"""Choosing and preparing review frames for one trial.

Balls cross the view in about 0.3-0.5 s and can leave 150 ms apart, so evenly spaced frames miss
them. Instead, each ball gets its own window starting at its wheel contact (the start of its
flywheel dip in the encoder log): a burst of frames through its flight, plus one motion-trail
image (the window's moving pixels overlaid on its first frame) that shows the whole path in one
picture. A reference frame before the gate opens and rest-state frames after the burst are added.
When the encoder shows fewer dips than balls loaded, the remaining windows are spread over the
rest of the launch.
"""

from pathlib import Path

from .camera import choose_review_times, extract_frames, nearest_frames, read_frame_times
from .store import write_json

DEFAULTS = {
    "frames_per_ball": 9,        # burst frames per ball window
    "ball_frame_step_ms": 66,    # spacing inside a window (every other frame at 30 fps)
    "ball_window_lead_ms": 33,   # start this long before wheel contact
    "rest_offsets_ms": (500, 1200),  # rest-state frames after the launch ends
    "min_contact_gap_ms": 60,    # dips closer than this are the same ball
    "trail_images": True,
}


def ball_contacts(metrics, summary, loaded):
    """Robot wall-clock ms of each ball's wheel contact, in launch order (may be fewer than loaded)."""
    fire = summary["timestamps"]["fire_ms"]
    out = []
    for d in metrics.get("dips", []):
        t = fire + d["start_ms"]
        if not out or t - out[-1] >= DEFAULTS["min_contact_gap_ms"]:
            out.append(t)
    return out[:loaded]


def plan(summary, metrics, loaded, cfg=None):
    """[{"kind", "ball", "robot_ms", "caption"}] in display order."""
    c = dict(DEFAULTS, **(cfg or {}))
    ts = summary["timestamps"]
    fire, end = ts["fire_ms"], ts.get("launch_end_ms") or ts["fire_ms"] + 3000
    contacts = ball_contacts(metrics, summary, loaded)
    span = c["ball_frame_step_ms"] * (c["frames_per_ball"] - 1)
    if len(contacts) < loaded:  # spread the missing windows after the last known contact
        start = (contacts[-1] + span) if contacts else fire
        missing = loaded - len(contacts)
        step = max(span, (end - start) / max(missing, 1))
        contacts += [start + i * step for i in range(missing)]
    detected = len(ball_contacts(metrics, summary, loaded))
    items = [{"kind": "reference", "ball": None, "robot_ms": fire - 300,
              "caption": "reference: before the gate opens (balls still inside)"}]
    for k, cms in enumerate(contacts, start=1):
        how = "wheel contact from the encoder" if k <= detected else "estimated (no encoder dip found)"
        for i in range(c["frames_per_ball"]):
            off = -c["ball_window_lead_ms"] + i * c["ball_frame_step_ms"]
            items.append({"kind": "ball", "ball": k, "robot_ms": cms + off,
                          "caption": f"ball {k} window ({how}): {off:+d} ms from its wheel contact, "
                                     f"{cms + off - fire:+.0f} ms from gate open"})
    for off in c["rest_offsets_ms"]:
        items.append({"kind": "rest", "ball": None, "robot_ms": end + off,
                      "caption": f"rest state: {off} ms after the launch ended"})
    return items


def build_trail(paths, out_path, threshold=28):
    """Overlays the moving pixels of a frame sequence onto its first frame (a motion trail)."""
    from .camera import _cv2
    cv2 = _cv2()
    import numpy as np  # noqa: PLC0415  (installed with opencv-python)
    frames = [cv2.imread(str(p)) for p in paths]
    frames = [f for f in frames if f is not None]
    if len(frames) < 2:
        return None
    base = frames[0].copy()
    bg = cv2.GaussianBlur(cv2.cvtColor(frames[0], cv2.COLOR_BGR2GRAY), (5, 5), 0)
    kernel = np.ones((5, 5), np.uint8)
    for f in frames[1:]:
        g = cv2.GaussianBlur(cv2.cvtColor(f, cv2.COLOR_BGR2GRAY), (5, 5), 0)
        mask = cv2.dilate((cv2.absdiff(g, bg) > threshold).astype(np.uint8), kernel) > 0
        base[mask] = f[mask]
    cv2.imwrite(str(out_path), base, [cv2.IMWRITE_JPEG_QUALITY, 85])
    return out_path


def prepare(tdir, summary, metrics, video, frames_csv, offset, cfg=None):
    """Extracts the planned frames (and trails) for a trial; writes review_frames.json.

    offset = (robot_minus_laptop_ms, uncertainty_ms, method). Returns the frame list for review.
    """
    c = dict(DEFAULTS, **(cfg or {}))
    tdir = Path(tdir)
    ts = summary.get("timestamps", {})
    if video is None or frames_csv is None or ts.get("fire_ms") is None:
        return []
    loaded = summary.get("count_at_fire") or 4
    frame_times = read_frame_times(frames_csv)
    items = plan(summary, metrics, loaded, c)
    idx = nearest_frames(frame_times, [it["robot_ms"] - offset[0] for it in items])
    saved = extract_frames(video, idx, tdir / "review_frames", c.get("frame_max_side", 960))
    frames, by_ball = [], {}
    for it, i in zip(items, idx):
        if i is None or i not in saved:
            continue
        robot_ms = frame_times[i] + offset[0]
        f = {"path": str(saved[i]), "frame_index": i, "laptop_ms": frame_times[i], "kind": it["kind"], "ball": it["ball"],
             "t_from_gate_open_ms": robot_ms - ts["fire_ms"], "caption": it["caption"]}
        frames.append(f)
        if it["kind"] == "ball":
            by_ball.setdefault(it["ball"], []).append(f)
    if c["trail_images"]:
        out = []
        for f in frames:
            out.append(f)
            k = f["ball"]
            last_of_ball = f["kind"] == "ball" and by_ball[k][-1] is f
            if last_of_ball:
                p = build_trail([x["path"] for x in by_ball[k]], tdir / "review_frames" / f"trail_ball_{k}.jpg")
                if p:
                    out.append({"path": str(p), "kind": "trail", "ball": k, "frame_index": None, "laptop_ms": None,
                                "t_from_gate_open_ms": by_ball[k][0]["t_from_gate_open_ms"],
                                "caption": f"ball {k} MOTION TRAIL: everything that moved during its window, "
                                           f"overlaid on the window's first frame (shows the whole path)"})
        frames = out
    write_json(tdir / "review_frames.json", {"method": "per-ball windows", "clock_offset_ms": offset[0],
                                             "clock_uncertainty_ms": offset[1], "clock_method": offset[2],
                                             "frames": frames})
    return frames


def even_frames(tdir, summary, video, frames_csv, offset, count=16, pre_ms=300, post_ms=1000, max_side=960):
    """The old evenly spaced selection (kept for comparison)."""
    ts = summary["timestamps"]
    frame_times = read_frame_times(frames_csv)
    wanted = choose_review_times(ts["fire_ms"], ts.get("launch_end_ms"), count, pre_ms, post_ms)
    idx = nearest_frames(frame_times, [w - offset[0] for w in wanted])
    saved = extract_frames(video, idx, Path(tdir) / "review_frames", max_side)
    return [{"path": str(saved[i]), "t_from_gate_open_ms": frame_times[i] + offset[0] - ts["fire_ms"],
             "caption": f"{frame_times[i] + offset[0] - ts['fire_ms']:+.0f} ms from gate open"}
            for i in idx if i is not None and i in saved]
