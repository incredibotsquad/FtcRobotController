"""Laptop camera capture (OpenCV), per-trial recording, and review-frame extraction.

OpenCV is imported lazily so the rest of the tool (and the tests) work without it.
Every recorded frame gets a laptop timestamp (time.time() in ms) in frames.csv; robot telemetry
is mapped onto those timestamps with the clock offset measured over ADB.
"""

import csv
import os
import threading
import time
from pathlib import Path


def _cv2():
    os.environ.setdefault("OPENCV_LOG_LEVEL", "ERROR")  # hide backend warnings from device probing
    try:
        import cv2  # noqa: PLC0415
    except ImportError as e:
        raise RuntimeError("OpenCV is not installed: pip install -r requirements.txt") from e
    return cv2


def list_cameras(max_index=6):
    """Cameras that open and return a frame. Device numbers are contiguous, so the scan stops at the
    first number that doesn't open (fewer "out device of bound" messages from OpenCV)."""
    cv2 = _cv2()
    found = []
    for i in range(max_index):
        cap = cv2.VideoCapture(i)
        opened = cap.isOpened()
        ok, frame = cap.read() if opened else (False, None)
        cap.release()
        if not opened:
            break
        if ok and frame is not None:
            found.append({"index": i, "width": frame.shape[1], "height": frame.shape[0]})
    return found


class Recorder:
    """Keeps a camera open and records clips on demand (start() ... stop())."""

    def __init__(self, index=0, width=1280, height=720, fps=60):
        self.index, self.width, self.height, self.fps = index, width, height, fps
        self._cap = None
        self._thread = None
        self._stop = threading.Event()
        self._writer = None
        self._frames = []
        self.actual_fps = None

    def open(self):
        cv2 = _cv2()
        cap = cv2.VideoCapture(self.index)
        if not cap.isOpened():
            raise RuntimeError(f"camera {self.index} can't be opened (check permissions / index: run `cameras`)")
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        cap.set(cv2.CAP_PROP_FPS, self.fps)
        ok, frame = cap.read()
        if not ok:
            cap.release()
            raise RuntimeError(f"camera {self.index} returned no frame")
        self.width, self.height = frame.shape[1], frame.shape[0]
        self.actual_fps = cap.get(cv2.CAP_PROP_FPS) or self.fps
        self._cap = cap
        return self

    def snapshot(self, path):
        cv2 = _cv2()
        ok, frame = self._cap.read()
        if ok:
            cv2.imwrite(str(path), frame)
        return ok

    def start(self, video_path):
        cv2 = _cv2()
        self._frames = []
        self._stop.clear()
        fourcc = cv2.VideoWriter_fourcc(*"mp4v")
        self._writer = cv2.VideoWriter(str(video_path), fourcc, float(self.actual_fps), (self.width, self.height))
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def _loop(self):
        while not self._stop.is_set():
            ok, frame = self._cap.read()
            t = time.time() * 1000
            if not ok:
                continue
            self._writer.write(frame)
            self._frames.append(t)

    def stop(self, frames_csv_path):
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=5)
        if self._writer:
            self._writer.release()
        with open(frames_csv_path, "w", newline="", encoding="utf-8") as f:
            w = csv.writer(f)
            w.writerow(["frame", "laptop_ms"])
            for i, t in enumerate(self._frames):
                w.writerow([i, f"{t:.1f}"])
        return len(self._frames)

    def close(self):
        if self._cap is not None:
            self._cap.release()
            self._cap = None


def read_frame_times(frames_csv_path):
    with open(frames_csv_path, newline="", encoding="utf-8") as f:
        return [float(r["laptop_ms"]) for r in csv.DictReader(f)]


def choose_review_times(fire_ms, end_ms, count=16, pre_ms=300, post_ms=1000):
    """Evenly spaced review times (robot wall ms) from just before the gate opens to after it ends."""
    start = fire_ms - pre_ms
    stop = (end_ms if end_ms is not None else fire_ms + 3000) + post_ms
    if count <= 1:
        return [start]
    step = (stop - start) / (count - 1)
    return [start + i * step for i in range(count)]


def nearest_frames(frame_times, wanted_laptop_ms):
    """Index of the recorded frame nearest each wanted time (None when outside the clip)."""
    out = []
    for w in wanted_laptop_ms:
        if not frame_times or w < frame_times[0] - 100 or w > frame_times[-1] + 100:
            out.append(None)
            continue
        out.append(min(range(len(frame_times)), key=lambda i: abs(frame_times[i] - w)))
    return out


def extract_frames(video_path, frame_indices, out_dir, max_side=960):
    """Saves the given frame indices as JPEGs (downscaled so the long side is <= max_side)."""
    cv2 = _cv2()
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    wanted = sorted({i for i in frame_indices if i is not None})
    saved = {}
    cap = cv2.VideoCapture(str(video_path))
    i = 0
    try:
        while wanted:
            ok, frame = cap.read()
            if not ok:
                break
            if i == wanted[0]:
                h, w = frame.shape[:2]
                scale = min(1.0, max_side / max(h, w))
                if scale < 1.0:
                    frame = cv2.resize(frame, (int(w * scale), int(h * scale)))
                p = out_dir / f"frame_{i:05d}.jpg"
                cv2.imwrite(str(p), frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
                saved[i] = p
                wanted.pop(0)
            i += 1
    finally:
        cap.release()
    return saved
