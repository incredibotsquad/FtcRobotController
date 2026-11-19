"""Command line interface. Run `python camera_launch_tuning.py --help`."""

import argparse
import json
import shutil
import sys
from pathlib import Path

from . import __version__, policy
from .env import describe_credentials, load_env
from .settings import DEFAULT_BOUNDS, PARAMS, from_production, merge_bounds, validate_settings, with_id
from .store import TOOL_DIR, RunStore, load_profile, new_profile, save_profile, select_settings
from .transport import AdbTransport, LocalDirTransport, TransportError

RUNS_DIR = TOOL_DIR / "runs"


def find_adb(explicit=None):
    if explicit:
        return explicit
    found = shutil.which("adb")
    if found:
        return found
    for base in (Path.home() / "Library/Android/sdk", Path.home() / "Android/Sdk",
                 Path.home() / "AppData/Local/Android/Sdk"):
        for name in ("adb", "adb.exe"):
            p = base / "platform-tools" / name
            if p.exists():
                return str(p)
    return "adb"


def make_transport(args):
    if getattr(args, "local_robot_dir", None):
        return LocalDirTransport(args.local_robot_dir)
    connect = getattr(args, "connect", None)
    if connect and ":" not in connect:
        connect += ":5555"
    t = AdbTransport(adb=find_adb(args.adb), serial=args.serial, connect=connect)
    if connect:
        try:
            t.reconnect()
        except TransportError as e:
            print(f"warning: {e}")
    return t


def add_link_args(p):
    p.add_argument("--adb", help="path to adb (default: PATH, then the usual Android SDK locations)")
    p.add_argument("--serial", help="adb device serial (when more than one device is connected)")
    p.add_argument("--connect", metavar="HOST[:PORT]",
                   help="Wi-Fi ADB: connect to the Control Hub (e.g. 192.168.43.1) and reconnect automatically if adb drops it")
    p.add_argument("--local-robot-dir", help=argparse.SUPPRESS)  # tests / simulators


def cmd_doctor(args):
    print(f"camera_launch_tuning {__version__}, Python {sys.version.split()[0]}")
    for mod, why in (("cv2", "camera capture (opencv-python)"), ("openai", "OpenAI reviewer (API key)"),
                     ("anthropic", "Anthropic reviewer (API key)")):
        try:
            __import__(mod)
            print(f"  [ok]      {mod:9s} {why}")
        except ImportError:
            print(f"  [missing] {mod:9s} {why}")
    from .providers import claude_cli
    ver = claude_cli.version(args.claude)
    if ver:
        print(f"  [ok]      claude    claude-cli reviewer, your Claude login ({ver})")
        if args.probe_claude:
            ok, detail = claude_cli.probe(args.claude)
            print(f"  [{'ok' if ok else 'FAIL'}]{' ' * (6 if ok else 4)}claude    {detail}")
    else:
        print("  [missing] claude    claude-cli reviewer (Claude Code CLI not on PATH; or pass --claude)")
    print("  credentials:", json.dumps(describe_credentials()))
    adb = find_adb(args.adb)
    print(f"  adb: {adb}")
    t = make_transport(args)
    try:
        print("  devices:", t.devices() or "none")
        st = t.read_status()
        print(f"  robot: {st['opmode']} run {st['robot_run_id']} phase {st['phase']}")
        off = t.clock_offset(status=st)
        print(f"  clock offset robot-laptop: {off[0]:.0f} ms (+/- {off[1]:.0f} ms, {off[2]})")
    except TransportError as e:
        print(f"  robot: not reachable ({e})")
    return 0


def cmd_cameras(args):
    from .camera import list_cameras
    for c in list_cameras():
        print(f"  camera {c['index']}: {c['width']}x{c['height']}")
    return 0


def cmd_preview(args):
    """Saves a camera snapshot (and, with --live, shows a live window) to check the view."""
    from .camera import Recorder
    rec = Recorder(args.camera, args.width, args.height, args.fps).open()
    try:
        out = Path(args.out)
        if not rec.snapshot(out):
            print("camera returned no frame")
            return 1
        print(f"saved {out.resolve()} ({rec.width}x{rec.height}) - open it and check: launcher exit, flight path, CELL opening")
        if args.live:
            import cv2
            print("live preview: press q in the window to close")
            while True:
                ok, frame = rec._cap.read()
                if ok:
                    cv2.imshow("camera_launch_tuning preview (q to close)", frame)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
            cv2.destroyAllWindows()
    finally:
        rec.close()
    return 0


def cmd_status(args):
    st = make_transport(args).read_status()
    keep = ("robot_run_id", "phase", "trial_id", "message", "artifact_count", "needs_empty_confirmation",
            "refill_authorized_trial", "settings", "settings_error", "left_rpm", "right_rpm", "battery_v",
            "s1_mm", "s1_baseline_mm", "s2_blocked", "s3_blocked", "s4_mm", "s4_baseline_mm")
    print(json.dumps({k: st.get(k) for k in keep}, indent=2))
    return 0


def cmd_spot_init(args):
    overrides = {k: getattr(args, k) for k in PARAMS if getattr(args, k) is not None}
    production = None
    if args.from_robot:
        production = make_transport(args).read_status()["production"]
        settings = from_production(production, args.distance, overrides.get("turret_position"))
        source = "robot production tables" + (f" at {args.distance} in" if args.distance is not None else "")
    else:
        settings = {k: None for k in PARAMS}
        source = "manual"
    settings.update(overrides)
    problems = validate_settings(settings, DEFAULT_BOUNDS)
    if problems:
        print("Starting settings incomplete or out of bounds:\n  " + "\n  ".join(problems))
        print("Give the missing values (e.g. --target-rpm 2668 or --from-robot --distance 48.75).")
        return 2
    try:
        load_profile(args.spot)
        if not args.force:
            print(f"Profile {args.spot!r} exists (use --force to replace its starting settings).")
            return 2
    except FileNotFoundError:
        pass
    prof = new_profile(args.spot, settings, args.description or "", args.distance, source)
    save_profile(prof)
    print(json.dumps(prof["starting_settings"], indent=2))
    return 0


def cmd_spot_show(args):
    print(json.dumps(load_profile(args.spot), indent=2))
    return 0


def build_reviewer(args):
    from .providers import make_reviewer
    if args.reviewer == "agent-file":
        return make_reviewer("agent-file", timeout_s=args.agent_timeout)
    if args.reviewer == "claude-cli":
        return make_reviewer("claude-cli", claude=args.claude, model=args.claude_model, timeout_s=args.agent_timeout)
    return make_reviewer(args.reviewer)


def cmd_run(args):
    from .session import Session
    profile = load_profile(args.spot)
    transport = make_transport(args)
    reviewer = build_reviewer(args)
    if args.resume:
        store = RunStore(RUNS_DIR / args.resume if not Path(args.resume).exists() else args.resume)
        goal = store.run["goal"]
    else:
        st = transport.read_status()
        bounds = merge_bounds(DEFAULT_BOUNDS, st.get("limits"))
        start = profile["selected"]["settings"] if (args.start_from_selected and profile.get("selected")) else profile["starting_settings"]
        problems = validate_settings(start, bounds)
        if problems:
            print("Starting settings outside the merged bounds:\n  " + "\n  ".join(problems))
            return 2
        goal = args.goal
        store = RunStore.create(RUNS_DIR, args.spot, goal, start, bounds,
                                {"max_trials": args.max_trials, "reviewer": args.reviewer},
                                robot_snapshot=st.get("production"))
    recorder = None
    if not args.no_camera:
        from .camera import Recorder
        recorder = Recorder(args.camera, args.width, args.height, args.fps).open()
        # Camera numbers aren't stable (macOS can list the built-in camera first or second), so
        # show the actual view before any trial. Watch which camera's light turns on, too.
        snap = store.dir / "camera_check.jpg"
        recorder.snapshot(snap)
        print(f"Camera {args.camera} view saved to {snap.resolve()}")
        if not args.skip_camera_check:
            if input("Open it: does it show the launcher exit, the flight path and the CELL? [y/N] ").strip().lower() != "y":
                recorder.close()
                print("Stopped. Try another --camera number (check with `preview --camera N`).")
                return 2
    print(f"Run folder: {store.dir}")
    session = Session(store, transport, reviewer, recorder, profile, config={
        "goal": goal, "max_trials": args.max_trials, "auto_apply": args.auto_apply,
        "validation_required": args.validation_required, "frames_per_ball": args.frames_per_ball,
        "confirm_labels": args.confirm_labels, "max_examples": args.max_examples})
    try:
        v = session.run()
    finally:
        if recorder:
            recorder.close()
    print(f"Done. Validation streak {v['streak']}/{v['required']}. Run folder: {store.dir}")
    return 0


def resolve_run(arg):
    p = Path(arg)
    return RunStore(p if p.exists() else RUNS_DIR / arg)


def cmd_undo(args):
    store = resolve_run(args.run)
    rec, restored = store.discard_last(args.reason, "cli")
    print(f"Discarded trial {rec['n']} (files kept). Next settings: {json.dumps(restored)}")
    if args.push:
        from .transport import build_command
        import time
        t = make_transport(args)
        st = t.read_status()
        revoked = store.revoked_robot_trials(st["robot_run_id"])
        t.write_command(build_command(int(time.time()), "cli-undo", st["robot_run_id"], settings=restored,
                                      revoked_trials=revoked, link_required=True))
        store.audit("push_command", source="cli-undo", settings_id=restored["settings_id"], revoked_trials=revoked)
        print("Pushed; the robot applies it at its next idle checkpoint (never mid-shot).")
    return 0


def cmd_history(args):
    store = resolve_run(args.run)
    for t in store.trials():
        labels = " ".join((t.get("summary") or {}).get("labels") or [])
        ch = t.get("accepted_change")
        ag = t.get("label_agreement")
        if ag and ag.get("corrected"):
            labels += f"  (reviewer said {' '.join(ag['ai'])})"
        print(f"{t['n']:4d} {t['status']:9s} {t.get('outcome') or '-':12s} {labels:40s} "
              f"{t['settings']['settings_id']} {'change ' + ch['parameter'] + ' -> ' + str(ch['new']) if ch else ''}")
    v = policy.validation_status(store.trials(), args.validation_required)
    print(f"validation streak {v['streak']}/{v['required']}; current settings {store.current_settings()['settings_id']}")
    return 0


def cmd_review(args):
    """Re-reviews a saved trial with another reviewer. Saves the answer; changes no decisions."""
    from .providers.base import ReviewRequest
    from .store import read_json, write_json
    import time
    store = resolve_run(args.run)
    t = store.trials()[args.trial - 1]
    tdir = store.trial_dir(args.trial)
    frames = read_json(tdir / "review_frames.json")["frames"] if (tdir / "review_frames.json").exists() else []
    if args.reframe:
        from .frames import prepare
        prev = read_json(tdir / "review_frames.json") if (tdir / "review_frames.json").exists() else {}
        offset = (prev.get("clock_offset_ms", 0.0), prev.get("clock_uncertainty_ms"), prev.get("clock_method", "saved"))
        summary = read_json(next(tdir.glob("robot_trial_*.json")))
        frames = prepare(tdir, summary, t.get("metrics") or {}, tdir / "video.mp4", tdir / "frames.csv", offset,
                         {"frames_per_ball": args.frames_per_ball})
        print(f"re-selected {len(frames)} images (per-ball windows + trails)")
    req = ReviewRequest(trial_n=t["n"], loaded_count=(t.get("robot_summary") or {}).get("count_at_fire") or 4,
                        goal=store.run["goal"], settings=t["settings"], bounds=store.run["bounds"],
                        metrics=t.get("metrics") or {}, robot_summary=t.get("robot_summary") or {},
                        history=[h for h in store.context_history() if h["trial"] < t["n"]], spot={"spot": store.run["spot"]},
                        frames=frames, examples=_examples_for(store.run["spot"], args.max_examples, exclude_trial=(store.run["run_id"], t["n"])))
    raw = build_reviewer(args).review(req, folder=tdir / f"rereview_{int(time.time())}")
    norm = policy.normalize_assessment(raw, req.loaded_count)
    out = tdir / f"rereview_{args.reviewer}_{int(time.time())}.json"
    write_json(out, {"raw": raw, "normalized": norm, "summary": policy.summarize(norm)})
    print(json.dumps(policy.summarize(norm), indent=2))
    print(f"saved {out} (decisions unchanged)")
    return 0


def cmd_select(args):
    store = resolve_run(args.run)
    v = policy.validation_status(store.trials(), args.validation_required)
    if not v["validated"] and not args.unvalidated:
        print(f"Not validated (streak {v['streak']}/{v['required']}). Use --unvalidated to record it anyway.")
        return 2
    prof = load_profile(store.run["spot"])
    settings = store.kept_trials()[-1]["settings"]
    select_settings(prof, settings, {"run": store.run["run_id"], "trials": v["trials"], "validated": v["validated"]})
    save_profile(prof)
    print(f"Selected {settings['settings_id']} for spot {prof['spot']!r}.")
    return 0


def cmd_example_add(args):
    from .examples import add_example, parse_labels
    store = resolve_run(args.run)
    t = store.trials()[args.trial - 1]
    loaded = (t.get("robot_summary") or {}).get("count_at_fire") or 4
    if args.labels:
        labels = parse_labels(args.labels, loaded)
    else:
        a = read_json_safe(store.trial_dir(args.trial) / "assessment.json")
        human = t.get("reviewer") == "manual" or "operator_corrected" in (a or {}).get("flags", [])
        if not a or not human:
            print("This trial's labels aren't from a person: give them with --labels (e.g. --labels 'l l l h').")
            return 2
        labels = [b["label"] for b in a["balls"]]
    ex = add_example(store.run["spot"], store.trial_dir(args.trial), labels,
                     {"run": store.run["run_id"], "trial": args.trial}, note=args.note or "")
    print(f"Saved example {ex} for spot {store.run['spot']!r}: {labels}")
    return 0


def cmd_example_list(args):
    from .examples import list_examples
    for ex in list_examples(args.spot):
        print(f"{ex['id']}  {' '.join(ex['labels']):32s} from run {ex['source'].get('run')} trial {ex['source'].get('trial')}  {ex.get('note', '')}")
    return 0


def cmd_example_remove(args):
    from .examples import remove_example
    remove_example(args.spot, args.id)
    print(f"Removed example {args.id}")
    return 0


def _examples_for(spot, limit, exclude_trial=None):
    """Calibration examples for a review, never including the trial being reviewed (that would leak its answer)."""
    from .examples import list_examples, load_for_review
    keep = [e for e in list_examples(spot) if (e["source"].get("run"), e["source"].get("trial")) != exclude_trial]
    ids = {e["id"] for e in keep[-limit:]} if limit > 0 else set()
    return [e for e in load_for_review(spot, 999) if e["id"] in ids]


def read_json_safe(path):
    from .store import read_json
    return read_json(path) if Path(path).exists() else None


def cmd_export(args):
    prof = load_profile(args.spot)
    sel = prof.get("selected")
    if not sel:
        print("No selected settings for this spot yet.")
        return 2
    s = sel["settings"]
    d = prof.get("distance_in")
    print(f"// Spot {prof['spot']!r} ({prof.get('description') or 'no description'}), validated by {sel['evidence']}")
    print(f"// Review, then edit by hand. Distance {d} in (robot center to CELL).")
    print(f"// LaunchReadinessCommand.SHOT_DISTANCES_IN / SHOT_RPMS: put {s['target_rpm']:.0f} RPM at {d} in.")
    print(f"// LaunchSubsystem gain table row at {d} in: P {s['p']}, kS {s['ks']}, kV {s['kv']} (I/D unchanged).")
    print(f"// SmartIntakeCommand.LAUNCH_TRANSFER_POWER = {s['transfer_power']}; LaunchBallsCommand.MAX_FIRE_MS = {s['max_fire_ms']};")
    print(f"// turret_position {s['turret_position']} was a fixed aim for this spot: compare with TeleOp's auto-aim there.")
    return 0


def main(argv=None):
    ap = argparse.ArgumentParser(prog="camera_launch_tuning", description="Camera-assisted launch tuning for CameraLaunchTuner.")
    ap.add_argument("--env-file", help="explicit .env path (default: tools/camera_launch_tuning/.env if present)")
    ap.add_argument("--version", action="version", version=__version__)
    sub = ap.add_subparsers(dest="command", required=True)

    p = sub.add_parser("doctor", help="check dependencies, keys (presence only), adb, robot and clock sync")
    p.add_argument("--claude", help="path to the claude CLI (default: PATH)")
    p.add_argument("--probe-claude", action="store_true",
                   help="also make one tiny headless claude call to confirm login + structured output (uses a little usage)")
    add_link_args(p); p.set_defaults(func=cmd_doctor)
    p = sub.add_parser("cameras", help="list cameras OpenCV can open"); p.set_defaults(func=cmd_cameras)
    p = sub.add_parser("preview", help="snapshot (and optional live window) from the camera to check the view")
    p.add_argument("--camera", type=int, default=0)
    p.add_argument("--width", type=int, default=1280)
    p.add_argument("--height", type=int, default=720)
    p.add_argument("--fps", type=int, default=30)
    p.add_argument("--out", default="camera_preview.jpg")
    p.add_argument("--live", action="store_true", help="also open a live window (q closes it)")
    p.set_defaults(func=cmd_preview)
    p = sub.add_parser("status", help="print the robot's status.json"); add_link_args(p); p.set_defaults(func=cmd_status)

    p = sub.add_parser("spot-init", help="create a spot profile with starting settings")
    p.add_argument("--spot", required=True)
    p.add_argument("--description")
    p.add_argument("--distance", type=float, help="robot center to CELL, inches (for --from-robot)")
    p.add_argument("--from-robot", action="store_true", help="start from the robot's production tables")
    p.add_argument("--force", action="store_true")
    for k in PARAMS:
        p.add_argument("--" + k.replace("_", "-"), dest=k, type=float)
    add_link_args(p); p.set_defaults(func=cmd_spot_init)
    p = sub.add_parser("spot-show", help="print a spot profile"); p.add_argument("--spot", required=True); p.set_defaults(func=cmd_spot_show)

    p = sub.add_parser("run", help="tuning session with the robot")
    p.add_argument("--spot", required=True)
    p.add_argument("--reviewer", choices=["manual", "openai", "anthropic", "claude-cli", "agent-file"], default="manual")
    p.add_argument("--goal", choices=sorted(policy.GOALS), default="3of4")
    p.add_argument("--max-trials", type=int, default=20)
    p.add_argument("--validation-required", type=int, default=3)
    p.add_argument("--auto-apply", action="store_true", help="apply validated proposals without asking")
    p.add_argument("--confirm-labels", action="store_true",
                   help="after each AI review, accept (Enter) or correct its labels; corrections are used and can become examples")
    p.add_argument("--max-examples", type=int, default=2, help="calibration examples shown to the reviewer per trial (0 = none)")
    p.add_argument("--resume", help="continue an existing run (folder or run id)")
    p.add_argument("--start-from-selected", action="store_true", help="start from the spot's selected settings")
    p.add_argument("--camera", type=int, default=0)
    p.add_argument("--width", type=int, default=1280)
    p.add_argument("--height", type=int, default=720)
    p.add_argument("--fps", type=int, default=60)
    p.add_argument("--frames-per-ball", type=int, default=9, help="review frames per ball window (plus a motion trail per ball)")
    p.add_argument("--no-camera", action="store_true", help="no recording (manual scoring by eye)")
    p.add_argument("--skip-camera-check", action="store_true", help="don't ask to confirm the camera view at the start")
    p.add_argument("--agent-timeout", type=int, default=1800, help="agent-file / claude-cli: seconds to wait per review")
    p.add_argument("--claude", help="claude-cli: path to the claude CLI (default: PATH)")
    p.add_argument("--claude-model", help="claude-cli: model alias or name (default: the CLI's default)")
    add_link_args(p); p.set_defaults(func=cmd_run)

    p = sub.add_parser("undo", help="discard the immediately last kept trial of a saved run")
    p.add_argument("--run", required=True); p.add_argument("--reason", default="discarded from the command line")
    p.add_argument("--push", action="store_true", help="also send the restored settings to the connected robot")
    add_link_args(p); p.set_defaults(func=cmd_undo)
    p = sub.add_parser("history", help="list a run's trials")
    p.add_argument("--run", required=True); p.add_argument("--validation-required", type=int, default=3)
    p.set_defaults(func=cmd_history)
    p = sub.add_parser("review", help="re-review a saved trial (saves the answer, changes nothing)")
    p.add_argument("--run", required=True); p.add_argument("--trial", type=int, required=True)
    p.add_argument("--reviewer", choices=["manual", "openai", "anthropic", "claude-cli", "agent-file"], required=True)
    p.add_argument("--agent-timeout", type=int, default=1800)
    p.add_argument("--claude"); p.add_argument("--claude-model")
    p.add_argument("--reframe", action="store_true", help="re-select frames from the saved video with per-ball windows first")
    p.add_argument("--max-examples", type=int, default=2, help="calibration examples to show (0 = none)")
    p.add_argument("--frames-per-ball", type=int, default=9)
    p.set_defaults(func=cmd_review)
    p = sub.add_parser("select", help="record a run's validated settings as the spot's selected profile")
    p.add_argument("--run", required=True); p.add_argument("--validation-required", type=int, default=3)
    p.add_argument("--unvalidated", action="store_true"); p.set_defaults(func=cmd_select)
    p = sub.add_parser("example-add", help="save a person-labeled trial as a calibration example for its spot")
    p.add_argument("--run", required=True); p.add_argument("--trial", type=int, required=True)
    p.add_argument("--labels", help="the true labels in launch order, e.g. 'l l l h' (default: the trial's manual labels)")
    p.add_argument("--note"); p.set_defaults(func=cmd_example_add)
    p = sub.add_parser("example-list", help="list a spot's calibration examples")
    p.add_argument("--spot", required=True); p.set_defaults(func=cmd_example_list)
    p = sub.add_parser("example-remove", help="delete one calibration example")
    p.add_argument("--spot", required=True); p.add_argument("--id", required=True); p.set_defaults(func=cmd_example_remove)
    p = sub.add_parser("export", help="print a promotion note for a spot's selected settings (no files changed)")
    p.add_argument("--spot", required=True); p.set_defaults(func=cmd_export)

    args = ap.parse_args(argv)
    try:
        load_env(args.env_file)
    except FileNotFoundError as e:
        print(e)
        return 2
    try:
        return args.func(args)
    except (TransportError, FileNotFoundError, ValueError, RuntimeError, TimeoutError) as e:
        print(f"error: {e}")
        return 1
