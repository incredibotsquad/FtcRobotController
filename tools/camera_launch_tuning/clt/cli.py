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
    return AdbTransport(adb=find_adb(args.adb), serial=args.serial)


def add_link_args(p):
    p.add_argument("--adb", help="path to adb (default: PATH, then the usual Android SDK locations)")
    p.add_argument("--serial", help="adb device serial (when more than one device is connected)")
    p.add_argument("--local-robot-dir", help=argparse.SUPPRESS)  # tests / simulators


def cmd_doctor(args):
    print(f"camera_launch_tuning {__version__}, Python {sys.version.split()[0]}")
    for mod, why in (("cv2", "camera capture (opencv-python)"), ("openai", "OpenAI reviewer"),
                     ("anthropic", "Anthropic reviewer")):
        try:
            __import__(mod)
            print(f"  [ok]      {mod:9s} {why}")
        except ImportError:
            print(f"  [missing] {mod:9s} {why}")
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
    print(f"Run folder: {store.dir}")
    session = Session(store, transport, reviewer, recorder, profile, config={
        "goal": goal, "max_trials": args.max_trials, "auto_apply": args.auto_apply,
        "validation_required": args.validation_required, "review_frames": args.frames})
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
    req = ReviewRequest(trial_n=t["n"], loaded_count=(t.get("robot_summary") or {}).get("count_at_fire") or 4,
                        goal=store.run["goal"], settings=t["settings"], bounds=store.run["bounds"],
                        metrics=t.get("metrics") or {}, robot_summary=t.get("robot_summary") or {},
                        history=[h for h in store.context_history() if h["trial"] < t["n"]], spot={"spot": store.run["spot"]},
                        frames=frames)
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
    add_link_args(p); p.set_defaults(func=cmd_doctor)
    p = sub.add_parser("cameras", help="list cameras OpenCV can open"); p.set_defaults(func=cmd_cameras)
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
    p.add_argument("--reviewer", choices=["manual", "openai", "anthropic", "agent-file"], default="manual")
    p.add_argument("--goal", choices=sorted(policy.GOALS), default="3of4")
    p.add_argument("--max-trials", type=int, default=20)
    p.add_argument("--validation-required", type=int, default=3)
    p.add_argument("--auto-apply", action="store_true", help="apply validated proposals without asking")
    p.add_argument("--resume", help="continue an existing run (folder or run id)")
    p.add_argument("--start-from-selected", action="store_true", help="start from the spot's selected settings")
    p.add_argument("--camera", type=int, default=0)
    p.add_argument("--width", type=int, default=1280)
    p.add_argument("--height", type=int, default=720)
    p.add_argument("--fps", type=int, default=60)
    p.add_argument("--frames", type=int, default=16, help="review frames per trial")
    p.add_argument("--no-camera", action="store_true", help="no recording (manual scoring by eye)")
    p.add_argument("--agent-timeout", type=int, default=1800)
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
    p.add_argument("--reviewer", choices=["manual", "openai", "anthropic", "agent-file"], required=True)
    p.add_argument("--agent-timeout", type=int, default=1800)
    p.set_defaults(func=cmd_review)
    p = sub.add_parser("select", help="record a run's validated settings as the spot's selected profile")
    p.add_argument("--run", required=True); p.add_argument("--validation-required", type=int, default=3)
    p.add_argument("--unvalidated", action="store_true"); p.set_defaults(func=cmd_select)
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
