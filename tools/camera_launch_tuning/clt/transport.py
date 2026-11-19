"""Robot link: file exchange over ADB (or a local folder for tests and dry runs).

Protocol (version 1), all files in REMOTE_DIR on the Control Hub:
  status.json   robot -> laptop. Rewritten atomically (temp file + rename) every ~250 ms, and on
                every phase change. Not rewritten during the burst itself.
  command.json  laptop -> robot. Pushed as command.json.tmp, then renamed with `mv -f`, so the
                robot never reads half a file. The robot polls it every ~250 ms.
  trials/trial_<robot_run_id>_<NNNN>.json / .csv   written by the robot when a trial ends.
See AGENT_GUIDE.md for the field list.
"""

import json
import os
import shutil
import subprocess
import tempfile
import time
from pathlib import Path

from . import PROTOCOL_VERSION

REMOTE_DIR = "/sdcard/FIRST/data/camera_launch_tuning"


class TransportError(RuntimeError):
    pass


def trial_basename(robot_run_id, trial_id):
    return f"trial_{robot_run_id}_{int(trial_id):04d}"


def build_command(seq, session_id, robot_run_id=None, settings=None, fire_trial=None,
                  refill_authorized_trial=None, revoked_trials=(), link_required=None):
    """The laptop -> robot command document. Only the fields given are included."""
    cmd = {"protocol": PROTOCOL_VERSION, "seq": int(seq), "session_id": session_id, "sent_ms": int(time.time() * 1000)}
    if robot_run_id:
        cmd["robot_run_id"] = robot_run_id
    if settings is not None:
        keys = ("settings_id", "target_rpm", "p", "ks", "kv", "transfer_power", "max_fire_ms", "turret_position")
        missing = [k for k in keys if settings.get(k) is None]
        if missing:
            raise ValueError(f"settings missing {missing}")
        cmd["settings"] = {k: settings[k] for k in keys}
    if fire_trial is not None:
        cmd["fire_trial"] = int(fire_trial)
    if refill_authorized_trial is not None:
        cmd["refill_authorized_trial"] = int(refill_authorized_trial)
    cmd["revoked_trials"] = sorted(int(t) for t in revoked_trials)
    if link_required is not None:
        cmd["link_required"] = bool(link_required)
    return cmd


def check_status(status):
    if not isinstance(status, dict) or status.get("protocol") != PROTOCOL_VERSION:
        raise TransportError(f"robot status protocol mismatch (expected {PROTOCOL_VERSION}): is CameraLaunchTuner running?")
    return status


class AdbTransport:
    """Talks to the Control Hub with the `adb` command line tool (USB or Wi-Fi adb)."""

    def __init__(self, adb="adb", serial=None, remote_dir=REMOTE_DIR, runner=subprocess.run, timeout_s=10,
                 connect=None):
        self.adb = adb
        # Wi-Fi ADB address (e.g. 192.168.43.1:5555). When set, it is also the device serial, and a lost
        # connection (the adb server restarted, e.g. by Android Studio) is re-made once per failing call.
        self.connect_addr = connect
        self.serial = serial or connect
        self.remote_dir = remote_dir.rstrip("/")
        self.runner = runner
        self.timeout_s = timeout_s

    def _cmd(self, *args):
        base = [self.adb]
        if self.serial:
            base += ["-s", self.serial]
        return base + list(args)

    def _run(self, *args, text=True):
        try:
            return self._run_once(*args, text=text)
        except TransportError as e:
            msg = str(e).lower()
            lost = any(s in msg for s in ("no devices", "not found", "offline", "device unauthorized"))
            if not (self.connect_addr and lost and args[:1] != ("connect",)):
                raise
            self.reconnect()
            return self._run_once(*args, text=text)

    def reconnect(self):
        """`adb connect` to the Wi-Fi address. Raises TransportError if it can't."""
        out = self.runner([self.adb, "connect", self.connect_addr], capture_output=True, text=True, timeout=self.timeout_s)
        text = ((out.stdout or "") + (out.stderr or "")).strip()
        if "connected to" not in text:
            hint = (" (macOS: allow your terminal app under System Settings > Privacy & Security > Local Network,"
                    " then run `adb kill-server`)") if "no route to host" in text.lower() else ""
            raise TransportError(f"adb connect {self.connect_addr} failed: {text[:200]}{hint}")
        return text

    def _run_once(self, *args, text=True):
        try:
            res = self.runner(self._cmd(*args), capture_output=True, text=text, timeout=self.timeout_s)
        except FileNotFoundError as e:
            raise TransportError(f"adb not found ({self.adb}): install Android platform-tools or pass --adb") from e
        except subprocess.TimeoutExpired as e:
            raise TransportError(f"adb timed out: {' '.join(args)}") from e
        if res.returncode != 0:
            raise TransportError(f"adb {' '.join(args)} failed: {(res.stderr or '').strip()[:300]}")
        return res.stdout

    def devices(self):
        out = self._run("devices")
        return [line.split()[0] for line in out.splitlines()[1:] if line.strip().endswith("device")]

    def read_status(self, attempts=3):
        # The robot rewrites status.json a few times a second; a read can land mid-rewrite, so retry briefly.
        for i in range(attempts):
            out = self._run("exec-out", "cat", f"{self.remote_dir}/status.json")
            try:
                return check_status(json.loads(out))
            except json.JSONDecodeError as e:
                if i == attempts - 1:
                    raise TransportError("status.json not readable (is CameraLaunchTuner initialized?)") from e
                time.sleep(0.1)

    def write_command(self, command):
        fd, tmp = tempfile.mkstemp(suffix=".json")
        try:
            with os.fdopen(fd, "w", encoding="utf-8") as f:
                json.dump(command, f, indent=2)
            self._run("push", tmp, f"{self.remote_dir}/command.json.tmp")
            self._run("shell", "mv", "-f", f"{self.remote_dir}/command.json.tmp", f"{self.remote_dir}/command.json")
        finally:
            os.unlink(tmp)

    def fetch_trial(self, robot_run_id, trial_id, dest_dir):
        dest = Path(dest_dir)
        dest.mkdir(parents=True, exist_ok=True)
        base = trial_basename(robot_run_id, trial_id)
        out = {}
        for ext in ("json", "csv"):
            local = dest / f"robot_{base}.{ext}"
            self._run("pull", f"{self.remote_dir}/trials/{base}.{ext}", str(local))
            out[ext] = local
        return out["json"], out["csv"]

    def clock_offset(self, samples=5, status=None):
        """Estimates robot_wall_ms - laptop_ms. Returns (offset_ms, uncertainty_ms, method).

        Tries `date +%s%N` on the hub (ms resolution if supported); otherwise /proc/uptime combined
        with the wall/boot pair in status.json. Uses the sample with the shortest round trip.
        """
        best = None
        for _ in range(samples):
            t0 = time.time() * 1000
            out = self._run("shell", "date", "+%s%N").strip()
            t1 = time.time() * 1000
            if out.isdigit() and len(out) >= 16:
                robot_ms = int(out) / 1e6
                cand = (t1 - t0, robot_ms - (t0 + t1) / 2, "date+%N")
                best = cand if best is None or cand[0] < best[0] else best
        if best is None:
            status = status or self.read_status()
            wall_minus_boot = status["wall_ms"] - status["boot_ms"]
            for _ in range(samples):
                t0 = time.time() * 1000
                out = self._run("shell", "cat", "/proc/uptime").split()
                t1 = time.time() * 1000
                robot_ms = float(out[0]) * 1000 + wall_minus_boot
                cand = (t1 - t0, robot_ms - (t0 + t1) / 2, "uptime")
                best = cand if best is None or cand[0] < best[0] else best
        rtt, offset, method = best
        resolution = 1 if method == "date+%N" else 10
        return offset, rtt / 2 + resolution, method


class LocalDirTransport:
    """Same interface on a local folder laid out like REMOTE_DIR. For tests and simulators."""

    def __init__(self, root):
        self.root = Path(root)
        (self.root / "trials").mkdir(parents=True, exist_ok=True)

    def devices(self):
        return ["local"]

    def read_status(self):
        p = self.root / "status.json"
        if not p.exists():
            raise TransportError("no status.json")
        return check_status(json.loads(p.read_text(encoding="utf-8")))

    def write_command(self, command):
        tmp = self.root / "command.json.tmp"
        tmp.write_text(json.dumps(command, indent=2), encoding="utf-8")
        os.replace(tmp, self.root / "command.json")

    def read_command(self):
        p = self.root / "command.json"
        return json.loads(p.read_text(encoding="utf-8")) if p.exists() else None

    def fetch_trial(self, robot_run_id, trial_id, dest_dir):
        dest = Path(dest_dir)
        dest.mkdir(parents=True, exist_ok=True)
        base = trial_basename(robot_run_id, trial_id)
        paths = []
        for ext in ("json", "csv"):
            src = self.root / "trials" / f"{base}.{ext}"
            if not src.exists():
                raise TransportError(f"missing {src.name}")
            dst = dest / f"robot_{base}.{ext}"
            shutil.copyfile(src, dst)
            paths.append(dst)
        return paths[0], paths[1]

    def clock_offset(self, samples=5, status=None):
        return 0.0, 1.0, "local"
