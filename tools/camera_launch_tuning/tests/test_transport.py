import json
import subprocess
import tempfile
import unittest
from pathlib import Path

from helpers import BASE_SETTINGS
from clt.settings import with_id
from clt.transport import AdbTransport, LocalDirTransport, TransportError, build_command, check_status


class FakeRunner:
    def __init__(self, outputs=None):
        self.calls = []
        self.outputs = outputs or {}
        self.pushed = None

    def __call__(self, cmd, capture_output, text, timeout):
        self.calls.append(cmd)
        if "push" in cmd:
            self.pushed = json.loads(Path(cmd[cmd.index("push") + 1]).read_text())
        key = next((k for k in self.outputs if k in " ".join(cmd)), None)
        out = self.outputs.get(key, "")
        if callable(out):
            out = out()
        return subprocess.CompletedProcess(cmd, 0, stdout=out, stderr="")


class CommandContract(unittest.TestCase):
    def test_command_fields(self):
        s = with_id(BASE_SETTINGS)
        c = build_command(7, "sess", "R1", settings=s, fire_trial=3, refill_authorized_trial=2, revoked_trials=[5, 4],
                          link_required=True)
        self.assertEqual(c["protocol"], 1)
        self.assertEqual(c["settings"]["settings_id"], s["settings_id"])
        self.assertEqual(set(c["settings"]), {"settings_id", "target_rpm", "p", "ks", "kv", "transfer_power",
                                              "max_fire_ms", "turret_position"})
        self.assertEqual(c["revoked_trials"], [4, 5])
        self.assertEqual((c["fire_trial"], c["refill_authorized_trial"]), (3, 2))
        with self.assertRaises(ValueError):
            build_command(1, "s", settings=dict(s, kv=None))

    def test_status_protocol_checked(self):
        with self.assertRaises(TransportError):
            check_status({"protocol": 99})


class AdbContract(unittest.TestCase):
    def test_atomic_push_then_rename_with_serial(self):
        r = FakeRunner()
        t = AdbTransport(adb="adb", serial="ABC", runner=r)
        t.write_command(build_command(1, "s", "R1", settings=with_id(BASE_SETTINGS)))
        self.assertEqual(r.calls[0][:3], ["adb", "-s", "ABC"])
        self.assertEqual(r.calls[0][3], "push")
        self.assertTrue(r.calls[0][-1].endswith("/command.json.tmp"))
        self.assertEqual(r.calls[1][3:6], ["shell", "mv", "-f"])
        self.assertTrue(r.calls[1][-1].endswith("/command.json"))
        self.assertEqual(r.pushed["seq"], 1)

    def test_status_and_trial_paths(self):
        r = FakeRunner({"status.json": json.dumps({"protocol": 1, "phase": "IDLE"})})
        t = AdbTransport(runner=r)
        self.assertEqual(t.read_status()["phase"], "IDLE")
        self.assertEqual(r.calls[0][1:3], ["exec-out", "cat"])
        with tempfile.TemporaryDirectory() as d:
            j, c = t.fetch_trial("20261117-1200", 3, d)
        pulls = [cmd for cmd in r.calls if "pull" in cmd]
        self.assertTrue(pulls[0][2].endswith("/trials/trial_20261117-1200_0003.json"))
        self.assertTrue(pulls[1][2].endswith("/trials/trial_20261117-1200_0003.csv"))

    def test_clock_offset_with_and_without_nanoseconds(self):
        import time
        r = FakeRunner({"date": lambda: str(int((time.time() * 1000 + 5000) * 1e6))})
        off, unc, method = AdbTransport(runner=r).clock_offset(samples=3)
        self.assertEqual(method, "date+%N")
        self.assertAlmostEqual(off, 5000, delta=200)
        r2 = FakeRunner({"date": "1700000000N", "uptime": "100.00 50.00"})
        off2, _, method2 = AdbTransport(runner=r2).clock_offset(samples=2, status={"wall_ms": 1_000_000, "boot_ms": 90_000})
        self.assertEqual(method2, "uptime")

    def test_missing_adb(self):
        def boom(*a, **k):
            raise FileNotFoundError
        with self.assertRaises(TransportError):
            AdbTransport(adb="/nope/adb", runner=boom).devices()


class LocalContract(unittest.TestCase):
    def test_roundtrip(self):
        with tempfile.TemporaryDirectory() as d:
            t = LocalDirTransport(d)
            (Path(d) / "status.json").write_text(json.dumps({"protocol": 1, "phase": "IDLE"}))
            self.assertEqual(t.read_status()["phase"], "IDLE")
            t.write_command({"seq": 2})
            self.assertEqual(t.read_command()["seq"], 2)
            self.assertFalse((Path(d) / "command.json.tmp").exists())


if __name__ == "__main__":
    unittest.main()


class Reconnect(unittest.TestCase):
    def test_reconnects_once_after_lost_device(self):
        calls = []

        def runner(cmd, capture_output, text, timeout):
            calls.append(cmd)
            if cmd[1] == "connect":
                return subprocess.CompletedProcess(cmd, 0, stdout="connected to 192.168.43.1:5555", stderr="")
            if len([c for c in calls if "cat" in c]) == 1:
                return subprocess.CompletedProcess(cmd, 1, stdout="", stderr="error: no devices/emulators found")
            return subprocess.CompletedProcess(cmd, 0, stdout=json.dumps({"protocol": 1}), stderr="")

        t = AdbTransport(runner=runner, connect="192.168.43.1:5555")
        self.assertEqual(t.read_status()["protocol"], 1)
        self.assertEqual(calls[1], ["adb", "connect", "192.168.43.1:5555"])
        self.assertEqual(calls[0][1:3], ["-s", "192.168.43.1:5555"])

    def test_no_route_gives_local_network_hint(self):
        def runner(cmd, capture_output, text, timeout):
            if cmd[1] == "connect":
                return subprocess.CompletedProcess(cmd, 1, stdout="failed to connect to '192.168.43.1:5555': No route to host", stderr="")
            return subprocess.CompletedProcess(cmd, 1, stdout="", stderr="error: no devices/emulators found")

        with self.assertRaises(TransportError) as cm:
            AdbTransport(runner=runner, connect="192.168.43.1:5555").read_status()
        self.assertIn("Local Network", str(cm.exception))

    def test_without_connect_errors_pass_through(self):
        def runner(cmd, capture_output, text, timeout):
            return subprocess.CompletedProcess(cmd, 1, stdout="", stderr="error: no devices/emulators found")
        with self.assertRaises(TransportError):
            AdbTransport(runner=runner).read_status()
