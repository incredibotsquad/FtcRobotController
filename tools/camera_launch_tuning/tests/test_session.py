import json
import os
import tempfile
import unittest
from pathlib import Path

from helpers import BASE_SETTINGS, assessment, bounds, robot_summary, write_json, write_telemetry
from clt.providers.base import Reviewer
from clt.session import Session
from clt.settings import with_id
from clt.store import RunStore
from clt.telemetry import compute_metrics, read_csv
from clt.transport import LocalDirTransport


class ScriptedReviewer(Reviewer):
    name = "scripted"

    def __init__(self, answers):
        self.answers = list(answers)
        self.requests = []

    def review(self, req, folder=None):
        self.requests.append(req)
        return self.answers.pop(0)


class AutoAckTransport(LocalDirTransport):
    """Local robot folder whose status echoes the last settings pushed (an idle robot)."""

    def write_command(self, command):
        super().write_command(command)
        st = {"protocol": 1, "robot_run_id": "R1", "phase": "IDLE", "trial_id": 1, "settings_error": "",
              "settings": command.get("settings") or {}}
        (self.root / "status.json").write_text(json.dumps(st))


class TelemetryMetrics(unittest.TestCase):
    def test_dips_recovery_and_flags(self):
        with tempfile.TemporaryDirectory() as d:
            p = Path(d) / "t.csv"
            write_telemetry(p, dips=((100, 150, 120), (400, 150, 600)))
            m = compute_metrics(read_csv(p), robot_summary())
        self.assertEqual(len(m["dips"]), 2)
        self.assertLessEqual(m["dips"][0]["recovery_ms"], 150)
        self.assertTrue(m["recovery_slow"])
        self.assertFalse(m["tracking_good"])
        self.assertEqual(m["count_drop_times_ms"], [100, 400])

    def test_steady_bias(self):
        with tempfile.TemporaryDirectory() as d:
            p = Path(d) / "t.csv"
            write_telemetry(p, steady_error=-90, dips=())
            m = compute_metrics(read_csv(p), robot_summary())
        self.assertTrue(m["steady_bias"])
        self.assertAlmostEqual(m["steady"]["mean_error_rpm"], -90, delta=1)


class ProcessTrial(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        root = Path(self.tmp.name)
        self.store = RunStore.create(root / "runs", "right1", "3of4", BASE_SETTINGS, bounds(), {}, run_id="r")
        self.transport = AutoAckTransport(root / "robot")
        self.files = root / "robotfiles"
        self.files.mkdir()

    def tearDown(self):
        self.tmp.cleanup()

    def session(self, answers, auto_apply=True):
        s = Session(self.store, self.transport, ScriptedReviewer(answers), config={"auto_apply": auto_apply},
                    say=lambda *_: None, ask=lambda _: "", sleep=lambda _: None)
        s.robot_run_id = "R1"
        return s

    def trial(self, session, robot_trial, count_at_end=0, frames=True):
        rec = self.store.new_trial(self.store.current_settings(), "R1", robot_trial)
        sp, cp = self.files / f"s{robot_trial}.json", self.files / f"t{robot_trial}.csv"
        write_json(sp, robot_summary(robot_trial, settings=self.store.current_settings(), count_at_end=count_at_end))
        write_telemetry(cp)
        session.review_frames = lambda *a: [{"path": "x.jpg", "t_from_gate_open_ms": 0.0}] if frames else []
        return session.process_trial(rec, sp, cp)

    def test_stuck_fourth_meets_goal_but_blocks_refill(self):
        s = self.session([assessment(["scored", "scored", "scored", "stuck"])])
        rec = self.trial(s, 1)
        self.assertEqual(rec["outcome"], "success")
        self.assertFalse(rec["refill_eligible"])
        self.assertIsNone(self.transport.read_command().get("refill_authorized_trial"))

    def test_all_confirmed_out_authorizes_refill_and_undo_revokes(self):
        s = self.session([assessment(["scored", "scored", "short", "scored"])])
        self.trial(s, 1)
        self.assertEqual(self.transport.read_command()["refill_authorized_trial"], 1)
        s.undo_last("test")
        cmd = self.transport.read_command()
        self.assertNotIn("refill_authorized_trial", cmd)
        self.assertEqual(cmd["revoked_trials"], [1])

    def test_valid_proposal_applied_and_sent_invalid_rejected(self):
        prop = {"parameter": "target_rpm", "new_value": 2768, "reason": "short"}
        s = self.session([assessment(["short", "short", "scored", "short"], proposal=prop),
                          assessment(["short", "short", "scored", "short"],
                                     proposal={"parameter": "target_rpm", "new_value": 9999, "reason": "x"})])
        rec = self.trial(s, 1)
        self.assertEqual(rec["accepted_change"]["new"], 2768.0)
        self.assertEqual(self.transport.read_command()["settings"]["target_rpm"], 2768.0)
        rec2 = self.trial(s, 2)
        self.assertIsNone(rec2["accepted_change"])
        self.assertEqual(self.store.current_settings()["target_rpm"], 2768.0)

    def test_no_frames_means_unknown_not_invented(self):
        s = self.session([])
        s.reviewer.needs_frames = True
        rec = self.trial(s, 1, frames=False)
        self.assertEqual(rec["summary"]["unknown"], 4)
        self.assertEqual(rec["outcome"], "inconclusive")

    def test_keys_never_saved(self):
        os.environ["OPENAI_API_KEY"] = "sk-never-saved-0123456789"
        try:
            s = self.session([assessment(["scored"] * 4)])
            self.trial(s, 1)
            for p in self.store.dir.rglob("*"):
                if p.is_file():
                    self.assertNotIn("sk-never-saved-0123456789", p.read_text(errors="ignore"), p)
        finally:
            del os.environ["OPENAI_API_KEY"]

    def test_three_successes_validate(self):
        s = self.session([assessment(["scored", "scored", "scored", "stuck"])] * 3)
        for i in (1, 2, 3):
            self.trial(s, i)
        from clt import policy
        v = policy.validation_status(self.store.trials())
        self.assertTrue(v["validated"])
        self.assertEqual(with_id(self.store.trials()[0]["settings"])["settings_id"],
                         with_id(self.store.trials()[2]["settings"])["settings_id"])


if __name__ == "__main__":
    unittest.main()


class SimulatedRobot(LocalDirTransport):
    """Walks IDLE -> LOADING -> SPINUP -> (fire permission) -> LAUNCHING -> IDLE, like CameraLaunchTuner."""

    def __init__(self, root, files_root):
        super().__init__(root)
        self.files_root = Path(files_root)
        self.phase, self.trial, self.reads, self.settings = "IDLE", 0, 0, {}
        self.fired_with_permission = False

    def write_command(self, command):
        super().write_command(command)
        if command.get("settings") and self.phase == "IDLE":
            self.settings = command["settings"]
        if command.get("fire_trial") == self.trial and self.phase == "SPINUP":
            self.fired_with_permission = True
            self.phase = "LAUNCHING"

    def read_status(self):
        self.reads += 1
        if self.phase == "IDLE" and self.reads > 3 and self.trial == 0:
            self.trial, self.phase = 1, "LOADING"
        elif self.phase == "LOADING":
            self.phase = "SPINUP"
        elif self.phase == "LAUNCHING":
            self.phase = "IDLE"
            base = self.root / "trials" / "trial_R1_0001"
            write_json(str(base) + ".json", robot_summary(1, settings=self.settings))
            write_telemetry(str(base) + ".csv")
        return {"protocol": 1, "robot_run_id": "R1", "phase": self.phase, "trial_id": self.trial,
                "trial_open": self.phase != "IDLE", "settings": self.settings, "trial_settings": self.settings,
                "settings_error": "", "wall_ms": 0, "boot_ms": 0}


class FakeRecorder:
    def __init__(self):
        self.started = self.stopped = False

    def start(self, path):
        self.started = True

    def stop(self, frames_csv):
        self.stopped = True
        Path(frames_csv).write_text("frame,laptop_ms\n")


class EndToEndSimulated(unittest.TestCase):
    def test_one_trial_handshake(self):
        with tempfile.TemporaryDirectory() as d:
            store = RunStore.create(Path(d) / "runs", "right1", "3of4", BASE_SETTINGS, bounds(), {}, run_id="r")
            robot = SimulatedRobot(Path(d) / "robot", d)
            rec_ = FakeRecorder()
            answers = iter(["", "q"])
            s = Session(store, robot, ScriptedReviewer([assessment(["scored", "scored", "scored", "stuck"])]),
                        recorder=rec_, config={"max_trials": 1}, say=lambda *_: None, ask=lambda _: next(answers),
                        sleep=lambda _: None)
            s.review_frames = lambda *a: [{"path": "x.jpg", "t_from_gate_open_ms": 0.0}]
            s.run()
            self.assertTrue(robot.fired_with_permission)
            self.assertTrue(rec_.started and rec_.stopped)
            t = store.trials()[0]
            self.assertEqual((t["outcome"], t["refill_eligible"]), ("success", False))
            self.assertTrue((store.trial_dir(1) / "decision.json").exists())


class FramePlan(unittest.TestCase):
    def test_windows_start_at_encoder_dips(self):
        from clt.frames import plan
        summary = robot_summary()  # fire 10000, end 11200
        metrics = {"dips": [{"start_ms": 150}, {"start_ms": 180}, {"start_ms": 400}]}  # 180 is the same ball
        items = plan(summary, metrics, 4)
        balls = [i for i in items if i["kind"] == "ball"]
        self.assertEqual(len(balls), 4 * 9)
        firsts = [b["robot_ms"] for b in balls if b["caption"].startswith(f"ball {b['ball']} window") and "-33 ms" in b["caption"]]
        self.assertEqual(firsts[:2], [10150 - 33, 10400 - 33])
        self.assertIn("estimated", [b for b in balls if b["ball"] == 3][0]["caption"])
        self.assertEqual(items[0]["kind"], "reference")
        self.assertEqual([i["kind"] for i in items[-2:]], ["rest", "rest"])


class Calibration(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        root = Path(self.tmp.name)
        self.profiles = root / "profiles"
        self.tdir = root / "trial"
        (self.tdir / "review_frames").mkdir(parents=True)
        frames = []
        for k in (1, 2):
            p = self.tdir / "review_frames" / f"trail_ball_{k}.jpg"
            p.write_bytes(b"jpg")
            frames.append({"path": str(p), "kind": "trail", "ball": k, "t_from_gate_open_ms": 0, "caption": "t"})
        r = self.tdir / "review_frames" / "rest.jpg"
        r.write_bytes(b"jpg")
        frames.append({"path": str(r), "kind": "rest", "ball": None, "t_from_gate_open_ms": 0, "caption": "r"})
        write_json(self.tdir / "review_frames.json", {"frames": frames})

    def tearDown(self):
        self.tmp.cleanup()

    def test_example_roundtrip_and_prompt_order(self):
        from clt.examples import add_example, load_for_review, parse_labels
        from clt.providers.base import ReviewRequest, build_prompt, review_images
        self.assertEqual(parse_labels("l, l h k", 4), ["long", "long", "short", "stuck"])
        with self.assertRaises(ValueError):
            parse_labels("l l", 4)
        ex = add_example("spot", self.tdir, ["long", "short"], {"run": "r", "trial": 1}, root=self.profiles)
        exs = load_for_review("spot", 2, self.profiles)
        self.assertEqual(exs[0]["id"], ex)
        self.assertEqual(len(exs[0]["images"]), 3)
        req = ReviewRequest(trial_n=2, loaded_count=2, goal="3of4", settings={}, bounds={}, metrics={}, robot_summary={},
                            history=[], spot={}, frames=[{"path": "x.jpg", "t_from_gate_open_ms": 5.0, "caption": "ball 1"}],
                            examples=exs)
        imgs = review_images(req)
        self.assertTrue(imgs[0]["caption"].startswith("EXAMPLE"))
        self.assertTrue(imgs[-1]["caption"].startswith("THIS TRIAL"))
        self.assertIn("person_labels", build_prompt(req))
        with self.assertRaises(ValueError):
            add_example("spot", self.tdir, ["unknown", "unknown"], {}, root=self.profiles)

    def test_confirm_override_counts_agreement_and_saves_example(self):
        store = RunStore.create(Path(self.tmp.name) / "runs", "spot", "3of4", BASE_SETTINGS, bounds(), {}, run_id="r")
        answers = iter(["l l l h", "y"])
        s = Session(store, LocalDirTransport(Path(self.tmp.name) / "robot"), ScriptedReviewer([]),
                    profile={"spot": "spot"}, config={"confirm_labels": True}, say=lambda *_: None,
                    ask=lambda _: next(answers), sleep=lambda _: None, profiles_root=self.profiles)
        rec = store.new_trial(BASE_SETTINGS, "R1", 1)
        from clt import policy
        a = policy.normalize_assessment(assessment(["unknown", "unknown", "long", "short"]), 4)
        fixed, ag = s.confirm_labels(rec, a, 4, self.tdir)
        self.assertEqual([b["label"] for b in fixed["balls"]], ["long", "long", "long", "short"])
        self.assertEqual((ag["matched"], ag["total"], ag["corrected"]), (2, 4, True))
        from clt.examples import list_examples
        self.assertEqual(list_examples("spot", self.profiles)[0]["labels"], ["long", "long", "long", "short"])
