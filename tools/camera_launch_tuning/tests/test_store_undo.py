import io
import json
import tempfile
import unittest
from contextlib import redirect_stdout
from pathlib import Path

from helpers import BASE_SETTINGS, bounds
from clt import cli, policy
from clt.store import RunStore


def make_run(root):
    return RunStore.create(root, "right1", "3of4", BASE_SETTINGS, bounds(), {}, run_id="run1")


def add_trial(store, robot_trial, outcome="failure", change=None, eligible=False):
    rec = store.new_trial(store.current_settings(), "R1", robot_trial)
    (store.trial_dir(rec["n"]) / "video.mp4").write_bytes(b"fake")
    return store.update_trial(rec["n"], outcome=outcome, accepted_change=change, refill_eligible=eligible,
                              summary={"labels": ["short"] * 4})


class UndoTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.store = make_run(self.tmp.name)

    def tearDown(self):
        self.tmp.cleanup()

    def test_settings_replay_and_discard_restores_preceding(self):
        add_trial(self.store, 1, change={"parameter": "target_rpm", "old": 2668.0, "new": 2768.0})
        self.assertEqual(self.store.current_settings()["target_rpm"], 2768.0)
        add_trial(self.store, 2, change={"parameter": "target_rpm", "old": 2768.0, "new": 2868.0})
        self.assertEqual(self.store.current_settings()["target_rpm"], 2868.0)
        rec, restored = self.store.discard_last("bad video", "test")
        self.assertEqual(rec["n"], 2)
        self.assertEqual(restored["target_rpm"], 2768.0)  # the settings trial 2 ran with
        rec, restored = self.store.discard_last("bad too", "test")
        self.assertEqual(restored["target_rpm"], 2668.0)  # session initial settings
        with self.assertRaises(ValueError):
            self.store.discard_last("nothing left", "test")

    def test_discard_keeps_files_audits_and_leaves_context(self):
        add_trial(self.store, 1, outcome="success")
        add_trial(self.store, 2, outcome="success", eligible=True)
        self.store.discard_last("operator", "test")
        d = self.store.trial_dir(2)
        self.assertTrue((d / "video.mp4").exists())
        self.assertTrue((d / "DISCARDED.json").exists())
        audit = [json.loads(l) for l in (self.store.dir / "audit.jsonl").read_text().splitlines()]
        self.assertEqual(audit[-1]["action"], "discard_trial")
        self.assertEqual([h["trial"] for h in self.store.context_history()], [1])
        self.assertEqual(policy.validation_status(self.store.trials())["streak"], 1)
        t2 = self.store.trials()[1]
        self.assertFalse(t2["refill_eligible"])  # a discarded review never authorizes a refill
        self.assertFalse(policy.refill_eligible(t2)[0])
        self.assertEqual(self.store.revoked_robot_trials("R1"), [2])
        with self.assertRaises(ValueError):
            self.store.update_trial(2, outcome="success")

    def test_cli_undo_on_saved_run(self):
        add_trial(self.store, 1, change={"parameter": "p", "old": 0.023, "new": 0.03})
        buf = io.StringIO()
        with redirect_stdout(buf):
            rc = cli.main(["undo", "--run", str(self.store.dir), "--reason", "cli test"])
        self.assertEqual(rc, 0)
        self.assertIn("Discarded trial 1", buf.getvalue())
        self.assertEqual(RunStore(self.store.dir).current_settings()["p"], 0.023)


if __name__ == "__main__":
    unittest.main()
