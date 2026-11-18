import unittest

from helpers import BASE_SETTINGS, assessment, bounds
from clt import policy
from clt.settings import validate_settings, with_id


def ctx(outcome="failure", labels=("short", "short", "scored", "scored"), metrics=None, history=(), reason="empty"):
    a = policy.normalize_assessment(assessment(list(labels)), 4)
    return {"outcome": outcome, "summary": policy.summarize(a),
            "metrics": metrics if metrics is not None else {"steady": {"mean_error_rpm": 5, "noise_rpm": 4}, "tracking_good": True},
            "robot_summary": {"completion_reason": reason}, "history": list(history)}


class ProposalBounds(unittest.TestCase):
    def check(self, param, value, **kw):
        return policy.check_proposal({"parameter": param, "new_value": value, "reason": "r"}, BASE_SETTINGS, bounds(), ctx(**kw))

    def test_rpm_up_for_short_misses_within_step(self):
        r = self.check("target_rpm", 2768)
        self.assertTrue(r["accepted"], r["reasons"])
        self.assertEqual(r["change"]["new"], 2768.0)

    def test_rejects_non_finite_and_non_numbers(self):
        for v in (float("nan"), float("inf"), None, "2700", True):
            self.assertFalse(self.check("target_rpm", v)["accepted"], v)

    def test_rejects_absolute_limits_and_max_step(self):
        self.assertIn("exceeds max step", " ".join(self.check("target_rpm", 2900)["reasons"]))
        s = dict(BASE_SETTINGS, target_rpm=4450)
        r = policy.check_proposal({"parameter": "target_rpm", "new_value": 4550}, s, bounds(), ctx())
        self.assertIn("outside absolute limits", " ".join(r["reasons"]))

    def test_rejects_unknown_or_unsupported_parameter(self):
        self.assertFalse(self.check("hood_angle", 1.0)["accepted"])
        c = ctx(); c["supported"] = {"target_rpm"}
        r = policy.check_proposal({"parameter": "turret_position", "new_value": 0.56}, BASE_SETTINGS, bounds(), c)
        self.assertIn("not supported", r["reasons"][0])

    def test_wrong_direction_and_no_misses(self):
        self.assertFalse(self.check("target_rpm", 2568)["accepted"])  # lowering for short misses
        self.assertFalse(self.check("target_rpm", 2768, labels=("left", "left", "scored", "scored"))["accepted"])

    def test_no_change_after_success_or_ambiguity(self):
        self.assertFalse(self.check("target_rpm", 2768, outcome="success")["accepted"])
        self.assertFalse(self.check("target_rpm", 2768, outcome="inconclusive")["accepted"])

    def test_gains_rejected_when_tracking_good_or_mechanical(self):
        self.assertIn("tracking is good", " ".join(self.check("kv", 0.00034)["reasons"]))
        jam = {"steady": {"mean_error_rpm": 3, "noise_rpm": 2}, "tracking_good": True}
        r = self.check("p", 0.03, labels=("scored", "scored", "scored", "stuck"), metrics=jam, reason="max_fire_ms")
        self.assertIn("mechanical", " ".join(r["reasons"]))

    def test_kv_allowed_with_steady_bias(self):
        bias = {"steady": {"mean_error_rpm": -90, "noise_rpm": 4}, "steady_bias": True, "tracking_good": False}
        self.assertTrue(self.check("kv", 0.00034, metrics=bias)["accepted"])

    def test_ks_needs_two_operating_speeds(self):
        bias = {"steady": {"mean_error_rpm": -90}, "steady_bias": True, "tracking_good": False}
        one = [{"status": "active", "settings": BASE_SETTINGS, "metrics": {"steady": {"samples": 30}}}]
        r = self.check("ks", 0.18, metrics=bias, history=one)
        self.assertIn("one operating speed", " ".join(r["reasons"]))
        two = one + [{"status": "active", "settings": dict(BASE_SETTINGS, target_rpm=3170), "metrics": {"steady": {"samples": 30}}}]
        self.assertTrue(self.check("ks", 0.18, metrics=bias, history=two)["accepted"])

    def test_settings_validation(self):
        self.assertEqual(validate_settings(BASE_SETTINGS, bounds()), [])
        self.assertTrue(validate_settings(dict(BASE_SETTINGS, kv=float("nan")), bounds()))
        self.assertTrue(validate_settings(dict(BASE_SETTINGS, max_fire_ms=2500.5), bounds()))


class Ambiguity(unittest.TestCase):
    def test_low_confidence_duplicates_missing_and_bad_labels_become_unknown(self):
        raw = {"footage_quality": "good", "balls": [
            {"index": 1, "label": "scored", "confidence": 0.95, "evidence": ""},
            {"index": 2, "label": "scored", "confidence": 0.3, "evidence": ""},
            {"index": 3, "label": "bounced", "confidence": 0.9, "evidence": ""},
            {"index": 3, "label": "scored", "confidence": 0.9, "evidence": ""},
            {"index": 9, "label": "scored", "confidence": 0.9, "evidence": ""}]}
        a = policy.normalize_assessment(raw, 4)
        self.assertEqual([b["label"] for b in a["balls"]], ["scored", "unknown", "unknown", "unknown"])
        self.assertIn("ball_4_missing", a["flags"])

    def test_poor_footage_and_malformed(self):
        a = policy.normalize_assessment(assessment(["scored"] * 4, quality="poor"), 4)
        self.assertEqual(policy.summarize(a)["unknown"], 4)
        self.assertEqual(policy.summarize(policy.normalize_assessment("garbage", 4))["unknown"], 4)

    def test_goal_outcomes(self):
        summ = lambda labels: policy.summarize(policy.normalize_assessment(assessment(labels), 4))
        self.assertEqual(policy.evaluate_goal(summ(["scored", "scored", "scored", "stuck"]), "3of4"), "success")
        self.assertEqual(policy.evaluate_goal(summ(["scored", "scored", "scored", "stuck"]), "4of4"), "failure")
        self.assertEqual(policy.evaluate_goal(summ(["scored", "scored", "unknown", "short"]), "3of4"), "inconclusive")
        self.assertEqual(policy.evaluate_goal(summ(["scored", "short", "long", "unknown"]), "3of4"), "failure")
        self.assertEqual(policy.evaluate_goal(summ(["scored"] * 4), "3of4", burst_started=False), "inconclusive")

    def test_exits_tracked_separately_from_scores(self):
        s = policy.summarize(policy.normalize_assessment(assessment(["scored", "long", "left", "stuck"]), 4))
        self.assertEqual((s["confirmed_scores"], s["confirmed_exits"], s["stuck"]), (1, 3, 1))


class Refill(unittest.TestCase):
    def trial(self, labels, status="active", count_at_end=0, burst=True):
        return {"status": status, "summary": policy.summarize(policy.normalize_assessment(assessment(labels), 4)),
                "robot_summary": {"burst_started": burst, "count_at_fire": 4, "count_at_end": count_at_end}}

    def test_stuck_fourth_blocks_refill_even_when_sensors_read_empty(self):
        ok, why = policy.refill_eligible(self.trial(["scored", "scored", "scored", "stuck"], count_at_end=0))
        self.assertFalse(ok)
        self.assertIn("press X", why)

    def test_unknown_blocks_refill(self):
        self.assertFalse(policy.refill_eligible(self.trial(["scored", "scored", "scored", "unknown"]))[0])

    def test_all_out_and_count_zero_allows_refill(self):
        self.assertTrue(policy.refill_eligible(self.trial(["scored", "short", "scored", "long"]))[0])

    def test_discarded_robot_count_and_no_burst_block_refill(self):
        self.assertFalse(policy.refill_eligible(self.trial(["scored"] * 4, status="discarded"))[0])
        self.assertFalse(policy.refill_eligible(self.trial(["scored"] * 4, count_at_end=1))[0])
        self.assertFalse(policy.refill_eligible(self.trial(["scored"] * 4, burst=False))[0])


class Validation(unittest.TestCase):
    def t(self, n, outcome, settings=BASE_SETTINGS, status="active"):
        return {"n": n, "status": status, "outcome": outcome, "settings": with_id(settings)}

    def test_three_consecutive_identical_successes(self):
        trials = [self.t(1, "failure"), self.t(2, "success"), self.t(3, "success"), self.t(4, "success")]
        v = policy.validation_status(trials)
        self.assertTrue(v["validated"])
        self.assertEqual(v["trials"], [2, 3, 4])

    def test_different_settings_or_inconclusive_break_the_streak(self):
        other = dict(BASE_SETTINGS, target_rpm=2700)
        self.assertEqual(policy.validation_status([self.t(1, "success", other), self.t(2, "success"), self.t(3, "success")])["streak"], 2)
        self.assertEqual(policy.validation_status([self.t(1, "success"), self.t(2, "inconclusive"), self.t(3, "success")])["streak"], 1)

    def test_discarded_trials_are_skipped(self):
        trials = [self.t(1, "success"), self.t(2, "success"), self.t(3, "failure", status="discarded"), self.t(4, "success")]
        self.assertTrue(policy.validation_status(trials)["validated"])
        trials[3]["status"] = "discarded"
        self.assertFalse(policy.validation_status(trials)["validated"])


if __name__ == "__main__":
    unittest.main()
