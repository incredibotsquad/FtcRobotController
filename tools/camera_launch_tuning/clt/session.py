"""The tuning loop: arm -> record -> fire -> fetch -> review -> decide -> checkpoint.

The robot runs the mechanism; this side records video, reviews the finished trial (never a live
feed), validates any proposed change, and sends the next settings at the review checkpoint.
"""

import time
import uuid

from . import policy
from .camera import choose_review_times, extract_frames, nearest_frames, read_frame_times
from .providers.base import ReviewError, ReviewRequest
from .settings import PARAMS, with_id
from .store import read_json, save_profile, select_settings, write_json
from .telemetry import compute_metrics, read_csv
from .transport import TransportError, build_command

ACTIVE_PHASES = ("LOADING", "SPINUP", "LAUNCHING", "POST")

DEFAULT_CONFIG = {
    "goal": "3of4",
    "max_trials": 20,
    "validation_required": 3,
    "min_confidence": policy.DEFAULT_MIN_CONFIDENCE,
    "review_frames": 16,
    "review_pre_ms": 300,
    "review_post_ms": 1000,
    "frame_max_side": 960,
    "status_poll_s": 0.3,
    "settings_ack_timeout_s": 15,
    "load_timeout_s": 600,
    "burst_timeout_s": 60,
    "record_tail_s": 0.7,
    "auto_apply": False,
    "telemetry_thresholds": {},
}


class Session:
    def __init__(self, store, transport, reviewer, recorder=None, profile=None, config=None,
                 say=print, ask=input, sleep=time.sleep, clock=time.monotonic, profiles_root=None):
        self.store = store
        self.transport = transport
        self.reviewer = reviewer
        self.recorder = recorder
        self.profile = profile
        self.cfg = dict(DEFAULT_CONFIG, **(config or {}))
        self.say, self.ask, self.sleep, self.clock = say, ask, sleep, clock
        self.profiles_root = profiles_root
        self.session_id = uuid.uuid4().hex[:12]
        self.seq = int(time.time())
        self.robot_run_id = None
        self.refill_trial = None
        self.bounds = store.run["bounds"]

    # ---------------------------------------------------------------- robot link
    def push(self, settings=None, fire_trial=None, refill=None):
        self.seq += 1
        cmd = build_command(
            self.seq, self.session_id, robot_run_id=self.robot_run_id, settings=settings, fire_trial=fire_trial,
            refill_authorized_trial=refill if refill is not None else self.refill_trial,
            revoked_trials=self.store.revoked_robot_trials(self.robot_run_id) if self.robot_run_id else (),
            link_required=True)
        self.transport.write_command(cmd)
        self.store.audit("push_command", seq=self.seq, settings_id=(settings or {}).get("settings_id"),
                         fire_trial=fire_trial, refill_authorized_trial=cmd.get("refill_authorized_trial"),
                         revoked_trials=cmd["revoked_trials"])
        return cmd

    def wait_status(self, predicate, timeout_s, what):
        deadline = self.clock() + timeout_s
        last_err = None
        while self.clock() < deadline:
            try:
                st = self.transport.read_status()
                if predicate(st):
                    return st
            except TransportError as e:
                last_err = e
            self.sleep(self.cfg["status_poll_s"])
        raise TimeoutError(f"timed out waiting for {what}" + (f" ({last_err})" if last_err else ""))

    def send_settings(self, settings):
        self.push(settings=settings)
        st = self.wait_status(lambda s: (s.get("settings") or {}).get("settings_id") == settings["settings_id"]
                              or bool(s.get("settings_error")), self.cfg["settings_ack_timeout_s"], "the robot to apply settings")
        if st.get("settings_error") and st["settings"].get("settings_id") != settings["settings_id"]:
            raise RuntimeError(f"robot rejected settings: {st['settings_error']}")
        return st

    # ---------------------------------------------------------------- main loop
    def run(self):
        st = self.transport.read_status()
        self.robot_run_id = st["robot_run_id"]
        if st.get("phase") in ACTIVE_PHASES:
            raise RuntimeError("robot is mid-trial: finish or abort it (gamepad 1 B) first")
        settings = self.store.current_settings()
        self.send_settings(settings)
        self.say(f"Robot ready (run {self.robot_run_id}). Settings {settings['settings_id']} applied.")
        while True:
            n_done = len(self.store.trials())
            if n_done >= self.cfg["max_trials"]:
                self.say(f"Trial limit ({self.cfg['max_trials']}) reached.")
                break
            v = policy.validation_status(self.store.trials(), self.cfg["validation_required"])
            if v["validated"]:
                self.finish_validated(v)
                break
            rec = self.run_one()
            if rec is None:
                break
            if not self.checkpoint():
                break
        return policy.validation_status(self.store.trials(), self.cfg["validation_required"])

    def run_one(self):
        st = self.transport.read_status()
        last_trial = st.get("trial_id", 0)
        if st.get("needs_empty_confirmation") and not self.refill_trial:
            self.say("Robot needs an empty confirmation: clear it if needed (hold LT), check the dead zone, press X.")
        self.say("On the robot: press gamepad 1 A and feed 4 balls.")
        st = self.wait_status(lambda s: s.get("trial_id", 0) > last_trial and s.get("phase") in ACTIVE_PHASES + ("IDLE",),
                              self.cfg["load_timeout_s"], "the robot to arm a trial")
        robot_trial = st["trial_id"]
        settings = with_id(st.get("trial_settings") or st["settings"])
        rec = self.store.new_trial(settings, self.robot_run_id, robot_trial)
        tdir = self.store.trial_dir(rec["n"])
        self.say(f"Trial {rec['n']} (robot trial {robot_trial}): loading...")
        st = self.wait_status(lambda s: s.get("phase") in ("SPINUP", "IDLE", "AWAIT_EMPTY_ACK", "CLEARING"),
                              self.cfg["load_timeout_s"], "loading to finish")
        video = frames_csv = None
        offset = (0.0, None, "none")
        if st.get("phase") == "SPINUP":
            offset = self.transport.clock_offset(status=st)
            if self.recorder:
                video, frames_csv = tdir / "video.mp4", tdir / "frames.csv"
                self.recorder.start(video)
            self.push(fire_trial=robot_trial)
            self.say("Recording. The robot fires once both flywheel motors are stable.")
            self.wait_status(lambda s: s.get("phase") not in ACTIVE_PHASES and not s.get("trial_open"),
                             self.cfg["burst_timeout_s"], "the burst to finish")
            if self.recorder:
                self.sleep(self.cfg["record_tail_s"])
                self.recorder.stop(frames_csv)
        summary_path, csv_path = self.transport.fetch_trial(self.robot_run_id, robot_trial, tdir)
        return self.process_trial(rec, summary_path, csv_path, video, frames_csv, offset)

    # ---------------------------------------------------------------- review + decision
    def review_frames(self, tdir, summary, video, frames_csv, offset, metrics=None):
        if metrics is not None:
            from .frames import prepare
            return prepare(tdir, summary, metrics, video, frames_csv, offset,
                           {"frames_per_ball": self.cfg.get("frames_per_ball", 9), "frame_max_side": self.cfg["frame_max_side"]})
        ts = summary.get("timestamps", {})
        if video is None or frames_csv is None or ts.get("fire_ms") is None:
            return []
        offset_ms = offset[0]
        frame_times = read_frame_times(frames_csv)
        wanted_robot = choose_review_times(ts["fire_ms"], ts.get("launch_end_ms"), self.cfg["review_frames"],
                                           self.cfg["review_pre_ms"], self.cfg["review_post_ms"])
        idx = nearest_frames(frame_times, [w - offset_ms for w in wanted_robot])
        saved = extract_frames(video, idx, tdir / "review_frames", self.cfg["frame_max_side"])
        frames = []
        for i in idx:
            if i is None or i not in saved:
                continue
            robot_ms = frame_times[i] + offset_ms
            frames.append({"path": str(saved[i]), "frame_index": i, "laptop_ms": frame_times[i],
                           "t_from_gate_open_ms": robot_ms - ts["fire_ms"]})
        write_json(tdir / "review_frames.json", {"clock_offset_ms": offset_ms, "clock_uncertainty_ms": offset[1],
                                                 "clock_method": offset[2], "frames": frames})
        return frames

    def process_trial(self, rec, summary_path, csv_path, video=None, frames_csv=None, offset=(0.0, None, "none")):
        n = rec["n"]
        tdir = self.store.trial_dir(n)
        summary = read_json(summary_path)
        rows = read_csv(csv_path)
        metrics = compute_metrics(rows, summary, self.cfg["telemetry_thresholds"])
        write_json(tdir / "metrics.json", metrics)
        loaded = summary.get("count_at_fire") or 0
        robot_summary = {k: summary.get(k) for k in ("completion_reason", "burst_started", "gate_opened", "count_at_fire",
                                                     "count_at_end", "robot_counted_exits", "empty_confirmed_at_timeout",
                                                     "left_rpm_at_fire", "right_rpm_at_fire", "battery_v_at_fire", "settings")}
        if robot_summary["settings"] and robot_summary["settings"].get("settings_id") != rec["settings"]["settings_id"]:
            self.say("Note: the robot reports different trial settings; using the robot's values.")
            rec = self.store.update_trial(n, settings=with_id(robot_summary["settings"]))

        if not summary.get("burst_started"):
            assessment = policy.normalize_assessment({"footage_quality": "poor", "balls": []}, max(loaded, 0))
            assessment["flags"].append("no_burst:" + str(summary.get("completion_reason")))
            raw = None
        else:
            frames = self.review_frames(tdir, summary, video, frames_csv, offset, metrics)
            req = ReviewRequest(trial_n=n, loaded_count=loaded, goal=self.cfg["goal"], settings=rec["settings"],
                                bounds=self.bounds, metrics=metrics, robot_summary=robot_summary,
                                history=self.store.context_history(), spot=self.spot_info(), frames=frames,
                                camera_notes=f"clock offset uncertainty ~{offset[1]} ms ({offset[2]})",
                                examples=self.calibration_examples())
            raw = self.call_reviewer(req, tdir, have_frames=bool(frames))
            assessment = policy.normalize_assessment(raw or {}, loaded, self.cfg["min_confidence"])
            if raw is None:
                assessment["flags"].append("review_failed")
        write_json(tdir / "assessment_raw.json", {"reviewer": self.reviewer.name, "raw": raw})
        agreement = None
        if self.cfg.get("confirm_labels") and summary.get("burst_started") and self.reviewer.name != "manual":
            assessment, agreement = self.confirm_labels(rec, assessment, loaded, tdir)
        write_json(tdir / "assessment.json", assessment)
        summ = policy.summarize(assessment)
        outcome = policy.evaluate_goal(summ, self.cfg["goal"], bool(summary.get("burst_started")))
        rec = self.store.update_trial(n, summary=summ, outcome=outcome, metrics=metrics, robot_summary=robot_summary,
                                      reviewer=self.reviewer.name, label_agreement=agreement)
        eligible, why = policy.refill_eligible(rec)
        rec = self.store.update_trial(n, refill_eligible=eligible, refill_reason=why)
        self.refill_trial = rec["robot_trial_id"] if eligible else None

        proposal = (raw or {}).get("proposal") if isinstance(raw, dict) else None
        check = policy.check_proposal(proposal, rec["settings"], self.bounds, {
            "outcome": outcome, "summary": summ, "metrics": metrics, "robot_summary": robot_summary,
            "history": self.store.kept_trials(), "supported": set(PARAMS)})
        accepted = None
        if check["accepted"]:
            ch = check["change"]
            if self.cfg["auto_apply"] or self.ask(f"Apply {ch['parameter']} {ch['old']} -> {ch['new']} ({ch['reason']})? [y/N] ").strip().lower() == "y":
                accepted = ch
        decision = {"proposal": proposal, "check": check, "accepted_change": accepted, "outcome": outcome,
                    "refill_eligible": eligible, "refill_reason": why}
        write_json(tdir / "decision.json", decision)
        rec = self.store.update_trial(n, accepted_change=accepted, proposal_check=check)
        self.store.event("trial_reviewed", n=n, outcome=outcome, labels=summ["labels"], accepted_change=accepted,
                         rejected_reasons=None if check["accepted"] else check["reasons"], refill_eligible=eligible)
        self.report(rec, summ, outcome, check, accepted, eligible, why)
        self.send_settings(self.store.current_settings())
        return rec

    def call_reviewer(self, req, tdir, have_frames):
        if not have_frames and getattr(self.reviewer, "needs_frames", False):
            self.say("No review frames (camera off or clip missing): balls are unknown. Use --reviewer manual.")
            return None
        try:
            return self.reviewer.review(req, folder=tdir / "review_request")
        except ReviewError as e:
            self.say(f"Review failed ({e}): every ball is unknown for this trial.")
            return None

    def spot_info(self):
        p = self.profile or {}
        return {"spot": p.get("spot"), "description": p.get("description"), "distance_in": p.get("distance_in")}

    def calibration_examples(self):
        spot = (self.profile or {}).get("spot")
        if not spot or self.cfg.get("max_examples", 2) <= 0:
            return []
        from .examples import load_for_review
        return load_for_review(spot, self.cfg.get("max_examples", 2), self.profiles_root)

    def confirm_labels(self, rec, assessment, loaded, tdir):
        """Shows the reviewer's labels; the operator accepts (Enter) or corrects them. Corrections become the
        trial's labels and can be saved as a calibration example. Returns (assessment, agreement)."""
        from .examples import add_example, parse_labels
        ai = [b["label"] for b in assessment["balls"]]
        self.say(f"{self.reviewer.name} labels: " + "  ".join(f"{b['index']}={b['label']}({b['confidence']:.1f})" for b in assessment["balls"]))
        for b in assessment["balls"]:
            if b["evidence"]:
                self.say(f"    {b['index']}: {b['evidence'][:150]}")
        while True:
            a = self.ask(f"Enter = correct, or type the right {loaded} labels (e.g. l l l h): ").strip()
            if not a:
                return assessment, {"matched": loaded, "total": loaded, "corrected": False, "ai": ai}
            try:
                human = parse_labels(a, loaded)
                break
            except ValueError as e:
                self.say(f"  {e}")
        corrected = dict(assessment)
        corrected["balls"] = [{"index": i + 1, "label": l, "confidence": 1.0, "evidence": "operator correction"}
                              for i, l in enumerate(human)]
        corrected["flags"] = list(assessment["flags"]) + ["operator_corrected"]
        corrected["reviewer_labels"] = ai
        matched = sum(1 for x, y in zip(ai, human) if x == y)
        spot = (self.profile or {}).get("spot")
        if spot and human != ai and self.ask("Save this trial as a calibration example for this spot? [Y/n] ").strip().lower() != "n":
            try:
                ex = add_example(spot, tdir, human, {"run": self.store.run["run_id"], "trial": rec["n"]},
                                 note="corrected during tuning", root=self.profiles_root)
                self.say(f"  saved example {ex}; the reviewer sees it from the next trial on")
            except (ValueError, FileNotFoundError, FileExistsError) as e:
                self.say(f"  not saved as an example: {e}")
        return corrected, {"matched": matched, "total": loaded, "corrected": True, "ai": ai}

    def agreement_so_far(self):
        m = t = 0
        for tr in self.store.kept_trials():
            ag = tr.get("label_agreement")
            if ag:
                m += ag["matched"]
                t += ag["total"]
        return m, t

    def report(self, rec, summ, outcome, check, accepted, eligible, why):
        self.say(f"Trial {rec['n']}: {' '.join(summ['labels'])} -> {outcome.upper()} "
                 f"({summ['confirmed_scores']} scored, {summ['confirmed_exits']} confirmed out, {summ['unknown']} unknown)")
        if accepted:
            self.say(f"  change: {accepted['parameter']} {accepted['old']} -> {accepted['new']}")
        elif check["change"]:
            self.say("  proposal rejected: " + "; ".join(check["reasons"]))
        self.say("  refill: " + ("allowed (" + why + ")" if eligible else "NOT allowed - " + why))
        v = policy.validation_status(self.store.trials(), self.cfg["validation_required"])
        self.say(f"  validation streak: {v['streak']}/{v['required']}")
        m, t = self.agreement_so_far()
        if t:
            self.say(f"  reviewer agreement with you so far: {m}/{t} balls ({100 * m / t:.0f}%)")

    # ---------------------------------------------------------------- checkpoint / undo
    def checkpoint(self):
        while True:
            a = self.ask("[Enter] next trial | u = discard the last trial | q = quit: ").strip().lower()
            if a == "":
                return True
            if a == "q":
                return False
            if a == "u":
                self.undo_last("interactive")

    def undo_last(self, actor, reason="operator discarded the last trial"):
        rec, restored = self.store.discard_last(reason, actor)
        if self.refill_trial == rec.get("robot_trial_id"):
            self.refill_trial = None  # a discarded review never authorizes a refill
        self.send_settings(restored)  # the robot applies it at its idle checkpoint, never mid-shot
        self.say(f"Discarded trial {rec['n']}; restored settings {restored['settings_id']}. "
                 "If the robot isn't confirmed empty, clear it and press X before loading.")
        return rec, restored

    def finish_validated(self, v):
        settings = self.store.kept_trials()[-1]["settings"]
        state = self.store.state
        state["validated"] = {"settings": settings, "trials": v["trials"]}
        self.store.save_state(state)
        self.store.event("validated", settings=settings, trials=v["trials"])
        self.say(f"VALIDATED: {v['streak']} consecutive goal successes at {settings['settings_id']} (trials {v['trials']}).")
        if self.profile is not None:
            select_settings(self.profile, settings, {"run": self.store.run["run_id"], "trials": v["trials"],
                                                     "goal": self.cfg["goal"]})
            save_profile(self.profile, self.profiles_root)
            self.say(f"Saved as the selected settings for spot {self.profile['spot']!r} (production code unchanged).")
