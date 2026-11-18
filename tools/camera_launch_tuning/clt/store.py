"""Run folders, trial records, history, undo (discard), and spot profiles.

Layout (all under tools/camera_launch_tuning/, git-ignored):
  runs/<run_id>/run.json                 session config: spot, goal, initial settings, bounds, robot snapshot
  runs/<run_id>/state.json               trial index (status, settings, outcome, accepted change)
  runs/<run_id>/history.jsonl            append-only event log (never rewritten)
  runs/<run_id>/audit.jsonl              append-only discard / settings-push audit log
  runs/<run_id>/trials/NNNN/             one folder per trial (files are never deleted)
        robot_trial_*.json / .csv        raw robot summary and per-loop telemetry
        video.mp4, frames.csv            camera clip and per-frame laptop timestamps
        review_frames/, review_frames.json   timed frames sent for review
        metrics.json, assessment_raw.json, assessment.json, decision.json
        DISCARDED.json                   present only if the trial was discarded
  profiles/<spot>.json                   per-spot starting and selected settings
"""

import json
import os
import time
from pathlib import Path

from .settings import with_id

TOOL_DIR = Path(__file__).resolve().parent.parent


def _now_iso():
    return time.strftime("%Y-%m-%dT%H:%M:%S", time.localtime())


def write_json(path, obj):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(path.suffix + ".tmp")
    tmp.write_text(json.dumps(obj, indent=2, default=str), encoding="utf-8")
    os.replace(tmp, path)


def read_json(path):
    return json.loads(Path(path).read_text(encoding="utf-8"))


class RunStore:
    def __init__(self, run_dir):
        self.dir = Path(run_dir)

    # ---------------------------------------------------------------- create / load
    @classmethod
    def create(cls, runs_root, spot, goal, initial_settings, bounds, config, robot_snapshot=None, run_id=None):
        run_id = run_id or time.strftime("%Y%m%d-%H%M%S") + f"-{spot}"
        store = cls(Path(runs_root) / run_id)
        if store.dir.exists():
            raise FileExistsError(store.dir)
        (store.dir / "trials").mkdir(parents=True)
        initial = with_id(initial_settings)
        write_json(store.dir / "run.json", {
            "run_id": run_id, "created": _now_iso(), "spot": spot, "goal": goal,
            "initial_settings": initial, "bounds": bounds, "config": config, "robot_snapshot": robot_snapshot,
        })
        write_json(store.dir / "state.json", {"trials": [], "validated": None})
        store.event("run_created", spot=spot, goal=goal, initial_settings=initial)
        return store

    @property
    def run(self):
        return read_json(self.dir / "run.json")

    @property
    def state(self):
        return read_json(self.dir / "state.json")

    def save_state(self, state):
        write_json(self.dir / "state.json", state)

    def event(self, kind, **data):
        with open(self.dir / "history.jsonl", "a", encoding="utf-8") as f:
            f.write(json.dumps({"t": _now_iso(), "event": kind, **data}, default=str) + "\n")

    def audit(self, kind, **data):
        with open(self.dir / "audit.jsonl", "a", encoding="utf-8") as f:
            f.write(json.dumps({"t": _now_iso(), "action": kind, **data}, default=str) + "\n")

    # ---------------------------------------------------------------- trials
    def trials(self):
        return self.state["trials"]

    def kept_trials(self):
        return [t for t in self.trials() if t["status"] == "active"]

    def trial_dir(self, n):
        return self.dir / "trials" / f"{n:04d}"

    def new_trial(self, settings, robot_run_id, robot_trial_id):
        state = self.state
        n = len(state["trials"]) + 1
        rec = {"n": n, "status": "active", "settings": with_id(settings), "robot_run_id": robot_run_id,
               "robot_trial_id": robot_trial_id, "created": _now_iso(), "outcome": None, "summary": None,
               "metrics": None, "robot_summary": None, "accepted_change": None, "refill_eligible": False}
        state["trials"].append(rec)
        self.save_state(state)
        self.trial_dir(n).mkdir(parents=True, exist_ok=True)
        self.event("trial_started", n=n, settings=rec["settings"], robot_run_id=robot_run_id, robot_trial_id=robot_trial_id)
        return rec

    def update_trial(self, n, **fields):
        state = self.state
        rec = state["trials"][n - 1]
        if rec["status"] != "active":
            raise ValueError(f"trial {n} is discarded")
        rec.update(fields)
        self.save_state(state)
        return rec

    def current_settings(self):
        """Initial settings plus the accepted changes of kept trials, in order."""
        s = dict(self.run["initial_settings"])
        for t in self.kept_trials():
            ch = t.get("accepted_change")
            if ch:
                s[ch["parameter"]] = ch["new"]
        return with_id(s)

    def context_history(self, limit=12):
        """Compact kept-trial history for review prompts (discarded trials never appear)."""
        out = []
        for t in self.kept_trials()[-limit:]:
            m = t.get("metrics") or {}
            out.append({
                "trial": t["n"], "settings": {k: v for k, v in t["settings"].items() if k != "settings_id"},
                "outcome": t.get("outcome"), "labels": (t.get("summary") or {}).get("labels"),
                "completion_reason": (t.get("robot_summary") or {}).get("completion_reason"),
                "steady_error_rpm": (m.get("steady") or {}).get("mean_error_rpm"),
                "dips": [{k: d.get(k) for k in ("drop_rpm", "recovery_ms")} for d in m.get("dips", [])],
                "accepted_change": t.get("accepted_change"),
            })
        return out

    # ---------------------------------------------------------------- undo
    def discard_last(self, reason, actor):
        """Discards the most recent kept trial. Files stay; the trial leaves all future context,
        validation and refill decisions. Returns (discarded record, settings to restore)."""
        state = self.state
        kept = [t for t in state["trials"] if t["status"] == "active"]
        if not kept:
            raise ValueError("no kept trial to discard")
        rec = kept[-1]
        before = self.current_settings()
        rec["status"] = "discarded"
        rec["discarded"] = {"t": _now_iso(), "reason": reason, "by": actor,
                            "removed_change": rec.get("accepted_change"), "refill_eligible_was": rec.get("refill_eligible")}
        rec["refill_eligible"] = False
        state["validated"] = None
        self.save_state(state)
        restored = self.current_settings()
        write_json(self.trial_dir(rec["n"]) / "DISCARDED.json", rec["discarded"])
        self.audit("discard_trial", n=rec["n"], reason=reason, by=actor, settings_before=before, settings_restored=restored,
                   robot_run_id=rec.get("robot_run_id"), robot_trial_id=rec.get("robot_trial_id"))
        self.event("trial_discarded", n=rec["n"], reason=reason, restored_settings=restored)
        return rec, restored

    def revoked_robot_trials(self, robot_run_id):
        return [t["robot_trial_id"] for t in self.trials()
                if t["status"] == "discarded" and t.get("robot_run_id") == robot_run_id]


# ------------------------------------------------------------------ spot profiles

def profiles_dir(root=None):
    return Path(root) if root else TOOL_DIR / "profiles"


def load_profile(spot, root=None):
    p = profiles_dir(root) / f"{spot}.json"
    if not p.exists():
        raise FileNotFoundError(f"no profile for spot {spot!r}: run `spot-init` first ({p})")
    return read_json(p)


def save_profile(profile, root=None):
    write_json(profiles_dir(root) / f"{profile['spot']}.json", profile)


def new_profile(spot, starting_settings, description="", distance_in=None, source="manual"):
    return {"spot": spot, "description": description, "distance_in": distance_in, "created": _now_iso(),
            "starting_settings": with_id(starting_settings), "starting_source": source,
            "selected": None, "history": []}


def select_settings(profile, settings, evidence):
    """Records validated settings as the spot's selected (reviewable) profile. Never touches code."""
    profile["history"].append({"t": _now_iso(), "previous_selected": profile.get("selected")})
    profile["selected"] = {"settings": with_id(settings), "evidence": evidence, "t": _now_iso()}
    return profile
