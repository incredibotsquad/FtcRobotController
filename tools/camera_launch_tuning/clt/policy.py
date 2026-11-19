"""Decision rules: ball labels, goals, refill eligibility, validation streaks, proposal checks.

Everything here is pure (no I/O) so it can be tested offline.
"""

from .settings import PARAMS, UNITS, canonical, is_finite_number, settings_key

LABELS = ("scored", "short", "long", "left", "right", "stuck", "unknown")
EXIT_LABELS = frozenset(("scored", "short", "long", "left", "right"))
MISS_LABELS = frozenset(("short", "long", "left", "right"))
GOALS = {"3of4": 3, "4of4": 4}
FOOTAGE_QUALITY = ("good", "partial", "poor")
GAIN_PARAMS = frozenset(("p", "ks", "kv"))
DEFAULT_MIN_CONFIDENCE = 0.6


# ------------------------------------------------------------------ assessments

def normalize_assessment(raw, loaded_count, min_confidence=DEFAULT_MIN_CONFIDENCE):
    """Turns a reviewer's answer into a safe assessment.

    Anything ambiguous becomes "unknown": low confidence, an unrecognized label, a duplicate or
    missing ball, poor footage, or a malformed answer. Never invents a result.
    """
    flags = []
    if not isinstance(raw, dict):
        raw = {}
        flags.append("malformed_response")
    quality = raw.get("footage_quality")
    if quality not in FOOTAGE_QUALITY:
        flags.append("footage_quality_missing")
        quality = "poor"
    by_index = {}
    balls_in = raw.get("balls") if isinstance(raw.get("balls"), list) else []
    if not isinstance(raw.get("balls"), list):
        flags.append("balls_missing")
    for b in balls_in:
        if not isinstance(b, dict):
            flags.append("ball_entry_malformed")
            continue
        idx = b.get("index")
        if not isinstance(idx, int) or isinstance(idx, bool) or not 1 <= idx <= loaded_count:
            flags.append(f"ball_index_invalid:{idx}")
            continue
        if idx in by_index:
            flags.append(f"ball_index_duplicate:{idx}")
            by_index[idx] = None  # contradictory: force unknown
            continue
        by_index[idx] = b
    balls = []
    for i in range(1, loaded_count + 1):
        b = by_index.get(i)
        label, conf, evidence = "unknown", 0.0, ""
        if b:
            label = b.get("label") if b.get("label") in LABELS else "unknown"
            if b.get("label") not in LABELS:
                flags.append(f"ball_{i}_label_invalid")
            c = b.get("confidence")
            conf = float(c) if is_finite_number(c) and 0 <= c <= 1 else 0.0
            evidence = str(b.get("evidence", ""))[:500]
            if label != "unknown" and conf < min_confidence:
                flags.append(f"ball_{i}_low_confidence")
                label = "unknown"
        elif i not in by_index:
            flags.append(f"ball_{i}_missing")
        if quality == "poor" and label != "unknown":
            label = "unknown"
        balls.append({"index": i, "label": label, "confidence": round(conf, 3), "evidence": evidence})
    if quality == "poor":
        flags.append("poor_footage_all_unknown")
    return {"balls": balls, "footage_quality": quality, "notes": str(raw.get("notes", ""))[:2000], "flags": flags}


def summarize(assessment):
    labels = [b["label"] for b in assessment["balls"]]
    return {
        "labels": labels,
        "confirmed_exits": sum(1 for x in labels if x in EXIT_LABELS),
        "confirmed_scores": sum(1 for x in labels if x == "scored"),
        "misses": {k: sum(1 for x in labels if x == k) for k in sorted(MISS_LABELS)},
        "stuck": sum(1 for x in labels if x == "stuck"),
        "unknown": sum(1 for x in labels if x == "unknown"),
    }


def evaluate_goal(summary, goal, burst_started=True):
    """success / failure / inconclusive. Unknown balls never count as scored."""
    need = GOALS[goal]
    if not burst_started:
        return "inconclusive"
    scores, unknown = summary["confirmed_scores"], summary["unknown"]
    if scores >= need:
        return "success"
    if scores + unknown >= need:
        return "inconclusive"  # the unknown balls could decide it
    return "failure"


# ------------------------------------------------------------------ refill eligibility

def refill_eligible(trial):
    """May the robot load another full set without a manual empty check?

    Only when this kept (not discarded) trial's review confirmed that EVERY loaded ball left the
    robot, and the robot's own count ended at 0. Storage sensors clearing is not enough: a ball in
    the transfer-to-flywheel dead zone is invisible to them.
    """
    if trial.get("status") != "active":
        return False, "trial discarded"
    robot = trial.get("robot_summary") or {}
    if not robot.get("burst_started"):
        return False, "no burst: balls are still held, resume loading instead"
    summary = trial.get("summary")
    if not summary:
        return False, "no assessment"
    loaded = robot.get("count_at_fire")
    if not isinstance(loaded, int) or loaded <= 0:
        return False, "unknown loaded count"
    if summary["confirmed_exits"] < loaded:
        missing = loaded - summary["confirmed_exits"]
        return False, f"{missing} ball(s) not confirmed out (stuck or unknown): clear and press X"
    if robot.get("count_at_end") != 0:
        return False, "robot count not 0 at the end of the burst"
    return True, "all loaded balls confirmed out by the camera review"


# ------------------------------------------------------------------ validation streak

def validation_status(trials, required=3):
    """Consecutive goal successes at identical settings, counting back from the latest kept trial.

    Discarded trials are skipped entirely (they are neither evidence nor a break). Any other
    outcome (failure, inconclusive, not fired) ends the streak.
    """
    kept = [t for t in trials if t.get("status") == "active"]
    streak, key, ids = 0, None, []
    for t in reversed(kept):
        k = settings_key(t["settings"])
        if t.get("outcome") != "success":
            break
        if key is None:
            key = k
        elif k != key:
            break
        streak += 1
        ids.append(t["n"])
    return {"streak": streak, "required": required, "validated": streak >= required,
            "settings_key": key, "trials": list(reversed(ids))}


# ------------------------------------------------------------------ proposals

def distinct_operating_rpms(trials, min_separation=300.0):
    """Distinct target RPMs (kept, fired trials with steady data), merged within min_separation."""
    rpms = sorted({round(t["settings"]["target_rpm"]) for t in trials
                   if t.get("status") == "active" and (t.get("metrics") or {}).get("steady")})
    groups = []
    for r in rpms:
        if not groups or r - groups[-1] >= min_separation:
            groups.append(r)
    return groups


def check_proposal(proposal, settings, bounds, context):
    """Validates one proposed change. Returns {"accepted": bool, "reasons": [...], "change": {...}}.

    context keys: outcome, summary, metrics, history (kept trials), supported (set of params),
    robot_summary.
    """
    reasons = []
    if not isinstance(proposal, dict):
        return {"accepted": False, "reasons": ["no proposal"], "change": None}
    param = proposal.get("parameter")
    if param in (None, "none"):
        return {"accepted": False, "reasons": ["no change proposed"], "change": None}
    if param not in PARAMS:
        return {"accepted": False, "reasons": [f"unknown parameter {param!r}"], "change": None}
    supported = context.get("supported", set(PARAMS))
    if param not in supported:
        return {"accepted": False, "reasons": [f"{param} is not supported by this robot"], "change": None}
    new = proposal.get("new_value")
    if not is_finite_number(new):
        return {"accepted": False, "reasons": ["new_value is not a finite number"], "change": None}
    old = settings[param]
    b = bounds[param]
    if param == "max_fire_ms" and float(new) != round(float(new)):
        reasons.append("max_fire_ms must be a whole number")
    if new < b["min"] or new > b["max"]:
        reasons.append(f"{new} outside absolute limits [{b['min']}, {b['max']}] {UNITS[param]}")
    if abs(new - old) > b["max_step"] + 1e-12:
        reasons.append(f"change {new - old:+g} exceeds max step {b['max_step']}")
    if canonical({param: new}).get(param) == canonical({param: old}).get(param):
        reasons.append("no effective change")

    outcome = context.get("outcome")
    summary = context.get("summary") or {}
    metrics = context.get("metrics") or {}
    robot = context.get("robot_summary") or {}
    if outcome == "inconclusive":
        reasons.append("trial is inconclusive: no tuning decision from ambiguous evidence")
    if outcome == "success":
        reasons.append("goal met: keep settings to build the validation streak")

    jam = summary.get("stuck", 0) > 0 or robot.get("completion_reason") in ("max_fire_ms", "spinup_timeout")
    if param in GAIN_PARAMS:
        if not metrics.get("steady"):
            reasons.append("no encoder data: gains can't be judged")
        elif jam and not (metrics.get("steady_bias") or metrics.get("recovery_slow")):
            reasons.append("a ball stuck / timed out while speed tracking was fine: mechanical, not a gain problem")
        elif metrics.get("tracking_good"):
            reasons.append("speed tracking is good: adjust target_rpm (or turret) for short/long/side misses instead")
        if param == "ks" and len(distinct_operating_rpms(context.get("history", []))) < 2:
            reasons.append("kS and kV can't be separated at one operating speed: adjust kV (or collect steady data at a 2nd RPM >= 300 apart)")
        if param == "kv" and not metrics.get("steady_bias"):
            reasons.append("kV changes need a steady-state speed error in the encoder data")
        if param == "p" and not (metrics.get("recovery_slow") or (metrics.get("steady") or {}).get("noise_rpm", 0) > 25):
            reasons.append("P changes need slow recovery or oscillation in the encoder data")
    if param == "target_rpm":
        if metrics.get("steady_bias") or metrics.get("motor_mismatch"):
            reasons.append("flywheel isn't reaching its target: fix tracking (kV/P) before trusting RPM changes")
        misses = summary.get("misses", {})
        short, long_ = misses.get("short", 0), misses.get("long", 0)
        if short == 0 and long_ == 0:
            reasons.append("RPM changes need short or long misses")
        elif short and long_:
            # Over AND under in one burst: one RPM can't fix both (lowering it makes the short ones worse).
            reasons.append(f"misses went both ways ({long_} over, {short} under): a shot-to-shot consistency problem "
                           "(feed / dead zone / recovery), not an RPM one")
        elif new > old and not short:
            reasons.append("raising RPM is for balls that came in under (short); these went over")
        elif new < old and not long_:
            reasons.append("lowering RPM is for balls that went over (long); these came in under")
    if param == "turret_position":
        misses = summary.get("misses", {})
        if misses.get("left", 0) + misses.get("right", 0) == 0:
            reasons.append("turret changes need left/right misses")
    if param == "transfer_power" and not (metrics.get("recovery_slow") or summary.get("stuck", 0) > 0):
        reasons.append("transfer changes need slow recovery between balls or a stuck ball")
    if param == "max_fire_ms" and robot.get("completion_reason") != "max_fire_ms":
        reasons.append("timeout changes need a launch that ended at max_fire_ms")

    change = {"parameter": param, "old": old, "new": float(new) if param != "max_fire_ms" else int(round(new)),
              "reason": str(proposal.get("reason", ""))[:1000]}
    return {"accepted": not reasons, "reasons": reasons, "change": change}


def apply_change(settings, change):
    s = dict(settings)
    s[change["parameter"]] = change["new"]
    return s
