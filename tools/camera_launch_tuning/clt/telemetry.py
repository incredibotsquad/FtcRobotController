"""Robot telemetry parsing and flywheel metrics (encoder evidence for tuning decisions)."""

import csv
import math
import statistics

NUMERIC = (
    "t_wall_ms", "t_boot_ms", "t_rel_ms", "target_rpm", "left_rpm", "right_rpm", "mean_rpm", "power",
    "bang_bang", "battery_v", "count", "s1_blocked", "s1_mm", "s2_blocked", "s3_blocked", "s4_blocked",
    "s4_mm", "gate_open", "intake_power", "transfer_power", "launch_running", "loop_ms",
)

# Defaults for the metric thresholds (overridable through the session config).
DEFAULT_THRESHOLDS = {
    "steady_window_ms": 300,      # window before the gate opens used for steady-state error
    "steady_tolerance_rpm": 40,   # |mean error| at or below this = speed tracking is good
    "mismatch_tolerance_rpm": 60, # left vs right encoder disagreement
    "dip_detect_rpm": 60,         # a drop this far below the TARGET = one ball's dip (encoder noise is ~+/-20)
    "recovery_ok_ms": 300,        # back within steady_tolerance of target this fast = recovered
}


def read_csv(path):
    rows = []
    with open(path, newline="", encoding="utf-8") as f:
        for r in csv.DictReader(f):
            row = {}
            for k, v in r.items():
                if k in NUMERIC:
                    try:
                        row[k] = float(v)
                    except (TypeError, ValueError):
                        row[k] = math.nan
                else:
                    row[k] = v
            rows.append(row)
    return rows


def _mean(xs):
    xs = [x for x in xs if not math.isnan(x)]
    return statistics.fmean(xs) if xs else math.nan


def compute_metrics(rows, summary, thresholds=None):
    """Summarizes one trial's telemetry. All times are robot wall-clock ms.

    Returns a dict with: steady (error, left/right, mismatch, power, noise), burst (dips with
    recovery times, min RPM, saturation), exits seen by the count dropping, and the derived flags
    tracking_good / recovery_slow / steady_bias / motor_mismatch used by the proposal policy.
    """
    th = dict(DEFAULT_THRESHOLDS, **(thresholds or {}))
    ts = summary.get("timestamps", {})
    fire = ts.get("fire_ms")
    end = ts.get("launch_end_ms") or ts.get("post_end_ms")
    target = (summary.get("settings") or {}).get("target_rpm")
    m = {"has_data": bool(rows), "fired": fire is not None, "target_rpm": target, "thresholds": th}
    if not rows or target is None:
        return m

    # ---- steady state just before the gate opens
    if fire is not None:
        steady = [r for r in rows if fire - th["steady_window_ms"] <= r["t_wall_ms"] < fire]
    else:
        steady = [r for r in rows if r.get("phase") == "SPINUP"][-30:]
    if steady:
        err = [r["mean_rpm"] - target for r in steady]
        m["steady"] = {
            "samples": len(steady),
            "mean_error_rpm": round(_mean(err), 1),
            "noise_rpm": round(statistics.pstdev(err), 1) if len(err) > 1 else 0.0,
            "left_rpm": round(_mean([r["left_rpm"] for r in steady]), 1),
            "right_rpm": round(_mean([r["right_rpm"] for r in steady]), 1),
            "mean_power": round(_mean([r["power"] for r in steady]), 4),
            "battery_v": round(_mean([r["battery_v"] for r in steady]), 2),
        }
        m["steady"]["mismatch_rpm"] = round(abs(m["steady"]["left_rpm"] - m["steady"]["right_rpm"]), 1)

    # ---- the burst: per-ball dips and recovery
    if fire is not None:
        stop = end if end is not None else fire + 10000
        burst = [r for r in rows if fire <= r["t_wall_ms"] <= stop + 1500]
        dips = []
        dip = None
        for r in burst:
            rpm, t = r["mean_rpm"], r["t_wall_ms"]
            if dip is None:
                if target - rpm >= th["dip_detect_rpm"]:
                    dip = {"start_ms": t - fire, "min_rpm": rpm, "min_ms": t - fire, "recovery_ms": None}
            else:
                if rpm < dip["min_rpm"]:
                    dip["min_rpm"], dip["min_ms"] = rpm, t - fire
                if abs(rpm - target) <= th["steady_tolerance_rpm"]:
                    dip["recovery_ms"] = (t - fire) - dip["start_ms"]
                    dips.append(dip)
                    dip = None
        if dip is not None:
            dips.append(dip)  # never recovered within the log
        for d in dips:
            d["drop_rpm"] = round(target - d["min_rpm"], 1)
            d["min_rpm"] = round(d["min_rpm"], 1)
        m["dips"] = dips
        m["burst_min_rpm"] = round(min(r["mean_rpm"] for r in burst), 1) if burst else None
        m["burst_saturated_fraction"] = round(sum(1 for r in burst if r["power"] >= 0.999) / len(burst), 3) if burst else None
        exits = []
        prev = None
        for r in burst:
            c = r["count"]
            if prev is not None and not math.isnan(c) and c < prev:
                exits.extend([round(r["t_wall_ms"] - fire)] * int(prev - c))
            prev = c
        m["count_drop_times_ms"] = exits
        gaps = [b - a for a, b in zip(exits, exits[1:])]
        m["ball_spacing_ms"] = gaps
        m["loop_ms_mean"] = round(_mean([r["loop_ms"] for r in burst]), 1) if burst else None

    # ---- flags for the policy
    st = m.get("steady")
    dips = m.get("dips", [])
    m["steady_bias"] = bool(st) and abs(st["mean_error_rpm"]) > th["steady_tolerance_rpm"]
    m["motor_mismatch"] = bool(st) and st["mismatch_rpm"] > th["mismatch_tolerance_rpm"]
    m["recovery_slow"] = any(d["recovery_ms"] is None or d["recovery_ms"] > th["recovery_ok_ms"] for d in dips)
    m["tracking_good"] = bool(st) and not m["steady_bias"] and not m["motor_mismatch"] and not m["recovery_slow"]
    return m
