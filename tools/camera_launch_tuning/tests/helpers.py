import csv
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from clt.settings import DEFAULT_BOUNDS  # noqa: E402

BASE_SETTINGS = {"target_rpm": 2668.0, "p": 0.023, "ks": 0.16, "kv": 0.00032419,
                 "transfer_power": 1.0, "max_fire_ms": 3000, "turret_position": 0.55}

COLUMNS = ("t_wall_ms,t_boot_ms,t_rel_ms,phase,target_rpm,left_rpm,right_rpm,mean_rpm,power,bang_bang,battery_v,"
           "count,s1_blocked,s1_mm,s2_blocked,s3_blocked,s4_blocked,s4_mm,gate_open,intake_power,transfer_power,"
           "smart_state,launch_running,loop_ms").split(",")


def bounds():
    return {k: dict(v) for k, v in DEFAULT_BOUNDS.items()}


def write_telemetry(path, target=2668.0, fire=10_000, steady_error=0.0, dips=((100, 150, 120),), balls=4, end=None):
    """Synthetic 10 ms telemetry: spin-up steady at target+steady_error, then per-ball dips.

    dips: (start_ms_after_fire, drop_rpm, recovery_ms)."""
    rows = []
    count = balls
    end = end or fire + 1200
    for t in range(fire - 600, end + 1500, 10):
        rpm = target + steady_error
        for start, drop, rec in dips:
            if start <= t - fire < start + rec:
                rpm = target - drop
        if t >= fire:
            for i, (start, _, _) in enumerate(dips):
                if t - fire == start and count > 0:
                    count -= 1
        phase = "SPINUP" if t < fire else ("LAUNCHING" if t < end else "POST")
        rows.append({"t_wall_ms": t, "t_boot_ms": t, "t_rel_ms": t - fire, "phase": phase, "target_rpm": target,
                     "left_rpm": rpm, "right_rpm": rpm, "mean_rpm": rpm, "power": 0.6, "bang_bang": 0,
                     "battery_v": 12.6, "count": count, "s1_blocked": 0, "s1_mm": 108, "s2_blocked": 0,
                     "s3_blocked": 0, "s4_blocked": 0, "s4_mm": 99, "gate_open": int(fire <= t < end),
                     "intake_power": 0, "transfer_power": 1, "smart_state": "FULL", "launch_running": int(fire <= t < end),
                     "loop_ms": 10})
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=COLUMNS)
        w.writeheader()
        w.writerows(rows)


def robot_summary(trial_id=1, settings=None, fire=10_000, end=11_200, count_at_fire=4, count_at_end=0,
                  reason="empty", burst=True):
    s = dict(settings or BASE_SETTINGS)
    s.setdefault("settings_id", "x")
    return {"protocol": 1, "robot_run_id": "R1", "trial_id": trial_id, "settings": s,
            "completion_reason": reason, "burst_started": burst, "gate_opened": burst,
            "count_at_fire": count_at_fire if burst else None, "count_at_end": count_at_end,
            "robot_counted_exits": (count_at_fire - count_at_end) if burst else 0,
            "timestamps": {"armed_ms": fire - 5000, "fire_ms": fire if burst else None,
                           "launch_end_ms": end if burst else None, "post_end_ms": end + 1500 if burst else None}}


def write_json(path, obj):
    Path(path).write_text(json.dumps(obj))


def assessment(labels, conf=0.9, quality="good", proposal=None):
    return {"footage_quality": quality, "notes": "",
            "balls": [{"index": i + 1, "label": l, "confidence": conf, "evidence": "seen"} for i, l in enumerate(labels)],
            "proposal": proposal or {"parameter": "none", "new_value": None, "reason": ""}}
