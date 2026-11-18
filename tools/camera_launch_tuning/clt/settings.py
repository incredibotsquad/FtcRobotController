"""Trial settings: the seven tunable parameters, their units, and absolute bounds."""

import hashlib
import json
import math

# Parameters the robot accepts (CameraLaunchTuner.Settings). Units:
#   target_rpm      flywheel RPM (motor RPM; the flywheel belt is 1:1)
#   p               LaunchSubsystem PID P, power per (ticks/s) of error (goBILDA bare motor, 28 CPR)
#   ks              feedforward static term, motor power (0..1)
#   kv              feedforward velocity term, power per (ticks/s), at NOMINAL_VOLTAGE
#   transfer_power  SmartIntakeCommand.LAUNCH_TRANSFER_POWER for the burst, motor power
#   max_fire_ms     LaunchBallsCommand.MAX_FIRE_MS for the burst, milliseconds (whole number)
#   turret_position turret servo position (0..1, limited to TURRET_MIN..TURRET_MAX)
PARAMS = ("target_rpm", "p", "ks", "kv", "transfer_power", "max_fire_ms", "turret_position")

UNITS = {
    "target_rpm": "RPM",
    "p": "power per tick/s of error",
    "ks": "motor power",
    "kv": "power per tick/s (at nominal voltage)",
    "transfer_power": "motor power",
    "max_fire_ms": "ms",
    "turret_position": "servo position",
}

# Laptop-side limits. Each session intersects these with the limits the robot reports.
# max_step = the largest change one proposal may make.
DEFAULT_BOUNDS = {
    "target_rpm": {"min": 1500.0, "max": 4500.0, "max_step": 150.0},
    "p": {"min": 0.0, "max": 0.15, "max_step": 0.01},
    "ks": {"min": 0.0, "max": 0.35, "max_step": 0.03},
    "kv": {"min": 0.0001, "max": 0.0008, "max_step": 0.00003},
    "transfer_power": {"min": 0.4, "max": 1.0, "max_step": 0.1},
    "max_fire_ms": {"min": 1500.0, "max": 6000.0, "max_step": 500.0},
    "turret_position": {"min": 0.2, "max": 1.0, "max_step": 0.01},
}

# Rounding used to decide whether two settings are "identical" (validation streaks).
_ROUND = {"target_rpm": 1, "p": 6, "ks": 5, "kv": 8, "transfer_power": 3, "max_fire_ms": 0, "turret_position": 4}


def is_finite_number(v):
    return isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v)


def canonical(settings):
    """The parameter values only, rounded, in a fixed order."""
    out = {}
    for k in PARAMS:
        v = settings.get(k)
        if not is_finite_number(v):
            out[k] = None
        elif k == "max_fire_ms":
            out[k] = int(round(v))
        else:
            out[k] = round(float(v), _ROUND[k])
    return out


def settings_key(settings):
    """Stable identity for 'identical settings' comparisons."""
    blob = json.dumps(canonical(settings), sort_keys=True)
    return hashlib.sha256(blob.encode()).hexdigest()[:12]


def with_id(settings):
    s = canonical(settings)
    s["settings_id"] = "s-" + settings_key(s)
    return s


def merge_bounds(laptop_bounds, robot_limits):
    """Tightest of laptop bounds and robot-reported [min, max] limits."""
    merged = {}
    for k in PARAMS:
        b = dict(laptop_bounds[k])
        lim = (robot_limits or {}).get(k)
        if isinstance(lim, (list, tuple)) and len(lim) == 2 and all(is_finite_number(x) for x in lim):
            b["min"] = max(b["min"], float(lim[0]))
            b["max"] = min(b["max"], float(lim[1]))
        merged[k] = b
    return merged


def validate_settings(settings, bounds):
    """Returns a list of problems (empty = valid). Every parameter must be present and finite."""
    problems = []
    for k in PARAMS:
        v = settings.get(k)
        if not is_finite_number(v):
            problems.append(f"{k}: missing or not a finite number")
            continue
        b = bounds[k]
        if v < b["min"] or v > b["max"]:
            problems.append(f"{k}: {v} outside [{b['min']}, {b['max']}]")
        if k == "max_fire_ms" and float(v) != round(float(v)):
            problems.append("max_fire_ms: must be a whole number of ms")
    return problems


def interpolate(xs, ys, x):
    """Linear interpolation clamped to the ends (same rule as the robot's tables)."""
    n = min(len(xs), len(ys))
    if n == 0:
        raise ValueError("empty table")
    if x <= xs[0]:
        return float(ys[0])
    for i in range(1, n):
        if x <= xs[i]:
            span = xs[i] - xs[i - 1]
            t = (x - xs[i - 1]) / span if span > 0 else 1.0
            return float(ys[i - 1] + t * (ys[i] - ys[i - 1]))
    return float(ys[n - 1])


def from_production(production, distance_in=None, turret_position=None):
    """Starting settings that match what TeleOp would use at `distance_in`.

    RPM comes from SHOT_DISTANCES_IN/SHOT_RPMS; P/kS/kV from the gain table when USE_GAIN_TABLE is
    on (else the single FLYWHEEL_* set); transfer power and timeout from the production statics.
    Without a distance, target_rpm is None and must be given.
    """
    rpm = None
    if distance_in is not None:
        rpm = interpolate(production["shot_distances_in"], production["shot_rpms"], distance_in)
    if production.get("use_gain_table") and distance_in is not None and production.get("gain_table_distances_in"):
        d = production["gain_table_distances_in"]
        p = interpolate(d, production["gain_table_p"], distance_in)
        ks = interpolate(d, production["gain_table_ks"], distance_in)
        kv = interpolate(d, production["gain_table_kv"], distance_in)
    else:
        p, ks, kv = production["flywheel_p"], production["flywheel_ks"], production["flywheel_kv"]
    turret = turret_position if turret_position is not None else production.get("stored_turret_position")
    if is_finite_number(turret):
        turret = min(max(turret, production.get("turret_min", 0.0)), production.get("turret_max", 1.0))
    return {
        "target_rpm": rpm,
        "p": p,
        "ks": ks,
        "kv": kv,
        "transfer_power": production["launch_transfer_power"],
        "max_fire_ms": int(production["max_fire_ms"]),
        "turret_position": turret,
    }
