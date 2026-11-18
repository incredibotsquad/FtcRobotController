"""Shared reviewer contract: request bundle, output schema, prompt text."""

import base64
import json
from dataclasses import dataclass, field
from pathlib import Path

from ..policy import FOOTAGE_QUALITY, GOALS, LABELS
from ..settings import PARAMS, UNITS


class ReviewError(RuntimeError):
    pass


# One JSON schema for every provider. Kept to the subset both structured-output implementations
# accept (objects with additionalProperties false, every property required, enums, nullable via a
# type list). Ranges, counts and cross-field rules are checked locally in policy.normalize_assessment
# and policy.check_proposal, never trusted from the model.
ASSESSMENT_SCHEMA = {
    "type": "object",
    "properties": {
        "footage_quality": {"type": "string", "enum": list(FOOTAGE_QUALITY)},
        "balls": {
            "type": "array",
            "items": {
                "type": "object",
                "properties": {
                    "index": {"type": "integer"},
                    "label": {"type": "string", "enum": list(LABELS)},
                    "confidence": {"type": "number"},
                    "evidence": {"type": "string"},
                },
                "required": ["index", "label", "confidence", "evidence"],
                "additionalProperties": False,
            },
        },
        "notes": {"type": "string"},
        "proposal": {
            "type": "object",
            "properties": {
                "parameter": {"type": "string", "enum": ["none"] + list(PARAMS)},
                "new_value": {"type": ["number", "null"]},
                "reason": {"type": "string"},
            },
            "required": ["parameter", "new_value", "reason"],
            "additionalProperties": False,
        },
    },
    "required": ["footage_quality", "balls", "notes", "proposal"],
    "additionalProperties": False,
}


@dataclass
class ReviewRequest:
    trial_n: int
    loaded_count: int
    goal: str
    settings: dict
    bounds: dict
    metrics: dict
    robot_summary: dict
    history: list
    spot: dict
    frames: list = field(default_factory=list)  # [{"path": str, "t_from_gate_open_ms": float}]
    camera_notes: str = ""

    def image_b64(self, frame):
        return base64.standard_b64encode(Path(frame["path"]).read_bytes()).decode("ascii")


class Reviewer:
    name = "base"

    def review(self, request, folder=None):
        """Returns the raw assessment dict (validated/normalized by the caller)."""
        raise NotImplementedError


SYSTEM_PROMPT = """You review FTC robot launch trials from camera frames.
The robot is stationary and launches a burst of up to 4 balls (POLLEN, 2.8 in yellow plastic balls with holes) from a
turret flywheel into a target CELL. You label each loaded ball, in launch order, as exactly one of:
scored (went into the target), short, long, left, right (missed, direction as seen from behind the robot toward the
target), stuck (never left the robot; e.g. stayed in the transfer-to-flywheel dead zone), unknown (not clearly visible,
ambiguous, or out of frame).
Rules:
- Use only what the frames show. If you cannot clearly see a ball's outcome, label it unknown with low confidence.
  Never infer a result from the telemetry alone. Do not guess the order of balls you cannot distinguish: use unknown.
- The robot's storage sensors cannot see the dead zone above sensor 4, so "count reached 0" does not prove a ball left.
- confidence is 0..1 for each label.
- Propose at most ONE parameter change, or parameter "none". Prefer target_rpm for consistent short/long misses when the
  encoder data shows the flywheel tracking its target; turret_position for consistent left/right misses; P / kV only
  when the encoder data shows poor tracking or slow recovery; never change kS/kV/P to fix a mechanical jam or a stuck
  ball. One operating speed cannot separate kS from kV. Stay within the bounds and max step given. If the goal was met,
  propose "none".
Return JSON matching the schema."""


def build_prompt(req):
    """Text part of the review prompt (the frames are attached separately)."""
    payload = {
        "trial": req.trial_n,
        "loaded_balls": req.loaded_count,
        "goal": f"{GOALS[req.goal]} confirmed scores out of {req.loaded_count} loaded",
        "spot": req.spot,
        "settings": {k: req.settings.get(k) for k in PARAMS},
        "units": UNITS,
        "bounds": req.bounds,
        "robot_summary": {k: req.robot_summary.get(k) for k in (
            "completion_reason", "count_at_fire", "count_at_end", "robot_counted_exits",
            "empty_confirmed_at_timeout", "left_rpm_at_fire", "right_rpm_at_fire", "battery_v_at_fire")},
        "encoder_metrics": req.metrics,
        "history_kept_trials": req.history,
        "camera_notes": req.camera_notes,
        "frames": [{"frame": i + 1, "t_from_gate_open_ms": round(f["t_from_gate_open_ms"])} for i, f in enumerate(req.frames)],
    }
    return ("Trial context (JSON). Frames follow in the same order as the 'frames' list; "
            "times are relative to the moment the gate opened (approximate, see clock uncertainty).\n"
            + json.dumps(payload, indent=1, default=str))
