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
    examples: list = field(default_factory=list)  # [{"id", "labels", "images": [{"path", "caption"}]}]

    def image_b64(self, frame):
        return base64.standard_b64encode(Path(frame["path"]).read_bytes()).decode("ascii")


def review_images(req):
    """Every image to show, in order: calibration examples first, then this trial's frames."""
    out = []
    for ex in req.examples:
        for im in ex["images"]:
            out.append({"path": im["path"], "caption": f"EXAMPLE {ex['id']} (person's labels {ex['labels']}): {im['caption']}",
                        "kind": "example", "t_from_gate_open_ms": 0.0})
    for f in req.frames:
        out.append(dict(f, caption="THIS TRIAL: " + f.get("caption", f"{f['t_from_gate_open_ms']:+.0f} ms from gate open")))
    return out


class Reviewer:
    name = "base"
    needs_frames = False  # True: without review frames every ball is unknown (no call is made)

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
  Never infer a result from the telemetry alone.
- Deciding outcomes (as seen from behind the robot toward the target; the camera may be at the side, see the spot
  description and camera notes):
  * scored: the ball enters the up-facing CELL's opening.
  * long: the ball goes OVER the HIVE / CELL without entering. You do not need to see where it lands: once it has
    clearly passed over the top without going in, it is long, even if it then leaves the frame.
  * short: the ball doesn't get high or far enough to enter: it hits the OUTSIDE of the HIVE / CELL (including the
    CELL's lower / near rim from below), or it never even reaches the raised CELL (its arc tops out below the CELL, or
    it comes down in front of the HIVE). Either way, short.
  * A ball that hits the CELL's upper / far rim from above after clearing the opening and bounces out is long. Say
    which rim and from which side in the evidence.
  * left / right: misses beside the CELL. From a side camera these are often not visible; then use what you can see.
  * The rest-state images are a cross-check only: the number of balls labeled scored should match the new balls
    inside the CELL.
- Order only matters when outcomes differ. If balls you can't tell apart all clearly had the same outcome (for
  example three balls of one burst all clearly pass over the HIVE), give each of them that label.
  Use unknown only for a ball whose own outcome is unclear.
- The encoder can merge two balls that hit the wheel within about 100 ms into one dip, so ball windows are
  approximate. Count balls from the trajectories; the number of loaded balls is exact.
- EXAMPLE images, when present, come from earlier trials at this spot with the same camera, labeled by a person.
  Use them to calibrate what scored / short / long look like from this view. Never copy their labels.
- The robot's storage sensors cannot see the dead zone above sensor 4, so "count reached 0" does not prove a ball left.
- Images are grouped per ball: ball k's window starts at its wheel contact (the start of its flywheel speed dip in
  the encoder log) and follows its flight; a MOTION TRAIL image overlays everything that moved in that window, so the
  ball's whole path shows in one picture. Rest-state images at the end show where balls came to rest. Use each ball's
  window and trail for that ball. Windows of balls launched close together overlap; if you can't tell which ball is
  which, use unknown for those balls only.
- confidence is 0..1 for each label.
- Propose at most ONE parameter change, or parameter "none". RPM direction: balls that go OVER need LOWER target_rpm;
  balls that come in UNDER (short) need HIGHER target_rpm. If the misses in a trial go both ways, no RPM change can fix
  both: propose "none" (or transfer_power when the encoder shows balls arriving too close together or a late, slow
  ball) and say it is a consistency problem. Prefer target_rpm for consistent misses in one direction when the
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
        "calibration_examples": [{"id": ex["id"], "person_labels": ex["labels"], "note": ex.get("note", "")} for ex in req.examples],
        "images": [{"image": i + 1, "kind": f.get("kind", "frame"), "ball": f.get("ball"),
                    "caption": f["caption"]} for i, f in enumerate(review_images(req))],
    }
    return ("Trial context (JSON). Images follow in the same order as the 'images' list; times are relative to the "
            "moment the gate opened (approximate, see clock uncertainty).\n"
            + json.dumps(payload, indent=1, default=str))
