"""Calibration examples: person-labeled trials shown to the reviewer next to each new trial.

Stored per spot (same camera view) under profiles/<spot>_examples/<id>/:
  example.json   {"id", "labels", "source": {"run", "trial"}, "note", "images": [{"file", "caption"}]}
  *.jpg          each ball's motion trail plus one rest-state frame from that trial
This is context, not training: the reviewer sees the examples on every review; nothing is learned.
"""

import json
import shutil
import time
from pathlib import Path

from .policy import LABELS
from .store import profiles_dir, read_json, write_json

MAX_IMAGES_PER_EXAMPLE = 6


def examples_dir(spot, root=None):
    return profiles_dir(root) / f"{spot}_examples"


def add_example(spot, trial_dir, labels, source, note="", root=None):
    """Copies a trial's motion trails and last rest frame as a labeled example. Returns its id."""
    if not labels or any(l not in LABELS for l in labels):
        raise ValueError(f"labels must be from {LABELS}")
    if all(l == "unknown" for l in labels):
        raise ValueError("an example needs at least one known label")
    rf = Path(trial_dir) / "review_frames.json"
    if not rf.exists():
        raise FileNotFoundError(f"{rf} missing: the trial has no review frames (re-select with `review --reframe`)")
    frames = read_json(rf)["frames"]
    trails = [f for f in frames if f.get("kind") == "trail"]
    rests = [f for f in frames if f.get("kind") == "rest"]
    if not trails:
        raise ValueError("the trial has no motion-trail images (re-select with `review --reframe`)")
    ex_id = time.strftime("%Y%m%d-%H%M%S")
    d = examples_dir(spot, root) / ex_id
    d.mkdir(parents=True, exist_ok=False)
    images = []
    for f in sorted(trails, key=lambda x: x["ball"])[:MAX_IMAGES_PER_EXAMPLE - 1]:
        k = f["ball"]
        name = f"trail_ball_{k}.jpg"
        shutil.copyfile(f["path"], d / name)
        lab = labels[k - 1] if k - 1 < len(labels) else "unknown"
        images.append({"file": name, "caption": f"ball {k} motion trail; the person labeled this ball {lab}"})
    if rests:
        shutil.copyfile(rests[-1]["path"], d / "rest.jpg")
        images.append({"file": "rest.jpg", "caption": f"rest state after the launch ({labels.count('scored')} scored)"})
    write_json(d / "example.json", {"id": ex_id, "labels": list(labels), "source": source, "note": note, "images": images})
    return ex_id


def list_examples(spot, root=None):
    base = examples_dir(spot, root)
    if not base.exists():
        return []
    out = []
    for p in sorted(base.iterdir()):
        j = p / "example.json"
        if j.exists():
            out.append(read_json(j))
    return out


def load_for_review(spot, limit=2, root=None):
    """The newest `limit` examples, with absolute image paths, for a ReviewRequest."""
    base = examples_dir(spot, root)
    out = []
    for ex in list_examples(spot, root)[-limit:] if limit > 0 else []:
        out.append({"id": ex["id"], "labels": ex["labels"], "note": ex.get("note", ""),
                    "images": [{"path": str(base / ex["id"] / im["file"]), "caption": im["caption"]} for im in ex["images"]]})
    return out


def remove_example(spot, ex_id, root=None):
    d = examples_dir(spot, root) / ex_id
    if not (d / "example.json").exists():
        raise FileNotFoundError(f"no example {ex_id} for spot {spot}")
    shutil.rmtree(d)


def parse_labels(text, count):
    """'l l l h' / 'l,l,l,h' / 'long long long short' -> labels. Shortcuts as in the manual reviewer."""
    from .providers.manual import SHORTCUTS
    parts = [p for p in text.replace(",", " ").split() if p]
    labels = [SHORTCUTS.get(p.lower(), p.lower()) for p in parts]
    if len(labels) != count or any(l not in LABELS for l in labels):
        raise ValueError(f"need {count} labels from s/h/l/f/r/k/u (or full names)")
    return labels


def dumps(examples):
    return json.dumps(examples, indent=2)
