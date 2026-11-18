"""Manual scoring at the terminal (watch the saved video, then type a label per ball)."""

from ..policy import LABELS
from ..settings import PARAMS
from .base import Reviewer

SHORTCUTS = {"s": "scored", "h": "short", "l": "long", "f": "left", "r": "right", "k": "stuck", "u": "unknown", "": "unknown"}


class ManualReviewer(Reviewer):
    name = "manual"

    def __init__(self, ask=input, say=print):
        self.ask = ask
        self.say = say

    def review(self, req, folder=None):
        self.say(f"Trial {req.trial_n}: label each of the {req.loaded_count} balls in launch order.")
        self.say("  s=scored h=short l=long f=left r=right k=stuck u=unknown (Enter = unknown)")
        balls = []
        for i in range(1, req.loaded_count + 1):
            while True:
                a = self.ask(f"  ball {i}: ").strip().lower()
                label = SHORTCUTS.get(a, a)
                if label in LABELS:
                    break
                self.say("    not a label")
            balls.append({"index": i, "label": label, "confidence": 1.0 if label != "unknown" else 0.0, "evidence": "manual"})
        quality = self.ask("Footage quality good/partial/poor [good]: ").strip().lower() or "good"
        notes = self.ask("Notes (optional): ").strip()
        param = self.ask(f"Propose one change ({', '.join(PARAMS)}) or Enter for none: ").strip()
        proposal = {"parameter": "none", "new_value": None, "reason": ""}
        if param in PARAMS:
            raw = self.ask(f"  new {param}: ").strip()
            try:
                value = float(raw)
            except ValueError:
                value = None
            proposal = {"parameter": param, "new_value": value, "reason": self.ask("  reason: ").strip()}
        return {"footage_quality": quality if quality in ("good", "partial", "poor") else "poor",
                "balls": balls, "notes": notes, "proposal": proposal}
