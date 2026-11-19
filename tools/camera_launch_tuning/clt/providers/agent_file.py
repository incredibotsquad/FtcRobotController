"""File-based review by an interactive agent session (Codex, Claude Code, ...) you run yourself.

The tool writes review_request/REVIEW_REQUEST.md, request.json, schema.json and the frames into the
trial folder, then waits for the agent (or you) to write review_request/assessment.json. Nothing
watches a live feed: the agent reviews the saved frames of one finished trial at a time. The
agent's own login is separate from API keys; no key is needed for this mode.
"""

import json
import shutil
import time
from pathlib import Path

from .base import ASSESSMENT_SCHEMA, SYSTEM_PROMPT, ReviewError, Reviewer, build_prompt, review_images

INSTRUCTIONS = """# Launch trial review request

{system}

## How to answer
1. Look at every image in `frames/`, in the order listed under "images" in request.json (each has a caption).
2. Read `request.json` (settings, bounds, encoder metrics, robot summary, kept history).
3. Write `assessment.json` in this folder, matching `schema.json` exactly. Write it to
   `assessment.json.tmp` first and rename it, so the tool never reads half a file.
4. Use "unknown" for anything you can't clearly see. Propose at most one change, or "none".

## Context
{prompt}
"""


class AgentFileReviewer(Reviewer):
    name = "agent-file"

    def __init__(self, timeout_s=1800, poll_s=2.0, sleep=time.sleep, clock=time.monotonic, notify=print):
        self.timeout_s = timeout_s
        self.poll_s = poll_s
        self.sleep = sleep
        self.clock = clock
        self.notify = notify

    def write_request(self, req, folder):
        folder = Path(folder)
        frames_dir = folder / "frames"
        frames_dir.mkdir(parents=True, exist_ok=True)
        listed = []
        for i, f in enumerate(review_images(req)):
            dst = frames_dir / f"{i + 1:02d}_{'ex_' if f.get('kind') == 'example' else ''}{Path(f['path']).name}"
            shutil.copyfile(f["path"], dst)
            listed.append({"image": i + 1, "file": f"frames/{dst.name}", "kind": f.get("kind", "frame"), "ball": f.get("ball"),
                           "caption": f["caption"]})
        (folder / "schema.json").write_text(json.dumps(ASSESSMENT_SCHEMA, indent=2), encoding="utf-8")
        (folder / "request.json").write_text(json.dumps({"images": listed, "context": json.loads(build_prompt(req).split("\n", 1)[1])},
                                                        indent=2, default=str), encoding="utf-8")
        (folder / "REVIEW_REQUEST.md").write_text(INSTRUCTIONS.format(system=SYSTEM_PROMPT, prompt=build_prompt(req)),
                                                  encoding="utf-8")
        return folder / "assessment.json"

    def review(self, req, folder=None):
        if folder is None:
            raise ReviewError("agent-file review needs the trial folder")
        answer = self.write_request(req, folder)
        if answer.exists():
            answer.unlink()
        self.notify(f"Waiting for an agent to write {answer} (see {Path(folder) / 'REVIEW_REQUEST.md'}).")
        deadline = self.clock() + self.timeout_s
        while self.clock() < deadline:
            if answer.exists():
                try:
                    return json.loads(answer.read_text(encoding="utf-8"))
                except json.JSONDecodeError as e:
                    raise ReviewError("assessment.json is not valid JSON") from e
            self.sleep(self.poll_s)
        raise ReviewError(f"no assessment.json within {self.timeout_s} s")
