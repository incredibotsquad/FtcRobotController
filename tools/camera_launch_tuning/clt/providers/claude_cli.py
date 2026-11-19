"""Reviewer that runs the locally installed Claude Code CLI headless (`claude -p`), under your own
Claude login instead of an API key.

Per trial it writes the same review bundle as agent-file (REVIEW_REQUEST.md, request.json,
schema.json, frames/), then runs:

  claude -p <prompt> --output-format json --json-schema <ASSESSMENT_SCHEMA>
         --tools Read --allowedTools Read --add-dir <bundle> --permission-mode dontAsk
         --no-session-persistence

in the bundle folder. Only the Read tool exists in that session, so it can look at the frames and
nothing else (no shell, no edits, no web). The answer is the `structured_output` field of the JSON
result. OPENAI_API_KEY / ANTHROPIC_API_KEY are removed from the child's environment, so the CLI uses
its own login rather than a key loaded from .env.
"""

import json
import os
import shutil
import subprocess
from pathlib import Path

from ..env import SECRET_KEYS
from .agent_file import AgentFileReviewer
from .base import ASSESSMENT_SCHEMA, ReviewError, Reviewer

PROMPT = """You are reviewing one finished FTC launch trial. Everything you need is in this folder: {folder}
1. Read REVIEW_REQUEST.md (instructions and rules) and request.json (context and the ordered frame list).
2. Read EVERY image listed under "images" in request.json, in order (files under frames/), using its caption.
3. Answer with the assessment object only, following the rules: label a ball unknown whenever its outcome
   is not clearly visible; propose at most one change, or parameter "none"."""


def find_claude(explicit=None):
    return explicit or shutil.which("claude")


def child_env(environ=None):
    """The environment for the CLI: everything except the API keys this tool may have loaded."""
    env = dict(os.environ if environ is None else environ)
    for k in SECRET_KEYS:
        env.pop(k, None)
    return env


class ClaudeCliReviewer(Reviewer):
    name = "claude-cli"
    needs_frames = True

    def __init__(self, claude=None, model=None, timeout_s=900, runner=subprocess.run):
        self.claude = find_claude(claude)
        if not self.claude:
            raise ReviewError("`claude` CLI not found on PATH (install Claude Code, or pass --claude PATH)")
        self.model = model
        self.timeout_s = timeout_s
        self.runner = runner

    def command(self, folder):
        cmd = [self.claude, "-p", PROMPT.format(folder=folder), "--output-format", "json",
               "--json-schema", json.dumps(ASSESSMENT_SCHEMA), "--tools", "Read", "--allowedTools", "Read",
               "--add-dir", str(folder), "--permission-mode", "dontAsk", "--no-session-persistence"]
        if self.model:
            cmd += ["--model", self.model]
        return cmd

    def review(self, req, folder=None):
        if folder is None:
            raise ReviewError("claude-cli review needs the trial folder")
        folder = Path(folder).resolve()
        AgentFileReviewer(notify=lambda *_: None).write_request(req, folder)
        try:
            res = self.runner(self.command(folder), cwd=str(folder), env=child_env(), capture_output=True,
                              text=True, timeout=self.timeout_s)
        except subprocess.TimeoutExpired as e:
            raise ReviewError(f"claude CLI took longer than {self.timeout_s} s") from e
        except FileNotFoundError as e:
            raise ReviewError(f"claude CLI not runnable: {self.claude}") from e
        (folder / "claude_cli_stderr.txt").write_text(res.stderr or "", encoding="utf-8")
        try:
            out = json.loads(res.stdout)
        except (json.JSONDecodeError, TypeError) as e:
            raise ReviewError(f"claude CLI exit {res.returncode}, output not JSON (see claude_cli_stderr.txt)") from e
        (folder / "claude_cli_result.json").write_text(json.dumps(out, indent=2), encoding="utf-8")
        if res.returncode != 0 or out.get("is_error") or out.get("subtype") != "success":
            raise ReviewError(f"claude CLI failed: {out.get('subtype')} {str(out.get('result', ''))[:200]}")
        answer = out.get("structured_output")
        if answer is None:
            try:
                answer = json.loads(out.get("result") or "")
            except json.JSONDecodeError as e:
                raise ReviewError("claude CLI returned no structured output") from e
        if not isinstance(answer, dict):
            raise ReviewError("claude CLI structured output is not an object")
        return answer


def probe(claude=None, timeout_s=180, runner=subprocess.run, workdir=None):
    """Tiny headless call (no images) to confirm the CLI is logged in and returns structured output.
    Returns (ok, detail). Uses a little of your Claude usage."""
    exe = find_claude(claude)
    if not exe:
        return False, "claude not found"
    schema = {"type": "object", "properties": {"ok": {"type": "boolean"}}, "required": ["ok"], "additionalProperties": False}
    cmd = [exe, "-p", "Reply with ok = true.", "--output-format", "json", "--json-schema", json.dumps(schema),
           "--tools", "", "--permission-mode", "dontAsk", "--no-session-persistence"]
    try:
        res = runner(cmd, cwd=workdir, env=child_env(), capture_output=True, text=True, timeout=timeout_s)
        out = json.loads(res.stdout)
    except (subprocess.TimeoutExpired, json.JSONDecodeError, TypeError, FileNotFoundError) as e:
        return False, f"probe failed: {type(e).__name__}"
    if out.get("subtype") == "success" and (out.get("structured_output") or {}).get("ok") is True:
        model = next(iter(out.get("modelUsage") or {}), "?")
        return True, f"headless structured output works (model {model}, {out.get('duration_ms', '?')} ms)"
    return False, f"probe returned {out.get('subtype')}: {str(out.get('result', ''))[:150]}"


def version(claude=None, runner=subprocess.run):
    exe = find_claude(claude)
    if not exe:
        return None
    try:
        res = runner([exe, "--version"], capture_output=True, text=True, timeout=20)
        return (res.stdout or "").strip().splitlines()[0] if res.stdout else None
    except (subprocess.TimeoutExpired, FileNotFoundError, OSError):
        return None
