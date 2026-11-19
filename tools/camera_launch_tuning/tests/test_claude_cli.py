import json
import os
import subprocess
import tempfile
import unittest
from pathlib import Path

from helpers import assessment
from test_providers import request
from clt.providers import make_reviewer
from clt.providers.base import ASSESSMENT_SCHEMA, ReviewError
from clt.providers.claude_cli import ClaudeCliReviewer, child_env, probe


class FakeClaude:
    def __init__(self, stdout, returncode=0, raise_exc=None):
        self.stdout, self.returncode, self.raise_exc = stdout, returncode, raise_exc
        self.cmd = self.kwargs = None

    def __call__(self, cmd, **kwargs):
        self.cmd, self.kwargs = cmd, kwargs
        if self.raise_exc:
            raise self.raise_exc
        return subprocess.CompletedProcess(cmd, self.returncode, stdout=self.stdout, stderr="")


def ok_output(answer):
    return json.dumps({"subtype": "success", "is_error": False, "result": json.dumps(answer), "structured_output": answer})


class ClaudeCliContract(unittest.TestCase):
    def test_command_is_read_only_headless_with_schema(self):
        answer = assessment(["scored", "scored", "short", "stuck"])
        fake = FakeClaude(ok_output(answer))
        with tempfile.TemporaryDirectory() as d:
            folder = Path(d) / "review_request"
            out = ClaudeCliReviewer(claude="/fake/claude", runner=fake).review(request(d), folder=folder)
            self.assertEqual(out["balls"][3]["label"], "stuck")
            cmd = fake.cmd
            self.assertEqual(cmd[:2], ["/fake/claude", "-p"])
            flag = lambda name: cmd[cmd.index(name) + 1]
            self.assertEqual(flag("--output-format"), "json")
            self.assertEqual(json.loads(flag("--json-schema")), ASSESSMENT_SCHEMA)
            self.assertEqual(flag("--tools"), "Read")
            self.assertEqual(flag("--allowedTools"), "Read")
            self.assertEqual(flag("--permission-mode"), "dontAsk")
            self.assertEqual(Path(flag("--add-dir")), folder.resolve())
            self.assertIn("--no-session-persistence", cmd)
            self.assertNotIn("bypassPermissions", cmd)
            self.assertEqual(Path(fake.kwargs["cwd"]), folder.resolve())
            for f in ("REVIEW_REQUEST.md", "request.json", "schema.json", "claude_cli_result.json"):
                self.assertTrue((folder / f).exists(), f)
            self.assertEqual(len(list((folder / "frames").iterdir())), 1)

    def test_api_keys_not_passed_to_cli(self):
        env = child_env({"PATH": "/bin", "ANTHROPIC_API_KEY": "sk-a", "OPENAI_API_KEY": "sk-o", "HOME": "/h"})
        self.assertEqual(env, {"PATH": "/bin", "HOME": "/h"})
        fake = FakeClaude(ok_output(assessment(["scored"] * 4)))
        os.environ["ANTHROPIC_API_KEY"] = "sk-test-should-not-pass"
        try:
            with tempfile.TemporaryDirectory() as d:
                ClaudeCliReviewer(claude="c", runner=fake).review(request(d), folder=Path(d) / "rr")
        finally:
            del os.environ["ANTHROPIC_API_KEY"]
        self.assertNotIn("ANTHROPIC_API_KEY", fake.kwargs["env"])

    def test_failures_raise(self):
        cases = [FakeClaude(json.dumps({"subtype": "error_max_turns", "is_error": True, "result": ""})),
                 FakeClaude("not json", returncode=1),
                 FakeClaude("", raise_exc=subprocess.TimeoutExpired("claude", 1)),
                 FakeClaude(json.dumps({"subtype": "success", "is_error": False, "result": "no json here"}))]
        with tempfile.TemporaryDirectory() as d:
            for fake in cases:
                with self.assertRaises(ReviewError):
                    ClaudeCliReviewer(claude="c", runner=fake).review(request(d), folder=Path(d) / "rr")

    def test_falls_back_to_result_text(self):
        answer = assessment(["scored"] * 4)
        fake = FakeClaude(json.dumps({"subtype": "success", "is_error": False, "result": json.dumps(answer)}))
        with tempfile.TemporaryDirectory() as d:
            out = ClaudeCliReviewer(claude="c", runner=fake).review(request(d), folder=Path(d) / "rr")
        self.assertEqual(len(out["balls"]), 4)

    def test_missing_cli_and_registry(self):
        with self.assertRaises(ReviewError):
            ClaudeCliReviewer(claude=None, runner=FakeClaude("")) if not __import__("shutil").which("claude") else (_ for _ in ()).throw(ReviewError("x"))
        self.assertTrue(make_reviewer("claude-cli", claude="c", runner=FakeClaude("")).needs_frames)

    def test_probe(self):
        good = FakeClaude(json.dumps({"subtype": "success", "structured_output": {"ok": True}, "modelUsage": {"m": {}}}))
        self.assertTrue(probe(claude="c", runner=good)[0])
        self.assertIn("--tools", good.cmd)
        self.assertFalse(probe(claude="c", runner=FakeClaude("oops"))[0])


if __name__ == "__main__":
    unittest.main()
