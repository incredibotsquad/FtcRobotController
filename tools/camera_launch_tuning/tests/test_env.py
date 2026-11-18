import tempfile
import unittest
from pathlib import Path

from helpers import *  # noqa: F401,F403  (sets sys.path)
from clt import env


class EnvTests(unittest.TestCase):
    def test_parse_is_literal_and_never_executes(self):
        text = ("# comment\nexport OPENAI_API_KEY='sk-test-1234567890'\nANTHROPIC_MODEL=claude-opus-5-5  # note\n"
                "EVIL=$(touch /tmp/should_not_exist_clt)\nQUOTED=\"a b # c\"\nnot a line\n")
        values = env.parse_env_text(text)
        self.assertEqual(values["OPENAI_API_KEY"], "sk-test-1234567890")
        self.assertEqual(values["ANTHROPIC_MODEL"], "claude-opus-5-5")
        self.assertEqual(values["EVIL"], "$(touch /tmp/should_not_exist_clt)")
        self.assertEqual(values["QUOTED"], "a b # c")
        self.assertFalse(Path("/tmp/should_not_exist_clt").exists())

    def test_existing_environment_wins_and_blanks_are_skipped(self):
        with tempfile.TemporaryDirectory() as d:
            p = Path(d) / ".env"
            p.write_text("OPENAI_API_KEY=from-file-123456\nANTHROPIC_API_KEY=\nOPENAI_MODEL=m1\n")
            environ = {"OPENAI_API_KEY": "from-shell-abcdef"}
            res = env.load_env(p, environ)
            self.assertEqual(environ["OPENAI_API_KEY"], "from-shell-abcdef")
            self.assertEqual(environ["OPENAI_MODEL"], "m1")
            self.assertNotIn("ANTHROPIC_API_KEY", environ)
            self.assertEqual(res["kept"], ["OPENAI_API_KEY"])
            self.assertNotIn("from-file-123456", repr(res))

    def test_explicit_missing_path_raises_default_missing_is_fine(self):
        with self.assertRaises(FileNotFoundError):
            env.load_env("/nonexistent/clt.env", {})
        self.assertEqual(env.load_env(None, {})["path"] in (None, str(env.DEFAULT_ENV_PATH)), True)

    def test_describe_and_redact_never_reveal_values(self):
        environ = {"OPENAI_API_KEY": "sk-secret-value-xyz", "ANTHROPIC_API_KEY": ""}
        desc = env.describe_credentials(environ)
        self.assertEqual(desc["OPENAI_API_KEY"], "set")
        self.assertEqual(desc["ANTHROPIC_API_KEY"], "missing")
        self.assertEqual(desc["ANTHROPIC_MODEL"], "claude-opus-5-5")
        self.assertNotIn("sk-secret-value-xyz", repr(desc))
        self.assertEqual(env.redact("key=sk-secret-value-xyz", environ), "key=[REDACTED]")

    def test_env_example_is_blank(self):
        values = env.parse_env_text((Path(env.__file__).parent.parent / ".env.example").read_text())
        self.assertEqual(set(values), set(env.KNOWN_KEYS))
        self.assertTrue(all(v == "" for v in values.values()))


if __name__ == "__main__":
    unittest.main()
