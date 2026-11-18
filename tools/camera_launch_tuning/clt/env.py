"""Local credential loading.

Reads a git-ignored .env file as plain KEY=VALUE text. Nothing in the file is executed, expanded
or interpolated. Variables already set in the process environment win over the file. Key values
are never printed: use describe_credentials() for a safe summary.
"""

import os
import re
from pathlib import Path

DEFAULT_ENV_PATH = Path(__file__).resolve().parent.parent / ".env"

KNOWN_KEYS = ("OPENAI_API_KEY", "OPENAI_MODEL", "ANTHROPIC_API_KEY", "ANTHROPIC_MODEL")
SECRET_KEYS = ("OPENAI_API_KEY", "ANTHROPIC_API_KEY")

DEFAULT_MODELS = {
    "OPENAI_MODEL": "gpt-6.2-astra",
    "ANTHROPIC_MODEL": "claude-opus-5-5",
}

_LINE = re.compile(r"^\s*(?:export\s+)?([A-Za-z_][A-Za-z0-9_]*)\s*=\s*(.*?)\s*$")


def parse_env_text(text):
    """Parses KEY=VALUE lines. Returns a dict. Never evaluates anything.

    Supported: blank lines, # comments, an optional leading `export `, values in single or double
    quotes (taken literally, no escapes or ${VAR} expansion), and trailing ` # comment` after an
    unquoted value. Lines that don't match are ignored.
    """
    values = {}
    for raw in text.splitlines():
        line = raw.strip()
        if not line or line.startswith("#"):
            continue
        m = _LINE.match(line)
        if not m:
            continue
        key, value = m.group(1), m.group(2)
        if len(value) >= 2 and value[0] == value[-1] and value[0] in ("'", '"'):
            value = value[1:-1]
        else:
            value = re.split(r"\s+#", value, maxsplit=1)[0].strip()
        values[key] = value
    return values


def load_env(path=None, environ=None):
    """Loads a .env file into `environ` (default os.environ) without overriding existing values.

    `path` None = the tool's own .env (missing is fine). An explicit path that doesn't exist raises
    FileNotFoundError. Returns {"path": str or None, "loaded": [names set from the file],
    "kept": [names already set in the environment]}. Values are not returned.
    """
    environ = os.environ if environ is None else environ
    explicit = path is not None
    p = Path(path).expanduser() if explicit else DEFAULT_ENV_PATH
    result = {"path": None, "loaded": [], "kept": []}
    if not p.exists():
        if explicit:
            raise FileNotFoundError(f"env file not found: {p}")
        return result
    result["path"] = str(p)
    for key, value in parse_env_text(p.read_text(encoding="utf-8")).items():
        if key in environ and environ[key] != "":
            result["kept"].append(key)
            continue
        if value == "":
            continue
        environ[key] = value
        result["loaded"].append(key)
    return result


def get_model(name, environ=None):
    environ = os.environ if environ is None else environ
    return environ.get(name) or DEFAULT_MODELS[name]


def describe_credentials(environ=None):
    """Safe summary: which keys are present (never their values) and which models will be used."""
    environ = os.environ if environ is None else environ
    return {
        "OPENAI_API_KEY": "set" if environ.get("OPENAI_API_KEY") else "missing",
        "OPENAI_MODEL": get_model("OPENAI_MODEL", environ),
        "ANTHROPIC_API_KEY": "set" if environ.get("ANTHROPIC_API_KEY") else "missing",
        "ANTHROPIC_MODEL": get_model("ANTHROPIC_MODEL", environ),
    }


def redact(text, environ=None):
    """Replaces any secret value found in `text` with [REDACTED]. Use before logging or saving."""
    environ = os.environ if environ is None else environ
    if not isinstance(text, str):
        return text
    for key in SECRET_KEYS:
        value = environ.get(key)
        if value and len(value) >= 8:
            text = text.replace(value, "[REDACTED]")
    return text
