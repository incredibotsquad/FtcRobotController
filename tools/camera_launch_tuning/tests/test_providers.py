import json
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace as NS

from helpers import BASE_SETTINGS, assessment, bounds
from clt.providers.agent_file import AgentFileReviewer
from clt.providers.anthropic_provider import FALLBACK_BETA, AnthropicReviewer
from clt.providers.base import ASSESSMENT_SCHEMA, ReviewError, ReviewRequest
from clt.providers.manual import ManualReviewer
from clt.providers.openai_provider import OpenAIReviewer


def request(tmp):
    img = Path(tmp) / "frame_00001.jpg"
    img.write_bytes(b"\xff\xd8\xff\xe0fakejpeg")
    return ReviewRequest(trial_n=1, loaded_count=4, goal="3of4", settings=BASE_SETTINGS, bounds=bounds(), metrics={},
                         robot_summary={"completion_reason": "empty"}, history=[], spot={"spot": "right1"},
                         frames=[{"path": str(img), "t_from_gate_open_ms": 120.0}])


def walk_objects(schema):
    if isinstance(schema, dict):
        if schema.get("type") == "object":
            yield schema
        for v in schema.values():
            yield from walk_objects(v)
    elif isinstance(schema, list):
        for v in schema:
            yield from walk_objects(v)


class SchemaContract(unittest.TestCase):
    def test_strict_schema_rules(self):
        for obj in walk_objects(ASSESSMENT_SCHEMA):
            self.assertIs(obj.get("additionalProperties"), False)
            self.assertEqual(set(obj["required"]), set(obj["properties"]))


class FakeOpenAI:
    def __init__(self, response):
        self.kwargs = None
        self.responses = NS(create=self._create)
        self._response = response

    def _create(self, **kw):
        self.kwargs = kw
        return self._response


class OpenAIContract(unittest.TestCase):
    def test_request_shape_and_parse(self):
        answer = assessment(["scored", "scored", "scored", "stuck"])
        resp = NS(status="completed", output=[NS(type="message", content=[NS(type="output_text", text=json.dumps(answer))])])
        client = FakeOpenAI(resp)
        with tempfile.TemporaryDirectory() as d:
            out = OpenAIReviewer(client=client, model="m").review(request(d))
        self.assertEqual(out["balls"][3]["label"], "stuck")
        fmt = client.kwargs["text"]["format"]
        self.assertEqual((fmt["type"], fmt["strict"]), ("json_schema", True))
        self.assertIs(fmt["schema"], ASSESSMENT_SCHEMA)
        user = client.kwargs["input"][1]["content"]
        img = [c for c in user if c["type"] == "input_image"][0]
        self.assertTrue(img["image_url"].startswith("data:image/jpeg;base64,"))

    def test_refusal_and_incomplete_raise(self):
        refuse = NS(status="completed", output=[NS(type="message", content=[NS(type="refusal", refusal="no")])])
        with tempfile.TemporaryDirectory() as d:
            with self.assertRaises(ReviewError):
                OpenAIReviewer(client=FakeOpenAI(refuse), model="m").review(request(d))
            with self.assertRaises(ReviewError):
                OpenAIReviewer(client=FakeOpenAI(NS(status="incomplete", incomplete_details="max", output=[])), model="m").review(request(d))


class FakeAnthropic:
    def __init__(self, response):
        self.kwargs = None
        self.beta = NS(messages=NS(create=self._create))
        self.messages = NS(create=self._create)
        self._response = response

    def _create(self, **kw):
        self.kwargs = kw
        return self._response


class AnthropicContract(unittest.TestCase):
    def test_request_shape_and_parse(self):
        answer = assessment(["scored"] * 4)
        client = FakeAnthropic(NS(stop_reason="end_turn", content=[NS(type="text", text=json.dumps(answer))]))
        with tempfile.TemporaryDirectory() as d:
            out = AnthropicReviewer(client=client).review(request(d))
        self.assertEqual(len(out["balls"]), 4)
        kw = client.kwargs
        self.assertEqual(kw["model"], "claude-opus-5-5")
        self.assertEqual(kw["betas"], [FALLBACK_BETA])
        self.assertEqual(kw["fallbacks"], "default")
        self.assertEqual(kw["output_config"]["format"]["type"], "json_schema")
        img = [c for c in kw["messages"][0]["content"] if c["type"] == "image"][0]
        self.assertEqual(img["source"]["type"], "base64")
        self.assertNotIn("thinking", kw)  # always on for this model; budget/disabled would be rejected

    def test_refusal_and_truncation_raise(self):
        with tempfile.TemporaryDirectory() as d:
            for stop in ("refusal", "max_tokens"):
                client = FakeAnthropic(NS(stop_reason=stop, stop_details=None, content=[]))
                with self.assertRaises(ReviewError):
                    AnthropicReviewer(client=client).review(request(d))

    def test_missing_keys_fail_cleanly(self):
        import os
        saved = {k: os.environ.pop(k, None) for k in ("OPENAI_API_KEY", "ANTHROPIC_API_KEY")}
        try:
            with self.assertRaises(ReviewError):
                OpenAIReviewer()
            with self.assertRaises(ReviewError):
                AnthropicReviewer()
        finally:
            for k, v in saved.items():
                if v is not None:
                    os.environ[k] = v


class FileAndManualContract(unittest.TestCase):
    def test_agent_file_roundtrip(self):
        with tempfile.TemporaryDirectory() as d:
            folder = Path(d) / "review_request"
            answer = assessment(["scored", "short", "scored", "scored"])

            def fake_sleep(_):
                (folder / "assessment.json").write_text(json.dumps(answer))

            r = AgentFileReviewer(timeout_s=5, poll_s=0, sleep=fake_sleep, notify=lambda *_: None)
            out = r.review(request(d), folder=folder)
            self.assertEqual(out["balls"][1]["label"], "short")
            self.assertTrue((folder / "REVIEW_REQUEST.md").exists())
            self.assertTrue((folder / "schema.json").exists())
            self.assertEqual(len(list((folder / "frames").iterdir())), 1)

    def test_agent_file_timeout(self):
        with tempfile.TemporaryDirectory() as d:
            t = [0.0]
            r = AgentFileReviewer(timeout_s=1, poll_s=0, sleep=lambda _: t.__setitem__(0, t[0] + 2),
                                  clock=lambda: t[0], notify=lambda *_: None)
            with self.assertRaises(ReviewError):
                r.review(request(d), folder=Path(d) / "rr")

    def test_manual(self):
        answers = iter(["s", "h", "x", "k", "", "good", "", "target_rpm", "2768", "short twice"])
        with tempfile.TemporaryDirectory() as d:
            out = ManualReviewer(ask=lambda _: next(answers), say=lambda *_: None).review(request(d))
        self.assertEqual([b["label"] for b in out["balls"]], ["scored", "short", "stuck", "unknown"])
        self.assertEqual(out["proposal"]["new_value"], 2768.0)


if __name__ == "__main__":
    unittest.main()
