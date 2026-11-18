"""OpenAI Responses API reviewer (image input + Structured Outputs with a strict JSON schema).

Per the official docs: images go in the user message as {"type": "input_image", "image_url":
"data:image/jpeg;base64,..."}; the schema goes in text={"format": {"type": "json_schema", "name",
"schema", "strict": True}}. A refusal or incomplete answer is treated as an unreadable review
(every ball unknown), never as a result.
"""

import json
import os

from ..env import get_model
from .base import ASSESSMENT_SCHEMA, SYSTEM_PROMPT, ReviewError, Reviewer, build_prompt


class OpenAIReviewer(Reviewer):
    name = "openai"

    def __init__(self, client=None, model=None, detail="high", max_output_tokens=4000):
        self.model = model or get_model("OPENAI_MODEL")
        self.detail = detail
        self.max_output_tokens = max_output_tokens
        if client is None:
            if not os.environ.get("OPENAI_API_KEY"):
                raise ReviewError("OPENAI_API_KEY is not set (add it to tools/camera_launch_tuning/.env)")
            from openai import OpenAI  # noqa: PLC0415
            client = OpenAI()  # reads OPENAI_API_KEY from the environment
        self.client = client

    def build_input(self, req):
        content = [{"type": "input_text", "text": build_prompt(req)}]
        for i, f in enumerate(req.frames):
            content.append({"type": "input_text", "text": f"Frame {i + 1}: t = {f['t_from_gate_open_ms']:+.0f} ms"})
            content.append({"type": "input_image", "image_url": f"data:image/jpeg;base64,{req.image_b64(f)}",
                            "detail": self.detail})
        return [{"role": "system", "content": SYSTEM_PROMPT}, {"role": "user", "content": content}]

    def review(self, req, folder=None):
        response = self.client.responses.create(
            model=self.model,
            input=self.build_input(req),
            max_output_tokens=self.max_output_tokens,
            text={"format": {"type": "json_schema", "name": "launch_trial_review",
                             "schema": ASSESSMENT_SCHEMA, "strict": True}},
        )
        if getattr(response, "status", None) == "incomplete":
            raise ReviewError(f"incomplete response: {getattr(response, 'incomplete_details', None)}")
        for item in getattr(response, "output", []) or []:
            if getattr(item, "type", None) != "message":
                continue
            for c in getattr(item, "content", []) or []:
                if getattr(c, "type", None) == "refusal":
                    raise ReviewError(f"model refused: {getattr(c, 'refusal', '')[:200]}")
                if getattr(c, "type", None) == "output_text":
                    try:
                        return json.loads(c.text)
                    except json.JSONDecodeError as e:
                        raise ReviewError("response was not valid JSON") from e
        raise ReviewError("no response content")
