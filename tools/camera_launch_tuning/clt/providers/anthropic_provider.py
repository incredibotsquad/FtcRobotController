"""Anthropic Messages API reviewer (base64 image blocks + structured outputs).

Uses output_config={"format": {"type": "json_schema", "schema": ...}} so the first text block is
JSON matching the schema, with effort "high" (thinking is always on for claude-opus-5-5). Server-side
refusal fallback is enabled (beta "server-side-fallback-2026-07-01", fallbacks="default"); a final
stop_reason of "refusal" or "max_tokens" is treated as an unreadable review, never as a result.
"""

import json
import os

from ..env import get_model
from .base import ASSESSMENT_SCHEMA, SYSTEM_PROMPT, ReviewError, Reviewer, build_prompt, review_images

FALLBACK_BETA = "server-side-fallback-2026-07-01"


class AnthropicReviewer(Reviewer):
    name = "anthropic"
    needs_frames = True

    def __init__(self, client=None, model=None, effort="high", max_tokens=16000, fallbacks=True):
        self.model = model or get_model("ANTHROPIC_MODEL")
        self.effort = effort
        self.max_tokens = max_tokens
        self.fallbacks = fallbacks
        if client is None:
            if not os.environ.get("ANTHROPIC_API_KEY"):
                raise ReviewError("ANTHROPIC_API_KEY is not set (add it to tools/camera_launch_tuning/.env)")
            import anthropic  # noqa: PLC0415
            client = anthropic.Anthropic()  # reads ANTHROPIC_API_KEY from the environment
        self.client = client

    def build_content(self, req):
        content = [{"type": "text", "text": build_prompt(req)}]
        for i, f in enumerate(review_images(req)):
            content.append({"type": "text", "text": f"Image {i + 1}: {f['caption']}"})
            content.append({"type": "image", "source": {"type": "base64", "media_type": "image/jpeg",
                                                         "data": req.image_b64(f)}})
        return content

    def review(self, req, folder=None):
        kwargs = dict(
            model=self.model,
            max_tokens=self.max_tokens,
            system=SYSTEM_PROMPT,
            messages=[{"role": "user", "content": self.build_content(req)}],
            output_config={"effort": self.effort, "format": {"type": "json_schema", "schema": ASSESSMENT_SCHEMA}},
        )
        if self.fallbacks:
            response = self.client.beta.messages.create(betas=[FALLBACK_BETA], fallbacks="default", **kwargs)
        else:
            response = self.client.messages.create(**kwargs)
        stop = getattr(response, "stop_reason", None)
        if stop == "refusal":
            details = getattr(response, "stop_details", None)
            raise ReviewError(f"model refused ({getattr(details, 'category', None)})")
        if stop == "max_tokens":
            raise ReviewError("response truncated at max_tokens")
        text = next((b.text for b in response.content if getattr(b, "type", None) == "text"), None)
        if text is None:
            raise ReviewError("no text block in response")
        try:
            return json.loads(text)
        except json.JSONDecodeError as e:
            raise ReviewError("response was not valid JSON") from e
