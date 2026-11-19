"""Trial reviewers: manual, OpenAI API, Anthropic API, and file-based agent review."""

from .base import ReviewRequest, Reviewer, ReviewError  # noqa: F401


def make_reviewer(name, **kwargs):
    if name == "manual":
        from .manual import ManualReviewer
        return ManualReviewer(**kwargs)
    if name == "openai":
        from .openai_provider import OpenAIReviewer
        return OpenAIReviewer(**kwargs)
    if name == "anthropic":
        from .anthropic_provider import AnthropicReviewer
        return AnthropicReviewer(**kwargs)
    if name == "claude-cli":
        from .claude_cli import ClaudeCliReviewer
        return ClaudeCliReviewer(**kwargs)
    if name == "agent-file":
        from .agent_file import AgentFileReviewer
        return AgentFileReviewer(**kwargs)
    raise ValueError(f"unknown reviewer {name!r} (manual, openai, anthropic, claude-cli, agent-file)")
