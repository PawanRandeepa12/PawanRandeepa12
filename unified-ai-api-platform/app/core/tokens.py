"""Token estimation helpers.

Used when a provider does not return exact token usage (and by the mock
provider, cost pre-estimates and routing decisions). The heuristic of ~4
characters per token is a deliberately simple approximation — exact usage
reported by a provider always takes precedence.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

if TYPE_CHECKING:  # pragma: no cover
    from app.schemas.chat import Message


def estimate_tokens(text: str | None) -> int:
    if not text:
        return 0
    return max(1, (len(text) + 3) // 4)


def estimate_messages_tokens(messages: list["Message"]) -> int:
    total = 3  # chat framing overhead
    for message in messages:
        total += 3 + estimate_tokens(message.content)
    return total
