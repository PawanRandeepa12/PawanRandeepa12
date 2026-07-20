"""Deterministic mock completions.

Mock mode lets you develop, demo and run the full test suite without provider
credentials or spend: the whole pipeline (auth -> routing -> cost tracking ->
caching -> metrics) executes for real, only the final provider call is
simulated. Enable with MOCK_PROVIDERS=always (or leave keys unset with the
default `auto`).
"""

from __future__ import annotations

from typing import TYPE_CHECKING

from app.core.tokens import estimate_messages_tokens, estimate_tokens

if TYPE_CHECKING:  # pragma: no cover
    from app.providers.base import ModelInfo, ProviderResult
    from app.schemas.chat import ChatCompletionRequest


def mock_completion(request: ChatCompletionRequest, model: ModelInfo) -> ProviderResult:
    from app.providers.base import ProviderResult

    last_user = next((m for m in reversed(request.messages) if m.role == "user"), request.messages[-1])
    preview = (last_user.content or "").strip().replace("\n", " ")[:160]
    content = (
        f"[MOCK {model.id}] Simulated response — no live {model.provider} API call was made.\n"
        f"You asked: \"{preview}\"\n"
        "Mock mode executes the full platform pipeline (auth, routing, cost tracking, caching, "
        "metrics) with deterministic output so it is safe for development, CI and demos. "
        "Set MOCK_PROVIDERS=never and add provider API keys to serve live completions."
    )
    prompt_tokens = estimate_messages_tokens(request.messages)
    completion_tokens = estimate_tokens(content)
    if request.max_tokens is not None:
        completion_tokens = min(completion_tokens, request.max_tokens)
    return ProviderResult(
        model_id=model.id,
        provider=model.provider,
        content=content,
        finish_reason="stop",
        prompt_tokens=prompt_tokens,
        completion_tokens=completion_tokens,
        mock=True,
    )
