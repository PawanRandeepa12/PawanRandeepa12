"""Chat gateway — orchestrates the request lifecycle.

Stage 1 (this version): explicit / first-available model selection and the
provider call. Later stages extend `complete()` in place:
  Stage 2 adds cost tracking + budget enforcement,
  Stage 3 adds strategy-based routing, health-aware failover and latency,
  Stage 4 adds caching, priority queueing and metrics.
"""

from __future__ import annotations

import time
import uuid

from app.config import Settings
from app.core.exceptions import ModelNotFoundError, NoAvailableProviderError
from app.providers.base import ModelInfo, ProviderAdapter
from app.providers.registry import ProviderRegistry
from app.schemas.chat import (
    ChatCompletionRequest,
    ChatCompletionResponse,
    Choice,
    Message,
    RoutingInfo,
    Usage,
)


class ChatGateway:
    def __init__(self, settings: Settings, registry: ProviderRegistry) -> None:
        self.settings = settings
        self.registry = registry

    # ------------------------------------------------------------- public API
    async def complete(self, request: ChatCompletionRequest, *, key_id: str = "anonymous") -> ChatCompletionResponse:  # noqa: ARG002
        adapter, model = self._select_model(request)

        started = time.perf_counter()
        result = await adapter.complete(request, model)
        latency_ms = round((time.perf_counter() - started) * 1000, 2)

        return ChatCompletionResponse(
            id=f"chatcmpl-{uuid.uuid4().hex[:24]}",
            created=int(time.time()),
            model=model.id,
            provider=adapter.name,
            choices=[
                Choice(
                    index=0,
                    message=Message(role="assistant", content=result.content),
                    finish_reason=result.finish_reason,
                )
            ],
            usage=Usage(
                prompt_tokens=result.prompt_tokens,
                completion_tokens=result.completion_tokens,
                total_tokens=result.total_tokens,
            ),
            latency_ms=latency_ms,
            routing=RoutingInfo(
                strategy="explicit",
                candidates=[model.id],
                mock=result.mock,
                reason="Direct provider selection (intelligent routing arrives in Stage 3).",
            ),
        )

    # ----------------------------------------------------------------- helpers
    def _select_model(self, request: ChatCompletionRequest) -> tuple[ProviderAdapter, ModelInfo]:
        ref = request.model if request.model != "auto" else self.settings.default_model

        if ref == "auto":
            # Stage 1 fallback: first model whose provider is usable. The Stage 3
            # router replaces this with strategy-driven selection.
            for adapter, model in self.registry.all_models():
                if adapter.mode() != "unconfigured":
                    return adapter, model
            raise NoAvailableProviderError(
                "No provider is available. Configure an API key or enable mock mode (MOCK_PROVIDERS=auto/always)."
            )

        matches = self.registry.resolve(ref)
        if not matches:
            raise ModelNotFoundError(f"Unknown model '{ref}'. GET /v1/models lists the catalog.")
        for adapter, model in matches:
            if adapter.mode() != "unconfigured":
                return adapter, model
        raise NoAvailableProviderError(f"Model '{ref}' is unavailable: its provider is not configured.")
