"""Chat gateway — orchestrates the request lifecycle.

Stage 1: explicit / first-available model selection + provider call.
Stage 2 (this version): adds pre-call global budget enforcement and post-call
cost computation, usage recording and budget threshold alerts.
Stage 3 will replace `_select_model` with strategy-driven routing & failover.
Stage 4 will add caching, priority queueing and metrics.
"""

from __future__ import annotations

import time
import uuid

from app.config import Settings
from app.core.exceptions import BudgetExceededError, ModelNotFoundError, NoAvailableProviderError
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
from app.services.cost.budgets import BudgetManager
from app.services.cost.pricing import PricingDatabase
from app.services.cost.tracker import CostTracker


class ChatGateway:
    def __init__(
        self,
        settings: Settings,
        registry: ProviderRegistry,
        *,
        pricing: PricingDatabase | None = None,
        tracker: CostTracker | None = None,
        budgets: BudgetManager | None = None,
    ) -> None:
        self.settings = settings
        self.registry = registry
        self.pricing = pricing
        self.tracker = tracker
        self.budgets = budgets

    # ------------------------------------------------------------- public API
    async def complete(self, request: ChatCompletionRequest, *, key_id: str = "anonymous") -> ChatCompletionResponse:
        adapter, model = self._select_model(request)
        self._enforce_budgets()

        started = time.perf_counter()
        result = await adapter.complete(request, model)
        latency_ms = round((time.perf_counter() - started) * 1000, 2)

        response = ChatCompletionResponse(
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
        self._track_cost(request, key_id, response, result.prompt_tokens, result.completion_tokens, result.mock)
        return response

    # ------------------------------------------------------- Stage 2 internals
    def _enforce_budgets(self) -> None:
        if self.budgets is None or not self.settings.cost_tracking_enabled:
            return
        blocking = self.budgets.blocking_budget()
        if blocking is not None:
            spent = self.budgets.spend_for(blocking)
            raise BudgetExceededError(
                f"Global budget '{blocking.name}' is exhausted "
                f"(${spent:.6f} of ${blocking.limit_usd:.6f} {blocking.period}). New requests are blocked.",
                details={"budget_id": blocking.id, "spent_usd": spent, "limit_usd": blocking.limit_usd},
            )

    def _track_cost(
        self,
        request: ChatCompletionRequest,
        key_id: str,
        response: ChatCompletionResponse,
        prompt_tokens: int,
        completion_tokens: int,
        mock: bool,
    ) -> None:
        cost_usd = 0.0
        if self.pricing is not None:
            _, model = self.registry.resolve(response.model)[0]
            cost = self.pricing.estimate(model.id, prompt_tokens, completion_tokens, catalog_fallback=model)
            response.cost = cost
            cost_usd = cost.total_cost_usd
        if self.tracker is not None and self.settings.cost_tracking_enabled:
            self.tracker.record(
                api_key_id=key_id,
                provider=response.provider,
                model=response.model,
                strategy=response.routing.strategy if response.routing else "explicit",
                prompt_tokens=prompt_tokens,
                completion_tokens=completion_tokens,
                cost_usd=cost_usd,
                latency_ms=response.latency_ms,
                mock=mock,
            )
            if self.budgets is not None:
                self.budgets.check_and_alert()

    # ----------------------------------------------------------------- helpers
    def _select_model(self, request: ChatCompletionRequest) -> tuple[ProviderAdapter, ModelInfo]:
        ref = request.model if request.model != "auto" else self.settings.default_model

        if ref == "auto":
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
