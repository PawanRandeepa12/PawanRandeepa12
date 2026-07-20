"""Chat gateway — orchestrates the request lifecycle.

Stage 1: explicit / first-available model selection + provider call.
Stage 2: pre-call global budget enforcement + post-call cost tracking/alerts.
Stage 3 (this version): the routing engine picks ordered candidates per
strategy (cost / latency / balanced / explicit); the gateway walks the list
and fails over on provider errors while reporting outcomes to the health
monitor and latency tracker.
Stage 4 will add caching, priority queueing and metrics.
"""

from __future__ import annotations

import time
import uuid

from app.config import Settings
from app.core.exceptions import (
    BudgetExceededError,
    NoAvailableProviderError,
    ProviderError,
)
from app.core.logging import get_logger
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
from app.services.routing.health import HealthMonitor
from app.services.routing.latency import LatencyTracker
from app.services.routing.router import RoutingEngine

logger = get_logger(__name__)


class ChatGateway:
    def __init__(
        self,
        settings: Settings,
        registry: ProviderRegistry,
        *,
        pricing: PricingDatabase | None = None,
        tracker: CostTracker | None = None,
        budgets: BudgetManager | None = None,
        router: RoutingEngine | None = None,
        health: HealthMonitor | None = None,
        latency: LatencyTracker | None = None,
    ) -> None:
        self.settings = settings
        self.registry = registry
        self.pricing = pricing
        self.tracker = tracker
        self.budgets = budgets
        self.router = router
        self.health = health
        self.latency = latency

    # ------------------------------------------------------------- public API
    async def complete(self, request: ChatCompletionRequest, *, key_id: str = "anonymous") -> ChatCompletionResponse:
        if self.router is None:  # pragma: no cover - router is always wired since Stage 3
            raise NoAvailableProviderError("Routing engine is not configured.")

        plan = self.router.plan(request)
        if not plan.candidates:
            raise NoAvailableProviderError(
                "No provider can serve this request right now.",
                details={"strategy": plan.strategy, "excluded": plan.excluded},
            )
        self._enforce_budgets()

        max_fallbacks = (
            request.max_fallbacks if request.max_fallbacks is not None else self.settings.max_fallbacks
        )
        max_attempts = 1 + max_fallbacks

        attempted: list[str] = []
        last_error: ProviderError | None = None
        result = None
        chosen_index = 0
        started = time.perf_counter()

        for index, candidate in enumerate(plan.candidates[:max_attempts]):
            attempted.append(candidate.model.id)
            try:
                call_started = time.perf_counter()
                result = await candidate.adapter.complete(request, candidate.model)
                call_latency_ms = round((time.perf_counter() - call_started) * 1000, 2)
                if self.latency is not None:
                    self.latency.record(candidate.model.id, call_latency_ms)
                if self.health is not None:
                    self.health.record_success(candidate.adapter.name)
                chosen_index = index
                break
            except ProviderError as exc:
                last_error = exc
                if self.health is not None:
                    self.health.record_failure(candidate.adapter.name, exc.message)
                logger.warning(
                    "route attempt %d/%d failed on %s (%s, retryable=%s): %s",
                    index + 1,
                    max_attempts,
                    candidate.model.id,
                    exc.error_type,
                    exc.retryable,
                    exc.message,
                )

        if result is None:
            assert last_error is not None
            raise last_error

        latency_ms = round((time.perf_counter() - started) * 1000, 2)
        chosen = plan.candidates[chosen_index]
        response = ChatCompletionResponse(
            id=f"chatcmpl-{uuid.uuid4().hex[:24]}",
            created=int(time.time()),
            model=chosen.model.id,
            provider=chosen.adapter.name,
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
                strategy=plan.strategy,
                candidates=[c.model.id for c in plan.candidates],
                fallbacks_used=len(attempted) - 1,
                mock=result.mock,
                reason=chosen.reason,
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
        request: ChatCompletionRequest,  # noqa: ARG002 - reserved for per-request budget scopes
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
