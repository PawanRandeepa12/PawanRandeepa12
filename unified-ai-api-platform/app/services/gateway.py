"""Chat gateway — orchestrates the request lifecycle (all four stages).

Request flow:
    cache lookup (S4)  →  route plan via strategy (S3)  →  budget guard (S2)
    →  priority-queued provider call with failover (S3/S4)
    →  cost + usage tracking, budget alerts (S2)  →  cache store (S4)
    →  metrics throughout (S4)

Plus `compare()`: parallel fan-out of one prompt across providers with
side-by-side cost & latency (S4).
"""

from __future__ import annotations

import asyncio
import json
import time
import uuid

from app.config import Settings
from app.core.exceptions import (
    BudgetExceededError,
    ModelNotFoundError,
    NoAvailableProviderError,
    ProviderError,
)
from app.core.logging import get_logger
from app.providers.base import ModelInfo, ProviderAdapter
from app.providers.registry import ProviderRegistry
from app.schemas.chat import (
    ChatCompletionRequest,
    ChatCompletionResponse,
    Choice,
    CompareRequest,
    CompareResponse,
    CompareResultItem,
    Message,
    RoutingInfo,
    Usage,
)
from app.services.cost.budgets import BudgetManager
from app.services.cost.pricing import PricingDatabase
from app.services.cost.tracker import CostTracker
from app.services.performance.cache import CacheBackend, make_cache_key
from app.services.performance.metrics import MetricsRegistry
from app.services.performance.queue import PriorityExecutor
from app.services.routing.health import HealthMonitor
from app.services.routing.latency import LatencyTracker
from app.services.routing.router import RoutePlan, RoutingEngine

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
        cache: CacheBackend | None = None,
        executor: PriorityExecutor | None = None,
        metrics: MetricsRegistry | None = None,
    ) -> None:
        self.settings = settings
        self.registry = registry
        self.pricing = pricing
        self.tracker = tracker
        self.budgets = budgets
        self.router = router
        self.health = health
        self.latency = latency
        self.cache = cache
        self.executor = executor
        self.metrics = metrics

    # ------------------------------------------------------------- public API
    async def complete(self, request: ChatCompletionRequest, *, key_id: str = "anonymous") -> ChatCompletionResponse:
        started_total = time.perf_counter()

        # Stage 4: serve identical requests from cache (free: no provider call,
        # no budget consumption, no extra usage records).
        cache_key = None
        if self.cache is not None and request.cache:
            cache_key = make_cache_key("chat", self._cache_payload(request))
            cached_value = await self.cache.get(cache_key)
            if cached_value is not None:
                self._metric("cache_events", "inc", event="hit")
                response = ChatCompletionResponse(**json.loads(cached_value))
                response.cached = True
                response.latency_ms = round((time.perf_counter() - started_total) * 1000, 2)
                return response
            self._metric("cache_events", "inc", event="miss")

        if self.router is None:  # pragma: no cover - always wired
            raise NoAvailableProviderError("Routing engine is not configured.")
        plan = self.router.plan(request)
        if not plan.candidates:
            raise NoAvailableProviderError(
                "No provider can serve this request right now.",
                details={"strategy": plan.strategy, "excluded": plan.excluded},
            )
        self._enforce_budgets()

        response = await self._run_plan(plan, request)
        self._track_cost(key_id, response)

        if cache_key is not None:
            try:
                await self.cache.set(cache_key, response.model_dump_json(), self.settings.cache_ttl_seconds)
                self._metric("cache_events", "inc", event="store")
            except Exception:  # noqa: BLE001 - caching must never break a request
                logger.warning("cache store failed", exc_info=True)
        return response

    async def compare(self, body: CompareRequest, *, key_id: str = "anonymous") -> CompareResponse:
        """Run the same prompt against multiple models in parallel (Stage 4)."""
        pairs = self._resolve_compare_models(body)
        self._enforce_budgets()

        async def run_one(adapter: ProviderAdapter, model: ModelInfo) -> CompareResultItem:
            req = ChatCompletionRequest(
                model=model.id,
                messages=body.messages,
                temperature=body.temperature,
                max_tokens=body.max_tokens,
                cache=False,
            )
            started = time.perf_counter()
            try:
                result = await adapter.complete(req, model)
                latency_ms = round((time.perf_counter() - started) * 1000, 2)
                cost = (
                    self.pricing.estimate(model.id, result.prompt_tokens, result.completion_tokens, catalog_fallback=model)
                    if self.pricing is not None
                    else None
                )
                if self.latency is not None:
                    self.latency.record(model.id, latency_ms)
                if self.health is not None:
                    self.health.record_success(adapter.name)
                if self.metrics is not None:
                    self.metrics.provider_requests.inc(
                        provider=adapter.name, model=model.id, status="mock" if result.mock else "ok"
                    )
                if self.tracker is not None and self.settings.cost_tracking_enabled:
                    self.tracker.record(
                        api_key_id=key_id,
                        provider=adapter.name,
                        model=model.id,
                        strategy="compare",
                        prompt_tokens=result.prompt_tokens,
                        completion_tokens=result.completion_tokens,
                        cost_usd=cost.total_cost_usd if cost else 0.0,
                        latency_ms=latency_ms,
                        mock=result.mock,
                    )
                    if self.budgets is not None:
                        self.budgets.check_and_alert()
                return CompareResultItem(
                    model=model.id,
                    provider=adapter.name,
                    ok=True,
                    content=result.content,
                    latency_ms=latency_ms,
                    usage=Usage(
                        prompt_tokens=result.prompt_tokens,
                        completion_tokens=result.completion_tokens,
                        total_tokens=result.total_tokens,
                    ),
                    cost=cost,
                )
            except ProviderError as exc:
                latency_ms = round((time.perf_counter() - started) * 1000, 2)
                if self.health is not None:
                    self.health.record_failure(adapter.name, exc.message)
                if self.metrics is not None:
                    self.metrics.provider_errors.inc(provider=adapter.name, model=model.id, error_type=exc.error_type)
                return CompareResultItem(
                    model=model.id, provider=adapter.name, ok=False, latency_ms=latency_ms, error=exc.message
                )

        async def run_queued(adapter: ProviderAdapter, model: ModelInfo) -> CompareResultItem:
            if self.executor is not None:
                return await self.executor.submit(lambda: run_one(adapter, model), priority="high")
            return await run_one(adapter, model)

        results = list(await asyncio.gather(*(run_queued(adapter, model) for adapter, model in pairs)))
        ok_results = [r for r in results if r.ok]
        fastest = min(ok_results, key=lambda r: r.latency_ms, default=None)
        priced = [r for r in ok_results if r.cost is not None]
        cheapest = min(priced, key=lambda r: r.cost.total_cost_usd, default=None)
        return CompareResponse(
            results=results,
            fastest=fastest.model if fastest else None,
            cheapest=cheapest.model if cheapest else None,
        )

    # ---------------------------------------------------------- routing + call
    async def _run_plan(self, plan: RoutePlan, request: ChatCompletionRequest) -> ChatCompletionResponse:
        max_fallbacks = (
            request.max_fallbacks if request.max_fallbacks is not None else self.settings.max_fallbacks
        )
        max_attempts = 1 + max_fallbacks
        attempted: list[str] = []
        outcome: dict = {"result": None, "chosen_index": 0, "last_error": None}

        async def invoke() -> None:
            for index, candidate in enumerate(plan.candidates[:max_attempts]):
                if index > 0:
                    self._metric("fallbacks", "inc", strategy=plan.strategy)
                attempted.append(candidate.model.id)
                try:
                    call_started = time.perf_counter()
                    result = await candidate.adapter.complete(request, candidate.model)
                    call_latency_ms = round((time.perf_counter() - call_started) * 1000, 2)
                    if self.latency is not None:
                        self.latency.record(candidate.model.id, call_latency_ms)
                    if self.health is not None:
                        self.health.record_success(candidate.adapter.name)
                    if self.metrics is not None:
                        self.metrics.provider_requests.inc(
                            provider=candidate.adapter.name,
                            model=candidate.model.id,
                            status="mock" if result.mock else "ok",
                        )
                        self.metrics.provider_duration.observe(
                            call_latency_ms / 1000, provider=candidate.adapter.name, model=candidate.model.id
                        )
                        self.metrics.provider_tokens.inc(
                            result.prompt_tokens, provider=candidate.adapter.name, model=candidate.model.id, kind="prompt"
                        )
                        self.metrics.provider_tokens.inc(
                            result.completion_tokens,
                            provider=candidate.adapter.name,
                            model=candidate.model.id,
                            kind="completion",
                        )
                    outcome["result"] = result
                    outcome["chosen_index"] = index
                    return
                except ProviderError as exc:
                    outcome["last_error"] = exc
                    if self.health is not None:
                        self.health.record_failure(candidate.adapter.name, exc.message)
                    if self.metrics is not None:
                        self.metrics.provider_errors.inc(
                            provider=candidate.adapter.name, model=candidate.model.id, error_type=exc.error_type
                        )
                    logger.warning(
                        "route attempt %d/%d failed on %s (%s, retryable=%s): %s",
                        index + 1,
                        max_attempts,
                        candidate.model.id,
                        exc.error_type,
                        exc.retryable,
                        exc.message,
                    )
            raise outcome["last_error"]

        started = time.perf_counter()
        if self.executor is not None:
            await self.executor.submit(invoke, priority=request.priority)
        else:
            await invoke()
        latency_ms = round((time.perf_counter() - started) * 1000, 2)

        result = outcome["result"]
        chosen = plan.candidates[outcome["chosen_index"]]
        return ChatCompletionResponse(
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

    def _track_cost(self, key_id: str, response: ChatCompletionResponse) -> None:
        cost_usd = 0.0
        if self.pricing is not None:
            _, model = self.registry.resolve(response.model)[0]
            cost = self.pricing.estimate(
                model.id,
                response.usage.prompt_tokens,
                response.usage.completion_tokens,
                catalog_fallback=model,
            )
            response.cost = cost
            cost_usd = cost.total_cost_usd
            if self.metrics is not None and cost_usd > 0:
                self.metrics.provider_cost.inc(cost_usd, provider=response.provider, model=response.model)
        if self.tracker is not None and self.settings.cost_tracking_enabled:
            self.tracker.record(
                api_key_id=key_id,
                provider=response.provider,
                model=response.model,
                strategy=response.routing.strategy if response.routing else "explicit",
                prompt_tokens=response.usage.prompt_tokens,
                completion_tokens=response.usage.completion_tokens,
                cost_usd=cost_usd,
                latency_ms=response.latency_ms,
                mock=bool(response.routing and response.routing.mock),
            )
            if self.budgets is not None:
                for alert in self.budgets.check_and_alert():
                    self._metric("budget_alerts", "inc", budget=alert.budget_id)

    # ------------------------------------------------------------------ compare
    def _resolve_compare_models(self, body: CompareRequest) -> list[tuple[ProviderAdapter, ModelInfo]]:
        pairs: list[tuple[ProviderAdapter, ModelInfo]] = []
        if body.models:
            for ref in body.models:
                matches = self.registry.resolve(ref)
                if not matches:
                    raise ModelNotFoundError(f"Unknown model '{ref}'.")
                adapter, model = matches[0]
                if adapter.mode() == "unconfigured":
                    raise NoAvailableProviderError(f"Model '{ref}' is unavailable: provider not configured.")
                pairs.append((adapter, model))
        else:
            seen: set[str] = set()
            for adapter, model in self.registry.all_models():
                if model.tier != "flagship" or adapter.name in seen or adapter.mode() == "unconfigured":
                    continue
                seen.add(adapter.name)
                pairs.append((adapter, model))
        if not pairs:
            raise NoAvailableProviderError("No configured provider is available for comparison.")
        return pairs

    # ------------------------------------------------------------------ helpers
    @staticmethod
    def _cache_payload(request: ChatCompletionRequest) -> dict:
        return {
            "model": request.model,
            "messages": [m.model_dump() for m in request.messages],
            "temperature": request.temperature,
            "max_tokens": request.max_tokens,
            "strategy": request.strategy,
            "required_capabilities": sorted(request.required_capabilities),
        }

    def _metric(self, name: str, method: str, **labels) -> None:
        if self.metrics is not None:
            getattr(getattr(self.metrics, name), method)(**labels)
