"""Routing engine (Stage 3).

`RoutingEngine.plan()` turns a chat request into an ordered list of route
candidates plus a transparent exclusion list. Strategies:

* **cost**     — cheapest healthy model that satisfies required capabilities
* **latency**  — healthy model with the lowest observed average latency
                 (unobserved models get a neutral penalty rank)
* **balanced** — score = 0.6 * normalized estimated cost + 0.4 * normalized
                 observed latency; equal-score groups are round-robin rotated
                 for load balancing
* **explicit** — the exact model requested (`model != "auto"`)

The gateway walks the candidate list and fails over to the next one when a
provider call raises a `ProviderError`.
"""

from __future__ import annotations

import itertools
from dataclasses import dataclass, field

from app.config import Settings
from app.core.exceptions import ModelNotFoundError
from app.core.tokens import estimate_messages_tokens
from app.providers.base import ModelInfo, ProviderAdapter
from app.providers.registry import ProviderRegistry
from app.schemas.chat import ChatCompletionRequest
from app.services.cost.pricing import PricingDatabase
from app.services.routing.health import HealthMonitor
from app.services.routing.latency import LatencyTracker

STRATEGIES: dict[str, str] = {
    "cost": "Pick the cheapest healthy model that satisfies the request's capabilities.",
    "latency": "Pick the healthy model with the lowest observed average latency.",
    "balanced": "Optimize a 60/40 cost/latency score with round-robin load balancing on ties.",
    "explicit": "Use the exact model requested (failover order still applies to equivalents).",
}


@dataclass
class RouteCandidate:
    adapter: ProviderAdapter
    model: ModelInfo
    reason: str = ""
    est_cost_usd: float | None = None
    est_latency_ms: float | None = None
    score: float | None = None


@dataclass
class RoutePlan:
    strategy: str
    candidates: list[RouteCandidate] = field(default_factory=list)
    excluded: list[dict] = field(default_factory=list)
    prompt_tokens_est: int = 0
    completion_tokens_est: int = 0


class RoutingEngine:
    def __init__(
        self,
        settings: Settings,
        registry: ProviderRegistry,
        health: HealthMonitor,
        latency: LatencyTracker,
        pricing: PricingDatabase,
    ) -> None:
        self.settings = settings
        self.registry = registry
        self.health = health
        self.latency = latency
        self.pricing = pricing
        self._rotation = itertools.count()

    # --------------------------------------------------------------------- plan
    def plan(self, request: ChatCompletionRequest) -> RoutePlan:
        prompt_est = estimate_messages_tokens(request.messages)
        completion_est = request.max_tokens or self.settings.default_completion_tokens_estimate

        excluded: list[dict] = []
        if request.model != "auto":
            strategy = "explicit"
            pool = self.registry.resolve(request.model)
            if not pool:
                raise ModelNotFoundError(
                    f"Unknown model '{request.model}'. GET /v1/models lists the catalog."
                )
        else:
            strategy = request.strategy if request.strategy != "auto" else self.settings.default_strategy
            pool = self.registry.all_models()

        available: list[tuple[ProviderAdapter, ModelInfo]] = []
        for adapter, model in pool:
            missing = [
                cap
                for cap in request.required_capabilities
                if (cap == "vision" and not model.supports_vision) or (cap == "tools" and not model.supports_tools)
            ]
            if missing:
                excluded.append({"model": model.id, "reason": f"missing capabilities: {', '.join(missing)}"})
            elif adapter.mode() == "unconfigured":
                excluded.append({"model": model.id, "reason": "provider not configured"})
            elif not self.health.is_available(adapter.name):
                excluded.append({"model": model.id, "reason": "provider unhealthy (circuit breaker open)"})
            else:
                available.append((adapter, model))

        candidates = self._order(strategy, available, prompt_est, completion_est)
        return RoutePlan(
            strategy=strategy,
            candidates=candidates,
            excluded=excluded,
            prompt_tokens_est=prompt_est,
            completion_tokens_est=completion_est,
        )

    # -------------------------------------------------------------------- order
    def _order(
        self,
        strategy: str,
        pairs: list[tuple[ProviderAdapter, ModelInfo]],
        prompt_est: int,
        completion_est: int,
    ) -> list[RouteCandidate]:
        candidates = [
            RouteCandidate(
                adapter=adapter,
                model=model,
                est_cost_usd=self.pricing.estimate(
                    model.id, prompt_est, completion_est, catalog_fallback=model
                ).total_cost_usd,
                est_latency_ms=self.latency.average(model.id),
            )
            for adapter, model in pairs
        ]

        if strategy == "cost":
            candidates.sort(key=lambda c: (c.est_cost_usd if c.est_cost_usd is not None else float("inf"), c.model.id))
            for c in candidates:
                c.reason = f"estimated cost ${c.est_cost_usd:.8f} for this request"
        elif strategy == "latency":
            known = [c.est_latency_ms for c in candidates if c.est_latency_ms is not None]
            penalty = (max(known) * 1.5) if known else 1000.0
            candidates.sort(
                key=lambda c: (c.est_latency_ms if c.est_latency_ms is not None else penalty, c.model.id)
            )
            for c in candidates:
                observed = f"{c.est_latency_ms:.1f}ms observed" if c.est_latency_ms is not None else "no latency data yet"
                c.reason = f"latency rank ({observed})"
        elif strategy == "balanced" and candidates:
            max_cost = max((c.est_cost_usd or 0.0) for c in candidates) or 1e-9
            known_lat = [c.est_latency_ms for c in candidates if c.est_latency_ms is not None]
            max_lat = max(known_lat) if known_lat else 1000.0
            for c in candidates:
                norm_cost = (c.est_cost_usd or 0.0) / max_cost
                norm_lat = (c.est_latency_ms if c.est_latency_ms is not None else max_lat) / max_lat
                c.score = round(0.6 * norm_cost + 0.4 * norm_lat, 6)
                c.reason = f"balanced score {c.score:.4f} (cost {norm_cost:.3f}, latency {norm_lat:.3f})"
            candidates.sort(key=lambda c: (c.score if c.score is not None else float("inf"), c.model.id))
        else:
            for c in candidates:
                c.reason = "explicit model requested"

        # Load balancing: round-robin rotation within equal-rank groups so
        # identical requests spread across equivalent candidates.
        if strategy == "cost":
            candidates = self._rotate_ties(candidates, key=lambda c: c.est_cost_usd)
        elif strategy == "balanced":
            candidates = self._rotate_ties(candidates, key=lambda c: c.score)
        return candidates

    def _rotate_ties(self, candidates: list[RouteCandidate], key) -> list[RouteCandidate]:
        rotation = next(self._rotation)
        result: list[RouteCandidate] = []
        i = 0
        while i < len(candidates):
            j = i
            while j < len(candidates) and key(candidates[j]) == key(candidates[i]):
                j += 1
            group = candidates[i:j]
            if len(group) > 1:
                shift = rotation % len(group)
                group = group[shift:] + group[:shift]
            result.extend(group)
            i = j
        return result
