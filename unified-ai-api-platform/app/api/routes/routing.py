"""Routing introspection endpoints (Stage 3).

Dry-run the router (see the exact candidate order + exclusion reasons without
spending a cent), inspect provider health / circuit state, and review latency
observations.
"""

from __future__ import annotations

from fastapi import APIRouter, Depends, Request

from app.core.exceptions import NotFoundError
from app.core.security import get_api_key_context
from app.schemas.chat import ChatCompletionRequest
from app.services.routing.router import STRATEGIES

router = APIRouter(prefix="/v1/routing", tags=["routing"], dependencies=[Depends(get_api_key_context)])


@router.get("/strategies")
async def list_strategies(request: Request) -> dict:
    return {
        "default": request.app.state.settings.default_strategy,
        "strategies": [{"id": sid, "description": desc} for sid, desc in STRATEGIES.items()],
    }


@router.post("/route")
async def dry_run_route(request: Request, body: ChatCompletionRequest) -> dict:
    """Show the route plan for this request without executing it."""
    engine = request.app.state.router_engine
    plan = engine.plan(body)
    return {
        "strategy": plan.strategy,
        "prompt_tokens_est": plan.prompt_tokens_est,
        "completion_tokens_est": plan.completion_tokens_est,
        "candidates": [
            {
                "rank": rank,
                "model": c.model.id,
                "provider": c.model.provider,
                "reason": c.reason,
                "est_cost_usd": c.est_cost_usd,
                "est_latency_ms": c.est_latency_ms,
                "score": c.score,
            }
            for rank, c in enumerate(plan.candidates, start=1)
        ],
        "excluded": plan.excluded,
    }


@router.get("/health")
async def provider_health(request: Request) -> dict:
    return {"providers": request.app.state.health_monitor.snapshot()}


@router.post("/health/{provider}/reset")
async def reset_provider(provider: str, request: Request) -> dict:
    monitor = request.app.state.health_monitor
    if not monitor.reset(provider):
        raise NotFoundError(f"Unknown provider '{provider}'.")
    return {"provider": provider, "reset": True, "health": monitor.snapshot().get(provider)}


@router.get("/latency")
async def latency_stats(request: Request) -> dict:
    return {"models": request.app.state.latency.stats()}
