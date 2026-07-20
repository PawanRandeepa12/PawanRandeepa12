"""Admin & observability endpoints (Stage 4).

`/metrics` is exposed unauthenticated (Prometheus scrapes); everything under
`/v1/admin` requires the master key when one is configured.
"""

from __future__ import annotations

import time

from fastapi import APIRouter, Depends, Request
from fastapi.responses import PlainTextResponse

from app import __version__
from app.core.exceptions import ComponentNotEnabledError
from app.core.security import require_admin

# Prometheus scrape endpoint (unauthenticated by convention).
metrics_router = APIRouter(tags=["observability"])


@metrics_router.get("/metrics", include_in_schema=False)
async def prometheus_metrics(request: Request) -> PlainTextResponse:
    registry = request.app.state.metrics
    if registry is None:
        return PlainTextResponse("# metrics disabled\n")
    return PlainTextResponse(registry.render(), media_type="text/plain; version=0.0.4")


# Admin endpoints (master key when configured).
router = APIRouter(prefix="/v1/admin", tags=["admin"], dependencies=[Depends(require_admin)])


@router.get("/stats")
async def platform_stats(request: Request) -> dict:
    app = request.app
    tracker = app.state.cost_tracker
    queue = app.state.executor
    rate_limiter = app.state.rate_limiter
    cache = app.state.cache
    metrics = app.state.metrics
    return {
        "version": __version__,
        "environment": app.state.settings.environment,
        "uptime_seconds": round(time.time() - app.state.started_at, 2),
        "usage": {
            "requests_tracked": tracker.count(),
            "total_spend_usd": round(tracker.total_spend(), 8),
        },
        "cache": cache.stats() if cache is not None else {"backend": "disabled"},
        "queue": queue.snapshot() if queue is not None else None,
        "rate_limiter": rate_limiter.snapshot() if rate_limiter is not None else None,
        "providers": {
            name: {"mode": health["mode"], "circuit": health["circuit"], "available": health["available"]}
            for name, health in app.state.health_monitor.snapshot().items()
        },
        "counters": metrics.snapshot() if metrics is not None else {},
    }


@router.get("/cache/stats")
async def cache_stats(request: Request) -> dict:
    cache = request.app.state.cache
    if cache is None:
        raise ComponentNotEnabledError("Cache is disabled in this deployment.")
    return cache.stats()


@router.post("/cache/clear")
async def cache_clear(request: Request) -> dict:
    cache = request.app.state.cache
    if cache is None:
        raise ComponentNotEnabledError("Cache is disabled in this deployment.")
    removed = await cache.clear()
    return {"cleared_entries": removed}


@router.get("/queue")
async def queue_stats(request: Request) -> dict:
    return request.app.state.executor.snapshot()


@router.get("/config")
async def redacted_config(request: Request) -> dict:
    return request.app.state.settings.redacted()
