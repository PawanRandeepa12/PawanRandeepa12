"""Liveness & readiness probes (Stage 1)."""

from __future__ import annotations

import time

from fastapi import APIRouter, Request
from fastapi.responses import JSONResponse

from app import __version__
from app.schemas.common import HealthResponse, ProviderReadiness, ReadinessResponse

router = APIRouter(tags=["health"])


@router.get("/health", response_model=HealthResponse)
async def health(request: Request) -> HealthResponse:
    settings = request.app.state.settings
    return HealthResponse(
        status="ok",
        version=__version__,
        environment=settings.environment,
        uptime_seconds=round(time.time() - request.app.state.started_at, 2),
    )


@router.get("/ready")
async def ready(request: Request):
    registry = request.app.state.registry
    providers = {
        name: ProviderReadiness(
            mode=adapter.mode(),
            configured=adapter.is_configured(),
            models=len(adapter.models()),
        )
        for name, adapter in registry.adapters.items()
    }
    is_ready = any(p.mode != "unconfigured" for p in providers.values())
    payload = ReadinessResponse(
        status="ready" if is_ready else "not_ready",
        providers=providers,
    )
    return JSONResponse(status_code=200 if is_ready else 503, content=payload.model_dump())
