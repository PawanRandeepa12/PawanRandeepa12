"""Application entrypoint.

`create_app()` builds the FastAPI app and wires components in the lifespan.
Components for later stages are declared as `app.state` placeholders from day
one (`None` until their stage enables them) so every layer can already depend
on them uniformly.
"""

from __future__ import annotations

import time
from contextlib import asynccontextmanager

from fastapi import FastAPI

from app import __version__
from app.api.routes import chat, health, models
from app.config import Settings, get_settings
from app.core.exceptions import register_exception_handlers
from app.core.logging import get_logger, setup_logging
from app.core.middleware import RequestIDMiddleware
from app.providers.registry import ProviderRegistry
from app.services.gateway import ChatGateway

logger = get_logger(__name__)


def create_app(settings: Settings | None = None) -> FastAPI:
    settings = settings or get_settings()
    setup_logging(settings.log_level)

    @asynccontextmanager
    async def lifespan(app: FastAPI):
        # --- Stage 1: foundation -------------------------------------------
        app.state.started_at = time.time()
        registry = ProviderRegistry(settings)
        app.state.registry = registry
        app.state.gateway = ChatGateway(settings, registry)

        # --- Later stages (wired in by Stages 2-4) ---------------------------
        app.state.pricing = None          # Stage 2: PricingDatabase
        app.state.cost_tracker = None     # Stage 2: CostTracker
        app.state.budgets = None          # Stage 2: BudgetManager
        app.state.latency = None          # Stage 3: LatencyTracker
        app.state.health_monitor = None   # Stage 3: HealthMonitor
        app.state.router_engine = None    # Stage 3: RoutingEngine
        app.state.cache = None            # Stage 4: ResponseCache
        app.state.rate_limiter = None     # Stage 4: RateLimiter
        app.state.executor = None         # Stage 4: PriorityExecutor
        app.state.metrics = None          # Stage 4: MetricsRegistry

        logger.info(
            "platform started (env=%s, mock_providers=%s)",
            settings.environment,
            settings.mock_providers,
        )
        try:
            yield
        finally:
            await registry.aclose()
            logger.info("platform stopped")

    app = FastAPI(
        title=settings.app_name,
        version=__version__,
        description=(
            "One OpenAI-style API for OpenAI, Anthropic Claude and Google Gemini. "
            "Stage 1: foundation. Stage 2: cost optimization. Stage 3: intelligent "
            "routing. Stage 4: speed & performance."
        ),
        lifespan=lifespan,
    )
    app.state.settings = settings

    register_exception_handlers(app)
    app.add_middleware(RequestIDMiddleware)

    app.include_router(health.router)
    app.include_router(models.router)
    app.include_router(chat.router)

    @app.get("/", include_in_schema=False)
    async def root() -> dict:
        return {
            "name": settings.app_name,
            "version": __version__,
            "docs": "/docs",
            "stages": {
                "1_foundation": True,
                "2_cost_optimization": getattr(app.state, "cost_tracker", None) is not None,
                "3_intelligent_routing": getattr(app.state, "router_engine", None) is not None,
                "4_performance": getattr(app.state, "cache", None) is not None
                or getattr(app.state, "executor", None) is not None,
            },
            "endpoints": {
                "chat": "POST /v1/chat/completions",
                "models": "GET /v1/models",
                "health": "GET /health, GET /ready",
            },
        }

    return app


app = create_app()
