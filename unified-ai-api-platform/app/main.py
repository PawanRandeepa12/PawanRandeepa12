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
from app.api.routes import admin, chat, cost, health, models, routing
from app.config import Settings, get_settings
from app.core.exceptions import register_exception_handlers
from app.core.logging import get_logger, setup_logging
from app.core.middleware import MetricsMiddleware, RequestIDMiddleware
from app.providers.registry import ProviderRegistry
from app.services.cost.analytics import CostAnalytics
from app.services.cost.budgets import BudgetManager
from app.services.cost.pricing import PricingDatabase
from app.services.cost.tracker import CostTracker
from app.services.gateway import ChatGateway
from app.services.performance.cache import build_cache
from app.services.performance.metrics import MetricsRegistry
from app.services.performance.queue import PriorityExecutor
from app.services.performance.rate_limit import RateLimiter
from app.services.routing.health import HealthMonitor
from app.services.routing.latency import LatencyTracker
from app.services.routing.router import RoutingEngine

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

        # --- Stage 2: cost optimization --------------------------------------
        pricing = PricingDatabase.load(settings.pricing_file)
        store_path = settings.usage_store_path if settings.cost_tracking_enabled else None
        tracker = CostTracker(store_path)
        budgets = BudgetManager(tracker, settings.parsed_alert_thresholds)
        if settings.monthly_budget_usd > 0:
            budgets.ensure_default_budget(settings.monthly_budget_usd)
        app.state.pricing = pricing
        app.state.cost_tracker = tracker
        app.state.cost_analytics = CostAnalytics(tracker)
        app.state.budgets = budgets

        # --- Stage 3: intelligent routing --------------------------------------
        latency_tracker = LatencyTracker()
        health_monitor = HealthMonitor(registry, settings)
        await health_monitor.start()
        router_engine = RoutingEngine(settings, registry, health_monitor, latency_tracker, pricing)
        app.state.latency = latency_tracker
        app.state.health_monitor = health_monitor
        app.state.router_engine = router_engine

        # --- Stage 4: speed & performance --------------------------------------
        cache = await build_cache(settings)
        rate_limiter = (
            RateLimiter(settings.rate_limit_requests_per_minute, settings.rate_limit_burst)
            if settings.rate_limit_enabled
            else None
        )
        executor = PriorityExecutor(settings.queue_max_concurrency)
        await executor.start()
        metrics = MetricsRegistry() if settings.metrics_enabled else None
        app.state.cache = cache
        app.state.rate_limiter = rate_limiter
        app.state.executor = executor
        app.state.metrics = metrics

        app.state.gateway = ChatGateway(
            settings,
            registry,
            pricing=pricing,
            tracker=tracker,
            budgets=budgets,
            router=router_engine,
            health=health_monitor,
            latency=latency_tracker,
            cache=cache,
            executor=executor,
            metrics=metrics,
        )

        logger.info(
            "platform started (env=%s, mock_providers=%s)",
            settings.environment,
            settings.mock_providers,
        )
        try:
            yield
        finally:
            await app.state.health_monitor.stop()
            await app.state.executor.stop()
            if hasattr(app.state.cache, "aclose"):
                await app.state.cache.aclose()
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
    app.add_middleware(MetricsMiddleware)

    app.include_router(health.router)
    app.include_router(models.router)
    app.include_router(chat.router)
    app.include_router(cost.router)
    app.include_router(routing.router)
    app.include_router(admin.router)
    app.include_router(admin.metrics_router)

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
                "chat": "POST /v1/chat/completions, POST /v1/chat/compare",
                "models": "GET /v1/models",
                "cost": "POST /v1/cost/estimate, POST /v1/cost/compare, GET /v1/cost/summary, GET /v1/cost/budgets",
                "routing": "POST /v1/routing/route, GET /v1/routing/health, GET /v1/routing/latency",
                "admin": "GET /v1/admin/stats, GET /v1/admin/cache/stats, GET /v1/admin/queue, GET /metrics",
                "health": "GET /health, GET /ready",
            },
        }

    return app


app = create_app()
