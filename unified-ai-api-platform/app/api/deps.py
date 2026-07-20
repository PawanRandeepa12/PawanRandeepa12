"""Shared FastAPI dependencies."""

from __future__ import annotations

from fastapi import Depends, Request

from app.config import Settings
from app.core.exceptions import ComponentNotEnabledError, RateLimitExceeded
from app.core.security import ApiKeyContext, get_api_key_context
from app.providers.registry import ProviderRegistry
from app.services.gateway import ChatGateway


def get_settings(request: Request) -> Settings:
    return request.app.state.settings


def get_registry(request: Request) -> ProviderRegistry:
    return request.app.state.registry


def get_gateway(request: Request) -> ChatGateway:
    return request.app.state.gateway


def require_component(name: str):
    """Dependency factory for components enabled in later stages."""

    def dependency(request: Request):
        component = getattr(request.app.state, name, None)
        if component is None:
            raise ComponentNotEnabledError(
                f"Component '{name}' is not enabled in this deployment."
            )
        return component

    return dependency


async def enforce_rate_limit(
    request: Request,
    ctx: ApiKeyContext = Depends(get_api_key_context),
) -> None:
    """Token-bucket rate limiting per API key (or per client IP when anonymous).

    Only an authenticated master key bypasses limits; the open dev-mode
    "anonymous" identity is still rate-limited like any other client.
    """
    limiter = getattr(request.app.state, "rate_limiter", None)
    if limiter is None or (ctx.is_master and ctx.authenticated):
        return
    identifier = ctx.key_id if ctx.authenticated else (request.client.host if request.client else "anonymous")
    allowed, retry_after = limiter.check(identifier)
    if not allowed:
        metrics = getattr(request.app.state, "metrics", None)
        if metrics is not None:
            metrics.rate_limited.inc(client=identifier)
        raise RateLimitExceeded(retry_after)
