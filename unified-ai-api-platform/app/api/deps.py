"""Shared FastAPI dependencies."""

from __future__ import annotations

from fastapi import Request

from app.config import Settings
from app.core.exceptions import ComponentNotEnabledError
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
