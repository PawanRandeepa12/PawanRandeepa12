"""Shared API schemas."""

from __future__ import annotations

from pydantic import BaseModel


class HealthResponse(BaseModel):
    status: str
    version: str
    environment: str
    uptime_seconds: float


class ProviderReadiness(BaseModel):
    mode: str        # mock | live | unconfigured
    configured: bool
    models: int


class ReadinessResponse(BaseModel):
    status: str      # ready | not_ready
    providers: dict[str, ProviderReadiness]
