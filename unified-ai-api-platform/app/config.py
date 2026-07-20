"""Central configuration management (Stage 1 deliverable).

Everything is driven by environment variables (optionally via a local `.env`
file) and validated with `pydantic-settings`. Later stages read their own
section of these settings; defining the whole surface here keeps configuration
a single source of truth for the platform.
"""

from __future__ import annotations

from functools import lru_cache
from pathlib import Path
from typing import Literal

from pydantic_settings import BaseSettings, SettingsConfigDict

BASE_DIR = Path(__file__).resolve().parent.parent


class Settings(BaseSettings):
    """Runtime configuration for the platform."""

    model_config = SettingsConfigDict(
        env_file=".env",
        env_file_encoding="utf-8",
        extra="ignore",
        case_sensitive=False,
    )

    # ------------------------------------------------------------------ core
    app_name: str = "Unified AI API Platform"
    environment: Literal["development", "staging", "production"] = "development"
    log_level: str = "INFO"
    host: str = "0.0.0.0"
    port: int = 8000

    # -------------------------------------------------------------- providers
    openai_api_key: str | None = None
    anthropic_api_key: str | None = None
    google_api_key: str | None = None
    openai_base_url: str = "https://api.openai.com/v1"
    anthropic_base_url: str = "https://api.anthropic.com/v1"
    google_base_url: str = "https://generativelanguage.googleapis.com/v1beta"
    anthropic_version: str = "2023-06-01"
    provider_timeout_seconds: float = 30.0
    # auto | always | never — when to serve deterministic mock completions.
    mock_providers: Literal["auto", "always", "never"] = "auto"
    default_model: str = "auto"

    # --------------------------------------------------------------- security
    # Comma-separated API keys accepted by this platform (empty = auth off).
    api_keys: str = ""
    master_api_key: str | None = None

    # ---------------------------------------------- Stage 2: cost optimization
    pricing_file: Path = BASE_DIR / "config" / "pricing.json"
    cost_tracking_enabled: bool = True
    usage_store_path: Path = BASE_DIR / "data" / "usage.jsonl"
    monthly_budget_usd: float = 0.0  # 0 disables the default global budget
    budget_alert_thresholds: str = "0.5,0.8,1.0"
    default_completion_tokens_estimate: int = 512

    # ---------------------------------------------- Stage 3: intelligent routing
    default_strategy: Literal["cost", "latency", "balanced"] = "balanced"
    health_check_enabled: bool = True
    health_check_interval_seconds: float = 30.0
    circuit_breaker_threshold: int = 3
    circuit_breaker_cooldown_seconds: float = 60.0
    max_fallbacks: int = 2

    # ---------------------------------------------- Stage 4: speed & performance
    redis_url: str | None = None
    cache_enabled: bool = True
    cache_ttl_seconds: int = 3600
    cache_max_entries: int = 5000
    rate_limit_enabled: bool = True
    rate_limit_requests_per_minute: int = 120
    rate_limit_burst: int = 20
    queue_max_concurrency: int = 8
    metrics_enabled: bool = True

    # ------------------------------------------------------------ derived data
    @property
    def parsed_api_keys(self) -> list[str]:
        return [k.strip() for k in self.api_keys.split(",") if k.strip()]

    @property
    def auth_enabled(self) -> bool:
        return bool(self.parsed_api_keys or self.master_api_key)

    @property
    def parsed_alert_thresholds(self) -> list[float]:
        values: list[float] = []
        for part in self.budget_alert_thresholds.split(","):
            part = part.strip()
            if part:
                values.append(float(part))
        return sorted(set(values))

    def redacted(self) -> dict:
        """Config snapshot safe to expose via admin endpoints."""
        data = self.model_dump()
        for field in ("openai_api_key", "anthropic_api_key", "google_api_key", "master_api_key"):
            value = data.get(field)
            data[field] = f"****{value[-4:]}" if value else None
        keys = data.get("api_keys") or ""
        data["api_keys"] = f"{len(self.parsed_api_keys)} key(s) configured" if keys else ""
        data["usage_store_path"] = str(data["usage_store_path"])
        data["pricing_file"] = str(data["pricing_file"])
        return data


@lru_cache
def get_settings() -> Settings:
    return Settings()
