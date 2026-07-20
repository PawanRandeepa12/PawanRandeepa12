"""Test fixtures.

Every test runs fully offline: MOCK_PROVIDERS=always makes providers return
deterministic mock completions, health checks are disabled, and usage data is
written to a per-test tmp directory.
"""

from __future__ import annotations

import pytest
from fastapi.testclient import TestClient

from app.config import Settings
from app.main import create_app


def build_settings(tmp_path, **overrides) -> Settings:
    base: dict = dict(
        environment="development",
        log_level="WARNING",
        mock_providers="always",
        health_check_enabled=False,
        rate_limit_enabled=False,
        cache_enabled=True,
        usage_store_path=tmp_path / "usage.jsonl",
        monthly_budget_usd=0.0,
    )
    base.update(overrides)
    return Settings(**base)


class TestEnv:
    """App + TestClient pair with lifespan management."""

    def __init__(self, settings: Settings) -> None:
        self.settings = settings
        self.app = create_app(settings)
        self.client = TestClient(self.app)

    def __enter__(self) -> TestEnv:
        self.client.__enter__()
        return self

    def __exit__(self, *exc) -> None:
        self.client.__exit__(*exc)


@pytest.fixture()
def make_env(tmp_path):
    def factory(**overrides) -> TestEnv:
        return TestEnv(build_settings(tmp_path, **overrides))

    return factory


@pytest.fixture()
def env(make_env):
    with make_env() as test_env:
        yield test_env


@pytest.fixture()
def client(env) -> TestClient:
    return env.client
