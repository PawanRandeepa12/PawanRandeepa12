"""Stage 3: intelligent routing system."""

from __future__ import annotations

from app.core.exceptions import ProviderUnavailable
from app.providers.gemini_provider import GeminiProvider

MESSAGES = [{"role": "user", "content": "Explain what a circuit breaker is in one paragraph."}]


def _chat(client, **overrides):
    body = {"model": "auto", "messages": MESSAGES}
    body.update(overrides)
    return client.post("/v1/chat/completions", json=body)


# ------------------------------------------------------------------- strategies
def test_strategies_endpoint(client):
    resp = client.get("/v1/routing/strategies")
    assert resp.status_code == 200
    data = resp.json()
    assert data["default"] == "balanced"
    ids = {s["id"] for s in data["strategies"]}
    assert ids == {"cost", "latency", "balanced", "explicit"}


def test_cost_strategy_orders_by_estimated_cost(env):
    resp = env.client.post("/v1/routing/route", json={"model": "auto", "messages": MESSAGES, "strategy": "cost"})
    assert resp.status_code == 200
    data = resp.json()
    assert data["strategy"] == "cost"
    costs = [c["est_cost_usd"] for c in data["candidates"]]
    assert costs == sorted(costs)
    assert data["candidates"][0]["model"] == "gemini/gemini-1.5-flash"
    assert "$" in data["candidates"][0]["reason"]


def test_balanced_strategy_has_scores(env):
    resp = env.client.post("/v1/routing/route", json={"model": "auto", "messages": MESSAGES})
    data = resp.json()
    assert data["strategy"] == "balanced"
    scores = [c["score"] for c in data["candidates"]]
    assert all(s is not None for s in scores)
    assert scores == sorted(scores)


def test_latency_strategy_prefers_observed_fast_model(env):
    latency = env.app.state.latency
    for model in ("openai/gpt-4o-mini", "gemini/gemini-1.5-flash", "anthropic/claude-3-haiku-20240307"):
        latency.record(model, 900.0)
    latency.record("anthropic/claude-3-5-haiku-20241022", 45.0)

    resp = env.client.post("/v1/routing/route", json={"model": "auto", "messages": MESSAGES, "strategy": "latency"})
    data = resp.json()
    assert data["strategy"] == "latency"
    assert data["candidates"][0]["model"] == "anthropic/claude-3-5-haiku-20241022"


def test_explicit_model_dry_run(client):
    resp = client.post("/v1/routing/route", json={"model": "gpt-4o-mini", "messages": MESSAGES})
    data = resp.json()
    assert data["strategy"] == "explicit"
    assert [c["model"] for c in data["candidates"]] == ["openai/gpt-4o-mini"]

    resp = client.post("/v1/routing/route", json={"model": "nope-x", "messages": MESSAGES})
    assert resp.status_code == 404


# ------------------------------------------------------------------ capabilities
def test_capability_filter_excludes_and_blocks(env):
    resp = env.client.post(
        "/v1/routing/route",
        json={"model": "auto", "messages": MESSAGES, "strategy": "cost", "required_capabilities": ["vision"]},
    )
    data = resp.json()
    excluded_models = {e["model"] for e in data["excluded"]}
    assert "anthropic/claude-3-5-haiku-20241022" in excluded_models  # no vision
    assert "openai/gpt-3.5-turbo" in excluded_models  # no vision
    for candidate in data["candidates"]:
        assert candidate["model"] not in excluded_models

    # Explicit non-vision model + vision requirement => nothing can serve.
    resp = _chat(env.client, model="openai/gpt-3.5-turbo", required_capabilities=["vision"])
    assert resp.status_code == 503
    assert resp.json()["error"]["type"] == "no_available_provider"


# ---------------------------------------------------------------- chat + routing
def test_chat_reports_routing_metadata(env):
    resp = _chat(env.client, strategy="cost")
    assert resp.status_code == 200
    routing = resp.json()["routing"]
    assert routing["strategy"] == "cost"
    assert routing["fallbacks_used"] == 0
    assert routing["candidates"][0] == resp.json()["model"] == "gemini/gemini-1.5-flash"


def test_latency_is_recorded_after_chat(env):
    assert _chat(env.client, model="openai/gpt-4o-mini").status_code == 200
    stats = env.client.get("/v1/routing/latency").json()["models"]
    assert "openai/gpt-4o-mini" in stats
    assert stats["openai/gpt-4o-mini"]["samples"] == 1
    assert stats["openai/gpt-4o-mini"]["avg_ms"] >= 0


# ------------------------------------------------------------------ failover
def test_failover_chain_and_circuit_breaker(env, monkeypatch):
    client = env.client

    async def broken_complete(self, request, model):  # noqa: ARG001
        raise ProviderUnavailable("gemini", "simulated gemini outage", retryable=True)

    monkeypatch.setattr(GeminiProvider, "complete", broken_complete)

    # gemini owns the two cheapest candidates; both fail -> fail over to openai.
    # cache=False guarantees each call really executes the failover chain.
    resp = _chat(client, strategy="cost", max_fallbacks=5, cache=False)
    assert resp.status_code == 200
    data = resp.json()
    assert data["provider"] == "openai"
    assert data["model"] == "openai/gpt-4o-mini"
    assert data["routing"]["fallbacks_used"] == 2

    # Second call burns more failures -> circuit breaker opens (threshold = 3).
    resp = _chat(client, strategy="cost", max_fallbacks=5, cache=False)
    assert resp.status_code == 200

    health = client.get("/v1/routing/health").json()["providers"]
    assert health["gemini"]["circuit"] == "open"
    assert health["gemini"]["available"] is False
    assert health["gemini"]["consecutive_failures"] >= 3

    # Now the router excludes gemini up-front: no fallbacks needed at all.
    plan = client.post(
        "/v1/routing/route", json={"model": "auto", "messages": MESSAGES, "strategy": "cost"}
    ).json()
    assert not any(c["provider"] == "gemini" for c in plan["candidates"])
    assert any("circuit" in e["reason"] for e in plan["excluded"] if e["model"].startswith("gemini/"))

    resp = _chat(client, strategy="cost", cache=False)
    assert resp.status_code == 200
    assert resp.json()["routing"]["fallbacks_used"] == 0
    assert resp.json()["provider"] == "openai"

    # Operator reset closes the circuit; gemini becomes a candidate again.
    resp = client.post("/v1/routing/health/gemini/reset")
    assert resp.status_code == 200
    assert client.get("/v1/routing/health").json()["providers"]["gemini"]["circuit"] == "closed"


def test_circuit_breaker_half_open_probe(make_env):
    import time

    with make_env(circuit_breaker_cooldown_seconds=0.05) as env:
        monitor = env.app.state.health_monitor
        threshold = env.settings.circuit_breaker_threshold
        for _ in range(threshold):
            monitor.record_failure("openai", "boom")
        assert monitor.is_available("openai") is False
        time.sleep(0.07)
        assert monitor.is_available("openai") is True  # half-open probe allowed
        monitor.record_success("openai")  # probe succeeded -> circuit closed
        assert monitor.snapshot()["openai"]["circuit"] == "closed"


def test_unavailable_explicit_model_returns_503_or_provider_error(env, monkeypatch):
    async def broken(self, request, model):  # noqa: ARG001
        raise ProviderUnavailable("openai", "openai down", retryable=True)

    from app.providers.openai_provider import OpenAIProvider

    monkeypatch.setattr(OpenAIProvider, "complete", broken)
    resp = _chat(env.client, model="openai/gpt-4o-mini")
    assert resp.status_code == 503
    assert resp.json()["error"]["provider"] == "openai"
    assert resp.json()["error"]["retryable"] is True


# ------------------------------------------------------------------ health
def test_health_snapshot_all_healthy(env):
    health = env.client.get("/v1/routing/health").json()["providers"]
    for name in ("openai", "anthropic", "gemini"):
        assert health[name]["available"] is True
        assert health[name]["circuit"] == "closed"
        assert health[name]["mode"] == "mock"


def test_reset_unknown_provider_404(client):
    assert client.post("/v1/routing/health/nope/reset").status_code == 404
