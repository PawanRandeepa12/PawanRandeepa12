"""Stage 4: speed & performance."""

from __future__ import annotations

import asyncio

from app.services.performance.queue import PriorityExecutor

PROMPT = [{"role": "user", "content": "What is the capital of Sri Lanka?"}]


def _chat(client, prompt=None, **overrides):
    body = {"model": "openai/gpt-4o-mini", "messages": prompt or PROMPT}
    body.update(overrides)
    return client.post("/v1/chat/completions", json=body)


# --------------------------------------------------------------------- caching
def test_identical_request_is_served_from_cache(env):
    client = env.client
    first = _chat(client)
    assert first.status_code == 200
    assert first.json()["cached"] is False
    assert first.headers.get("x-cache") == "MISS"

    second = _chat(client)
    assert second.status_code == 200
    assert second.json()["cached"] is True
    assert second.headers.get("x-cache") == "HIT"
    assert second.json()["choices"][0]["message"]["content"] == first.json()["choices"][0]["message"]["content"]

    # Cached replies do not consume budget / do not add usage records.
    summary = client.get("/v1/cost/summary?days=1").json()
    assert summary["requests"] == 1

    # But a different prompt is a fresh miss.
    third = _chat(client, prompt=[{"role": "user", "content": "Something else entirely."}])
    assert third.json()["cached"] is False


def test_cache_can_be_bypassed_per_request(env):
    client = env.client
    assert _chat(client, cache=False).json()["cached"] is False
    assert _chat(client, cache=False).json()["cached"] is False


def test_cache_disabled_environment(make_env):
    with make_env(cache_enabled=False) as env:
        assert _chat(env.client).json()["cached"] is False
        assert "x-cache" not in {k.lower() for k in _chat(env.client).headers}


def test_admin_cache_endpoints(env):
    client = env.client
    _chat(client)
    _chat(client)  # one hit
    stats = client.get("/v1/admin/cache/stats").json()
    assert stats["backend"] == "memory"
    assert stats["entries"] == 1
    assert stats["hits"] >= 1 and stats["misses"] >= 1

    cleared = client.post("/v1/admin/cache/clear").json()
    assert cleared["cleared_entries"] == 1
    assert _chat(client).json()["cached"] is False


# ---------------------------------------------------------------- rate limiting
def test_rate_limiter_returns_429_with_retry_after(make_env):
    with make_env(rate_limit_enabled=True, rate_limit_requests_per_minute=60, rate_limit_burst=3) as env:
        client = env.client
        for _ in range(3):
            assert _chat(client).status_code == 200
        resp = _chat(client)
        assert resp.status_code == 429
        assert resp.json()["error"]["type"] == "rate_limit_exceeded"
        assert "retry-after" in {k.lower() for k in resp.headers}


# ---------------------------------------------------------------- priority queue
def test_priority_queue_orders_high_before_low():
    async def scenario():
        executor = PriorityExecutor(max_concurrency=1)
        order: list[str] = []

        async def make(name):
            order.append(name)

        # Enqueue before workers start: low is queued first, high must win.
        f_low = await executor.enqueue(lambda: make("low"), priority="low")
        f_high = await executor.enqueue(lambda: make("high"), priority="high")
        await executor.start()
        await asyncio.gather(f_low, f_high)
        await executor.stop()
        return order

    assert asyncio.run(scenario()) == ["high", "low"]


def test_priority_queue_bounds_concurrency():
    async def scenario():
        executor = PriorityExecutor(max_concurrency=2)
        await executor.start()
        running = 0
        peak = 0

        async def work():
            nonlocal running, peak
            running += 1
            peak = max(peak, running)
            await asyncio.sleep(0.01)
            running -= 1

        await asyncio.gather(*(executor.submit(work, priority="normal") for _ in range(8)))
        await executor.stop()
        return peak

    assert asyncio.run(scenario()) == 2


def test_queue_stats_endpoint(env):
    resp = env.client.get("/v1/admin/queue")
    assert resp.status_code == 200
    data = resp.json()
    assert data["workers"] == data["max_concurrency"] == env.settings.queue_max_concurrency


# -------------------------------------------- parallel comparison across models
def test_compare_endpoint_parallel_results(env):
    client = env.client
    resp = client.post(
        "/v1/chat/compare",
        json={
            "messages": PROMPT,
            "models": ["openai/gpt-4o-mini", "anthropic/claude-3-haiku-20240307", "gemini/gemini-1.5-flash"],
        },
    )
    assert resp.status_code == 200
    data = resp.json()
    assert len(data["results"]) == 3
    assert all(r["ok"] for r in data["results"])
    assert {r["provider"] for r in data["results"]} == {"openai", "anthropic", "gemini"}
    assert data["cheapest"] == "gemini/gemini-1.5-flash"
    assert data["fastest"] in {r["model"] for r in data["results"]}
    for item in data["results"]:
        assert item["cost"]["total_cost_usd"] > 0
        assert item["latency_ms"] >= 0

    # Comparison calls are real provider calls: they are tracked.
    summary = client.get("/v1/cost/summary?days=1").json()
    assert summary["requests"] == 3
    records = client.get("/v1/cost/records").json()
    assert {r["strategy"] for r in records} == {"compare"}


def test_compare_defaults_to_flagships(env):
    resp = env.client.post("/v1/chat/compare", json={"messages": PROMPT})
    data = resp.json()
    providers = {r["provider"] for r in data["results"]}
    assert providers == {"openai", "anthropic", "gemini"}


# --------------------------------------------------------------------- metrics
def test_prometheus_metrics_endpoint(env):
    _chat(env.client)
    resp = env.client.get("/metrics")
    assert resp.status_code == 200
    assert resp.headers["content-type"].startswith("text/plain")
    body = resp.text
    assert "uaip_provider_requests_total" in body
    assert "uaip_http_requests_total" in body
    assert 'provider="openai"' in body
    assert "uaip_provider_cost_usd_total" in body


def test_metrics_not_recorded_for_metrics_route(env):
    env.client.get("/metrics")
    counts = env.client.get("/metrics").text.count('path="/metrics"')
    assert counts == 0


def test_admin_stats_overview(env):
    _chat(env.client)
    stats = env.client.get("/v1/admin/stats").json()
    assert stats["usage"]["requests_tracked"] == 1
    assert stats["usage"]["total_spend_usd"] > 0
    assert stats["cache"]["backend"] == "memory"
    assert stats["queue"]["workers"] >= 1
    assert set(stats["providers"]) == {"openai", "anthropic", "gemini"}


def test_redacted_config_endpoint(env):
    config = env.client.get("/v1/admin/config").json()
    assert config["openai_api_key"] is None
    assert "rate_limit_requests_per_minute" in config
