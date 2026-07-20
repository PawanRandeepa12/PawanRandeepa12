"""Stage 2: cost optimization engine."""

from __future__ import annotations

import json

import pytest

from app.core.tokens import estimate_messages_tokens
from app.schemas.chat import Message

MESSAGES = [{"role": "user", "content": "Summarize the benefits of a unified AI gateway in two sentences."}]
MESSAGE_OBJS = [Message(role=m["role"], content=m["content"]) for m in MESSAGES]


def _chat(client, **overrides):
    body = {"model": "openai/gpt-4o-mini", "messages": MESSAGES}
    body.update(overrides)
    return client.post("/v1/chat/completions", json=body)


# ------------------------------------------------------------------- estimates
def test_estimate_with_exact_tokens(client):
    resp = client.post(
        "/v1/cost/estimate",
        json={"model": "openai/gpt-4o", "prompt_tokens": 1000, "completion_tokens": 1000},
    )
    assert resp.status_code == 200
    data = resp.json()
    assert data["tokens_estimated"] is False
    # 1000 in @ $0.0025/1K + 1000 out @ $0.01/1K
    assert data["cost"]["prompt_cost_usd"] == pytest.approx(0.0025)
    assert data["cost"]["completion_cost_usd"] == pytest.approx(0.0100)
    assert data["cost"]["total_cost_usd"] == pytest.approx(0.0125)
    assert data["cost"]["pricing_source"] == "pricing_file"


def test_estimate_from_messages_uses_token_heuristic(client):
    resp = client.post("/v1/cost/estimate", json={"model": "gemini/gemini-1.5-flash", "messages": MESSAGES})
    assert resp.status_code == 200
    data = resp.json()
    assert data["tokens_estimated"] is True
    assert data["prompt_tokens"] == estimate_messages_tokens(MESSAGE_OBJS)


def test_estimate_requires_tokens_or_messages(client):
    resp = client.post("/v1/cost/estimate", json={"model": "openai/gpt-4o"})
    assert resp.status_code == 400
    assert resp.json()["error"]["type"] == "bad_request"


def test_estimate_unknown_model_404(client):
    resp = client.post("/v1/cost/estimate", json={"model": "nope", "prompt_tokens": 5})
    assert resp.status_code == 404


# -------------------------------------------------------------------- compare
def test_compare_is_sorted_and_finds_cheapest(client):
    resp = client.post("/v1/cost/compare", json={"messages": MESSAGES})
    assert resp.status_code == 200
    data = resp.json()
    costs = [e["cost_usd"] for e in data["estimates"]]
    assert costs == sorted(costs)
    assert data["cheapest"] == "gemini/gemini-1.5-flash"
    # all estimates share the same token assumption
    assert len({e["completion_tokens"] for e in data["estimates"]}) == 1


def test_compare_subset_and_capability_filter(client):
    resp = client.post(
        "/v1/cost/compare",
        json={"messages": MESSAGES, "models": ["gpt-4o", "claude-3-haiku-20240307"]},
    )
    assert {e["model"] for e in resp.json()["estimates"]} == {
        "openai/gpt-4o",
        "anthropic/claude-3-haiku-20240307",
    }
    assert resp.json()["cheapest"] == "anthropic/claude-3-haiku-20240307"

    resp = client.post("/v1/cost/compare", json={"messages": MESSAGES, "required_capabilities": ["vision"]})
    models = {e["model"] for e in resp.json()["estimates"]}
    assert "openai/gpt-3.5-turbo" not in models  # no vision support
    assert "anthropic/claude-3-5-haiku-20241022" not in models  # no vision support


# ------------------------------------------------------------- live completion
def test_completion_carries_cost_and_is_tracked(env):
    client = env.client
    resp = _chat(client)
    assert resp.status_code == 200
    data = resp.json()
    assert data["cost"]["total_cost_usd"] > 0
    expected_prompt = data["usage"]["prompt_tokens"] / 1000 * 0.00015
    assert data["cost"]["prompt_cost_usd"] == pytest.approx(expected_prompt)

    summary = client.get("/v1/cost/summary?days=1").json()
    assert summary["requests"] == 1
    assert summary["total_spend_usd"] == pytest.approx(data["cost"]["total_cost_usd"])
    assert summary["by_provider"]["openai"]["requests"] == 1
    assert "openai/gpt-4o-mini" in summary["by_model"]

    records = client.get("/v1/cost/records").json()
    assert len(records) == 1
    assert records[0]["model"] == "openai/gpt-4o-mini"
    assert records[0]["api_key_id"] == "anonymous"
    assert records[0]["mock"] is True


def test_history_has_daily_buckets(env):
    _chat(env.client)
    history = env.client.get("/v1/cost/history?days=7").json()["history"]
    assert len(history) == 7
    totals = [b["requests"] for b in history]
    assert sum(totals) == 1
    assert history[-1]["requests"] == 1  # today is the last bucket


def test_usage_is_persisted_to_jsonl(env):
    _chat(env.client)
    store = env.settings.usage_store_path
    assert store.exists()
    lines = [json.loads(line) for line in store.read_text().splitlines() if line.strip()]
    assert len(lines) == 1
    assert lines[0]["model"] == "openai/gpt-4o-mini"
    assert lines[0]["cost_usd"] > 0


# --------------------------------------------------------------------- budgets
def test_budget_lifecycle_with_alerts_and_blocking(env):
    client = env.client

    # A budget so small that a single mock completion exhausts it.
    resp = client.post(
        "/v1/cost/budgets",
        json={"name": "tiny-test-budget", "limit_usd": 0.000001, "thresholds": [0.5, 1.0]},
    )
    assert resp.status_code == 201
    budget_id = resp.json()["id"]
    assert resp.json()["exceeded"] is False

    assert _chat(client, cache=False).status_code == 200  # first call allowed (spend was 0)

    status = (client.get("/v1/cost/budgets").json()["budgets"])
    budget = next(b for b in status if b["id"] == budget_id)
    assert budget["exceeded"] is True
    assert budget["used_pct"] > 100

    alerts = client.get("/v1/cost/budgets/alerts").json()["alerts"]
    budget_alerts = [a for a in alerts if a["budget_id"] == budget_id]
    assert {a["threshold"] for a in budget_alerts} == {0.5, 1.0}

    # Now blocked: the global budget is exhausted (cache=False forces a new
    # provider call — cached replies stay free by design).
    resp = _chat(client, cache=False)
    assert resp.status_code == 429
    assert resp.json()["error"]["type"] == "budget_exceeded"

    assert client.delete(f"/v1/cost/budgets/{budget_id}").status_code == 200
    assert _chat(client, cache=False).status_code == 200  # unblocked again


def test_budget_validation(client):
    resp = client.post("/v1/cost/budgets", json={"name": "x", "limit_usd": 1, "scope": "provider:nope"})
    assert resp.status_code == 400

    resp = client.post("/v1/cost/budgets", json={"name": "x", "limit_usd": 1, "scope": "weird"})
    assert resp.status_code == 400

    resp = client.delete("/v1/cost/budgets/b-missing")
    assert resp.status_code == 404


def test_env_configured_default_budget(make_env):
    with make_env(monthly_budget_usd=25.0) as env:
        budgets = env.client.get("/v1/cost/budgets").json()["budgets"]
        default = next(b for b in budgets if b["id"] == "default")
        assert default["limit_usd"] == 25.0
        assert default["period"] == "monthly"


def test_provider_scoped_budget(env):
    client = env.client
    client.post(
        "/v1/cost/budgets",
        json={"name": "openai-only", "limit_usd": 1.0, "scope": "provider:openai"},
    )
    _chat(client)
    budgets = client.get("/v1/cost/budgets").json()["budgets"]
    scoped = next(b for b in budgets if b["scope"] == "provider:openai")
    assert scoped["spent_usd"] > 0
    global_other = next(b for b in budgets if b["id"] == scoped["id"])
    assert global_other["exceeded"] is False
