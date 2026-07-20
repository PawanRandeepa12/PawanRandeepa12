"""Stage 1: foundation & core architecture."""

from __future__ import annotations


def chat(client, payload: dict | None = None, **overrides):
    body = {"model": "openai/gpt-4o-mini", "messages": [{"role": "user", "content": "Say hello."}]}
    if payload:
        body = payload
    body.update(overrides)
    return client.post("/v1/chat/completions", json=body)


def test_root_reports_stage_1(client):
    resp = client.get("/")
    assert resp.status_code == 200
    data = resp.json()
    assert data["stages"]["1_foundation"] is True


def test_health_and_readiness(client):
    resp = client.get("/health")
    assert resp.status_code == 200
    assert resp.json()["status"] == "ok"

    resp = client.get("/ready")
    assert resp.status_code == 200
    providers = resp.json()["providers"]
    assert set(providers) == {"openai", "anthropic", "gemini"}
    assert all(p["mode"] == "mock" for p in providers.values())


def test_model_catalog_lists_all_providers(client):
    resp = client.get("/v1/models")
    assert resp.status_code == 200
    data = resp.json()["data"]
    ids = {m["id"] for m in data}
    assert {
        "openai/gpt-4o",
        "anthropic/claude-3-5-sonnet-20241022",
        "gemini/gemini-1.5-flash",
    } <= ids
    for model in data:
        assert model["pricing_per_1k_tokens_usd"]["input"] > 0
        assert "vision" in model["capabilities"]


def test_model_detail_and_unknown_model(client):
    resp = client.get("/v1/models/openai/gpt-4o")
    assert resp.status_code == 200
    assert resp.json()["provider"] == "openai"

    resp = client.get("/v1/models/does-not-exist")
    assert resp.status_code == 404
    assert resp.json()["error"]["type"] == "model_not_found"


def test_chat_completion_mock_pipeline(client):
    resp = chat(client)
    assert resp.status_code == 200
    data = resp.json()
    assert data["provider"] == "openai"
    assert data["model"] == "openai/gpt-4o-mini"
    assert "MOCK" in data["choices"][0]["message"]["content"]
    usage = data["usage"]
    assert usage["total_tokens"] == usage["prompt_tokens"] + usage["completion_tokens"]
    assert usage["prompt_tokens"] > 0
    assert data["routing"]["mock"] is True
    assert data["latency_ms"] >= 0
    assert "x-request-id" in {k.lower() for k in resp.headers}


def test_bare_model_name_and_auto_resolution(client):
    resp = chat(client, model="gpt-4o-mini")
    assert resp.status_code == 200
    assert resp.json()["model"] == "openai/gpt-4o-mini"

    resp = chat(client, model="auto")
    assert resp.status_code == 200
    assert "/" in resp.json()["model"]

    resp = chat(client, model="claude-3-haiku-20240307")
    assert resp.status_code == 200
    assert resp.json()["provider"] == "anthropic"


def test_each_provider_serves_completion(client):
    for model in ("openai/gpt-4o", "anthropic/claude-3-5-haiku-20241022", "gemini/gemini-2.0-flash"):
        resp = chat(client, model=model)
        assert resp.status_code == 200, model
        assert resp.json()["model"] == model


def test_unknown_model_returns_404(client):
    resp = chat(client, model="openai/nope-9000")
    assert resp.status_code == 404
    assert resp.json()["error"]["type"] == "model_not_found"


def test_validation_error_envelope(client):
    resp = client.post("/v1/chat/completions", json={"messages": "not-a-list"})
    assert resp.status_code == 422
    assert resp.json()["error"]["type"] == "validation_error"


def test_api_key_auth(make_env):
    with make_env(api_keys="secret-key-1,secret-key-2") as env:
        assert env.client.post("/v1/chat/completions", json={
            "model": "auto", "messages": [{"role": "user", "content": "hi"}]
        }).status_code == 401

        headers = {"X-API-Key": "wrong"}
        assert env.client.post("/v1/chat/completions", json={
            "model": "auto", "messages": [{"role": "user", "content": "hi"}]
        }, headers=headers).status_code == 401

        headers = {"X-API-Key": "secret-key-2"}
        resp = env.client.post("/v1/chat/completions", json={
            "model": "auto", "messages": [{"role": "user", "content": "hi"}]
        }, headers=headers)
        assert resp.status_code == 200


def test_master_key_auth(make_env):
    with make_env(master_api_key="boss-key") as env:
        headers = {"Authorization": "Bearer boss-key"}
        resp = env.client.post("/v1/chat/completions", json={
            "model": "auto", "messages": [{"role": "user", "content": "hi"}]
        }, headers=headers)
        assert resp.status_code == 200
