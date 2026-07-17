# Platform Reference Implementation

## Structure

```
platform/
  gateway/          # Go data plane - 11μs target (Bifrost pattern)
    main.go         # Envoy-inspired Gin, OpenAI compat + MCP + Commerce
    config.yaml     # LiteLLM compatible + OMNI extensions
  controlplane/     # Python FastAPI - Cost AI, Registry, Billing
    app.py          # Virtual keys, Pareto recommender, MCP registry, x402/MPP sessions
  sdk/
    python/omni.py  # Drop-in SDK - OpenAI + cost reco + MCP + paidTool decorator
```

## Quick Start

### 1. Data Plane (Go)

```bash
cd platform/gateway
go mod init omni/gateway
go mod tidy
go run main.go

# Test
curl -X POST http://localhost:8080/v1/chat/completions \
  -H "Authorization: Bearer omni_demo_12345" \
  -H "Content-Type: application/json" \
  -d '{"model":"auto","messages":[{"role":"user","content":"hello"}],"model_preferences":{"cost":0.8}}'

# MCP
curl -X POST http://localhost:8080/mcp/v1 \
  -H "Authorization: Bearer omni_demo_12345" \
  -H "Content-Type: application/json" \
  -d '{"jsonrpc":"2.0","id":1,"method":"tools/list"}'
```

### 2. Control Plane (Python)

```bash
cd platform/controlplane
pip install fastapi uvicorn pydantic
uvicorn app:app --reload --port 8001

# Test
curl http://localhost:8001/health
curl http://localhost:8001/v1/mcp/registry
curl -X POST http://localhost:8001/v1/cost/recommend -H "Content-Type: application/json" -d '{"prompt_template":"classify sentiment","quality_threshold":0.9,"preferences":{"cost":0.8}}'
```

### 3. SDK

```bash
cd platform/sdk/python
pip install openai requests
python omni.py
```

## Docker Compose (Prod-like)

```yaml
# docker-compose.yml (to be added)
services:
  gateway:
    build: ./gateway
    ports: ["8080:8080"]
    env: [REDIS_URL]
  controlplane:
    build: ./controlplane
    ports: ["8001:8001"]
  redis:
    image: redis:7
  qdrant:
    image: qdrant/qdrant
  postgres:
    image: postgres:15
```

## Why Go for Data Plane?

- LiteLLM Python overhead is known bottleneck (per Cubxxw market analysis)
- Bifrost Go = 11μs at 5k RPS best in class
- Envoy Proxy is battle-tested (Envoy AI Gateway with Bloomberg/Tetrate)
- Helicone Rust architecture, Agent Gateway Rust - speed matters as gateway latency compounds per agent step

## Next Steps to Production

- Replace in-memory maps with Redis Lua for rate limiting + Qdrant for semantic cache + Postgres for virtual keys (see config.yaml)
- Add Envoy WASM filters for PII masking (Higress pattern)
- Add OTel collector -> ClickHouse
- Implement real provider HTTP calls with KMS decryption (currently mock)
- Add x402 facilitator verification via Coinbase CDP SDK
- Add MPP via Stripe MPP SDK (launched March 18 2026)
