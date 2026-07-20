# Unified AI API Platform

One OpenAI-style REST API in front of **OpenAI**, **Anthropic Claude** and
**Google Gemini** — with a cost optimization engine, an intelligent routing
layer and performance tooling, built with **FastAPI** and **Docker**.

The platform is delivered in four stages:

| Stage | Focus | Status |
|------:|-------|--------|
| 1 | Foundation & core architecture | ✅ |
| 2 | Cost optimization engine | ⏳ planned |
| 3 | Intelligent routing system | ⏳ planned |
| 4 | Speed & performance | ⏳ planned |

> **Mock mode:** no provider keys? No problem. With `MOCK_PROVIDERS=auto`
> (default) the platform serves deterministic mock completions while running
> the *entire* real pipeline — perfect for development, CI and demos.

---

## Quickstart

### Local

```bash
cd unified-ai-api-platform
python -m venv .venv && source .venv/bin/activate
pip install -r requirements.txt
uvicorn app.main:app --reload
```

### Docker

```bash
docker compose up --build
```

Then:

```bash
# Liveness / readiness
curl localhost:8000/health
curl localhost:8000/ready

# List the unified model catalog
curl localhost:8000/v1/models

# Unified chat completion (works without keys via mock mode)
curl -X POST localhost:8000/v1/chat/completions \
  -H 'Content-Type: application/json' \
  -d '{
        "model": "openai/gpt-4o-mini",
        "messages": [{"role": "user", "content": "Hello from the platform!"}]
      }'
```

Interactive API docs: **http://localhost:8000/docs**

---

## Stage 1 — Foundation & core architecture

- **Project setup** — FastAPI app factory (`app/main.py`), `Dockerfile`
  (non-root, health-checked) and `docker-compose.yml`, `Makefile`, typed
  settings via `pydantic-settings` with `.env` support.
- **Base API structure** — unified `POST /v1/chat/completions`,
  `GET /v1/models`, `/health`, `/ready`; consistent error envelope for every
  failure (`{"error": {"type", "message", ...}}`).
- **Provider integrations** — adapter layer (`ProviderAdapter`) with real
  HTTP integrations for OpenAI (`/chat/completions`), Anthropic (`/messages`)
  and Gemini (`:generateContent`), plus a deterministic mock mode.
- **Configuration management** — every knob is an environment variable
  documented in `.env.example`.
- **Logging & security** — structured JSON logs with request-id correlation
  (`X-Request-ID`), API-key auth (`API_KEYS` / `MASTER_API_KEY`) with master
  key support.

### Unified model ids

Models are addressed as `provider/name`, e.g. `openai/gpt-4o`,
`anthropic/claude-3-5-sonnet-20241022`, `gemini/gemini-1.5-flash`.
Bare names (`gpt-4o-mini`) work too; `model: "auto"` lets the platform pick.

### Layout

```
app/
├── main.py              # app factory, lifespan wiring
├── config.py            # typed settings (all stages)
├── core/                # logging, errors, security, middleware, token utils
├── schemas/             # unified request/response contracts
├── providers/           # adapter base + OpenAI / Anthropic / Gemini + mock
├── services/            # gateway orchestration (cost/routing/perf submodules later)
└── api/routes/          # health, models, chat
tests/                   # offline test suite (mock providers)
config/                  # pricing database (Stage 2)
```

## Configuration

See `.env.example` for the fully commented list. Highlights:

| Variable | Default | Purpose |
|----------|---------|---------|
| `OPENAI_API_KEY` / `ANTHROPIC_API_KEY` / `GOOGLE_API_KEY` | – | Provider credentials |
| `MOCK_PROVIDERS` | `auto` | `auto` mocks keyless providers, `always` forces mocks, `never` = live only |
| `API_KEYS` / `MASTER_API_KEY` | – | Platform client auth (empty = open dev mode) |
| `PROVIDER_TIMEOUT_SECONDS` | `30` | Upstream timeout |

## Tests

```bash
pip install -r requirements-dev.txt
pytest
```

The suite runs fully offline against mock providers.

## Roadmap

- **Stage 2** — per-request cost tracking, editable pricing database, cost
  comparison, budgets with threshold alerts, spend analytics endpoints.
- **Stage 3** — cost/latency/balanced routing strategies, provider health
  monitoring with circuit breakers, automatic failover, capability-aware
  model selection.
- **Stage 4** — Redis response caching, priority request queue, rate
  limiting, parallel multi-provider comparison, Prometheus metrics.
