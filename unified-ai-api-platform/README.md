# Unified AI API Platform

One OpenAI-style REST API in front of **OpenAI**, **Anthropic Claude** and
**Google Gemini** — with a **cost optimization engine**, an **intelligent
routing layer** and **performance tooling**, built with FastAPI and Docker.

Write to one API. Switch providers by changing a model id — or let the
platform pick the cheapest / fastest healthy provider per request, track
every dollar, enforce budgets, fail over automatically when a provider goes
down, and cache repeat calls for free.

> **Try it with zero credentials:** with no provider keys set
> (`MOCK_PROVIDERS=auto`, the default) every provider serves deterministic
> mock completions while the *entire real pipeline* runs — auth, routing,
> cost tracking, budgets, caching, rate limiting, metrics. Ideal for
> development, CI and demos.

---

## The four stages

| Stage | Focus | What it adds |
|------:|-------|--------------|
| **1** | Foundation & core architecture | FastAPI app, Docker, provider adapters (OpenAI / Claude / Gemini + mock mode), config management, structured logging, API-key auth, unified error envelope |
| **2** | Cost optimization engine | Editable pricing database, per-request cost tracking (persisted), cost comparison, budgets with threshold alerts + hard blocking, spend analytics endpoints |
| **3** | Intelligent routing system | Cost / latency / balanced routing strategies, provider health monitoring with circuit breakers, automatic failover, capability-aware model selection, load balancing |
| **4** | Speed & performance | Redis / in-memory response cache, priority request queue, token-bucket rate limiting, parallel multi-model comparison, Prometheus metrics |

Stage-by-stage commits: `Stage 1` → `Stage 2` → `Stage 3` → `Stage 4` in the
git history — each stage leaves the platform fully working.

---

## Quickstart

### Local (Python 3.11+)

```bash
cd unified-ai-api-platform
python -m venv .venv && source .venv/bin/activate
pip install -r requirements.txt
uvicorn app.main:app --reload
```

### Docker (with Redis cache)

```bash
cp .env.example .env      # optional; sensible defaults without it
docker compose up --build
```

Interactive docs: **http://localhost:8000/docs**

### Your first requests

```bash
# Unified chat completion — model "auto" + strategy "cost" picks the cheapest
curl -X POST localhost:8000/v1/chat/completions \
  -H 'Content-Type: application/json' \
  -d '{
        "model": "auto",
        "strategy": "cost",
        "messages": [{"role": "user", "content": "Give me one tip for reducing LLM spend."}]
      }'
```

```json
{
  "model": "gemini/gemini-1.5-flash",
  "provider": "gemini",
  "choices": [{"message": {"role": "assistant", "content": "[MOCK ...] ..."}}],
  "usage": {"prompt_tokens": 14, "completion_tokens": 90, "total_tokens": 104},
  "cost": {"total_cost_usd": 0.00002805, "pricing_source": "pricing_file"},
  "routing": {"strategy": "cost", "fallbacks_used": 0, "reason": "estimated cost $0.00002805 for this request"},
  "cached": false,
  "latency_ms": 0.31
}
```

```bash
# What did that cost across ALL models, before spending anything?
curl -X POST localhost:8000/v1/cost/compare \
  -H 'Content-Type: application/json' \
  -d '{"messages": [{"role": "user", "content": "Hello"}]}'

# Which model would the router pick and why? (dry run, no call made)
curl -X POST localhost:8000/v1/routing/route \
  -H 'Content-Type: application/json' \
  -d '{"model": "auto", "strategy": "balanced", "messages": [{"role": "user", "content": "Hello"}]}'

# Run the same prompt on the flagship model of every provider, in parallel
curl -X POST localhost:8000/v1/chat/compare \
  -H 'Content-Type: application/json' \
  -d '{"messages": [{"role": "user", "content": "Explain caching in one sentence."}]}'

# Spend analytics
curl 'localhost:8000/v1/cost/summary?days=30'
curl 'localhost:8000/v1/cost/history?days=30'

# Prometheus metrics
curl localhost:8000/metrics
```

Or run the guided tour: `python examples/demo.py`

---

## API reference (summary)

| Method & path | Purpose |
|---|---|
| `POST /v1/chat/completions` | Unified completion. `model`: unified id / bare name / `auto`. Supports `strategy`, `priority`, `cache`, `max_fallbacks`, `required_capabilities` |
| `POST /v1/chat/compare` | Parallel fan-out of one prompt to N models → content + cost + latency each, `fastest` & `cheapest` |
| `GET /v1/models` | Catalog: pricing, context window, capabilities, tier, provider mode |
| `POST /v1/cost/estimate` | Cost estimate for one model (exact tokens or estimated from messages) |
| `POST /v1/cost/compare` | Every model's estimated cost for the same request, cheapest first |
| `GET /v1/cost/summary?days=N` | Spend/usage totals by provider & model |
| `GET /v1/cost/history?days=N` | Per-day spend series |
| `GET /v1/cost/records` | Recent per-request usage records |
| `GET/POST /v1/cost/budgets` | Budget CRUD + live status (`used_pct`, `exceeded`, …) |
| `GET /v1/cost/budgets/alerts` | Fired threshold alerts |
| `POST /v1/routing/route` | Routing dry-run: ordered candidates + exclusion reasons |
| `GET /v1/routing/health` | Provider health + circuit breaker state |
| `POST /v1/routing/health/{provider}/reset` | Manually close a provider's circuit |
| `GET /v1/routing/latency` | Observed per-model latency stats (avg/p50/p95) |
| `GET /v1/admin/stats` | Platform overview (usage, cache, queue, circuits, counters) |
| `GET /v1/admin/cache/stats` · `POST /v1/admin/cache/clear` | Cache operations |
| `GET /v1/admin/queue` · `GET /v1/admin/config` | Queue snapshot / redacted config |
| `GET /metrics` | Prometheus exposition |
| `GET /health` · `GET /ready` | Probes |

**Auth:** pass `X-API-Key: <key>` (or `Authorization: Bearer <key>`) when
`API_KEYS`/`MASTER_API_KEY` are configured, and `/v1/admin/*` needs the master key.
With no keys configured the platform runs open (dev mode). Clients get
HTTP 401/403/429 responses in the shared error envelope.

---

## How it works

```
                    ┌─────────────────────────── FastAPI ────────────────────────────┐
 client ──► auth ──►│ rate limit ──► cache? ──► router.plan ──► budget guard ──►     │──► model A (openai)
    ▲               │      (S4)      HIT│ (S4)      │  (S3)         │  (S2)          │      ▼ fail
    │               │                   └── return  │ cost/latency/ │ block when     │──► model B (anthropic)
    │               │      metrics (S4) ◄───────────┤ balanced      │ exhausted      │      ▼ fail
    └───────────────┴─────────── cost track (S2) ◄──┴───────────────┴────────────────┴──► model C (gemini)
                                  usage ──► alert thresholds (S2)   latency/health feedback (S3)
```

### Stage 1 — Foundation

- **Provider adapters** normalize OpenAI `/chat/completions`, Anthropic
  `/messages` and Gemini `:generateContent` into one `ProviderResult`
  (content, finish reason, token usage). Errors are classified
  retryable/non-retryable (timeouts, 429/5xx vs 4xx) which Stage 3 uses.
- **Mock mode** (`MOCK_PROVIDERS=auto|always|never`) makes the platform
  fully usable offline — providers without keys serve deterministic mocks.
- **Config**: everything is an env var (see `.env.example`), validated with
  pydantic-settings. APIs keys hashed before logging; admin config endpoint
  redacts secrets.
- **Logging**: structured JSON logs with `X-Request-ID` correlation.

### Stage 2 — Cost optimization

- **Pricing database** (`config/pricing.json`): editable, no redeploy needed;
  resolution order pricing file → adapter catalog → default.
- Every real call records a `UsageRecord` (tokens, cost, latency, strategy,
  API key) in memory + a JSONL store; cached replies are free by design.
- **Budgets** at `global` / `provider:<name>` / `key:<id>` scope over
  monthly/daily/all-time periods. Threshold crossings (e.g. 50%, 80%, 100%)
  fire alerts; an exhausted **global** budget blocks new provider calls with
  `429 budget_exceeded`.
- Compare before you spend: `/v1/cost/compare` ranks every model's estimated
  cost for the *actual* request payload.

### Stage 3 — Intelligent routing

- **Strategies** (`strategy` per request, `DEFAULT_STRATEGY` globally):
  - `cost` — cheapest healthy model that satisfies `required_capabilities`
  - `latency` — lowest observed average latency (unobserved models start mid-pack)
  - `balanced` — `0.6 × normalized cost + 0.4 × normalized latency`
  - `explicit` — exact model requested
- **Failover**: the gateway walks the plan's candidates on any provider error
  (timeout/5xx/429/...); `routing.fallbacks_used` shows how many hops were needed.
- **Circuit breaker**: after `CIRCUIT_BREAKER_THRESHOLD` consecutive failures
  a provider is skipped for `CIRCUIT_BREAKER_COOLDOWN_SECONDS`, then probed
  half-open; resettable via API.
- **Load balancing**: equal-rank candidates rotate round-robin.
- Transparency: `POST /v1/routing/route` shows the exact candidate order and
  per-model exclusion reasons without executing anything.

### Stage 4 — Speed & performance

- **Cache**: Redis when `REDIS_URL` is set (compose provides it), bounded
  in-memory TTL otherwise. Identical requests (`X-Cache: HIT`) skip provider
  calls, spend and budget; `cache: false` per request opts out.
- **Priority queue**: `priority: high|normal|low` orders queued requests;
  `QUEUE_MAX_CONCURRENCY` caps simultaneous upstream calls.
- **Rate limiting**: per-API-key (or per-IP) token bucket → `429` +
  `Retry-After`; authenticated master key bypasses.
- **Compare endpoint**: parallel fan-out with per-model latency/cost and
  `fastest`/`cheapest` verdicts — real calls, tracked like any other spend.
- **Metrics**: `/metrics` (Prometheus) — HTTP + provider latency histograms,
  request/error/token/cost counters, cache events, fallovers, rate-limit hits.

---

## Configuration

All settings are environment variables (or `.env`) — the full commented list
is in [`.env.example`](.env.example). The most useful ones:

| Variable | Default | Purpose |
|---|---|---|
| `OPENAI_API_KEY` / `ANTHROPIC_API_KEY` / `GOOGLE_API_KEY` | – | Provider credentials |
| `MOCK_PROVIDERS` | `auto` | `auto` mocks keyless providers · `always` forces mocks · `never` live only |
| `API_KEYS` / `MASTER_API_KEY` | – | Platform auth (empty = open dev mode) |
| `MONTHLY_BUDGET_USD` | `0` | Default global monthly budget (0 = off) |
| `DEFAULT_STRATEGY` | `balanced` | `cost` · `latency` · `balanced` |
| `MAX_FALLBACKS` / `CIRCUIT_BREAKER_*` | `2` / `3`,`60s` | Failover depth, breaker sensitivity |
| `REDIS_URL` | – | Cache backend (empty = in-memory) |
| `RATE_LIMIT_REQUESTS_PER_MINUTE` / `_BURST` | `120` / `20` | Per-client limits |
| `QUEUE_MAX_CONCURRENCY` | `8` | Max parallel upstream calls |

Going **live** (real provider calls):

```bash
export OPENAI_API_KEY=sk-...            # and/or ANTHROPIC_API_KEY, GOOGLE_API_KEY
export MOCK_PROVIDERS=never             # providers without keys become "unconfigured"
```

## Project layout

```
unified-ai-api-platform/
├── app/
│   ├── main.py                     # app factory; lifespan wires S1→S4 components
│   ├── config.py                   # typed settings for every stage
│   ├── core/                       # logging, error envelope, auth, middleware, tokens
│   ├── schemas/                    # chat, compare, cost contracts
│   ├── providers/                  # adapter base + openai / anthropic / gemini + mock
│   ├── services/
│   │   ├── gateway.py              # orchestration: cache→route→budget→call→track
│   │   ├── cost/                   # S2: pricing, tracker, budgets, analytics
│   │   ├── routing/               # S3: router, health/circuit breaker, latency
│   │   └── performance/           # S4: cache, rate limit, priority queue, metrics
│   └── api/routes/                 # health, models, chat, cost, routing, admin
├── config/pricing.json             # editable pricing database
├── tests/                          # 51 offline tests (mock providers)
├── examples/demo.py                # guided CLI tour of the API
├── Dockerfile · docker-compose.yml · Makefile
└── .env.example
```

## Development

```bash
pip install -r requirements-dev.txt
pytest          # 51 tests, fully offline
ruff check app tests
```

## Roadmap

- Streaming completions (SSE) end-to-end per provider
- Streaming/token-aware live usage for long generations
- Persistent analytics DB (SQLite/Postgres) behind the `CostTracker` interface
- More providers (Mistral, Cohere, Azure OpenAI) via the adapter interface
- Per-key budgets auto-derived from API keys; webhook budget alerts
