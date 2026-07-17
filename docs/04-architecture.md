# Architecture - OMNI Unified AI API Platform

## High-Level Architecture

```
                 ┌─────────────────────────────────────────────────┐
                 │        Control Plane (Python + TS)              │
                 │  Dashboard, Policy Engine, Cost Intelligence,   │
                 │  Registry, Observability, Billing (Stripe/x402) │
                 └─────────────────────────────────────────────────┘
                                  ▲
                                  │ gRPC / REST (mTLS)
                                  ▼
┌──────────────────────────────────────────────────────────────────────┐
│                    Data Plane (Go/Rust - Envoy based)               │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐ │
│  │ AI Gateway  │  │ MCP Gateway │  │ Media Plane │  │Commerce GW │ │
│  │ 11μs target │  │ stateless   │  │ fal.ai compat│  │ x402+MPP   │ │
│  │ + caching   │  │ federation  │  │             │  │            │ │
│  └─────────────┘  └─────────────┘  └─────────────┘  └─────────────┘ │
│        └─────────────────────────────────────────────────────────┘    │
│                          Envoy Proxy (L7)                             │
│  AuthZ | Rate Limit (token-aware) | PII/Masking | Prompt Injection   │
│  Semantic Cache (Redis + Qdrant) | Fallback | Observability (OTel)   │
└──────────────────────────────────────────────────────────────────────┘
                                  ▼
┌──────────────────────────────────────────────────────────────────────┐
│                    Upstream Providers / MCP Servers                   │
│ OpenAI, Anthropic, Gemini, DeepSeek, Groq, Together, Fireworks,     │
│ fal.ai, Replicate, ElevenLabs, Deepgram, Mistral OCR, DeepL, Hive,  │
│ 17k+ MCP Servers (GitHub, Slack, Postgres, etc)                      │
└──────────────────────────────────────────────────────────────────────┘
```

Data plane MUST be Go (like Bifrost) or Rust (like Helicone's Rust architecture, Agent Gateway Rust). Python is too slow for 11μs overhead and 5k RPS.

## Data Plane Deep Dive (Go)

### Envoy AI Gateway pattern (from Envoy AI Gateway blog with Bloomberg/Tetrate)

Use Envoy Proxy as base - battle-tested, trusted by thousands of enterprises. We extend with:

- **Filter 1: AuthN/AuthZ - Virtual Keys**
  - Virtual key → lookup team → rate limit bucket → provider key decrypt via KMS
  - JWT validation for MCP (iss per RFC 9207, SEP-2468), OAuth for remote MCP

- **Filter 2: Token-Aware Rate Limiting**
  - Traditional API GW does req/min. AI needs tokens/min.
  - Estimate tokens pre-request via tiktoken, enforce: `user_tokens_per_minute`, `team_tokens_per_hour`
  - Hierarchical budgets: org 10M tokens/day, team 1M, user 100k. Like Bifrost Virtual Keys budget.

- **Filter 3: Semantic Cache**
  - Layer 1: Exact cache - SHA256(request) in Redis - 1ms lookup
  - Layer 2: Semantic cache - embed prompt via small model (e.g., text-embedding-3-small), query Qdrant with threshold 0.95 similarity. Hit = return cached without upstream call.
  - Benchmark: Bifrost style semantic caching can give 40% hit rate, cut spend 50% (Maxim AI)
  - Write path: Cache key includes model + params + prompt hash. TTL configurable.

- **Filter 4: Security Guardrails**
  - Prompt injection detection: Scan tool descriptions + resources for injection patterns (like Zuplo prompt injection policy + Akamai AI Firewall)
  - PII detection: Regex + NER model for SSN, email, credit card, PHI
  - Secret masking: Redact `sk-`, `x-api-key`, `ghp_` in responses/logs
  - Budget enforcement: Kill runaway agent loops (e.g., >100 tool calls in 1 min)

- **Router:**
  - Weighted routing: 70% DeepSeek (cheap), 30% Claude (quality) for fallback
  - Automatic failover: If provider 5xx or latency > 2s, retry with backoff to alt provider. Track via circuit breaker.
  - Cost optimizer call: Consult control plane's ML recommender async for model suggestion

- **MCP Gateway Extension:**
  - `MCPGatewayExtension` CRD (Red Hat pattern) targeting Gateway API Gateway resource
  - Federation: Pull `tools/list` from N upstream MCP servers, merge, deduplicate with prefix option (e.g., `github__create_issue`)
  - Virtual server composition: Per team, compose subset of approved tools into one virtual MCP server
  - JSON-RPC 2.0 handling over Streamable HTTP (MCP 2026-07-28 spec, SSE deprecated). Support `Mcp-Method` header for routing, plain round-robin LB (stateless core)
  - Audit: Emit OTel spans with typed events for every tool call

### Performance Targets

- Data plane overhead: 11-15μs p50 at 5k RPS (Bifrost benchmark)
- Cache lookup p95: <5ms Redis, <15ms semantic
- Routing failover: <50ms detection
- Deploy: Kubernetes Gateway API, Envoy Proxy data plane, Helm chart

## Control Plane Deep Dive (Python/FastAPI + TypeScript policy)

### Services

1.  **Dashboard API (FastAPI):**
    - Team/org management, virtual key CRUD, budget config
    - Provider credential management: BYOK encrypted with AWS KMS/GCP KMS Vault
    - Prompt registry: Versioned prompts, A/B test groups

2.  **Policy Engine (TypeScript VM):**
    - Inspired by Zuplo TypeScript programmability
    - Users write policies in TS: `function rateLimit(ctx) { if ctx.user.plan == 'free' && ctx.tokens > 1000 return deny(); }`
    - Sandboxed QuickJS/V8 isolate, hot reload <1s

3.  **Cost Intelligence Engine (Python):**
    - Eval harness: FutureAGI pattern - dataset of 100+ labeled prompts, run against candidate models, score exact match, groundedness, format validity, custom LLM judge
    - Recommendation: For each prompt template, find cheapest model meeting quality threshold (e.g., GPT-5 nano vs GPT-5)
    - Prompt optimizer: Compression (LLMLingua), cache key optimization
    - Billing attribution: Per-call cost, per-team, per-model, per-tool

4.  **MCP Registry:**
    - Crawl mcp.so, Glama, Smithery, official AAIF registry
    - Curated metadata: trust score, category, auth type, pricing
    - One-click deploy: Docker container isolated per MCP server (Docker MCP Gateway pattern) with resource limits + image signing
    - Hosted VS BYO: Offer managed hosted version for popular servers (e.g., GitHub, Postgres) vs passthrough

5.  **Observability Stack:**
    - OpenTelemetry Collector -> ClickHouse + Prometheus + Jaeger (like Bifrost Prometheus)
    - Langfuse / traceAI integration for LLM traces
    - MLflow for agent traceability (Red Hat OpenShift AI pattern)
    - Dashboard: per-model usage, latency, error, cost, cache hit rate

6.  **Commerce Service:**
    - Stripe integration: subscriptions, metered usage, invoices, tax
    - x402 facilitator: Verify on-chain USDC payment proof, role of facilitator from Coinbase x402 spec
    - MPP handler: Session management, spending caps, allowlist payees, CloudTrail audit
    - Wallet: Agent wallets with programmable guardrails (like Crossmint multi-protocol API)

### Data Stores

- **PostgreSQL:** Teams, keys, budgets, prompt versions, registry metadata, billing
- **Redis:** Exact cache, rate limit counters (token buckets), session store for MPP
- **Qdrant/Pinecone:** Semantic cache vectors
- **ClickHouse:** Logs/traces (high ingest)
- **S3/R2:** Prompt assets, eval datasets

## Security Architecture

- Zero trust: mTLS between control/data plane
- Secret zero: Provider API keys never in plaintext logs, encrypted at rest (KMS), in transit (mTLS), in memory (sealed)
- Data plane can run air-gapped: No outbound to control plane except gRPC telemetry (push, not pull), and OTel export optional
- Compliance: Audit logs immutable (WORM S3), SOC2 controls around key management, EU AI Act: every AI decision traced (LLM -> tool chain)
- Malicious MCP protection: Tool descriptions scanned for prompt injection (like Snyk+Invariant Labs), container isolation for untrusted servers

## Deployment Topologies

### Topology A: Managed Cloud (SaaS) - for startups
```
Client -> Cloudflare/Edge (300+ PoPs) -> AWS/GCP regional data plane (Envoy) -> Providers
                            ↕
                      Control Plane (us-east-1)
```
- Lowest latency: Edge terminates TLS, routes via Anycast
- Multi-cloud managed dedicated: Like Zuplo - not pinned to AWS/Azure/GCP, so no cross-cloud egress penalties

### Topology B: Self-Hosted Data Plane (Hybrid) - for enterprise needing data sovereignty
```
Client -> VPC Envoy Data Plane (customer AWS) -> Providers (direct, zero data flow to us except billing telemetry)
                        ↕ (mTLS gRPC, only metadata)
              Control Plane (SaaS) - cost analytics, registry
```
- Similar to LiteLLM "zero external data flow"
- Data plane is Go binary + Helm, needs just Redis (or embedded)
- Billing: Push token counts, not prompts

### Topology C: Full Air-Gapped
```
Client -> On-Prem K8s (Envoy + Control Plane all in VPC) -> Providers via proxy
```
- No SaaS dependency
- Enterprise license, support contract

## API Design (World-Class Spec)

### OpenAI-Compatible (primary)

`POST /v1/chat/completions` with extra fields:
```json
{
  "model": "auto", // our intelligent router picks cheapest that meets quality
  "model_preferences": {"cost": 0.7, "quality": 0.3, "latency": 0.2},
  "fallback_models": ["gpt-5-mini", "gemini-2.5-flash"],
  "cache": {"enabled": true, "semantic": true, "threshold": 0.95},
  "budget": {"max_tokens": 10000},
  "metadata": {"team": "acme", "env": "prod"}
}
```

### Fal.ai-Compatible for Media

`POST /v1/videos/generations` (same as fal.ai) so existing fal.ai code works with base_url swap.

### MCP Spec Compliant

`POST /mcp/v1` Streamable HTTP, JSON-RPC 2.0, `Mcp-Method` header routing.

### Admin API

`GET /v1/admin/usage?team=acme&group_by=model` -> ClickHouse aggregated.

## Scalability Math

- Target: 10B tokens/day (Portkey claim) = ~115k tokens/sec avg = ~1150 req/sec @ 100 token avg
- At 5k RPS (Bifrost bench), need 3-4 Envoy pods (2 vCPU each Go) + Redis cluster (1 shard per 10k RPS)
- Semantic cache: Qdrant with HNSW, 10M vectors ~ 10GB RAM, 15ms p95 search
- Cost: Data plane infra ~ $500/mo for 5k RPS on GCP (vs LiteLLM $2k+ due to Python inefficiency)

This architecture lets us claim: "Bifrost perf, Zuplo MCP, Portkey governance, OpenRouter catalog, fal.ai media."
