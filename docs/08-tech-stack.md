# Tech Stack - World-Class Choices (July 2026)

## Data Plane - Performance Tier

- **Language:** Go 1.22+ (like Bifrost) or Rust (like Helicone, Agent Gateway). Go chosen for faster hiring, Envoy extensibility. Rust for WASM filters.
- **Framework:** Gin/Fiber for HTTP, Envoy Proxy for L7 (via Envoy AI Gateway pattern from Bloomberg/Tetrate). Higress Wasm for filters.
- **Proxy:** Envoy Proxy base - trusted by thousands of enterprises, supports Gateway API, MCPGatewayExtension CRD (Red Hat pattern).
- **Cache:** Redis for exact (SHA256 hash) + token buckets (Lua), Qdrant for semantic vectors (HNSW, 15ms p95). Alternative: Pinecone managed but higher cost.
- **Rate Limiting:** Redis + Lua token bucket - token-aware, not just req/min (critical per Zuplo).
- **Security:** 
  - Presidio or custom NER for PII
  - Secret masking regex
  - Prompt injection detection via small model (like Akamai AI Firewall pattern from Zuplo)
  - Container isolation per MCP server via Docker SDK + cosign image signing.

## Control Plane - Velocity Tier

- **Language:** Python 3.11 + FastAPI (for AI/ML velocity) + TypeScript isolate for policies (Zuplo pattern)
- **API Framework:** FastAPI with Pydantic v2
- **Policy Engine:** QuickJS or V8 Isolate in Go sidecar - TypeScript policies hot reload <1s (Zuplo TS programmability)
- **Database:**
  - Postgres (Neon serverless or managed) for teams, keys, budgets
  - ClickHouse for logs/traces (high ingest) - alternative: TimescaleDB but ClickHouse better for 10B tokens/day query
  - Qdrant cloud for semantic cache vectors
- **Observability:**
  - OpenTelemetry SDK -> OTel Collector -> Prometheus (metrics) + Jaeger/Tempo (traces) + ClickHouse (logs)
  - Langfuse integration for LLM trace UI (LiteLLM deep integration pattern)
  - MLflow for agent traceability (Red Hat OpenShift AI pattern)
  - Grafana dashboards
- **Queue:** NATS or Redis Streams for async eval harness jobs
- **Orchestration:** Kubernetes Gateway API + Helm chart (like Kong, APISIX)

## MCP Gateway Specifics

- **Spec:** MCP 2026-07-28 RC - stateless core, Streamable HTTP (SSE deprecated per 2025-03 rev), JSON-RPC 2.0, `Mcp-Method` header, `ttlMs` caching, OAuth 2.0 with iss validation per RFC 9207 (SEP-2468).
- **SDK:** Official TypeScript + Python MCP SDKs from modelcontextprotocol.io.
- **Federation:** Merge `tools/list` from N upstreams, deduplicate via prefix (github__create_issue) per Docker MCP Gateway.
- **Virtual Servers:** Compose approved tools per team (Zuplo virtual composition pattern).
- **Auth:** Bundled OAuth 2.0 authorization server (Zuplo pattern) + 4 credential models: hosted OAuth, BYO OAuth, API key proxy, none.

## Commerce - Agentic Payments

- **x402:** Coinbase CDP facilitator SDK (Go + Python), V2 launched Dec 2025, verify on-chain USDC proof on Base/Solana. Implement hybrid middleware per WorkOS (Stripe key, MPP session, x402 receipt).
- **MPP:** Stripe Machine Payments Protocol SDK (launched March 18 2026) - sessions with spending caps, allowlist payees, CloudTrail audit.
- **AP2/ACP:** Google AP2 mandates (60+ partners) for authorization audit, OpenAI ACP for checkout (future).
- **Wallet:** Crossmint or Privy for agent wallets with programmable guardrails, or self-built with Coinbase WaaS.
- **Billing:** Stripe Billing metered usage + Stripe Tax + Stripe Connect for payouts to tool builders (fiat), plus USDC payouts via Circle.

## Media & Expert AI Adapters

- **Media:** Reverse proxy to fal.ai queue (https://queue.fal.run), Replicate API (per second GPU), Together AI serverless. Implement fal.ai-compatible contract so SDK swap is base_url only.
- **STT:** Deepgram Nova-3 SDK (<300ms latency), Whisper via OpenAI compatible.
- **TTS:** ElevenLabs API ($0.30/1K chars), Cartesia (<100ms).
- **OCR:** Mistral OCR 4 API, Google Document AI fallback.
- **Translation:** DeepL, Google.
- **Moderation:** Hive AI multi-modal.

## SDK & DX

- **SDK Generation:** Speakeasy or Fern (from Top 5 AI-Powered API Dev Tools 2026) - generate typed SDKs in 10+ languages (Python, TS, Go, Java). Aligned with Stainless AI-native idiomatic code.
- **Languages:** Target OpenAI SDK compatibility first (no new SDK needed), then provide OMNI extensions SDK in Python/TS.
- **CLI:** Go + Cobra - `omni keys create`, `omni mcp publish --price 0.01`, `omni registry import`.

## Infrastructure

- **Edge:** Fly.io or Cloudflare Workers for data plane edge PoPs (300+ like Zuplo) + AWS Global Accelerator for Anycast. Multi-cloud managed dedicated so no cross-cloud egress penalties (Zuplo pattern).
- **Core:** AWS/GCP Kubernetes (EKS/GKE) for control plane + regional data plane pods.
- **IaC:** Terraform, Pulumi
- **CI/CD:** GitHub Actions + ArgoCD for GitOps (TensorZero GitOps pattern)
- **Secrets:** AWS KMS / GCP KMS + HashiCorp Vault for provider keys encryption.

## Evaluation & Eval Harness

- **Eval Framework:** FutureAGI evaluate function pattern - dataset of 100+ labeled prompts, metrics: exact match, groundedness, format validity, LLM-as-judge.
- **Judge Models:** Use Claude Sonnet 4.5 or GPT-5 mini as judge for quality parity (cost-effective).
- **Storage:** S3 for eval datasets.

## Compliance Tooling

- **SOC2:** Vanta or Drata for controls monitoring
- **PII:** Presidio
- **Audit:** Immutable logs in S3 WORM + ClickHouse + SIEM export (Datadog/Splunk)

## Why This Stack Beats Competitors

- LiteLLM Python only: we Go for perf + Python for velocity - best both.
- Portkey Node.js: Go lower latency.
- Bifrost Go but limited providers: we Go but with LiteLLM 100+ provider mapping reuse.
- Zuplo TS policies but closed: we TS policies but open-core.
- This stack is exactly what 2026 best-practice suggests: Envoy-based data plane (Envoy AI Gateway blog proves), FastAPI control plane, Qdrant semantic cache, OTel for observability.

