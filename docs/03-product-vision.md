# Product Vision - OMNI Unified AI API Platform

## North Star

**One API to rule them all.**

Developers should need exactly ONE integration to access ANY AI capability now and in the future, with enterprise governance, cost controls, and a path to monetize their own tools.

## Product Principles (World-Class Bar)

1.  **Compatibility over Novelty:** Be OpenAI-compatible for LLM, fal.ai-compatible for media, MCP spec 2026-07-28 compliant for tools. No new SDK to learn for basic use. Drop-in replacement for OpenRouter, LiteLLM, fal.ai.
2.  **Perf is a Feature:** Target 11-15μs overhead (Bifrost benchmark), <20ms p95 routing, sub-100ms TTFT when paired with Groq. Built on Go/Rust data plane (not Python) like Envoy Proxy + Envoy AI Gateway. Envoy is trusted by thousands of enterprises - we leverage same battle-tested base.
3.  **Governance by Default:** Every request logged, traced (OpenTelemetry), PII-masked, budget-checked, policy-enforced. EU AI Act ready from day 1. Unlike OpenRouter's limited governance.
4.  **Open-Core Moat:** Apache 2.0 gateway core (like LiteLLM, Bifrost) for trust + adoption, managed control plane with proprietary cost intelligence, registry, commerce.
5.  **Agentic-Native:** Not just LLM API - first gateway built for agents that use tools (MCP), talk to agents (A2A), and pay each other (x402/MPP).

## Architecture Pillars

### Pillar 1: The Unified AI Gateway (The Data Plane)

Single endpoint: `https://api.omni.ai/v1`

| Endpoint | Compatibility | Example |
|----------|---------------|---------|
| `/chat/completions` | OpenAI | GPT-5, Claude Sonnet 4.5, Gemini 2.5 Pro, DeepSeek V3 |
| `/completions` | OpenAI | Legacy |
| `/embeddings` | OpenAI | text-embedding-3, Embed-3 |
| `/images/generations` | OpenAI + fal.ai | Flux, Midjourney via kie.ai, GPT Image 2 |
| `/videos/generations` | fal.ai | Veo, Runway, Pollo, Suno Video |
| `/audio/speech` | OpenAI + ElevenLabs | TTS |
| `/audio/transcriptions` | OpenAI + Deepgram | Whisper Large v4, Nova-3 (<300ms latency) |
| `/moderations` | OpenAI + Hive | Multi-modal moderation |
| `/ocr` | Mistral OCR compat | 170 languages, bounding boxes |
| `/translate` | DeepL/Google compat | 130+ languages |
| `/rerank` | Cohere compat | Command R rerank |

Features:
- **Intelligent Routing:** Weighted, fallback (if OpenAI down -> Anthropic -> Google), latency-based, cost-based, quality-based (eval score).
- **Semantic Caching:** 40% hit rate target (Maxim AI benchmark) with vector similarity >0.95, reduces spend 30-50%.
- **Token Budgeting:** Hierarchical (org -> team -> user -> key), token-aware rate limiting (not just req/min - critical because one req can be 50 to 50k tokens).
- **Virtual Keys:** Like LiteLLM/Portkey - per customer key that maps to policies, budgets, provider keys hidden.

### Pillar 2: MCP Gateway (The Tool Plane)

Endpoint: `https://mcp.omni.ai`

- **Federation:** Aggregate 100s of MCP servers behind one spec-compliant URL (per Zuplo model). Agents see unified `tools/list`.
- **Virtual MCP Servers:** Compose approved tools from multiple upstream servers into one virtual server per agent/team (Zuplo pattern).
- **Registry:** Curated marketplace - start with top 200 from mcp.so/Glama/Smithery + enterprise connectors (Slack, GitHub, Postgres, Salesforce, Notion, Jira).
- **Governance:**
  - OAuth 2.0 authorization server bundled (MCP spec 2026-07-28 requires iss validation per RFC 9207).
  - 4 credential models (like Zuplo): hosted OAuth, BYO OAuth, API key proxy, no-auth.
  - Tool-level RBAC: `github:delete_repo` requires approval, `filesystem:read` auto-allow.
  - Rate limiting: prevent runaway loops (agent calls search 10k times).
  - Audit: MLflow + OTel tracing, log LLM reasoning -> tool call chain for EU AI Act.
  - Security: Prompt injection detection via tool descriptions, PII/secret masking (Akamai AI Firewall integration pattern), isolation via container (Docker MCP pattern).

Stateless core: Leverage MCP 2026-07-28 RC which is stateless - plain round-robin LB, `Mcp-Method` header routing, `ttlMs` caching.

### Pillar 3: Commerce Gateway (The Money Plane) - Differentiator

Endpoint: Handles 402 Payment Required natively.

Hybrid middleware pattern (from WorkOS research):

```python
# Three verification paths in one middleware
# Path 1: Stripe subscription (human)
# Path 2: MPP session (high-freq agent)
# Path 3: x402 receipt (autonomous one-off)
# Path 4: No cred -> return 402 with payment options
```

- **x402 Protocol:** USDC on Base/Solana, 5 lines server code via `x402-mcp` (Vercel). Instant. For $0.001-$0.10 microtransactions.
- **MPP Protocol:** Stripe Machine Payments Protocol - sessions with spending limits, fiat + stablecoin, for $10-$500 purchases. Merchant acceptance + dispute handling.
- **AP2 + ACP:** Support Google AP2 mandates for authorization audit + OpenAI ACP for checkout (future).
- **Dashboard:** For tool builders - usage, revenue, tax compliance via Stripe.

This enables:
- Tool builder publishes MCP server, sets price $0.01/call, we handle metering + settlement + USDC payout.
- Enterprise agent uses same tool, pays via existing Stripe subscription, CFO sees invoice.

First to do this: Apify proved demand (20k tools), but they are marketplace. We are infrastructure (gateway).

### Pillar 4: Control Plane (The Brain)

- **Cost Intelligence Engine:** 
  - Model recommender: "This classification task can use GPT-5 nano ($0.05/M) vs GPT-5 ($1.25/M) with 97% parity - tested on your eval set of 100 examples via LLM-as-judge" (FutureAGI pattern).
  - Prompt compressor, cache optimizer.
- **Observability 2.0:** Helicone-level tracing + FutureAGI eval - cost per trace, latency per provider, format validity, groundedness, custom judge.
- **Prompt Registry:** Portkey-style versioning, A/B testing, automatic failover per prompt.
- **Policy Engine:** TypeScript programmable policies (Zuplo DX) - custom rate limiting functions, e.g., `if user.tier == 'free' && tokens > 1000 then deny`.

### Pillar 5: Deployment Models

1.  **Managed Cloud:** Fully managed, 300+ edge (Cloudflare model but multi-cloud), SOC2/HIPAA, 99.99% SLA (Portkey claim). Best for startups.
2.  **Self-Hosted Data Plane:** Open-source Go binary, deploy in VPC, control plane still managed for cost analytics (LiteLLM zero data flow option). ~$2k/mo infra target, BYOK, PostgreSQL+Redis optional (we offer SQLite mode for small teams to cut cost).
3.  **Full Self-Hosted:** Apache 2.0, Helm chart, for banks/air-gapped. Enterprise license.

## User Journey: World-Class DX

```bash
# 1. Install (30 seconds)
pip install omni-ai  # or npm install @omni-ai/sdk

# 2. One key
export OMNI_API_KEY="omni_..."

# 3. Use anywhere - OpenAI compat
from openai import OpenAI
client = OpenAI(base_url="https://api.omni.ai/v1", api_key="omni_...")
client.chat.completions.create(model="claude-sonnet-4.5", messages=[...]) # Works
client.chat.completions.create(model="deepseek-v3", ...) # Auto cheaper
client.images.generate(model="flux-pro", ...) # Media
client.audio.transcriptions.create(model="deepgram-nova-3", ...) # Expert

# 4. MCP tools governed
# mcp.json points to https://mcp.omni.ai/v1/team -- all tools federated + governed

# 5. Monetize your tool
# omni mcp publish --price 0.01 --currency USDC
```

No need to learn new SDK. Migration from OpenRouter/LiteLLM is 1-line base_url change.

## Enterprise Readiness Checklist (World-Class Bar)

- [ ] SOC 2 Type II (like Together AI, Fireworks)
- [ ] HIPAA (for healthcare STT/medical)
- [ ] GDPR + EU AI Act logs (traceability of AI decisions)
- [ ] PII detection + masking + DLP via token scan
- [ ] Prompt injection detection (Zuplo policy)
- [ ] Secret masking (redact API keys in responses)
- [ ] BYOK encryption: provider keys encrypted with KMS, never logged
- [ ] Regional data residency (Azure OpenAI pattern)
- [ ] 99.99% uptime SLA with status page + failover validation
- [ ] SAML SSO + SCIM provisioning
- [ ] Audit log export to SIEM (Datadog, Splunk)

## What Makes it World-Class vs Clone?

| Clone (e.g., OpenRouter fork) | World-Class (OMNI) |
|-------------------------------|--------------------|
| 500 models, simple proxy | 500+ models + 1000+ media + 500+ expert + 1000+ MCP tools |
| Request routing only | ML-based quality/cost/latency routing with eval harness |
| No caching or basic exact | Semantic cache + prompt compression + token saving 40%+ |
| No MCP or basic list | Full MCP gateway with virtual servers, OAuth, audit, container isolation |
| No monetization | x402 + MPP native commerce |
| Single cloud | Multi-cloud + edge + self-hosted VPC |
| No compliance | SOC2/HIPAA/EU AI Act day-1 |

This is the product narrative: **We don't just proxy AI, we operationalize it.**
