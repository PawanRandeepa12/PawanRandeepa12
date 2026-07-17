# OMNI — Unified AI API Platform
### Any Model. Any Modality. Any Tool. One API.

> World-class company product spec + reference implementation. Built to compete with OpenRouter, Portkey, LiteLLM, Zuplo, Bifrost, fal.ai in July 2026.

[![Apache 2.0](https://img.shields.io/badge/license-Apache%202.0-blue)]()
[![MCP Spec](https://img.shields.io/badge/MCP-2026--07--28%20RC-brightgreen)]()
[![x402](https://img.shields.io/badge/x402-V2%20%2B%20MPP%20Ready-purple)]()

**OMNI is not another LLM proxy. It's the Stripe for AI: 3 gateways in one control plane.**

1.  **AI Gateway** (11µs target, like Bifrost): 500+ LLMs, 1000+ media models, 30+ expert AI via one OpenAI-compatible API
2.  **MCP Gateway** (Zuplo-class): Federate 17k+ MCP servers behind one spec-compliant endpoint with OAuth, RBAC, audit, container isolation
3.  **Commerce Gateway** (World's First): Native x402 (Coinbase) + Stripe MPP monetization for MCP tools - 5 lines to charge $0.01/call in USDC

---

## 🚀 Why OMNI Wins in 2026

**Market:** $12.5B foundation API spend in 2025, tripled enterprise spend, yet broken:

| Pain | Broken Today | OMNI Fix |
|------|--------------|----------|
| **Fragmented** | OpenRouter = only LLM, fal.ai = only media, Deepgram = only STT = 3 bills | **One API:** OpenAI compat + fal.ai compat + ElevenLabs compat, one key/bill |
| **Runaway Costs** | No cache, pay full price, 18x spread (DeepSeek $0.28 vs Claude $5) | **Cost Intelligence:** 40% semantic cache hit, ML router saves 50% auto (Maxim benchmark) |
| **Shadow IT Tools** | Agents直接连 MCP servers, no auth, no audit, EU AI Act risk | **Governed MCP:** OAuth 2.0 (RFC 9207 iss), virtual servers, PII masking, MLflow traces |
| **No Monetization** | MCP builders have 0 revenue, Apify manually enabled 20k tools with x402 | **x402 Native:** 5-line pricing, USDC on Base + Stripe MPP sessions, dashboard |

Gartner: **75% of API Gateway vendors will have MCP by end 2026** — window is now.

## 📊 Competitive Landscape (July 2026 Research)

We benchmarked 10 platforms (see `docs/02-competitive-landscape.md`):

- **OpenRouter** 500+ models: limited governance, no MCP, no media
- **Eden AI** 500+ (21/25 best score): multimodal yes, governance basic
- **Portkey** 160+ (Gartner Cool Vendor, 10B+ tokens/day claim): strong governance, 250+ models, MCP GA, but no media
- **LiteLLM** 100+ (46k stars, 295M downloads): open-source Python, $2k/mo infra, perf challenge
- **Bifrost** (Go, 11μs overhead best): perf king, LLM+MCP unified, Apache 2.0, but only 12+ providers
- **Zuplo** (Best MCP): 3 gateway types, TS policies, OAuth bundled, 300+ edge, sub-20s deploy, but closed + no media + no cost AI
- **Cloudflare AI Gateway**: free edge caching, but Cloudflare-locked

**OMNI Position:** *OpenRouter catalog + Portkey governance + Bifrost perf + Zuplo MCP + fal.ai media + Stripe/x402 billing*

## 🏗️ Architecture (See `docs/04-architecture.md`)

```
Control Plane (Python + TS): Dashboard, Policy Engine, Cost AI, Registry, Billing
         ↕ mTLS gRPC
Data Plane (Go/Rust Envoy): AI GW + MCP GW + Media Plane + Commerce GW (x402+MPP)
         ↕ 
Upstreams: OpenAI, Anthropic, Gemini, DeepSeek, Groq, Together, fal.ai, Deepgram, 17k MCP
```

**Perf Target:** <15μs p50, <20ms p95 routing, 5k RPS per Envoy pod, Redis + Qdrant semantic cache.

**Deployment:** 
- SaaS: 300+ edge (like Zuplo), 99.99% SLA
- Hybrid: Self-hosted data plane Go binary in VPC + SaaS control plane (zero data flow like LiteLLM)
- Air-gapped: Full Helm chart

## 📦 Quick Start (30 seconds)

```bash
# 1. Clone open-core gateway
git clone https://github.com/PawanRandeepa12/PawanRandeepa12
cd PawanRandeepa12/platform/gateway
go run main.go --config config.yaml # LiteLLM compat config.yaml

# 2. Use one SDK for everything - OpenAI compat
export OMNI_API_KEY=omni_xxx
python

from openai import OpenAI
client = OpenAI(base_url="https://api.omni.ai/v1", api_key="omni_xxx")

# LLM - auto router picks cheapest meeting quality
client.chat.completions.create(model="auto", messages=[...], 
  extra_body={"model_preferences": {"cost": 0.7, "quality": 0.3}})

# Media - fal.ai compat
client.images.generate(model="flux-pro", prompt="cyberpunk city")
client.audio.speech.create(model="elevenlabs-tts", input="hello")

# 3. MCP - one endpoint federates all tools
# mcp.json
{
  "mcpServers": {
    "omni": { "url": "https://mcp.omni.ai/v1/your-team", "headers": {"Authorization": "Bearer omni_xxx"} }
  }
}
# Agent sees unified tools/list with RBAC + audit

# 4. Monetize your MCP tool (5 lines with x402-mcp)
from omni_commerce import paidTool
@paidTool(price=0.01, currency="USDC", chain="base")
def search(query: str): ...
```

## 💰 Pricing (Transparent)

| Tier | Price | What you get |
|------|-------|--------------|
| Free | $0 | 10k req/mo, 1 team member, community |
| Pro | $49/mo | 100k req, 5 members, cache, virtual keys (like Portkey) |
| Scale | $499/mo | 1M req, SSO, audit, TS policies |
| Enterprise | $5k+ | 99.99% SLA, SOC2/HIPAA, self-hosted data plane, SIEM export |

Provider cost + platform fee: 15% Free, 12% Pro, 10% Scale, 8% Ent (transparent like Requesty 5% vs OpenRouter hidden).
MCP tax: 5% on tool payments (new revenue stream).

**Savings pitch:** "We charge 12% but save 50% via cache + DeepSeek routing = net -38%"

## 🗺️ Roadmap (12 Months to World-Class)

- **M0-2:** LiteLLM compat MVP, 100+ providers, Go data plane, 10 design partners
- **M2-4:** Governance + perf: <20μs, token budgets, semantic cache (Bifrost parity)
- **M4-7:** MCP Gateway: stateless 2026-07-28 RC, federation, 500 servers registry, launch with spec final July 28 2026
- **M7-9:** Media + Expert: 50 models (Veo, Flux, Suno), OCR/STT/TTS packs
- **M9-11:** Commerce + Cost AI: x402 + MPP hybrid middleware, eval harness Pareto
- **M11-12:** Edge 300+ PoPs, SOC2 Type I, 99.99% SLA, $1M ARR target

Full roadmap: `docs/06-roadmap.md`

## 📚 Docs

- `docs/00-executive-summary.md` — One-pager for investors
- `docs/01-market-analysis.md` — $12.5B market, segmentation, personas
- `docs/02-competitive-landscape.md` — 10-player teardown with score table
- `docs/03-product-vision.md` — 3 gateways, principles, enterprise checklist
- `docs/04-architecture.md` — Envoy Go data plane, FastAPI control plane, deployment topologies
- `docs/05-competitive-moat.md` — 5 moats: catalog, MCP registry, cost AI, perf, compliance
- `docs/06-roadmap.md` — 0→world-class in 12 months
- `docs/07-pricing-gtm.md` — Pricing + 4 GTM motions + partnership with Coinbase x402 Foundation

Plus reference code in `/platform`

## 🛡️ Enterprise Readiness

- [x] BYOK encrypted KMS, virtual keys
- [x] Token-aware rate limiting, hierarchical budgets
- [x] PII detection, secret masking, prompt injection (Zuplo pattern)
- [x] OpenTelemetry, MLflow traceability for EU AI Act
- [ ] SOC2 Type II (roadmap M9-15)
- [ ] HIPAA BAA, GDPR residency
- [ ] SAML SSO + SCIM (Scale tier)
- [ ] Self-hosted data plane Helm

## 🤝 Why This Repo?

This repo started as Arduino line-follower, now transformed into world-class AI company product spec. It's designed to be:

- **Investor-ready:** Market size, moat, roadmap, metrics
- **Engineer-ready:** Architecture with Envoy/Go perf targets, MCP 2026-07-28 RC implementation plan, x402 hybrid middleware code
- **GTM-ready:** Pricing that beats LiteLLM infra cost, 4 motions, brand narrative

## 🔥 The Unfair Advantage

- **LiteLLM compat layer:** Instant migration for 46k star community
- **MCP importer:** `omni registry import --source mcp.so` — be distribution layer
- **x402-first:** Stripe + Coinbase integrated Feb 2026, Foundation July 2026 - first gateway with native commerce
- **Cost Intelligence:** Not just routing, ML-based savings dashboard - CFO loved

**The window:** MCP spec final July 28 2026 + x402 Foundation July 16 2026 = 12-month rush to become default MCP gateway before Kong/Cloudflare consolidate similar to API GW consolidation.

---

**License:** Apache 2.0 data plane, proprietary control plane (open-core like Portkey/LiteLLM/Bifrost).

**Built with passion by PawanRandeepa12 - let's make this world-class.**

> Want to build? Start with `platform/gateway/main.go` and `docs/03-product-vision.md`

