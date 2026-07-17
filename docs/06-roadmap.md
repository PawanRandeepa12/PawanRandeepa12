# Roadmap - 0 to World-Class in 12 Months

## Phase 0: Foundation (Month 0-2) - The LiteLLM Compatibility MVP

**Goal:** Get 10 design partners, 46k star community low-friction migration.

- **Week 1-2: Repo & Brand**
  - Name: OMNI (or NEXUS)
  - Domain: api.omni.ai, mcp.omni.ai
  - GitHub: omni-ai/gateway (Apache 2.0)
  - Build reference implementation: Go data plane using Envoy + Gin/Fiber? Or fork Bifrost (Apache 2.0) as starting point for 11μs base.
  - FastAPI control plane mock: auth, virtual keys, provider registry

- **Week 3-4: LiteLLM Compat + OpenRouter Parity**
  - Implement `config.yaml` parser from LiteLLM (100+ providers). Reuse their LLM provider mapping code (open-source).
  - OpenAI-compatible endpoints: `/v1/chat/completions`, `/completions`, `/embeddings`
  - Features: fallback, retry, basic cost tracking (just token counts), virtual keys stored in Postgres
  - Deployment: Docker compose: gateway (Go) + Redis + Postgres + FastAPI control plane
  - Benchmark: p50 overhead <50μs (not yet 11μs, but better than Python)

- **Week 5-8: Beta Launch**
  - Landing page: Comparison table vs OpenRouter/Portkey/LiteLLM
  - Docs: Migration guide from LiteLLM (1-line change) and OpenRouter
  - SDK: `pip install omni-ai`, wrappers for OpenAI, Anthropic clones
  - Free tier: 10k req/mo (Helicone model)
  - Design partners: 10 startups spending $5k+/mo, offer free Pro for 3 months + cost savings report
  - Metrics to hit: 100 GitHub stars, 10 paying $0? Focus on feedback.

**Deliverable:** GitHub repo with 100+ provider support, OpenAI-compat, virtual keys, basic caching, demo showing fallback OpenAI->Claude.

## Phase 1: Governance & Performance (Month 2-4) - Portkey + Bifrost Parity + Beat

**Goal:** Match best governance and perf, become credible enterprise choice.

- **Month 2-3:**
  - **Perf Sprint:** Replace Python hot paths with Go, Envoy WASM filters for PII/secret masking (Higress pattern). Target <20μs p50.
  - **Security:** Prompt injection detection (regex + small model judge), PII detection (Presidio), secret masking.
  - **Token-aware rate limiting:** Implement token bucket in Redis (Lua script) counting input+output tokens (estimate via tiktoken pre-request, accurate post).
  - **Hierarchical budgets:** Org -> Team -> User -> Key, hourly/daily/monthly caps, Slack alert.

- **Month 3-4:**
  - **Observability:** OTel tracing, ClickHouse for logs, Prometheus metrics, Grafana dashboard. Per-model latency, cost, error. Portkey-level.
  - **Prompt Registry:** Versioned prompts, A/B testing (10% traffic to new prompt), fallback per prompt.
  - **Semantic Caching:** Qdrant integration, embedding via text-embedding-3-small, threshold tuning, 40% hit target demo.
  - **Compliance Start:** SOC2 Type I readiness, audit logs immutable (S3), BYOK encryption with KMS, DPA template.
  - **Hybrid Deploy:** Helm chart for self-hosted data plane, SaaS control plane. Zero data flow mode (like LiteLLM).

**Deliverable:** Benchmark blog post "We are 10x faster than LiteLLM, 2x faster than Portkey, with semantic caching saving 45%". Launch on HN. Target 500 stars.

## Phase 2: MCP Gateway (Month 4-7) - The Differentiator, Zuplo Parity + Registry

**Goal:** Own MCP narrative. Launch at time of MCP 2026-07-28 spec final.

- **Month 4-5:**
  - Implement MCP Gateway per 2026-07-28 RC: stateless, Streamable HTTP, JSON-RPC 2.0, `Mcp-Method` header, OAuth 2.0 with iss per RFC 9207, TTL caching.
  - Federation: Pull tools/list from N upstreams, merge with prefix option to avoid collisions.
  - Virtual MCP servers: Compose approved tools per team.
  - Deploy: Envoy AI Gateway pattern, `MCPGatewayExtension` CRD.

- **Month 5-6:**
  - **Managed Hosting:** Docker isolation per MCP server (Docker MCP Gateway), resource limits, image signing. Start with top 20: github, slack, postgres, filesystem, fetch, notion, linear, jira.
  - **Registry MVP:** Crawl mcp.so (17k), Glama, Smithery via APIs, store metadata: category, auth type, trust score. Curate top 200 manually with security notes.
  - **Governance:** Tool-level RBAC, approval workflow for sensitive calls (e.g., Slack delete), rate limiting per tool, audit log with MLflow (Red Hat pattern).
  - **Security:** Prompt injection scan for tool descriptions (like Invariant Labs), Snyk style advisory.

- **Month 6-7:**
  - **Registry Launch:** Public page registry.omni.ai - searchable, with docs, trust badges, one-click deploy to your virtual endpoint. SEO push.
  - **Integration with AI Gateway:** Allow chat completion to auto-call MCP tools (function calling bridge).
  - **Launch Event:** Coincide with MCP spec final July 28 2026. Blog: "The Post-MCP World: Why You Need a Gateway". Demo video: agent uses 10 tools federated.
  - **Target:** 100 MCP servers hosted, 1000 registered, 50 design partners using MCP gateway.

**Deliverable:** Only gateway that does both LLM + MCP unified (like Bifrost) but with curated registry + Docker isolation.

## Phase 3: Media & Expert AI (Month 7-9) - True Unified, Eden + fal.ai Parity

**Goal:** Become true unified, not just LLM.

- **Month 7-8:**
  - **Media Plane:** Proxy to fal.ai, Replicate, Together, Groq. Implement fal.ai-compatible API: `/v1/videos/generations`, `/v1/images/generations`, `/v1/audio/generations`. Use their SDKs under hood but one key.
  - Catalog: Start with 50 models: Flux, Veo 3, Runway Gen-3, Suno v4, Midjourney via kie.ai proxy, Llama 4, Qwen 3.
  - Caching for media? Not semantic but exact for same prompt.
  - Billing: Per image/second/video, like fal.ai credit-based, unified with token billing.

- **Month 8-9:**
  - **Expert AI:** Integrate:
    - OCR: Mistral OCR 4 ($4/1k pages), Google Document AI fallback
    - STT: Deepgram Nova-3 (<300ms), Whisper Large v4 ($0.006/min batch)
    - TTS: ElevenLabs ($0.30/1K chars), OpenAI TTS ($0.015/1K chars cheap path)
    - Translation: DeepL ($25/1M chars), Google
    - Moderation: Hive AI multi-modal, OpenAI moderation free
    - Embeddings: Embed-3, OpenAI
  - Unified endpoints: `/v1/ocr`, `/v1/transcriptions`, `/v1/translations` etc with compat layers.
  - Industry Packs: Pre-built workflows - "Legal Pack: Claude + Mistral OCR + DeepL", "Healthcare: Whisper + PHI filter + Claude".

**Deliverable:** Landing page: "One API for 500 LLM + 1000 Media + 30 Expert". Comparison vs Eden AI (21/25) but with governance.

## Phase 4: Commerce & Cost Intelligence (Month 9-11) - The Unfair Advantage

**Goal:** Launch moats that no competitor has.

- **Month 9-10: Agentic Payments**
  - Implement hybrid middleware: Stripe key, MPP session, x402 receipt.
  - x402 facilitator: Verify USDC on Base/Solana on-chain payment, per WorkOS/Coinbase spec. Use Coinbase CDP facilitator SDK.
  - MPP: Integrate Stripe MPP SDK (launched March 18 2026) - session creation, spend caps, allowlist payees, CloudTrail logs.
  - Dashboard for tool builders: Usage metrics, revenue, payout to wallet (USDC) + fiat via Stripe Connect.
  - Launch 20 curated MCP tools with pricing: e.g., `brave-search @ $0.001/search`, `github-advanced @ $0.01/call`. Dogfood.
  - Partnership: Talk to Coinbase x402 Foundation (formed July 16 2026 with Visa/Stripe/Google) - apply for grant, co-marketing.

- **Month 10-11: Cost Intelligence Engine**
  - Build eval harness: User uploads eval set (or auto-generate from prod traces via LLM-as-judge).
  - Run against candidate models, score quality vs cost Pareto.
  - Recommender service: Control plane suggests model per prompt template. Feature flag: `model: auto` with quality threshold.
  - Prompt compression: LLMLingua v2 integration.
  - Dashboard: "You saved $X this month vs all GPT-4, here's breakdown: cache 30%, DeepSeek routing 40%, compression 10%".

**Deliverable:** Blog post "Introducing the First AI Gateway with Built-in Monetization: x402 + MPP". First in category. PR via Coinbase/Stripe.

## Phase 5: Edge & Enterprise Hardening (Month 11-12) - World-Class Bar

**Goal:** Enterprise-ready, world-class company product.

- **Performance & Edge:**
  - Deploy data plane to Fly.io + Cloudflare Workers + AWS Global Accelerator: 300+ PoPs like Zuplo. Anycast.
  - Multi-cloud dedicated: GCP, AWS, Azure regions with no egress penalty design (control plane routes to nearest data plane).
  - Benchmark: Publish official benchmark vs Bifrost (11μs), vs LiteLLM (show 10x faster), vs Portkey (2x).

- **Compliance:**
  - SOC2 Type II audit start (need 3 months observation, so start month 9, aim completion by month 15 but Type I by month 12)
  - HIPAA BAA, GDPR data residency (EU data plane option), EU AI Act traceability docs.
  - SAML SSO, SCIM, RBAC, audit export to Datadog/Splunk.
  - 99.99% SLA with public status page (like Portkey claim).

- **GTM:**
  - Pricing page: Free 10k req, Pro $49/mo (10k logs like Portkey), Scale $499/mo, Enterprise custom ($5k-20k).
  - Marketplace fees: Transparent 12% provider margin + 5% MCP tax, vs OpenRouter opaque.
  - Sales: Hire first AE, target 20 enterprise logos spending $50k+/mo on inference (warm from design partners).
  - Docs: World-class docs like Stripe - interactive playground, migration guides, MCP registry SEO.

**Metrics @ Month 12:**
- 2000 GitHub stars (LiteLLM has 46k, but ambitious)
- 1000 free users, 200 Pro ($49 => ~$10k MRR), 10 Enterprise ($5k => $50k MRR) => ~$60k MRR = $720k ARR, on path to $1M ARR Year 1 goal
- 10B tokens/day routed (stretch but feasible with 2-3 large customers)
- 500 MCP servers registered, 100 hosted, 10 monetized via x402
- $50k saved for customers via cost intelligence (provable)

## Year 2 Vision (Beyond 12 months)

- **A2A Gateway:** Agent-to-Agent protocol (Google A2A) - federate agents like MCP tools, governance, billing.
- **Fine-tuning as Service:** Distill expensive prompts to cheap models automatically.
- **On-Prem Model Hosting:** Integrate vLLM, TensorRT-LLM for customers wanting to host Llama 4 themselves but via same API (like Together AI self-service H100 clusters).
- **Vertical SaaS:** Legal AI, Healthcare AI stacks with compliance packs.
- **Global Fabric:** 500+ edge, <10ms p95.

## Resource Plan

**Team of 6 for Year 1:**
- 1 CEO/PM (you)
- 2 Staff Engineers: Go/Rust data plane + Envoy (ex-Kong/Envoy background)
- 1 Senior Backend: Control plane Python/FastAPI + ClickHouse + K8s
- 1 Frontend + DX: Dashboard + docs + SDKs
- 1 DevRel/GTM: Design partners, registry curation, x402 partnerships

**Infra Cost:** ~$2k/mo initially (Fly.io, Qdrant cloud, Neon Postgres, ClickHouse Cloud), $5k/mo at 10B tokens/day (Redis, Envoy pods). Manageable.

**Key Risk Mitigation:**
- Risk: Performance target missed -> Mitigation: Start with Gin/Fiber Go, not custom Envoy early. Use Bifrost fork.
- Risk: MCP registry curation heavy -> Mitigation: Import mcp.so data, community contributions.
- Risk: x402/MPP immature -> Mitigation: Hybrid middleware supports Stripe traditional as fallback, not all-or-nothing.

This roadmap is designed for a world-class launch within 12 months, with each phase building a moat and aligning with market timing windows (MCP spec final July 2026, x402 Foundation July 2026).
