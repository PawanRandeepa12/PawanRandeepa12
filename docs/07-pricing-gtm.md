# Pricing & Go-To-Market Strategy

## Pricing Philosophy: Transparent + Cost-Saving Justified

Lessons from competitors:
- OpenRouter hides margin -> distrust
- Requesty 5% markup transparent -> trust, lower score but beloved
- Portkey $49/mo entry anchors Pro
- Helicone 10k free req table stakes
- LiteLLM self-host $2k+/mo fixed -> our managed at $500-2k/mo is justified

Our Model: **Seat + Consumption + Value-Share Hybrid**

| Tier | Price | Includes | Target | Overages |
|------|-------|----------|--------|----------|
| **Free** | $0 | 10k req/mo, 1M tokens, 100 media gens, 1 team member, community support, 100 MCP calls | Solo devs, evals | $0.002/1k tokens after? Actually soft cap, throttle |
| **Pro** | $49/mo | 100k req/mo, 10M tokens, 1000 media gens, 5 team members, semantic cache, budget caps, virtual keys, email support, 10k MCP calls | Series A startups | $8 per 1M extra tokens pool? Actually transparent provider cost + 12% |
| **Scale** | $499/mo | 1M req/mo, 100M tokens, 10k media, 25 team members, SSO, audit logs 30 days, custom rate limits, TS policies, dedicated Slack, 100k MCP calls | Scaleups spending $10k+ | Provider cost + 10% (volume discount) |
| **Enterprise** | $5k-20k/mo custom | Unlimited, 99.99% SLA, SOC2/HIPAA, data residency, self-hosted data plane option, audit to SIEM, 1yr log retention, BAA/DPA, premium support + TAM, A2A gateway | Enterprises | Provider cost + 8% + seat |

**Provider Cost Handling:** Transparent pass-through + platform fee, as TokenMix/AI/ML API model:

- LLM: Provider cost (e.g., DeepSeek $0.28/M) + platform fee (Free 15%, Pro 12%, Scale 10%, Enterprise 8%). Show in dashboard: "Provider $0.28/M + Platform $0.033 = $0.313/M"
- Media: Same: fal.ai cost + platform fee (but media margins higher, maybe 15% flat)
- Expert: Same
- MCP Tools: Tool builder sets price (e.g., $0.01/call), we take 5% tax + Stripe fees. Builder gets 95% (like App Store 30% but 5% to encourage supply). For free tools, 0%.

**Why this beats competitors:**
- More transparent than OpenRouter (hidden spread)
- Cheaper than self-hosting LiteLLM at scale ($2k fixed vs our $499 + usage where usage < $2k for most)
- Savings narrative: "We charge 12% but save you 50% via cache + routing = net -38%"

## Billing Mechanics

- **Metering:** ClickHouse aggregates: tokens (input/output cached), media seconds/images, MCP tool calls, x402 payments
- **Stripe Integration:** For fiat, metered billing (Stripe Billing metered usage), subscription management, invoicing, tax (Stripe Tax)
- **x402 Facilitator:** For USDC crypto, on-chain verification, payout via Coinbase CDP or Circle. Show dual balance: Fiat + Crypto.
- **MPP Sessions:** Pre-auth spending limits, debit per call, session token returned, Stripe handles settlement

**Example Invoice (Scale customer):**
```
OMNI Scale Base: $499
LLM Usage: 250M tokens @ avg $1.50/M provider + 10% = $412.50
  - Cache saved: 100M tokens (40% hit) = $150 saved
  - Routing saved: Used DeepSeek for 50M tokens vs GPT-5, saved $48.60
Media: 5k images @ $0.02 avg + 10% = $110
MCP Tools: 50k calls @ avg $0.002 + 5% tax = $105
Total: $1126.50
You Saved: $198.60 vs provider direct + no governance
```

Show savings prominently.

## Go-To-Market Motions

### Motion 1: Bottoms-Up Developer (LiteLLM/OpenRouter steal)

**Channel:** GitHub, Hacker News, Twitter/X, Reddit r/LocalLLaMA, Discord

**Tactics:**
- **GitHub:** Open-core gateway Apache 2.0, README with benchmark table vs LiteLLM/Portkey/Bifrost, migration guide `s/base_url=openrouter.ai/api/v1/base_url=api.omni.ai\/v1`. Target 100 stars Month 1, 1000 by Month 6 (LiteLLM has 46k, ambitious but doable with HN launches)
- **Launch Posts:** Each phase launch on HN: Phase 0 "Show HN: Open-source alternative to OpenRouter with Go perf", Phase 2 "Show HN: MCP Registry with 500 servers + x402 payments", Phase 4 "We saved $10k for startups with semantic cache"
- **Content:** Technical deep dives - "How we hit 15μs overhead", "MCP stateless 2026-07-28 spec explained", "x402 vs MPP benchmark". From our research: devs love architecture posts (Envoy AI Gateway blog).
- **DX:** One-line install, 30-sec quickstart, interactive playground like OpenRouter chat, TypeScript/Python/Go SDKs (generate via Speakeasy/Stainless pattern from Top 5 AI-Powered API Dev Tools 2026)

**Conversion:** Free -> Pro when hit 10k req limit or need team features. Use Product-Led Growth: dashboard shows "Upgrade to Pro for $49 to get virtual keys + budget caps (you've been rate-limited 3x this week)".

### Motion 2: Design Partner / Community (For MCP Registry)

**Channel:** MCP community Discord, Smithery, mcp.so, Glama, Anthropic Cookbook, Vercel AI SDK Discord

**Tactics:**
- Partner with top 20 MCP builder tools: offer free hosted + monetization via x402, take 0% fee first 3 months.
- Provide `omni mcp publish` CLI that publishes to all registries (mcp.so + Glama + Smithery + OMNI) at once - be distribution layer.
- Host MCP Builder Office Hours weekly.
- Co-marketing with Coinbase x402 Foundation: Apply for ecosystem grant, joint webinar "How to monetize your MCP server with x402"

**Conversion:** Builders bring their users (agents using their tools) to our gateway = free acquisition. Network effect.

### Motion 3: Top-Down Enterprise (Portkey/Kong/Zuplo win)

**Channel:** Vercel partnership, AWS Marketplace, Azure Marketplace, VC intro to portfolio, LinkedIn, conferences (KubeCon, AI Engineer Summit)

**Tactics:**
- **Vercel Integration:** Vercel AI Gateway is <20ms but limited to Vercel ecosystem. Offer Vercel marketplace integration: "Deploy OMNI gateway in 1 click in Vercel". Co-sell.
- **Enterprise Features Checklist Page:** Public comparison table vs Portkey vs Kong vs Zuplo vs Cloudflare on SOC2, HIPAA, PII, audit, MCP, performance. Enterprises love checklists.
- **Compliance First Blog:** "How to pass EU AI Act with an AI Gateway" - detailed guide with logs, traceability.
- **Case Studies:** Even with 2-3 enterprise design partners, publish case study: "How Acme saved $50k/mo and passed SOC2 with OMNI self-hosted data plane"
- **AWS/Azure Marketplace:** List as private offer, with BYOK docs, self-hosted Helm chart. Ease procurement.
- **Pricing:** Custom, but anchor $5k/mo minimum for enterprise, with TAM.

### Motion 4: Media / Creative Vertical (fal.ai steal)

**Channel:** Twitter creative AI community, Discord (Midjourney), Runway, Replicate, Product Hunt

**Tactics:**
- Launch on Product Hunt: "fal.ai + OpenRouter unified + one bill"
- Creator Pack: Bundle 100 image gens + 10 videos + 1000 audio minutes for $19 starter pack (like fal.ai credit but with LLM).
- Integration with ComfyUI, Automatic1111 as plugin: Use OMNI API key instead of multiple.

## Sales Playbook

**ICP Qualification (Ideal Customer Profile):**
- Spending $10k+/mo on AI (found via Clearbit enrichment of Free signups who hit limits)
- Using 2+ providers (ask in onboarding: which providers you use?)
- Team size 5+ (needs SSO/budget)
- Agent use case (MCP interest)

**Discovery Questions:**
1.  How much are you spending on AI infra monthly? (If >$10k, enterprise motion)
2.  How do you handle fallback when OpenAI is down? (If manual, pain)
3.  How do you prevent agent runaway cost loops? (If no budget caps, pain)
4.  Are you using MCP? How do you govern tools? (If direct connections, pain + EU AI Act risk)
5.  Are you using multiple APIs for LLM vs media? (If yes, catalog pain)

**Demo Flow (15 min):**
- 0-2m: One SDK for everything (show OpenAI + fal.ai compat)
- 2-5m: Fallback + cache demo - kill primary provider, show auto-failover <50ms, cache hit saving
- 5-8m: MCP gateway - federate 3 tools, show virtual server, RBAC, audit log
- 8-11m: Cost intelligence - show eval harness, Pareto frontier, "auto" router picking DeepSeek saving 80%
- 11-13m: Commerce - publish MCP tool with $0.01 price, pay via x402 live
- 13-15m: Enterprise - self-hosted data plane Helm install 1 command, audit export

**Objection Handling:**
- "We already use LiteLLM" -> "We are LiteLLM compat, drop-in, but Go perf 10x, plus managed + MCP + media, no need to run Postgres"
- "We use OpenRouter, it's simple" -> "OpenRouter limited governance, no MCP, no cache, no cost savings report, we transparent fee but net cheaper"
- "We built in-house gateway on AWS API GW" -> "That's $10k+ eng time + maintenance, plus no MCP + no semantic cache. Our self-hosted data plane gives you same data sovereignty but with features"
- "x402 is immature" -> "We support hybrid, Stripe traditional fallback, you can start with Stripe today, add x402 when ready. Same middleware"

## Metrics & KPIs

**North Star:** Token volume + MCP tool calls (usage)

**Leading:**
- GitHub stars, Discord members, registry servers count
- Free -> Pro conversion % (target 5-10% PLG standard)
- Cache hit rate, cost saved per customer (provable value)

**Lagging:**
- MRR, ARPU, Net Revenue Retention (should be >120% as usage grows)
- Gross Margin: Target 70%+ (provider cost is COGS, platform fee margin)
- Churn: <5% monthly for Pro

**Year 1 Targets (from Roadmap):**
- $1M ARR = ~$83k MRR = 200 Pro ($10k) + 10 Enterprise ($50k) + usage fees ($23k)
- Actually $720k ARR per roadmap, but push to $1M with usage overages

## Partnership Strategy

1.  **Coinbase x402 Foundation:** Ecosystem grant + co-marketing, list as official facilitator
2.  **Stripe:** MPP early partner, joint case study on agentic payments
3.  **Vercel:** AI Gateway integration, listing in Vercel Marketplace, co-sell to Next.js users
4.  **Cloudflare:** Use their edge network, maybe Workers AI integration
5.  **Envoy / Tetrate / Solo.io:** Envoy AI Gateway is open, collaborate, contribute MCP filters upstream
6.  **Security:** Snyk/Invariant Labs for MCP scanning, Akamai for AI Firewall
7.  **Model Providers:** Groq for perf showcase, Together AI for open models, fal.ai for media - revenue share deals

## Brand & Positioning

**Name Options:**
- OMNI - one for all (preferred, short, omni.ai might be taken but omni-ai.com)
- NEXUS - connection point
- CONDUIT - conduit for AI

**Tagline:** "Any Model. Any Modality. Any Tool. One API."

**Messaging Pillars:**
1.  For Developers: "Drop-in replacement for OpenRouter/LiteLLM/fal.ai with superpowers"
2.  For CTOs: "Cut AI spend 50% with governed, auditable gateway"
3.  For Platform Teams: "SOC2/HIPAA/EU AI Act ready, self-hosted data plane, 99.99% SLA"
4.  For MCP Builders: "Monetize your MCP server in 5 lines with x402 + Stripe"

**Visual:** Dark mode, terminal-first, like Vercel/PlanetScale. Show code snippets > marketing speak.

This pricing/GTM can compete: Free tier matches Helicone/Portkey, Pro $49 matches, but transparent fees + cost savings story + MCP registry + x402 is unique. No one else bundles all.
