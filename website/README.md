# OMNI Website

World-class landing page for Unified AI API Platform.

## Run Locally

```bash
cd website
python -m http.server 3000
# open http://localhost:3000
```

Or just open `index.html` directly — uses Tailwind CDN, no build needed.

## Stack

- Pure HTML + Tailwind CDN (no build) — like Stripe/Linear style
- Geist font (Vercel)
- No framework — fast, portable, world-class design with glassmorphism, gradients, grid-bg
- Interactive tabs (LLM/Media/MCP/x402), mock MCP registry search, benchmark chart, pricing
- Assets: `assets/logo.png`, `assets/hero-abstract.png` generated via AI

## Sections

- Hero: metric $12.5B, 40% cache, 11μs
- 3 Gateways (AI, MCP, Commerce x402+MPP)
- Cost Intelligence Engine Pareto chart
- MCP Registry npm-style (17k federated)
- Perf benchmark vs LiteLLM/Portkey/Bifrost
- Compare table vs OpenRouter/Portkey/LiteLLM/Zuplo
- Pricing Free/Pro/Scale/Enterprise transparent 8-15% + 5% MCP tax
- Roadmap to $1M ARR

## Deploy

- Vercel: `vercel --prod` (just static)
- Cloudflare Pages: drag `website/` folder
- Or copy to `platform/` gateway to serve at `/` route

## To make production Next.js:

If you want, we can convert to Next.js 14 app dir with `app/page.tsx` same content. For now static is fastest to validate.

