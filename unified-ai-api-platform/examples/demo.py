#!/usr/bin/env python3
"""Guided tour of the Unified AI API Platform.

Usage:
    uvicorn app.main:app &          # start the platform (mock mode is fine)
    python examples/demo.py         # run this tour

No provider keys needed — every step works against mock completions.
"""

from __future__ import annotations

import os
import sys

import httpx

BASE = os.environ.get("PLATFORM_URL", "http://localhost:8000")
HEADERS = {"X-API-Key": os.environ["PLATFORM_API_KEY"]} if os.environ.get("PLATFORM_API_KEY") else {}


def show(title: str, payload, limit: int = 600) -> None:
    print(f"\n=== {title} " + "=" * max(2, 68 - len(title)))
    text = payload if isinstance(payload, str) else __import__("json").dumps(payload, indent=2)
    print(text[:limit] + ("…" if len(text) > limit else ""))


def main() -> int:
    client = httpx.Client(base_url=BASE, headers=HEADERS, timeout=30)
    try:
        client.get("/health").raise_for_status()
    except httpx.HTTPError:
        print(f"Platform not reachable at {BASE}. Start it with: uvicorn app.main:app")
        return 1

    show("Stage 1 · model catalog", client.get("/v1/models").json()["data"][:2])

    # Stage 1.5 — a first completion through the unified API
    body = {"model": "openai/gpt-4o-mini", "messages": [{"role": "user", "content": "Say something cheerful."}]}
    resp = client.post("/v1/chat/completions", json=body)
    show("Stage 1 · unified completion", resp.json())

    # Stage 2 — cost estimation & comparison before spending
    body = {"messages": [{"role": "user", "content": "Compare vector databases for me."}]}
    compare = client.post("/v1/cost/compare", json=body).json()
    show("Stage 2 · cost comparison (cheapest first)", compare["estimates"][:3])
    print(f"           -> cheapest: {compare['cheapest']}")

    # Stage 3 — routing dry run + a cost-routed request
    route = client.post("/v1/routing/route", json={**body, "model": "auto", "strategy": "cost"}).json()
    show("Stage 3 · route dry run", route["candidates"][:3])

    resp = client.post("/v1/chat/completions", json={**body, "model": "auto", "strategy": "cost"}).json()
    print(f"           -> routed to {resp['model']} | cost ${resp['cost']['total_cost_usd']:.8f} | "
          f"fallbacks: {resp['routing']['fallbacks_used']}")

    # Stage 4 — cache + parallel comparison + metrics
    first = client.post("/v1/chat/completions", json=body | {"model": "openai/gpt-4o-mini"})
    second = client.post("/v1/chat/completions", json=body | {"model": "openai/gpt-4o-mini"})
    print(f"\n=== Stage 4 · cache ===  1st: {first.headers.get('x-cache')}  2nd: {second.headers.get('x-cache')} "
          f"(cached={second.json()['cached']})")

    race = client.post("/v1/chat/compare", json=body).json()
    print("=== Stage 4 · parallel comparison ===")
    for item in race["results"]:
        print(f"  {item['model']:45s} {item['latency_ms']:8.1f} ms   ${item['cost']['total_cost_usd']:.8f}")
    print(f"           -> fastest: {race['fastest']} | cheapest: {race['cheapest']}")

    summary = client.get("/v1/cost/summary?days=1").json()
    show("Analytics · today's spend", {k: summary[k] for k in ("requests", "total_spend_usd", "by_provider")})
    print(f"\nDone. Metrics at {BASE}/metrics — admin overview at /v1/admin/stats")
    return 0


if __name__ == "__main__":
    sys.exit(main())
