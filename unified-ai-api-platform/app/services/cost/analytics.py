"""Spend analytics over usage records (Stage 2).

Feeds the dashboard endpoints: aggregate summaries, per-day history and
recent records.
"""

from __future__ import annotations

import time
from datetime import UTC, datetime

from app.services.cost.tracker import CostTracker, UsageRecord


def _bucket_key(ts: float) -> str:
    return datetime.fromtimestamp(ts, UTC).date().isoformat()


class CostAnalytics:
    def __init__(self, tracker: CostTracker) -> None:
        self._tracker = tracker

    # ---------------------------------------------------------------- summary
    def summary(self, days: int = 30) -> dict:
        since = time.time() - days * 86_400
        records = self._tracker.query(since=since)
        total_spend = sum(r.cost_usd for r in records)
        total_tokens = sum(r.total_tokens for r in records)
        by_provider: dict[str, dict] = {}
        by_model: dict[str, dict] = {}
        for record in records:
            for index, key in ((by_provider, record.provider), (by_model, record.model)):
                bucket = index.setdefault(key, {"requests": 0, "tokens": 0, "spend_usd": 0.0})
                bucket["requests"] += 1
                bucket["tokens"] += record.total_tokens
                bucket["spend_usd"] += record.cost_usd
        for index in (by_provider, by_model):
            for bucket in index.values():
                bucket["spend_usd"] = round(bucket["spend_usd"], 8)

        latencies = [r.latency_ms for r in records]
        return {
            "period_days": days,
            "since": datetime.fromtimestamp(since, UTC).isoformat(),
            "requests": len(records),
            "cached_requests": sum(1 for r in records if r.cached),
            "mock_requests": sum(1 for r in records if r.mock),
            "total_tokens": total_tokens,
            "total_spend_usd": round(total_spend, 8),
            "avg_cost_per_request_usd": round(total_spend / len(records), 8) if records else 0.0,
            "avg_latency_ms": round(sum(latencies) / len(latencies), 2) if latencies else 0.0,
            "by_provider": dict(sorted(by_provider.items(), key=lambda kv: kv[1]["spend_usd"], reverse=True)),
            "by_model": dict(sorted(by_model.items(), key=lambda kv: kv[1]["spend_usd"], reverse=True)),
        }

    # ---------------------------------------------------------------- history
    def history(self, days: int = 30) -> list[dict]:
        since = time.time() - days * 86_400
        records = self._tracker.query(since=since)
        buckets: dict[str, dict] = {}
        for offset in range(days - 1, -1, -1):  # pre-fill empty days, oldest first
            day_ts = time.time() - offset * 86_400
            buckets[_bucket_key(day_ts)] = {"date": _bucket_key(day_ts), "requests": 0, "tokens": 0, "spend_usd": 0.0}
        for record in records:
            bucket = buckets.get(_bucket_key(record.ts))
            if bucket is None:
                continue
            bucket["requests"] += 1
            bucket["tokens"] += record.total_tokens
            bucket["spend_usd"] += record.cost_usd
        for bucket in buckets.values():
            bucket["spend_usd"] = round(bucket["spend_usd"], 8)
        return list(buckets.values())

    # ---------------------------------------------------------------- records
    def recent_records(self, limit: int = 50) -> list[dict]:
        return [self._record_view(r) for r in self._tracker.recent(limit)]

    @staticmethod
    def _record_view(record: UsageRecord) -> dict:
        return {
            "id": record.id,
            "ts": record.ts,
            "at": datetime.fromtimestamp(record.ts, UTC).isoformat(),
            "api_key_id": record.api_key_id,
            "provider": record.provider,
            "model": record.model,
            "strategy": record.strategy,
            "prompt_tokens": record.prompt_tokens,
            "completion_tokens": record.completion_tokens,
            "total_tokens": record.total_tokens,
            "cost_usd": record.cost_usd,
            "latency_ms": record.latency_ms,
            "cached": record.cached,
            "mock": record.mock,
        }
