"""Rolling latency tracking per model (Stage 3).

Every successful provider call records its wall-clock latency into a bounded
per-model window. The routing engine uses these observations for the
`latency` strategy and for the latency component of the `balanced` score.
"""

from __future__ import annotations

from collections import defaultdict, deque


def percentile(values: list[float], pct: float) -> float:
    if not values:
        return 0.0
    ordered = sorted(values)
    k = (len(ordered) - 1) * pct / 100
    low = int(k)
    high = min(low + 1, len(ordered) - 1)
    if low == high:
        return ordered[low]
    return ordered[low] + (ordered[high] - ordered[low]) * (k - low)


class LatencyTracker:
    def __init__(self, window: int = 200) -> None:
        self._samples: dict[str, deque[float]] = defaultdict(lambda: deque(maxlen=window))

    def record(self, model_id: str, latency_ms: float) -> None:
        self._samples[model_id].append(latency_ms)

    def average(self, model_id: str) -> float | None:
        samples = self._samples.get(model_id)
        if not samples:
            return None
        return sum(samples) / len(samples)

    def stats(self, model_id: str | None = None) -> dict:
        ids = [model_id] if model_id else sorted(self._samples)
        out: dict[str, dict] = {}
        for mid in ids:
            samples = list(self._samples.get(mid, ()))
            if not samples:
                out[mid] = {"samples": 0}
                continue
            out[mid] = {
                "samples": len(samples),
                "avg_ms": round(sum(samples) / len(samples), 2),
                "min_ms": round(min(samples), 2),
                "max_ms": round(max(samples), 2),
                "p50_ms": round(percentile(samples, 50), 2),
                "p95_ms": round(percentile(samples, 95), 2),
            }
        return out
