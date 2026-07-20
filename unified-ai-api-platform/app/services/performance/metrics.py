"""Performance monitoring & metrics (Stage 4).

A small dependency-free metrics registry that renders Prometheus exposition
format at GET /metrics, plus a JSON snapshot for the admin stats endpoint.
"""

from __future__ import annotations

DEFAULT_BUCKETS = (0.005, 0.01, 0.025, 0.05, 0.1, 0.25, 0.5, 1.0, 2.5, 5.0, 10.0)


def _label_key(labels: dict) -> tuple:
    return tuple(sorted(labels.items()))


def _escape(value: object) -> str:
    return str(value).replace("\\", "\\\\").replace('"', '\\"')


def _render_labels(key: tuple) -> str:
    if not key:
        return ""
    inner = ",".join(f'{k}="{_escape(v)}"' for k, v in key)
    return "{" + inner + "}"


class Counter:
    def __init__(self, name: str, description: str) -> None:
        self.name = name
        self.description = description
        self._values: dict[tuple, float] = {}

    def inc(self, amount: float = 1.0, **labels) -> None:
        key = _label_key(labels)
        self._values[key] = self._values.get(key, 0.0) + amount

    def snapshot(self) -> dict:
        return {str(dict(key)): value for key, value in self._values.items()}

    def render(self) -> str:
        lines = [f"# HELP {self.name} {self.description}", f"# TYPE {self.name} counter"]
        for key, value in sorted(self._values.items()):
            lines.append(f"{self.name}{_render_labels(key)} {value:g}")
        return "\n".join(lines)


class Histogram:
    def __init__(self, name: str, description: str, buckets: tuple = DEFAULT_BUCKETS) -> None:
        self.name = name
        self.description = description
        self.buckets = buckets
        self._counts: dict[tuple, list[int]] = {}
        self._sums: dict[tuple, float] = {}
        self._totals: dict[tuple, int] = {}

    def observe(self, value: float, **labels) -> None:
        key = _label_key(labels)
        counts = self._counts.setdefault(key, [0] * len(self.buckets))
        for i, bound in enumerate(self.buckets):
            if value <= bound:
                counts[i] += 1  # cumulative buckets (Prometheus convention)
        self._sums[key] = self._sums.get(key, 0.0) + value
        self._totals[key] = self._totals.get(key, 0) + 1

    def render(self) -> str:
        lines = [f"# HELP {self.name} {self.description}", f"# TYPE {self.name} histogram"]
        for key in sorted(self._counts):
            for bound, count in zip(self.buckets, self._counts[key], strict=True):
                labels = _label_key({**dict(key), "le": bound})
                lines.append(f"{self.name}_bucket{_render_labels(labels)} {count}")
            labels_inf = _label_key({**dict(key), "le": "+Inf"})
            lines.append(f"{self.name}_bucket{_render_labels(labels_inf)} {self._totals[key]}")
            lines.append(f"{self.name}_sum{_render_labels(key)} {self._sums[key]:g}")
            lines.append(f"{self.name}_count{_render_labels(key)} {self._totals[key]}")
        return "\n".join(lines)


class MetricsRegistry:
    def __init__(self) -> None:
        self.http_requests = Counter("uaip_http_requests_total", "HTTP requests handled by the platform.")
        self.http_duration = Histogram("uaip_http_request_duration_seconds", "HTTP request latency.")
        self.provider_requests = Counter("uaip_provider_requests_total", "Provider completions by outcome (ok/mock).")
        self.provider_errors = Counter("uaip_provider_errors_total", "Failed provider calls by error type.")
        self.provider_duration = Histogram("uaip_provider_request_duration_seconds", "Provider call latency.")
        self.provider_tokens = Counter("uaip_provider_tokens_total", "Tokens processed, by provider/model/kind.")
        self.provider_cost = Counter("uaip_provider_cost_usd_total", "Accumulated estimated spend in USD.")
        self.cache_events = Counter("uaip_cache_events_total", "Cache hits, misses and stores.")
        self.fallbacks = Counter("uaip_fallbacks_total", "Route candidate failovers, by strategy.")
        self.budget_alerts = Counter("uaip_budget_alerts_total", "Budget threshold alerts fired.")
        self.rate_limited = Counter("uaip_rate_limited_requests_total", "Requests rejected by the rate limiter.")
        self._all = [
            self.http_requests, self.http_duration, self.provider_requests, self.provider_errors,
            self.provider_duration, self.provider_tokens, self.provider_cost, self.cache_events,
            self.fallbacks, self.budget_alerts, self.rate_limited,
        ]

    def render(self) -> str:
        return "\n\n".join(metric.render() for metric in self._all) + "\n"

    def snapshot(self) -> dict:
        return {
            metric.name: metric.snapshot()
            for metric in self._all
            if isinstance(metric, Counter) and metric.snapshot()
        }
