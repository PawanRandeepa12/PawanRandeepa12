"""Per-client rate limiting — (Stage 4).

Simple token-bucket limiter keyed by API-key id (or client IP when auth is
disabled). Exceeding clients get HTTP 429 with a `Retry-After` hint.
"""

from __future__ import annotations

import time


class TokenBucket:
    __slots__ = ("capacity", "refill_per_sec", "tokens", "updated")

    def __init__(self, capacity: float, refill_per_sec: float) -> None:
        self.capacity = capacity
        self.refill_per_sec = refill_per_sec
        self.tokens = capacity
        self.updated = time.monotonic()

    def take(self) -> tuple[bool, float]:
        now = time.monotonic()
        self.tokens = min(self.capacity, self.tokens + (now - self.updated) * self.refill_per_sec)
        self.updated = now
        if self.tokens >= 1.0:
            self.tokens -= 1.0
            return True, 0.0
        retry_after = (1.0 - self.tokens) / self.refill_per_sec if self.refill_per_sec > 0 else 60.0
        return False, retry_after


class RateLimiter:
    def __init__(self, requests_per_minute: int, burst: int) -> None:
        self.requests_per_minute = max(1, requests_per_minute)
        self.burst = max(1, burst)
        self._buckets: dict[str, TokenBucket] = {}
        self._ops = 0

    def check(self, identifier: str) -> tuple[bool, float]:
        bucket = self._buckets.get(identifier)
        if bucket is None:
            bucket = TokenBucket(float(self.burst), self.requests_per_minute / 60.0)
            self._buckets[identifier] = bucket
        self._ops += 1
        if self._ops % 512 == 0:
            self._cleanup()
        return bucket.take()

    def _cleanup(self) -> None:
        now = time.monotonic()
        idle = [
            key
            for key, bucket in self._buckets.items()
            if now - bucket.updated > 600 and bucket.tokens >= bucket.capacity
        ]
        for key in idle:
            self._buckets.pop(key, None)

    def reset(self) -> None:
        self._buckets.clear()

    def snapshot(self) -> dict:
        return {
            "tracked_clients": len(self._buckets),
            "requests_per_minute": self.requests_per_minute,
            "burst": self.burst,
        }
