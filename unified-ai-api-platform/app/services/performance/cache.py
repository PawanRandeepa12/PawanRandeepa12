"""Response caching layer (Stage 4).

Backend selection: Redis when `REDIS_URL` is configured and reachable
(shared across replicas, survives restarts), otherwise a bounded in-memory
TTL cache. Identical chat requests are served from cache with `cached=true`
— skipping provider calls, cost tracking and budget consumption entirely.
"""

from __future__ import annotations

import hashlib
import json
import time
from typing import Protocol

from app.config import Settings
from app.core.logging import get_logger

logger = get_logger(__name__)

CACHE_PREFIX = "uaip:"


def make_cache_key(namespace: str, payload: dict) -> str:
    canonical = json.dumps(payload, sort_keys=True, separators=(",", ":"), default=str)
    digest = hashlib.sha256(canonical.encode()).hexdigest()
    return f"{CACHE_PREFIX}{namespace}:{digest}"


class CacheBackend(Protocol):
    name: str

    async def get(self, key: str) -> str | None: ...
    async def set(self, key: str, value: str, ttl_seconds: int) -> None: ...
    async def clear(self) -> int: ...
    def stats(self) -> dict: ...


class InMemoryTTLCache:
    name = "memory"

    def __init__(self, max_entries: int = 5000) -> None:
        self._items: dict[str, tuple[float, str]] = {}
        self._max_entries = max(100, max_entries)
        self.hits = 0
        self.misses = 0

    async def get(self, key: str) -> str | None:
        item = self._items.get(key)
        if item is None:
            self.misses += 1
            return None
        expires_at, value = item
        if expires_at < time.time():
            self._items.pop(key, None)
            self.misses += 1
            return None
        self.hits += 1
        return value

    async def set(self, key: str, value: str, ttl_seconds: int) -> None:
        if len(self._items) >= self._max_entries:
            self._evict()
        self._items[key] = (time.time() + max(1, ttl_seconds), value)

    def _evict(self) -> None:
        now = time.time()
        for key in [k for k, (exp, _) in self._items.items() if exp < now]:
            self._items.pop(key, None)
        if len(self._items) >= self._max_entries:  # still full: drop oldest ~10%
            ordered = sorted(self._items.items(), key=lambda kv: kv[1][0])
            for key, _ in ordered[: max(1, len(ordered) // 10)]:
                self._items.pop(key, None)

    async def clear(self) -> int:
        removed = len(self._items)
        self._items.clear()
        return removed

    def stats(self) -> dict:
        total = self.hits + self.misses
        return {
            "backend": self.name,
            "entries": len(self._items),
            "max_entries": self._max_entries,
            "hits": self.hits,
            "misses": self.misses,
            "hit_rate": round(self.hits / total, 4) if total else 0.0,
        }


class RedisCache:
    name = "redis"

    def __init__(self, url: str) -> None:
        from redis import asyncio as aioredis  # optional dependency, imported lazily

        self._redis = aioredis.from_url(url, decode_responses=True)
        self.hits = 0
        self.misses = 0

    async def ping(self) -> None:
        await self._redis.ping()

    async def get(self, key: str) -> str | None:
        value = await self._redis.get(key)
        if value is None:
            self.misses += 1
        else:
            self.hits += 1
        return value

    async def set(self, key: str, value: str, ttl_seconds: int) -> None:
        await self._redis.set(key, value, ex=max(1, ttl_seconds))

    async def clear(self) -> int:
        removed = 0
        async for key in self._redis.scan_iter(match=f"{CACHE_PREFIX}*", count=500):
            removed += await self._redis.delete(key)
        return removed

    def stats(self) -> dict:
        total = self.hits + self.misses
        return {
            "backend": self.name,
            "hits": self.hits,
            "misses": self.misses,
            "hit_rate": round(self.hits / total, 4) if total else 0.0,
        }

    async def aclose(self) -> None:
        await self._redis.aclose()


async def build_cache(settings: Settings) -> CacheBackend | None:
    if not settings.cache_enabled:
        logger.info("cache: disabled")
        return None
    if settings.redis_url:
        try:
            cache = RedisCache(settings.redis_url)
            await cache.ping()
            logger.info("cache: redis backend ready")
            return cache
        except Exception as exc:  # noqa: BLE001 - any redis failure falls back
            logger.warning("cache: redis unavailable (%s); falling back to in-memory", exc)
    logger.info("cache: in-memory TTL backend (max %d entries)", settings.cache_max_entries)
    return InMemoryTTLCache(settings.cache_max_entries)
