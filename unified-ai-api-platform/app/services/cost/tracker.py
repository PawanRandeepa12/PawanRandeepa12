"""Usage & cost tracking with JSONL persistence (Stage 2).

Every completed (non-cached) request appends one UsageRecord. Records are kept
in memory for fast aggregation and appended to a JSONL file so spend survives
restarts. The format is deliberately simple — swap this class for a real DB
backed implementation when volume grows.
"""

from __future__ import annotations

import json
import threading
import time
import uuid
from dataclasses import asdict, dataclass
from pathlib import Path

from app.core.logging import get_logger

logger = get_logger(__name__)


@dataclass
class UsageRecord:
    id: str
    ts: float
    api_key_id: str
    provider: str
    model: str
    strategy: str
    prompt_tokens: int
    completion_tokens: int
    total_tokens: int
    cost_usd: float
    latency_ms: float
    cached: bool = False
    mock: bool = False


class CostTracker:
    def __init__(self, store_path: Path | None) -> None:
        self._path = Path(store_path) if store_path else None
        self._records: list[UsageRecord] = []
        self._lock = threading.Lock()
        self._load()

    # ---------------------------------------------------------------- storage
    def _load(self) -> None:
        if not self._path or not self._path.exists():
            return
        loaded = 0
        for line in self._path.read_text(encoding="utf-8").splitlines():
            line = line.strip()
            if not line:
                continue
            try:
                self._records.append(UsageRecord(**json.loads(line)))
                loaded += 1
            except (TypeError, ValueError):
                continue  # tolerate older/corrupt lines
        if loaded:
            logger.info("loaded %d historical usage records from %s", loaded, self._path)

    def _persist(self, record: UsageRecord) -> None:
        if not self._path:
            return
        try:
            self._path.parent.mkdir(parents=True, exist_ok=True)
            with self._path.open("a", encoding="utf-8") as fh:
                fh.write(json.dumps(asdict(record)) + "\n")
        except OSError:
            logger.warning("could not persist usage record", exc_info=True)

    # ------------------------------------------------------------------ write
    def record(
        self,
        *,
        api_key_id: str,
        provider: str,
        model: str,
        strategy: str,
        prompt_tokens: int,
        completion_tokens: int,
        cost_usd: float,
        latency_ms: float,
        cached: bool = False,
        mock: bool = False,
    ) -> UsageRecord:
        record = UsageRecord(
            id=f"u-{uuid.uuid4().hex[:16]}",
            ts=time.time(),
            api_key_id=api_key_id,
            provider=provider,
            model=model,
            strategy=strategy,
            prompt_tokens=prompt_tokens,
            completion_tokens=completion_tokens,
            total_tokens=prompt_tokens + completion_tokens,
            cost_usd=cost_usd,
            latency_ms=latency_ms,
            cached=cached,
            mock=mock,
        )
        with self._lock:
            self._records.append(record)
            self._persist(record)
        return record

    # ------------------------------------------------------------------- read
    def query(
        self,
        *,
        since: float | None = None,
        api_key_id: str | None = None,
        provider: str | None = None,
    ) -> list[UsageRecord]:
        with self._lock:
            records = list(self._records)
        return [
            r
            for r in records
            if (since is None or r.ts >= since)
            and (api_key_id is None or r.api_key_id == api_key_id)
            and (provider is None or r.provider == provider)
        ]

    def total_spend(
        self,
        *,
        since: float | None = None,
        api_key_id: str | None = None,
        provider: str | None = None,
    ) -> float:
        return sum(r.cost_usd for r in self.query(since=since, api_key_id=api_key_id, provider=provider))

    def recent(self, limit: int = 50) -> list[UsageRecord]:
        with self._lock:
            return list(reversed(self._records[-limit:]))

    def count(self) -> int:
        with self._lock:
            return len(self._records)
