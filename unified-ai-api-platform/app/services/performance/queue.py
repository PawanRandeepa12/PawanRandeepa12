"""Request prioritization queue (Stage 4).

Provider calls execute through a bounded pool of async workers. Requests
carry a priority (high / normal / low); higher-priority work is dequeued
first while the worker count caps upstream concurrency — protecting provider
accounts from connection storms under bursty load.
"""

from __future__ import annotations

import asyncio
import itertools
from collections.abc import Awaitable, Callable
from typing import Any

PRIORITY_LEVELS = {"high": 0, "normal": 5, "low": 9}

QueueItem = tuple[int, int, Callable[[], Awaitable[Any]] | None, "asyncio.Future | None"]


class PriorityExecutor:
    def __init__(self, max_concurrency: int = 8) -> None:
        self.max_concurrency = max(1, max_concurrency)
        self._queue: asyncio.PriorityQueue[QueueItem] = asyncio.PriorityQueue()
        self._sequence = itertools.count()
        self._workers: list[asyncio.Task] = []
        self._started = False
        self._submitted = 0
        self._completed = 0
        self._in_progress = 0

    async def start(self) -> None:
        if self._started:
            return
        self._started = True
        self._workers = [
            asyncio.create_task(self._worker(), name=f"priority-worker-{i}")
            for i in range(self.max_concurrency)
        ]

    async def stop(self) -> None:
        for _ in self._workers:  # sentinels sort last, so queued work drains first
            await self._queue.put((10_000, next(self._sequence), None, None))
        for task in self._workers:
            try:
                await task
            except asyncio.CancelledError:
                pass
        self._workers = []
        self._started = False

    async def enqueue(self, fn: Callable[[], Awaitable[Any]], priority: str = "normal") -> asyncio.Future:
        """Queue work without awaiting its result (returns the future)."""
        loop = asyncio.get_running_loop()
        future: asyncio.Future = loop.create_future()
        level = PRIORITY_LEVELS.get(priority, PRIORITY_LEVELS["normal"])
        self._submitted += 1
        await self._queue.put((level, next(self._sequence), fn, future))
        return future

    async def submit(self, fn: Callable[[], Awaitable[Any]], priority: str = "normal") -> Any:
        future = await self.enqueue(fn, priority)
        return await future

    async def _worker(self) -> None:
        while True:
            _, _, fn, future = await self._queue.get()
            if fn is None:  # shutdown sentinel
                self._queue.task_done()
                return
            self._in_progress += 1
            try:
                result = await fn()
                if future is not None and not future.done():
                    future.set_result(result)
            except Exception as exc:  # noqa: BLE001 - forwarded to the waiter
                if future is not None and not future.done():
                    future.set_exception(exc)
            finally:
                self._in_progress -= 1
                self._completed += 1
                self._queue.task_done()

    def snapshot(self) -> dict:
        return {
            "queued": self._queue.qsize(),
            "in_progress": self._in_progress,
            "workers": len(self._workers),
            "max_concurrency": self.max_concurrency,
            "submitted": self._submitted,
            "completed": self._completed,
        }
