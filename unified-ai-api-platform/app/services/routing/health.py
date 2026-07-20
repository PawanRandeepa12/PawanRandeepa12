"""Provider health monitoring + circuit breaker (Stage 3).

Two signal sources keep the health picture current:

1. active  — a background task periodically pings each provider
             (`HEALTH_CHECK_INTERVAL_SECONDS`),
2. passive — the gateway reports the outcome of every real call
             (`record_success` / `record_failure`).

After `CIRCUIT_BREAKER_THRESHOLD` consecutive failures the provider's circuit
opens and the router skips it until `CIRCUIT_BREAKER_COOLDOWN_SECONDS` elapse;
the next request is then treated as a half-open probe and closes the circuit
again on success.
"""

from __future__ import annotations

import asyncio
import time
from dataclasses import dataclass

from app.config import Settings
from app.core.logging import get_logger
from app.providers.registry import ProviderRegistry

logger = get_logger(__name__)


@dataclass
class ProviderHealth:
    name: str
    mode: str                     # mock | live | unconfigured
    healthy: bool = True
    consecutive_failures: int = 0
    last_error: str | None = None
    last_checked: float | None = None
    circuit_open_until: float | None = None


class HealthMonitor:
    def __init__(self, registry: ProviderRegistry, settings: Settings) -> None:
        self._registry = registry
        self._settings = settings
        self._states: dict[str, ProviderHealth] = {}
        self._task: asyncio.Task | None = None

    # ------------------------------------------------------------- lifecycle
    async def start(self) -> None:
        self.refresh_modes()
        if not self._settings.health_check_enabled or self._task is not None:
            return
        interval = max(2.0, self._settings.health_check_interval_seconds)
        self._task = asyncio.create_task(self._loop(interval), name="provider-health-monitor")
        logger.info("health monitor started (interval=%ss)", interval)

    async def stop(self) -> None:
        if self._task is not None:
            self._task.cancel()
            try:
                await self._task
            except asyncio.CancelledError:
                pass
            self._task = None

    async def _loop(self, interval: float) -> None:
        while True:
            await asyncio.sleep(interval)
            await self.check_all()

    # --------------------------------------------------------------- checking
    async def check_all(self) -> None:
        for name, adapter in self._registry.adapters.items():
            try:
                ok, detail = await adapter.health_check()
            except Exception as exc:  # noqa: BLE001 - any probe failure counts
                ok, detail = False, str(exc)
            state = self._ensure(name)
            state.last_checked = time.time()
            if ok:
                self.record_success(name)
            else:
                self.record_failure(name, detail)

    # ------------------------------------------------------ passive reporting
    def record_success(self, name: str) -> None:
        state = self._ensure(name)
        if state.circuit_open_until is not None:
            logger.info("circuit closed for provider '%s' (recovered)", name)
        state.consecutive_failures = 0
        state.circuit_open_until = None
        state.last_error = None
        state.healthy = state.mode != "unconfigured"

    def record_failure(self, name: str, error: str) -> None:
        state = self._ensure(name)
        state.consecutive_failures += 1
        state.last_error = error[:300]
        state.healthy = False
        threshold = self._settings.circuit_breaker_threshold
        if state.consecutive_failures >= threshold and state.circuit_open_until is None:
            cooldown = self._settings.circuit_breaker_cooldown_seconds
            state.circuit_open_until = time.time() + cooldown
            logger.warning(
                "circuit OPEN for provider '%s' after %d consecutive failures (cooldown %.0fs)",
                name,
                state.consecutive_failures,
                cooldown,
            )

    # ------------------------------------------------------------------ queries
    def is_available(self, name: str) -> bool:
        state = self._ensure(name)
        if state.mode == "unconfigured":
            return False
        if state.circuit_open_until is not None and time.time() < state.circuit_open_until:
            return False
        return True  # closed, or half-open probe after cooldown

    def reset(self, name: str) -> bool:
        state = self._states.get(name)
        if state is None:
            return False
        state.consecutive_failures = 0
        state.circuit_open_until = None
        state.last_error = None
        state.healthy = state.mode != "unconfigured"
        logger.info("health state manually reset for provider '%s'", name)
        return True

    def refresh_modes(self) -> None:
        for name, adapter in self._registry.adapters.items():
            self._ensure(name).mode = adapter.mode()

    def snapshot(self) -> dict:
        self.refresh_modes()
        now = time.time()
        out: dict[str, dict] = {}
        for name, state in self._states.items():
            remaining = max(0.0, (state.circuit_open_until or 0) - now) if state.circuit_open_until else 0.0
            circuit = "closed"
            if state.circuit_open_until is not None:
                circuit = "open" if remaining > 0 else "half_open"
            out[name] = {
                "mode": state.mode,
                "available": self.is_available(name),
                "healthy": state.healthy,
                "consecutive_failures": state.consecutive_failures,
                "last_error": state.last_error,
                "last_checked": state.last_checked,
                "circuit": circuit,
                "circuit_open_seconds_remaining": round(remaining, 1),
            }
        return out

    # ------------------------------------------------------------------ internals
    def _ensure(self, name: str) -> ProviderHealth:
        state = self._states.get(name)
        if state is None:
            adapter = self._registry.get(name)
            mode = adapter.mode() if adapter else "unconfigured"
            state = ProviderHealth(name=name, mode=mode, healthy=mode != "unconfigured")
            self._states[name] = state
        return state
