"""Budget management & threshold alerts (Stage 2).

Budgets cap spend over a period (monthly / daily / all-time) at a scope:
  * `global`          — whole platform (a fully-consumed global budget blocks
                        new requests with HTTP 429 budget_exceeded)
  * `provider:<name>` — spend through one provider
  * `key:<key_id>`    — spend by one platform API key

When spend crosses a configured fraction of the limit (e.g. 0.5 / 0.8 / 1.0)
an alert is recorded and logged once per threshold.
"""

from __future__ import annotations

import time
import uuid
from dataclasses import dataclass, field
from datetime import UTC, datetime

from app.core.logging import get_logger
from app.services.cost.tracker import CostTracker

logger = get_logger(__name__)


@dataclass
class Budget:
    id: str
    name: str
    limit_usd: float
    scope: str = "global"
    period: str = "monthly"
    thresholds: list[float] = field(default_factory=lambda: [0.5, 0.8, 1.0])
    created_ts: float = field(default_factory=time.time)


@dataclass
class BudgetAlert:
    id: str
    budget_id: str
    budget_name: str
    threshold: float
    spent_usd: float
    limit_usd: float
    ts: float
    message: str


class BudgetManager:
    def __init__(self, tracker: CostTracker, default_thresholds: list[float] | None = None) -> None:
        self._tracker = tracker
        self._budgets: dict[str, Budget] = {}
        self._alerts: list[BudgetAlert] = []
        self._fired: set[tuple[str, float]] = set()
        self._default_thresholds = default_thresholds or [0.5, 0.8, 1.0]

    # --------------------------------------------------------------- lifecycle
    def create(
        self,
        *,
        name: str,
        limit_usd: float,
        scope: str = "global",
        period: str = "monthly",
        thresholds: list[float] | None = None,
        budget_id: str | None = None,
    ) -> Budget:
        budget = Budget(
            id=budget_id or f"b-{uuid.uuid4().hex[:12]}",
            name=name,
            limit_usd=limit_usd,
            scope=scope,
            period=period,
            thresholds=sorted(thresholds) if thresholds else list(self._default_thresholds),
        )
        self._budgets[budget.id] = budget
        logger.info("budget created: %s (%s, limit=$%.4f, scope=%s)", budget.id, name, limit_usd, scope)
        return budget

    def ensure_default_budget(self, limit_usd: float) -> Budget:
        existing = self._budgets.get("default")
        if existing is not None:
            return existing
        return self.create(name="Default monthly budget", limit_usd=limit_usd, budget_id="default")

    def list(self) -> list[Budget]:
        return list(self._budgets.values())

    def get(self, budget_id: str) -> Budget | None:
        return self._budgets.get(budget_id)

    def delete(self, budget_id: str) -> bool:
        return self._budgets.pop(budget_id, None) is not None

    def alerts(self) -> list[BudgetAlert]:
        return list(reversed(self._alerts[-100:]))

    # ----------------------------------------------------------------- verdicts
    @staticmethod
    def _period_start(budget: Budget) -> float | None:
        now = datetime.now(UTC)
        if budget.period == "daily":
            return now.replace(hour=0, minute=0, second=0, microsecond=0).timestamp()
        if budget.period == "monthly":
            return now.replace(day=1, hour=0, minute=0, second=0, microsecond=0).timestamp()
        return None  # all_time

    def spend_for(self, budget: Budget) -> float:
        provider = budget.scope.split(":", 1)[1] if budget.scope.startswith("provider:") else None
        key_id = budget.scope.split(":", 1)[1] if budget.scope.startswith("key:") else None
        return self._tracker.total_spend(
            since=self._period_start(budget),
            provider=provider,
            api_key_id=key_id,
        )

    def status(self, budget: Budget) -> dict:
        spent = self.spend_for(budget)
        ratio = spent / budget.limit_usd if budget.limit_usd > 0 else 0.0
        fired = sorted(threshold for (bid, threshold) in self._fired if bid == budget.id)
        return {
            "id": budget.id,
            "name": budget.name,
            "scope": budget.scope,
            "period": budget.period,
            "limit_usd": budget.limit_usd,
            "spent_usd": round(spent, 8),
            "remaining_usd": round(max(0.0, budget.limit_usd - spent), 8),
            "used_pct": round(ratio * 100, 4),
            "exceeded": spent >= budget.limit_usd,
            "thresholds": budget.thresholds,
            "fired_thresholds": fired,
            "created_ts": budget.created_ts,
        }

    def status_all(self) -> list[dict]:
        return [self.status(b) for b in self._budgets.values()]

    def blocking_budget(self) -> Budget | None:
        """A fully-consumed global budget blocks new provider calls."""
        for budget in self._budgets.values():
            if budget.scope == "global" and budget.limit_usd > 0 and self.spend_for(budget) >= budget.limit_usd:
                return budget
        return None

    def check_and_alert(self) -> list[BudgetAlert]:
        """Evaluate thresholds after a recorded request; fire new alerts."""
        new_alerts: list[BudgetAlert] = []
        for budget in self._budgets.values():
            if budget.limit_usd <= 0:
                continue
            ratio = self.spend_for(budget) / budget.limit_usd
            for threshold in budget.thresholds:
                if ratio >= threshold and (budget.id, threshold) not in self._fired:
                    self._fired.add((budget.id, threshold))
                    alert = BudgetAlert(
                        id=f"a-{uuid.uuid4().hex[:12]}",
                        budget_id=budget.id,
                        budget_name=budget.name,
                        threshold=threshold,
                        spent_usd=round(self.spend_for(budget), 8),
                        limit_usd=budget.limit_usd,
                        ts=time.time(),
                        message=(
                            f"Budget '{budget.name}' reached {threshold:.0%} of its "
                            f"${budget.limit_usd:.4f} {budget.period} limit."
                        ),
                    )
                    self._alerts.append(alert)
                    new_alerts.append(alert)
                    logger.warning("budget alert: %s", alert.message)
        return new_alerts
