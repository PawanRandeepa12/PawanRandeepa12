"""Provider pricing database (Stage 2).

Prices live in an editable JSON file (`config/pricing.json`) so costs can be
kept current without a redeploy. Resolution order for a model:
  1. pricing file
  2. the provider adapter's catalog price
  3. `default_pricing`
"""

from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path

from app.core.logging import get_logger
from app.providers.base import ModelInfo
from app.schemas.chat import CostInfo

logger = get_logger(__name__)


@dataclass(frozen=True)
class ModelPricing:
    input_per_1k: float
    output_per_1k: float
    source: str  # pricing_file | catalog | default


class PricingDatabase:
    def __init__(self, file_prices: dict[str, ModelPricing], default_pricing: ModelPricing, currency: str = "USD") -> None:
        self._file_prices = file_prices
        self._default_pricing = default_pricing
        self.currency = currency

    @classmethod
    def load(cls, path: Path) -> "PricingDatabase":
        file_prices: dict[str, ModelPricing] = {}
        default = ModelPricing(0.001, 0.003, "default")
        currency = "USD"
        try:
            data = json.loads(Path(path).read_text(encoding="utf-8"))
            currency = data.get("currency", "USD")
            raw_default = data.get("default_pricing") or {}
            default = ModelPricing(
                float(raw_default.get("input", 0.001)),
                float(raw_default.get("output", 0.003)),
                "default",
            )
            for model_id, prices in (data.get("models") or {}).items():
                file_prices[model_id.lower()] = ModelPricing(
                    float(prices["input"]), float(prices["output"]), "pricing_file"
                )
            logger.info("loaded pricing for %d models from %s", len(file_prices), path)
        except FileNotFoundError:
            logger.warning("pricing file %s not found; falling back to catalog/default prices", path)
        except (ValueError, KeyError, TypeError) as exc:
            logger.error("invalid pricing file %s (%s); falling back to catalog/default prices", path, exc)
        return cls(file_prices, default, currency)

    def get(self, model_id: str, catalog_fallback: ModelInfo | None = None) -> ModelPricing:
        hit = self._file_prices.get(model_id.lower())
        if hit is not None:
            return hit
        if catalog_fallback is not None:
            return ModelPricing(
                catalog_fallback.input_price_per_1k,
                catalog_fallback.output_price_per_1k,
                "catalog",
            )
        return self._default_pricing

    def estimate(
        self,
        model_id: str,
        prompt_tokens: int,
        completion_tokens: int,
        *,
        catalog_fallback: ModelInfo | None = None,
    ) -> CostInfo:
        pricing = self.get(model_id, catalog_fallback)
        prompt_cost = prompt_tokens / 1000 * pricing.input_per_1k
        completion_cost = completion_tokens / 1000 * pricing.output_per_1k
        return CostInfo(
            prompt_cost_usd=round(prompt_cost, 8),
            completion_cost_usd=round(completion_cost, 8),
            total_cost_usd=round(prompt_cost + completion_cost, 8),
            pricing_source=pricing.source,
        )

    def as_dict(self) -> dict:
        return {
            "currency": self.currency,
            "default": {"input": self._default_pricing.input_per_1k, "output": self._default_pricing.output_per_1k},
            "models": {
                model_id: {"input": p.input_per_1k, "output": p.output_per_1k}
                for model_id, p in sorted(self._file_prices.items())
            },
        }
