"""Cost optimization schemas (Stage 2)."""

from __future__ import annotations

from typing import Literal

from pydantic import BaseModel, Field

from app.schemas.chat import Capability, CostInfo, Message


class CostEstimateRequest(BaseModel):
    model: str = Field(description="Unified model id or bare model name.")
    messages: list[Message] | None = None
    prompt_tokens: int | None = Field(default=None, ge=0)
    completion_tokens: int | None = Field(default=None, ge=0)


class CostEstimateResponse(BaseModel):
    model: str
    prompt_tokens: int
    completion_tokens: int
    tokens_estimated: bool
    cost: CostInfo


class CostCompareRequest(BaseModel):
    messages: list[Message] = Field(min_length=1)
    max_tokens: int | None = Field(default=None, gt=0)
    models: list[str] = Field(default_factory=list, description="Empty = whole catalog.")
    required_capabilities: list[Capability] = Field(default_factory=list)


class ModelCostEstimate(BaseModel):
    model: str
    provider: str
    tier: str
    prompt_tokens: int
    completion_tokens: int
    cost_usd: float


class CostCompareResponse(BaseModel):
    completion_tokens_assumed: int
    estimates: list[ModelCostEstimate]
    cheapest: str | None


class BudgetCreate(BaseModel):
    name: str = Field(min_length=1, max_length=120)
    limit_usd: float = Field(gt=0)
    scope: str = Field(default="global", description="global | provider:<name> | key:<key_id>")
    period: Literal["monthly", "daily", "all_time"] = "monthly"
    thresholds: list[float] | None = Field(default=None, description="Fractions of the limit, e.g. [0.5, 0.8, 1.0]")


class UsageRecordView(BaseModel):
    id: str
    at: str
    api_key_id: str
    provider: str
    model: str
    strategy: str
    prompt_tokens: int
    completion_tokens: int
    total_tokens: int
    cost_usd: float
    latency_ms: float
    cached: bool
    mock: bool
