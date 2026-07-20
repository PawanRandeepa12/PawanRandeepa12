"""Unified chat completion schemas (the platform's OpenAI-style contract).

One request/response shape regardless of the upstream provider. Optional
fields (cost, routing) are populated as later stages are enabled.
"""

from __future__ import annotations

from typing import Any, Literal

from pydantic import BaseModel, Field

Role = Literal["system", "user", "assistant", "tool"]
Strategy = Literal["auto", "cost", "latency", "balanced"]
Priority = Literal["low", "normal", "high"]
Capability = Literal["vision", "tools"]


class Message(BaseModel):
    role: Role
    content: str


class ChatCompletionRequest(BaseModel):
    model: str = Field(
        default="auto",
        description="Unified model id ('openai/gpt-4o-mini'), a bare model name, or 'auto' for platform routing.",
    )
    messages: list[Message] = Field(min_length=1)
    temperature: float = Field(default=0.7, ge=0.0, le=2.0)
    max_tokens: int | None = Field(default=None, gt=0)
    # Stage 3 routing controls
    strategy: Strategy = "auto"
    max_fallbacks: int | None = Field(default=None, ge=0, le=10)
    required_capabilities: list[Capability] = Field(default_factory=list)
    # Stage 4 performance controls
    priority: Priority = "normal"
    cache: bool = True
    metadata: dict[str, Any] | None = None


class Usage(BaseModel):
    prompt_tokens: int
    completion_tokens: int
    total_tokens: int


class CostInfo(BaseModel):
    prompt_cost_usd: float
    completion_cost_usd: float
    total_cost_usd: float
    pricing_source: str = "pricing.json"


class RoutingInfo(BaseModel):
    strategy: str
    candidates: list[str]
    fallbacks_used: int = 0
    mock: bool = False
    reason: str | None = None


class Choice(BaseModel):
    index: int = 0
    message: Message
    finish_reason: str = "stop"


class ChatCompletionResponse(BaseModel):
    id: str
    object: str = "chat.completion"
    created: int
    model: str
    provider: str
    choices: list[Choice]
    usage: Usage
    latency_ms: float
    cached: bool = False
    cost: CostInfo | None = None       # populated by Stage 2
    routing: RoutingInfo | None = None  # populated by Stage 3


# ------------------------------------------------------- Stage 4: comparison
class CompareRequest(BaseModel):
    """Fan the same prompt out to several models in parallel."""

    messages: list[Message] = Field(min_length=1)
    models: list[str] = Field(
        default_factory=list,
        description="Models to compare. Empty = the flagship model of each configured provider.",
    )
    temperature: float = Field(default=0.7, ge=0.0, le=2.0)
    max_tokens: int | None = Field(default=None, gt=0)


class CompareResultItem(BaseModel):
    model: str
    provider: str
    ok: bool
    content: str | None = None
    latency_ms: float
    usage: Usage | None = None
    cost: CostInfo | None = None
    error: str | None = None


class CompareResponse(BaseModel):
    results: list[CompareResultItem]
    fastest: str | None = None
    cheapest: str | None = None
