"""Cost optimization endpoints (Stage 2).

Estimate & compare costs *before* calling a provider, manage budgets and
alerts, and explore spend analytics — everything a cost dashboard needs.
"""

from __future__ import annotations

from fastapi import APIRouter, Depends, Query, Request

from app.core.exceptions import BadRequestError, ModelNotFoundError, NotFoundError
from app.core.security import get_api_key_context
from app.core.tokens import estimate_messages_tokens
from app.schemas.cost import (
    BudgetCreate,
    CostCompareRequest,
    CostCompareResponse,
    CostEstimateRequest,
    CostEstimateResponse,
    ModelCostEstimate,
    UsageRecordView,
)

router = APIRouter(prefix="/v1/cost", tags=["cost"], dependencies=[Depends(get_api_key_context)])


# --------------------------------------------------------------------- pricing
@router.get("/pricing")
async def pricing_table(request: Request) -> dict:
    return request.app.state.pricing.as_dict()


# -------------------------------------------------------------------- estimate
@router.post("/estimate", response_model=CostEstimateResponse)
async def estimate_cost(request: Request, body: CostEstimateRequest) -> CostEstimateResponse:
    settings = request.app.state.settings
    registry = request.app.state.registry
    pricing = request.app.state.pricing

    matches = registry.resolve(body.model)
    if not matches:
        raise ModelNotFoundError(f"Unknown model '{body.model}'.")
    _, model = matches[0]

    tokens_estimated = body.prompt_tokens is None
    if body.prompt_tokens is not None:
        prompt_tokens = body.prompt_tokens
    elif body.messages:
        prompt_tokens = estimate_messages_tokens(body.messages)
    else:
        raise BadRequestError("Provide either 'messages' or 'prompt_tokens'.")

    completion_tokens = body.completion_tokens
    if completion_tokens is None:
        completion_tokens = settings.default_completion_tokens_estimate
        tokens_estimated = True

    return CostEstimateResponse(
        model=model.id,
        prompt_tokens=prompt_tokens,
        completion_tokens=completion_tokens,
        tokens_estimated=tokens_estimated,
        cost=pricing.estimate(model.id, prompt_tokens, completion_tokens, catalog_fallback=model),
    )


# -------------------------------------------------------------------- compare
@router.post("/compare", response_model=CostCompareResponse)
async def compare_costs(request: Request, body: CostCompareRequest) -> CostCompareResponse:
    settings = request.app.state.settings
    registry = request.app.state.registry
    pricing = request.app.state.pricing

    if body.models:
        pairs = []
        for ref in body.models:
            matches = registry.resolve(ref)
            if not matches:
                raise ModelNotFoundError(f"Unknown model '{ref}'.")
            pairs.append(matches[0])
    else:
        pairs = registry.all_models()

    prompt_tokens = estimate_messages_tokens(body.messages)
    completion_tokens = body.max_tokens or settings.default_completion_tokens_estimate

    estimates: list[ModelCostEstimate] = []
    for adapter, model in pairs:
        if adapter.mode() == "unconfigured":
            continue
        if "vision" in body.required_capabilities and not model.supports_vision:
            continue
        if "tools" in body.required_capabilities and not model.supports_tools:
            continue
        cost = pricing.estimate(model.id, prompt_tokens, completion_tokens, catalog_fallback=model)
        estimates.append(
            ModelCostEstimate(
                model=model.id,
                provider=model.provider,
                tier=model.tier,
                prompt_tokens=prompt_tokens,
                completion_tokens=completion_tokens,
                cost_usd=cost.total_cost_usd,
            )
        )
    estimates.sort(key=lambda e: e.cost_usd)
    return CostCompareResponse(
        completion_tokens_assumed=completion_tokens,
        estimates=estimates,
        cheapest=estimates[0].model if estimates else None,
    )


# -------------------------------------------------------------------- analytics
@router.get("/summary")
async def cost_summary(request: Request, days: int = Query(30, ge=1, le=365)) -> dict:
    return request.app.state.cost_analytics.summary(days)


@router.get("/history")
async def cost_history(request: Request, days: int = Query(30, ge=1, le=365)) -> dict:
    return {"history": request.app.state.cost_analytics.history(days)}


@router.get("/records", response_model=list[UsageRecordView])
async def recent_records(request: Request, limit: int = Query(50, ge=1, le=500)) -> list[dict]:
    return request.app.state.cost_analytics.recent_records(limit)


# --------------------------------------------------------------------- budgets
@router.get("/budgets")
async def list_budgets(request: Request) -> dict:
    return {"budgets": request.app.state.budgets.status_all()}


@router.post("/budgets", status_code=201)
async def create_budget(request: Request, body: BudgetCreate) -> dict:
    if body.scope.startswith("provider:"):
        provider = body.scope.split(":", 1)[1]
        if request.app.state.registry.get(provider) is None:
            raise BadRequestError(f"Unknown provider '{provider}' in scope.")
    elif body.scope != "global" and not body.scope.startswith("key:"):
        raise BadRequestError("Scope must be 'global', 'provider:<name>' or 'key:<key_id>'.")
    if body.thresholds is not None and any(t <= 0 or t > 1.5 for t in body.thresholds):
        raise BadRequestError("Thresholds must be fractions in (0, 1.5].")

    budgets = request.app.state.budgets
    budget = budgets.create(
        name=body.name,
        limit_usd=body.limit_usd,
        scope=body.scope,
        period=body.period,
        thresholds=body.thresholds,
    )
    return budgets.status(budget)


@router.get("/budgets/alerts")
async def list_alerts(request: Request) -> dict:
    alerts = request.app.state.budgets.alerts()
    return {
        "alerts": [
            {
                "id": a.id,
                "budget_id": a.budget_id,
                "budget_name": a.budget_name,
                "threshold": a.threshold,
                "spent_usd": a.spent_usd,
                "limit_usd": a.limit_usd,
                "ts": a.ts,
                "message": a.message,
            }
            for a in alerts
        ]
    }


@router.delete("/budgets/{budget_id}")
async def delete_budget(request: Request, budget_id: str) -> dict:
    if not request.app.state.budgets.delete(budget_id):
        raise NotFoundError(f"Budget '{budget_id}' not found.")
    return {"deleted": budget_id}
