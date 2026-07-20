"""Model catalog endpoints (Stage 1)."""

from __future__ import annotations

from fastapi import APIRouter, Request

from app.core.exceptions import ModelNotFoundError
from app.providers.base import ModelInfo, ProviderAdapter

router = APIRouter(prefix="/v1/models", tags=["models"])


def model_payload(adapter: ProviderAdapter, model: ModelInfo) -> dict:
    return {
        "id": model.id,
        "object": "model",
        "provider": model.provider,
        "name": model.name,
        "display_name": model.display_name,
        "tier": model.tier,
        "context_window": model.context_window,
        "capabilities": {"vision": model.supports_vision, "tools": model.supports_tools},
        "pricing_per_1k_tokens_usd": {
            "input": model.input_price_per_1k,
            "output": model.output_price_per_1k,
        },
        "provider_mode": adapter.mode(),  # mock | live | unconfigured
    }


@router.get("")
async def list_models(request: Request) -> dict:
    registry = request.app.state.registry
    return {
        "object": "list",
        "data": [model_payload(adapter, model) for adapter, model in registry.all_models()],
    }


@router.get("/{model_ref:path}")
async def get_model(model_ref: str, request: Request) -> dict:
    matches = request.app.state.registry.resolve(model_ref)
    if not matches:
        raise ModelNotFoundError(f"Unknown model '{model_ref}'.")
    adapter, model = matches[0]
    return model_payload(adapter, model)
