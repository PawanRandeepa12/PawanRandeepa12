"""Chat endpoints.

POST /v1/chat/completions — unified completion with routing, cost tracking,
                            caching, priority queueing and rate limiting.
POST /v1/chat/compare     — Stage 4: run one prompt against several models
                            in parallel, with cost & latency side by side.
"""

from __future__ import annotations

from fastapi import APIRouter, Depends, Request, Response

from app.api.deps import enforce_rate_limit
from app.core.security import ApiKeyContext, get_api_key_context
from app.schemas.chat import (
    ChatCompletionRequest,
    ChatCompletionResponse,
    CompareRequest,
    CompareResponse,
)

router = APIRouter(prefix="/v1/chat", tags=["chat"])


@router.post("/completions", response_model=ChatCompletionResponse)
async def create_completion(
    request: Request,
    response: Response,
    body: ChatCompletionRequest,
    ctx: ApiKeyContext = Depends(get_api_key_context),
    _: None = Depends(enforce_rate_limit),
) -> ChatCompletionResponse:
    gateway = request.app.state.gateway
    result = await gateway.complete(body, key_id=ctx.key_id)
    if request.app.state.cache is not None:
        response.headers["X-Cache"] = "HIT" if result.cached else "MISS"
    return result


@router.post("/compare", response_model=CompareResponse)
async def compare_models(
    request: Request,
    body: CompareRequest,
    ctx: ApiKeyContext = Depends(get_api_key_context),
    _: None = Depends(enforce_rate_limit),
) -> CompareResponse:
    gateway = request.app.state.gateway
    return await gateway.compare(body, key_id=ctx.key_id)
