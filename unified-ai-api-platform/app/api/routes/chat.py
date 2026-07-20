"""Unified chat completion endpoint (Stage 1)."""

from __future__ import annotations

from fastapi import APIRouter, Depends, Request

from app.core.security import ApiKeyContext, get_api_key_context
from app.schemas.chat import ChatCompletionRequest, ChatCompletionResponse

router = APIRouter(prefix="/v1/chat", tags=["chat"])


@router.post("/completions", response_model=ChatCompletionResponse)
async def create_completion(
    request: Request,
    body: ChatCompletionRequest,
    ctx: ApiKeyContext = Depends(get_api_key_context),
) -> ChatCompletionResponse:
    gateway = request.app.state.gateway
    return await gateway.complete(body, key_id=ctx.key_id)
