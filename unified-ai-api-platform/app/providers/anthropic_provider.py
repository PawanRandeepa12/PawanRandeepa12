"""Anthropic Claude provider adapter (Stage 1)."""

from __future__ import annotations

from typing import ClassVar

from app.core.tokens import estimate_messages_tokens, estimate_tokens
from app.providers.base import ModelInfo, ProviderAdapter, ProviderResult
from app.schemas.chat import ChatCompletionRequest


class AnthropicProvider(ProviderAdapter):
    name: ClassVar[str] = "anthropic"
    catalog: ClassVar[tuple[ModelInfo, ...]] = (
        ModelInfo(
            id="anthropic/claude-3-5-sonnet-20241022", provider="anthropic",
            name="claude-3-5-sonnet-20241022", display_name="Claude 3.5 Sonnet",
            context_window=200_000, input_price_per_1k=0.0030, output_price_per_1k=0.0150,
            supports_vision=True, tier="flagship",
        ),
        ModelInfo(
            id="anthropic/claude-3-5-haiku-20241022", provider="anthropic",
            name="claude-3-5-haiku-20241022", display_name="Claude 3.5 Haiku",
            context_window=200_000, input_price_per_1k=0.0008, output_price_per_1k=0.0040,
            supports_vision=False, tier="standard",
        ),
        ModelInfo(
            id="anthropic/claude-3-opus-20240229", provider="anthropic",
            name="claude-3-opus-20240229", display_name="Claude 3 Opus",
            context_window=200_000, input_price_per_1k=0.0150, output_price_per_1k=0.0750,
            supports_vision=True, tier="flagship",
        ),
        ModelInfo(
            id="anthropic/claude-3-haiku-20240307", provider="anthropic",
            name="claude-3-haiku-20240307", display_name="Claude 3 Haiku",
            context_window=200_000, input_price_per_1k=0.00025, output_price_per_1k=0.00125,
            supports_vision=True, tier="budget",
        ),
    )

    @property
    def api_key(self) -> str | None:
        return self.settings.anthropic_api_key

    @property
    def base_url(self) -> str:
        return self.settings.anthropic_base_url

    def _headers(self) -> dict:
        return {
            "x-api-key": self.api_key or "",
            "anthropic-version": self.settings.anthropic_version,
        }

    async def complete(self, request: ChatCompletionRequest, model: ModelInfo) -> ProviderResult:
        if self.use_mock():
            return self._mock_result(request, model)

        system_parts = [m.content for m in request.messages if m.role == "system"]
        convo: list[dict] = []
        for m in request.messages:
            if m.role == "system":
                continue
            role = "assistant" if m.role == "assistant" else "user"
            # Anthropic requires alternating roles — merge consecutive ones.
            if convo and convo[-1]["role"] == role:
                convo[-1]["content"] += "\n" + m.content
            else:
                convo.append({"role": role, "content": m.content})
        if not convo:
            convo = [{"role": "user", "content": ""}]

        payload: dict = {
            "model": model.name,
            "max_tokens": request.max_tokens or 1024,
            "messages": convo,
            "temperature": request.temperature,
        }
        if system_parts:
            payload["system"] = "\n".join(system_parts)

        resp = await self._request("POST", "/messages", headers=self._headers(), json_body=payload)
        data = resp.json()
        blocks = data.get("content") or []
        text = "".join(b.get("text", "") for b in blocks if isinstance(b, dict) and b.get("type") == "text")
        usage = data.get("usage") or {}
        return ProviderResult(
            model_id=model.id,
            provider=self.name,
            content=text,
            finish_reason=data.get("stop_reason") or "stop",
            prompt_tokens=usage.get("input_tokens") or estimate_messages_tokens(request.messages),
            completion_tokens=usage.get("output_tokens") or estimate_tokens(text),
        )

    async def _ping(self) -> None:
        await self._request("GET", "/models", headers=self._headers())
