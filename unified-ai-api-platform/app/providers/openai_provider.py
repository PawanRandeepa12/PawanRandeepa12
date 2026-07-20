"""OpenAI provider adapter (Stage 1)."""

from __future__ import annotations

from typing import ClassVar

from app.core.tokens import estimate_messages_tokens, estimate_tokens
from app.providers.base import ModelInfo, ProviderAdapter, ProviderResult
from app.schemas.chat import ChatCompletionRequest


class OpenAIProvider(ProviderAdapter):
    name: ClassVar[str] = "openai"
    catalog: ClassVar[tuple[ModelInfo, ...]] = (
        ModelInfo(
            id="openai/gpt-4o", provider="openai", name="gpt-4o", display_name="GPT-4o",
            context_window=128_000, input_price_per_1k=0.0025, output_price_per_1k=0.0100,
            supports_vision=True, tier="flagship",
        ),
        ModelInfo(
            id="openai/gpt-4o-mini", provider="openai", name="gpt-4o-mini", display_name="GPT-4o mini",
            context_window=128_000, input_price_per_1k=0.00015, output_price_per_1k=0.0006,
            supports_vision=True, tier="budget",
        ),
        ModelInfo(
            id="openai/gpt-4-turbo", provider="openai", name="gpt-4-turbo", display_name="GPT-4 Turbo",
            context_window=128_000, input_price_per_1k=0.0100, output_price_per_1k=0.0300,
            supports_vision=True, tier="flagship",
        ),
        ModelInfo(
            id="openai/gpt-3.5-turbo", provider="openai", name="gpt-3.5-turbo", display_name="GPT-3.5 Turbo",
            context_window=16_385, input_price_per_1k=0.0005, output_price_per_1k=0.0015,
            supports_vision=False, tier="budget",
        ),
    )

    @property
    def api_key(self) -> str | None:
        return self.settings.openai_api_key

    @property
    def base_url(self) -> str:
        return self.settings.openai_base_url

    async def complete(self, request: ChatCompletionRequest, model: ModelInfo) -> ProviderResult:
        if self.use_mock():
            return self._mock_result(request, model)

        payload: dict = {
            "model": model.name,
            "messages": [{"role": m.role, "content": m.content} for m in request.messages],
            "temperature": request.temperature,
        }
        if request.max_tokens is not None:
            payload["max_tokens"] = request.max_tokens

        resp = await self._request(
            "POST",
            "/chat/completions",
            headers={"Authorization": f"Bearer {self.api_key}"},
            json_body=payload,
        )
        data = resp.json()
        choice = (data.get("choices") or [{}])[0]
        usage = data.get("usage") or {}
        content = ((choice.get("message") or {}).get("content")) or ""
        return ProviderResult(
            model_id=model.id,
            provider=self.name,
            content=content,
            finish_reason=choice.get("finish_reason") or "stop",
            prompt_tokens=usage.get("prompt_tokens") or estimate_messages_tokens(request.messages),
            completion_tokens=usage.get("completion_tokens") or estimate_tokens(content),
        )

    async def _ping(self) -> None:
        await self._request("GET", "/models", headers={"Authorization": f"Bearer {self.api_key}"})
