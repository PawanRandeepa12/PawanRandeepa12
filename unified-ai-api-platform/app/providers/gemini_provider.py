"""Google Gemini provider adapter (Stage 1)."""

from __future__ import annotations

from typing import ClassVar

from app.core.tokens import estimate_messages_tokens, estimate_tokens
from app.providers.base import ModelInfo, ProviderAdapter, ProviderResult
from app.schemas.chat import ChatCompletionRequest


class GeminiProvider(ProviderAdapter):
    name: ClassVar[str] = "gemini"
    catalog: ClassVar[tuple[ModelInfo, ...]] = (
        ModelInfo(
            id="gemini/gemini-1.5-pro", provider="gemini", name="gemini-1.5-pro",
            display_name="Gemini 1.5 Pro",
            context_window=2_000_000, input_price_per_1k=0.00125, output_price_per_1k=0.0050,
            supports_vision=True, tier="flagship",
        ),
        ModelInfo(
            id="gemini/gemini-1.5-flash", provider="gemini", name="gemini-1.5-flash",
            display_name="Gemini 1.5 Flash",
            context_window=1_000_000, input_price_per_1k=0.000075, output_price_per_1k=0.0003,
            supports_vision=True, tier="budget",
        ),
        ModelInfo(
            id="gemini/gemini-2.0-flash", provider="gemini", name="gemini-2.0-flash",
            display_name="Gemini 2.0 Flash",
            context_window=1_048_576, input_price_per_1k=0.0001, output_price_per_1k=0.0004,
            supports_vision=True, tier="standard",
        ),
    )

    @property
    def api_key(self) -> str | None:
        return self.settings.google_api_key

    @property
    def base_url(self) -> str:
        return self.settings.google_base_url

    async def complete(self, request: ChatCompletionRequest, model: ModelInfo) -> ProviderResult:
        if self.use_mock():
            return self._mock_result(request, model)

        system_parts = [m.content for m in request.messages if m.role == "system"]
        contents = []
        for m in request.messages:
            if m.role == "system":
                continue
            role = "model" if m.role == "assistant" else "user"
            contents.append({"role": role, "parts": [{"text": m.content}]})

        payload: dict = {
            "contents": contents,
            "generationConfig": {
                "temperature": request.temperature,
                "maxOutputTokens": request.max_tokens or 1024,
            },
        }
        if system_parts:
            payload["systemInstruction"] = {"parts": [{"text": "\n".join(system_parts)}]}

        resp = await self._request(
            "POST",
            f"/models/{model.name}:generateContent",
            params={"key": self.api_key},
            json_body=payload,
        )
        data = resp.json()
        candidates = data.get("candidates") or []
        first = candidates[0] if candidates else {}
        parts = ((first.get("content") or {}).get("parts")) or []
        text = "".join(p.get("text", "") for p in parts if isinstance(p, dict))
        meta = data.get("usageMetadata") or {}
        return ProviderResult(
            model_id=model.id,
            provider=self.name,
            content=text,
            finish_reason=(first.get("finishReason") or "STOP").lower(),
            prompt_tokens=meta.get("promptTokenCount") or estimate_messages_tokens(request.messages),
            completion_tokens=meta.get("candidatesTokenCount") or estimate_tokens(text),
        )

    async def _ping(self) -> None:
        await self._request("GET", "/models", params={"key": self.api_key})
