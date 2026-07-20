"""Provider registry — holds the configured adapters and the model index."""

from __future__ import annotations

from app.config import Settings
from app.providers.anthropic_provider import AnthropicProvider
from app.providers.base import ModelInfo, ProviderAdapter
from app.providers.gemini_provider import GeminiProvider
from app.providers.openai_provider import OpenAIProvider

PROVIDER_CLASSES: tuple[type[ProviderAdapter], ...] = (
    OpenAIProvider,
    AnthropicProvider,
    GeminiProvider,
)


class ProviderRegistry:
    def __init__(self, settings: Settings) -> None:
        self._adapters: dict[str, ProviderAdapter] = {cls.name: cls(settings) for cls in PROVIDER_CLASSES}
        self._model_index: dict[str, list[tuple[ProviderAdapter, ModelInfo]]] = {}
        for adapter in self._adapters.values():
            for model in adapter.models():
                self._model_index.setdefault(model.id.lower(), []).append((adapter, model))
                self._model_index.setdefault(model.name.lower(), []).append((adapter, model))

    @property
    def adapters(self) -> dict[str, ProviderAdapter]:
        return dict(self._adapters)

    def get(self, name: str) -> ProviderAdapter | None:
        return self._adapters.get(name)

    def all_models(self) -> list[tuple[ProviderAdapter, ModelInfo]]:
        return [(adapter, model) for adapter in self._adapters.values() for model in adapter.models()]

    def resolve(self, model_ref: str) -> list[tuple[ProviderAdapter, ModelInfo]]:
        """All (adapter, model) pairs matching a unified id or bare model name."""
        return list(self._model_index.get(model_ref.strip().lower(), []))

    async def aclose(self) -> None:
        for adapter in self._adapters.values():
            await adapter.aclose()
