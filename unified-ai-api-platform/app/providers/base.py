"""Provider adapter base classes.

Every upstream AI provider implements `ProviderAdapter`. The base class
handles: model catalog metadata, mock-mode detection, shared HTTP plumbing
with timeout / error classification, and health checks.

Errors raised through `_request` are classified as retryable vs non-retryable
(`ProviderError.retryable`) which the Stage 3 router uses for failovers and
the circuit breaker.
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import ClassVar

import httpx

from app.config import Settings
from app.core.exceptions import (
    ProviderAuthenticationError,
    ProviderError,
    ProviderTimeout,
    ProviderUnavailable,
)
from app.schemas.chat import ChatCompletionRequest


@dataclass(frozen=True)
class ModelInfo:
    """Static metadata for one model offered by a provider."""

    id: str                    # unified id, e.g. "openai/gpt-4o-mini"
    provider: str              # provider key, e.g. "openai"
    name: str                  # native model name sent to the provider API
    display_name: str
    context_window: int
    input_price_per_1k: float  # USD per 1K tokens (fallback when pricing file lacks the model)
    output_price_per_1k: float
    supports_vision: bool = False
    supports_tools: bool = True
    tier: str = "standard"     # flagship | standard | budget


@dataclass
class ProviderResult:
    """Normalized result of one completion call."""

    model_id: str
    provider: str
    content: str
    finish_reason: str
    prompt_tokens: int
    completion_tokens: int
    mock: bool = False

    @property
    def total_tokens(self) -> int:
        return self.prompt_tokens + self.completion_tokens


class ProviderAdapter(ABC):
    name: ClassVar[str] = "base"
    catalog: ClassVar[tuple[ModelInfo, ...]] = ()

    def __init__(self, settings: Settings) -> None:
        self.settings = settings
        self._client: httpx.AsyncClient | None = None

    # ------------------------------------------------------------ configuration
    @property
    @abstractmethod
    def api_key(self) -> str | None: ...

    @property
    @abstractmethod
    def base_url(self) -> str: ...

    def is_configured(self) -> bool:
        return bool(self.api_key)

    def use_mock(self) -> bool:
        mode = self.settings.mock_providers
        if mode == "always":
            return True
        if mode == "never":
            return False
        return not self.is_configured()  # auto

    def mode(self) -> str:
        if not self.is_configured() and self.settings.mock_providers == "never":
            return "unconfigured"
        return "mock" if self.use_mock() else "live"

    # ------------------------------------------------------------------- catalog
    def models(self) -> list[ModelInfo]:
        return list(self.catalog)

    def get_model(self, ref: str) -> ModelInfo | None:
        ref_l = ref.strip().lower()
        for model in self.catalog:
            if ref_l in (model.id.lower(), model.name.lower()):
                return model
        return None

    # ------------------------------------------------------------------ http/io
    async def client(self) -> httpx.AsyncClient:
        if self._client is None:
            self._client = httpx.AsyncClient(
                base_url=self.base_url.rstrip("/"),
                timeout=httpx.Timeout(self.settings.provider_timeout_seconds),
            )
        return self._client

    async def aclose(self) -> None:
        if self._client is not None:
            await self._client.aclose()
            self._client = None

    @abstractmethod
    async def complete(self, request: ChatCompletionRequest, model: ModelInfo) -> ProviderResult: ...

    async def health_check(self) -> tuple[bool, str]:
        if self.use_mock():
            return True, "mock mode (no live traffic)"
        if not self.is_configured():
            return False, "missing API key"
        try:
            await self._ping()
            return True, "ok"
        except Exception as exc:  # noqa: BLE001 - surfaced as health detail
            return False, str(exc)

    async def _ping(self) -> None:
        """Lightweight liveness probe; subclasses override."""
        raise NotImplementedError

    # ------------------------------------------------------------------ helpers
    def _mock_result(self, request: ChatCompletionRequest, model: ModelInfo) -> ProviderResult:
        from app.providers.mock import mock_completion  # late import avoids a cycle

        return mock_completion(request, model)

    async def _request(
        self,
        method: str,
        path: str,
        *,
        headers: dict | None = None,
        params: dict | None = None,
        json_body: dict | None = None,
    ) -> httpx.Response:
        client = await self.client()
        try:
            resp = await client.request(method, path, headers=headers, params=params, json=json_body)
        except httpx.TimeoutException as exc:
            raise ProviderTimeout(self.name, f"{self.name} request timed out.") from exc
        except httpx.HTTPError as exc:
            raise ProviderUnavailable(self.name, f"Could not reach {self.name}: {exc}") from exc
        self._raise_for_status(resp)
        return resp

    def _raise_for_status(self, resp: httpx.Response) -> None:
        if resp.status_code < 400:
            return
        status = resp.status_code
        message = f"{self.name} returned HTTP {status}: {self._extract_error(resp)}"
        if status in (401, 403):
            raise ProviderAuthenticationError(self.name, message, status=status)
        if status == 429 or status >= 500:
            raise ProviderUnavailable(self.name, message, status=status, retryable=True)
        raise ProviderError(self.name, message, status=status, retryable=False)

    @staticmethod
    def _extract_error(resp: httpx.Response) -> str:
        try:
            data = resp.json()
            if isinstance(data, dict):
                err = data.get("error")
                if isinstance(err, dict):
                    return str(err.get("message") or err)[:300]
                if isinstance(err, str):
                    return err[:300]
        except ValueError:
            pass
        return (resp.text or "")[:300]
