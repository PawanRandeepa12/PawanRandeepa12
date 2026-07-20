"""Unified error model.

Every error the API returns uses the same envelope:

    {"error": {"type": ..., "message": ..., "details": {...}, "provider": ...}}

Provider failures are classified as retryable / non-retryable so the routing
layer (Stage 3) can decide whether to fail over to another provider.
"""

from __future__ import annotations

from fastapi import FastAPI, Request
from fastapi.encoders import jsonable_encoder
from fastapi.exceptions import RequestValidationError
from fastapi.responses import JSONResponse

from app.core.logging import get_logger

logger = get_logger(__name__)


class PlatformError(Exception):
    status_code = 500
    error_type = "internal_error"

    def __init__(self, message: str, *, details: dict | None = None) -> None:
        super().__init__(message)
        self.message = message
        self.details = details or {}


class AuthenticationError(PlatformError):
    status_code = 401
    error_type = "authentication_error"


class AuthorizationError(PlatformError):
    status_code = 403
    error_type = "authorization_error"


class ModelNotFoundError(PlatformError):
    status_code = 404
    error_type = "model_not_found"


class NoAvailableProviderError(PlatformError):
    status_code = 503
    error_type = "no_available_provider"


class ComponentNotEnabledError(PlatformError):
    status_code = 501
    error_type = "component_not_enabled"


class BudgetExceededError(PlatformError):
    status_code = 429
    error_type = "budget_exceeded"


class RateLimitExceeded(PlatformError):
    status_code = 429
    error_type = "rate_limit_exceeded"

    def __init__(self, retry_after_seconds: float) -> None:
        super().__init__(f"Rate limit exceeded. Retry in {retry_after_seconds:.1f} seconds.")
        self.retry_after_seconds = retry_after_seconds


class ProviderError(PlatformError):
    """A call to an upstream AI provider failed."""

    status_code = 502
    error_type = "provider_error"
    retryable = False

    def __init__(
        self,
        provider: str,
        message: str,
        *,
        status: int | None = None,
        retryable: bool | None = None,
        details: dict | None = None,
    ) -> None:
        super().__init__(message, details=details)
        self.provider = provider
        self.provider_status = status
        if retryable is not None:
            self.retryable = retryable


class ProviderTimeout(ProviderError):
    status_code = 504
    error_type = "provider_timeout"
    retryable = True


class ProviderUnavailable(ProviderError):
    status_code = 503
    error_type = "provider_unavailable"
    retryable = True


class ProviderAuthenticationError(ProviderError):
    status_code = 500
    error_type = "provider_authentication_error"
    retryable = False


def error_payload(exc: PlatformError) -> dict:
    body: dict = {"error": {"type": exc.error_type, "message": exc.message}}
    if exc.details:
        body["error"]["details"] = exc.details
    if isinstance(exc, ProviderError):
        body["error"]["provider"] = exc.provider
        if exc.provider_status is not None:
            body["error"]["provider_status"] = exc.provider_status
        body["error"]["retryable"] = exc.retryable
    return body


def register_exception_handlers(app: FastAPI) -> None:
    @app.exception_handler(PlatformError)
    async def _platform_error_handler(request: Request, exc: PlatformError) -> JSONResponse:  # noqa: ARG001
        headers: dict[str, str] = {}
        if isinstance(exc, RateLimitExceeded):
            headers["Retry-After"] = str(max(1, round(exc.retry_after_seconds)))
        if isinstance(exc, ProviderError) and exc.status_code >= 500:
            logger.warning("provider failure: %s", exc.message, extra={"provider": exc.provider})
        return JSONResponse(status_code=exc.status_code, content=error_payload(exc), headers=headers)

    @app.exception_handler(RequestValidationError)
    async def _validation_error_handler(request: Request, exc: RequestValidationError) -> JSONResponse:  # noqa: ARG001
        return JSONResponse(
            status_code=422,
            content={
                "error": {
                    "type": "validation_error",
                    "message": "Request validation failed.",
                    "details": jsonable_encoder(exc.errors()),
                }
            },
        )

    @app.exception_handler(Exception)
    async def _unhandled_error_handler(request: Request, exc: Exception) -> JSONResponse:  # noqa: ARG001
        logger.exception("unhandled exception while processing request")
        return JSONResponse(
            status_code=500,
            content={"error": {"type": "internal_error", "message": "Unexpected internal error."}},
        )
