"""API-key authentication for platform clients (Stage 1 deliverable).

Keys are configured through `API_KEYS` / `MASTER_API_KEY`. When no keys are
configured the platform runs in open development mode and every request is
treated as anonymous with admin rights.
"""

from __future__ import annotations

import hashlib
import hmac
from dataclasses import dataclass

from fastapi import Depends, Request

from app.core.exceptions import AuthenticationError, AuthorizationError


@dataclass(frozen=True)
class ApiKeyContext:
    key_id: str           # stable, non-reversible identifier used in logs/usage records
    is_master: bool
    authenticated: bool


def hash_api_key(raw: str) -> str:
    return hashlib.sha256(raw.encode()).hexdigest()[:16]


async def get_api_key_context(request: Request) -> ApiKeyContext:
    settings = request.app.state.settings
    raw = request.headers.get("x-api-key")
    if not raw:
        auth = request.headers.get("authorization", "")
        if auth.lower().startswith("bearer "):
            raw = auth[7:].strip()

    if not settings.auth_enabled:
        return ApiKeyContext(key_id="anonymous", is_master=True, authenticated=False)
    if not raw:
        raise AuthenticationError(
            "API key required. Pass it via the 'X-API-Key' header or 'Authorization: Bearer <key>'."
        )
    if settings.master_api_key and hmac.compare_digest(raw, settings.master_api_key):
        return ApiKeyContext(key_id=hash_api_key(raw), is_master=True, authenticated=True)
    for key in settings.parsed_api_keys:
        if hmac.compare_digest(raw, key):
            return ApiKeyContext(key_id=hash_api_key(raw), is_master=False, authenticated=True)
    raise AuthenticationError("Invalid API key.")


async def require_admin(
    request: Request,
    ctx: ApiKeyContext = Depends(get_api_key_context),
) -> ApiKeyContext:
    """Admin endpoints require the master key when one is configured."""
    settings = request.app.state.settings
    if settings.master_api_key and not ctx.is_master:
        raise AuthorizationError("This endpoint requires the master API key.")
    return ctx
