"""HTTP middleware (Stage 1: request id; Stage 4: metrics)."""

from __future__ import annotations

import time
import uuid

from starlette.middleware.base import BaseHTTPMiddleware
from starlette.requests import Request
from starlette.responses import Response

from app.core.logging import request_id_ctx


class RequestIDMiddleware(BaseHTTPMiddleware):
    """Attach/propagate a request correlation id."""

    async def dispatch(self, request: Request, call_next) -> Response:
        request_id = request.headers.get("x-request-id") or uuid.uuid4().hex[:12]
        request.state.request_id = request_id
        token = request_id_ctx.set(request_id)
        try:
            response = await call_next(request)
        finally:
            request_id_ctx.reset(token)
        response.headers["X-Request-ID"] = request_id
        return response


class MetricsMiddleware(BaseHTTPMiddleware):
    """Record HTTP request counts & latency into the metrics registry."""

    async def dispatch(self, request: Request, call_next) -> Response:
        registry = getattr(request.app.state, "metrics", None)
        if registry is None or request.url.path == "/metrics":
            return await call_next(request)
        started = time.perf_counter()
        response = await call_next(request)
        path = self._normalize_path(request.url.path)
        registry.http_requests.inc(method=request.method, path=path, status=response.status_code)
        registry.http_duration.observe(time.perf_counter() - started, method=request.method, path=path)
        return response

    @staticmethod
    def _normalize_path(path: str) -> str:
        # Collapse path parameters to keep label cardinality bounded.
        if path.startswith("/v1/models/"):
            return "/v1/models/:ref"
        if path.startswith("/v1/routing/health/"):
            return "/v1/routing/health/:provider/reset"
        if path.startswith("/v1/cost/budgets/"):
            return "/v1/cost/budgets/:id"
        return path
