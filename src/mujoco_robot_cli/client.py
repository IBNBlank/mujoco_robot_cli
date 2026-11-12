from __future__ import annotations

import json
from pathlib import Path
from typing import Any

import httpx


class RobotClient:
    def __init__(self, server: str = "http://127.0.0.1:8000", timeout_s: float = 20, request_id: str | None = None) -> None:
        self.server = server.rstrip("/")
        self.timeout_s = timeout_s
        self.request_id = request_id

    def request(self, method: str, path: str, payload: dict | None = None, params: dict | None = None) -> dict[str, Any]:
        headers = {"x-request-id": self.request_id} if self.request_id else {}
        try:
            response = httpx.request(method, self.server + path, json=payload, params=params, headers=headers, timeout=self.timeout_s)
        except httpx.TimeoutException as exc:
            raise ClientFailure(4, "TIMEOUT", str(exc)) from exc
        except httpx.HTTPError as exc:
            raise ClientFailure(3, "CONNECTION_FAILED", str(exc)) from exc
        if response.headers.get("content-type", "").startswith("image/"):
            return {"ok": True, "status_code": response.status_code, "content": response.content, "headers": dict(response.headers)}
        try:
            body = response.json()
        except json.JSONDecodeError as exc:
            raise ClientFailure(5, "INTERNAL_ERROR", response.text) from exc
        if response.status_code >= 500 and response.status_code != 504:
            raise ClientFailure(5, body.get("error", {}).get("code", "INTERNAL_ERROR"), body.get("error", {}).get("message", ""), body)
        if not body.get("ok", False):
            code = body.get("error", {}).get("code", "INVALID_ARGUMENT")
            raise ClientFailure(4 if code == "TIMEOUT" else 2, code, body.get("error", {}).get("message", ""), body)
        return body

    def save_image(self, response: dict, output: str) -> dict:
        path = Path(output)
        path.parent.mkdir(parents=True, exist_ok=True)
        temporary = path.with_suffix(path.suffix + ".tmp")
        temporary.write_bytes(response["content"])
        temporary.replace(path)
        return {"path": str(path.resolve()), "bytes": len(response["content"]), "headers": {key: response["headers"].get(key) for key in ("x-epoch", "x-step", "x-width", "x-height", "x-captured-at-ns", "x-tcp-position-base", "x-tcp-quaternion-base", "x-camera-position-tcp", "x-camera-rotation-tcp", "x-intrinsics", "content-type")}}


class ClientFailure(Exception):
    def __init__(self, exit_code: int, code: str, message: str, body: dict | None = None) -> None:
        self.exit_code = exit_code
        self.code = code
        self.message = message
        self.body = body or {"ok": False, "error": {"code": code, "message": message}}
