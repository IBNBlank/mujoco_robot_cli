from __future__ import annotations

from contextlib import asynccontextmanager
import io
import time
from uuid import uuid4

import numpy as np
from fastapi import FastAPI, Request
from fastapi.responses import JSONResponse, Response
from PIL import Image

from .config import Settings
from .schemas import Envelope, ErrorBody, GripperCommand, JointCommand, ObjectCommand, PoseCommand, ResetCommand
from .simulation.scene import CAMERAS
from .simulation.worker import WorkerClient


class ServiceError(Exception):
    def __init__(self, code: str, message: str, status: int, data: dict | None = None) -> None:
        self.code = code
        self.message = message
        self.status = status
        self.data = data


def create_app(settings: Settings, worker: WorkerClient | None = None) -> FastAPI:
    @asynccontextmanager
    async def lifespan(app: FastAPI):
        try:
            if app.state.owns_worker:
                app.state.worker.start()
            yield
        finally:
            if app.state.owns_worker:
                app.state.worker.close()

    app = FastAPI(title="mujoco_robot_cli", version="0.1.0", lifespan=lifespan)
    app.state.settings = settings
    app.state.worker = worker or WorkerClient(settings)
    app.state.owns_worker = worker is None

    @app.exception_handler(ServiceError)
    def service_error(_: Request, exc: ServiceError) -> JSONResponse:
        body = Envelope(ok=False, request_id=getattr(getattr(_, "state", None), "request_id", ""), data=exc.data, error=ErrorBody(code=exc.code, message=exc.message))
        return JSONResponse(body.model_dump(), status_code=exc.status)

    @app.middleware("http")
    async def request_context(request: Request, call_next):
        request.state.request_id = request.headers.get("x-request-id", uuid4().hex)
        try:
            response = await call_next(request)
        except Exception:
            body = Envelope(ok=False, request_id=request.state.request_id, error=ErrorBody(code="INTERNAL_ERROR", message="internal server error"))
            return JSONResponse(body.model_dump(), status_code=500)
        response.headers["x-request-id"] = request.state.request_id
        return response

    @app.get("/healthz")
    def health(request: Request):
        return ok(request, {"alive": app.state.worker.process.is_alive()})

    @app.get("/readyz")
    def ready(request: Request):
        snap = fresh(app.state.worker, settings)
        return ok(request, {"ready": True, "epoch": snap["epoch"]})

    @app.get("/v1/capabilities")
    def capabilities(request: Request):
        return ok(request, settings.model_dump() | {"protocol": "0.1.0", "robot": "archer_y6", "tcp": "ee_frame", "quaternion": "wxyz", "cameras": list(CAMERAS)})

    @app.get("/v1/arm/joints")
    def get_joints(request: Request):
        snap = fresh(app.state.worker, settings)
        return ok(request, snap["joints"] | meta(snap))

    @app.post("/v1/arm/joints")
    def set_joints(command: JointCommand, request: Request):
        return action(app, request, "joints", command.model_dump(), command.timeout_s)

    @app.get("/v1/arm/pose")
    def get_pose(request: Request, frame: str = "world"):
        snap = fresh(app.state.worker, settings)
        if frame not in {"world", "base"}:
            raise ServiceError("INVALID_ARGUMENT", "unsupported frame", 422)
        from .simulation.scene import transform
        position, quaternion = transform(np.array(snap["pose"]["position"]), np.array(snap["pose"]["quaternion"]), "world", frame, np.array(snap["pose"]["base_position"]), np.array(snap["pose"]["base_quaternion"]))
        return ok(request, {"frame": frame, "position": position.tolist(), "quaternion": quaternion.tolist()} | meta(snap))

    @app.post("/v1/arm/pose")
    def set_pose(command: PoseCommand, request: Request):
        return action(app, request, "pose", command.model_dump(), command.timeout_s)

    @app.get("/v1/gripper")
    def get_gripper(request: Request):
        snap = fresh(app.state.worker, settings)
        return ok(request, gripper_view(snap) | meta(snap))

    @app.post("/v1/gripper")
    def set_gripper(command: GripperCommand, request: Request):
        return action(app, request, "gripper", command.model_dump(), command.timeout_s)

    @app.get("/v1/object/pose")
    def get_object(request: Request):
        snap = fresh(app.state.worker, settings)
        return ok(request, object_view(snap) | meta(snap))

    @app.get("/v1/object/in-target")
    def object_in_target(request: Request):
        snap = fresh(app.state.worker, settings)
        view = object_view(snap)
        return ok(request, {"in_target": view["in_target"], "position": view["position"], "target": view["target"]} | meta(snap))

    @app.post("/v1/object/pose")
    def set_object(command: ObjectCommand, request: Request):
        return action(app, request, "object", command.model_dump(), command.timeout_s)

    @app.post("/v1/environment/reset")
    def reset(command: ResetCommand, request: Request):
        return action(app, request, "reset", command.model_dump(), command.timeout_s)

    @app.get("/v1/camera/image")
    def image(camera: str = "end", type: str = "rgb"):
        if camera not in CAMERAS:
            raise ServiceError("INVALID_ARGUMENT", f"camera must be one of: {', '.join(CAMERAS)}", 422)
        if type not in {"rgb", "depth"}:
            raise ServiceError("INVALID_ARGUMENT", "type must be rgb or depth", 422)
        image = app.state.worker.latest_image(camera)
        if image is None or time.monotonic() - image["monotonic_s"] > settings.freshness_s:
            raise ServiceError("NOT_READY", "image is stale", 503)
        array = image[type]
        output = io.BytesIO()
        Image.fromarray(array).save(output, format="PNG")
        return Response(output.getvalue(), media_type="image/png", headers={"x-epoch": str(image["epoch"]), "x-step": str(image["step"]), "x-captured-at-ns": str(image["captured_at_ns"]), "x-width": str(image["width"]), "x-height": str(image["height"])})

    return app


def fresh(worker: WorkerClient, settings: Settings) -> dict:
    snap = worker.latest()
    if snap is None or time.monotonic() - snap["monotonic_s"] > settings.freshness_s:
        raise ServiceError("NOT_READY", "state is stale", 503)
    if not worker.process.is_alive():
        raise ServiceError("INTERNAL_ERROR", "simulation worker exited", 500)
    return snap


def meta(snap: dict) -> dict:
    return {key: snap[key] for key in ("epoch", "step", "sim_time_s", "captured_at_ns")}


def object_view(snap: dict) -> dict:
    position = snap["object"]["position"]
    target = {"center": [0.6, 0.0, 0.301], "half_size": [0.04, 0.04, 0.001]}
    inside_xy = all(abs(position[index] - target["center"][index]) <= target["half_size"][index] + 0.02 for index in (0, 1))
    on_table = abs(position[2] - 0.322) <= 0.015
    return snap["object"] | {"in_target": inside_xy and on_table, "target": target}


def gripper_view(snap: dict) -> dict:
    value = snap["gripper"]
    ratio = value["opening_ratio"]
    status = "moving" if value["moving"] else "open" if ratio > 0.95 else "closed" if ratio < 0.05 else "partial"
    return value | {"status": status}


def action(app: FastAPI, request: Request, kind: str, payload: dict, timeout_s: float | None):
    timeout = min(timeout_s or app.state.settings.default_timeout_s, app.state.settings.max_timeout_s)
    result = app.state.worker.submit(kind, payload, timeout)
    snap = app.state.worker.latest() or {}
    data = {"result": result, "state": snap}
    if not result.get("ok", False):
        code = result.get("code", "INTERNAL_ERROR")
        status = {"INVALID_ARGUMENT": 422, "IK_FAILED": 422, "BUSY": 409, "TIMEOUT": 504, "NOT_READY": 503}.get(code, 500)
        raise ServiceError(code, result.get("message", "action failed"), status, data)
    return ok(request, data)


def ok(request: Request, data: dict) -> dict:
    return Envelope(ok=True, request_id=request.state.request_id, data=data).model_dump()
