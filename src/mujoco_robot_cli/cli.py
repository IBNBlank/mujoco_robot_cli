from __future__ import annotations

import argparse
import base64
import json
import sys

from .client import ClientFailure, RobotClient
from .schemas import DeltaPoseCommand, GripperCommand, JointCommand, ObjectCommand, PoseCommand, ResetCommand


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="One-shot client for mujoco_server")
    parser.add_argument("--server", default="http://127.0.0.1:8000")
    parser.add_argument("--timeout", type=float, default=20)
    parser.add_argument("--request-id", default=None)
    parser.add_argument("--request-json", default=None)
    sub = parser.add_subparsers(dest="command", required=True)
    for name in ("joints", "pose", "gripper", "image", "object", "environment"):
        item = sub.add_parser(name)
        item.add_argument("operation", nargs="?", default="get")
        item.add_argument("--positions", nargs=6, type=float)
        item.add_argument("--position", nargs=3, type=float)
        item.add_argument("--quaternion", nargs=4, type=float)
        item.add_argument("--frame", choices=["base"], default="base")
        item.add_argument("--delta-position", nargs=3, type=float, default=[0, 0, 0])
        item.add_argument("--delta-rpy", nargs=3, type=float, default=[0, 0, 0])
        item.add_argument("--pixel", nargs=2, type=int, metavar=("U", "V"))
        item.add_argument("--epoch", type=int)
        item.add_argument("--step", type=int)
        item.add_argument("--depth-m", type=float)
        item.add_argument("--tcp-position-base", nargs=3, type=float)
        item.add_argument("--tcp-quaternion-base", nargs=4, type=float)
        item.add_argument("--camera-position-tcp", nargs=3, type=float)
        item.add_argument("--camera-rotation-tcp", nargs=9, type=float)
        item.add_argument("--intrinsics", nargs=6, type=float, metavar=("FX", "FY", "CX", "CY", "WIDTH", "HEIGHT"))
        item.add_argument("--opening-ratio", type=float)
        item.add_argument("--camera", choices=["end", "opposite", "left"], default="end")
        item.add_argument("--type", default="rgb")
        item.add_argument("--output")
        item.add_argument("--base64", action="store_true")
        item.add_argument("--object-id", choices=["red", "orange", "yellow"], default="red")
        item.add_argument("--settle", type=float, default=0)
    args = parser.parse_args(argv)
    client = RobotClient(args.server, args.timeout + 5, args.request_id)
    try:
        payload = json.loads(args.request_json) if args.request_json else None
        result = dispatch(client, args, payload)
    except ClientFailure as exc:
        emit(exc.body)
        return exc.exit_code
    except (ValueError, json.JSONDecodeError) as exc:
        emit({"ok": False, "error": {"code": "INVALID_ARGUMENT", "message": str(exc)}})
        return 2
    emit(result)
    return 0


def dispatch(client: RobotClient, args, payload: dict | None) -> dict:
    if args.command == "joints" and args.operation == "set":
        data = payload or JointCommand(positions=required(args.positions, "positions"), timeout_s=args.timeout).model_dump()
        return client.request("POST", "/v1/arm/joints", data)
    if args.command == "joints":
        return client.request("GET", "/v1/arm/joints")
    if args.command == "pose" and args.operation == "delta":
        data = payload or DeltaPoseCommand(delta_position=args.delta_position, delta_rpy=args.delta_rpy, timeout_s=args.timeout).model_dump()
        return client.request("POST", "/v1/arm/pose/delta", data)
    if args.command == "pose" and args.operation == "set":
        data = payload or PoseCommand(position=required(args.position, "position"), quaternion=args.quaternion, frame=args.frame, timeout_s=args.timeout).model_dump()
        return client.request("POST", "/v1/arm/pose", data)
    if args.command == "pose":
        return client.request("GET", "/v1/arm/pose", params={"frame": args.frame})
    if args.command == "gripper" and args.operation in {"open", "close"}:
        data = payload or GripperCommand(action=args.operation, timeout_s=args.timeout).model_dump()
        return client.request("POST", "/v1/gripper", data)
    if args.command == "gripper" and args.operation == "set":
        data = payload or GripperCommand(opening_ratio=args.opening_ratio, timeout_s=args.timeout).model_dump()
        return client.request("POST", "/v1/gripper", data)
    if args.command == "gripper":
        return client.request("GET", "/v1/gripper")
    if args.command == "image" and args.operation == "point":
        data = payload if payload is not None else {
            "camera": args.camera, "pixel": required(args.pixel, "pixel"),
            "depth_m": required(args.depth_m, "depth-m"),
        }
        if payload is None:
            for key in ("tcp_position_base", "tcp_quaternion_base", "camera_position_tcp"):
                value = getattr(args, key)
                if value is not None:
                    data[key] = value
            if args.camera_rotation_tcp is not None:
                data["camera_rotation_tcp"] = [args.camera_rotation_tcp[index:index + 3] for index in (0, 3, 6)]
            if args.intrinsics is not None:
                data["intrinsics"] = point_intrinsics(args.intrinsics)
        if args.epoch is not None:
            data["epoch"] = args.epoch
        if args.step is not None:
            data["step"] = args.step
        return client.request("POST", "/v1/camera/point", data)
    if args.command == "image":
        response = client.request("GET", "/v1/camera/image", params={"camera": args.camera, "type": args.type})
        saved = client.save_image(response, args.output or "artifacts/end.png")
        if args.base64:
            saved["base64"] = base64.b64encode(response["content"]).decode()
        return {"ok": True, "data": saved}
    if args.command == "object" and args.operation == "list":
        return client.request("GET", "/v1/objects")
    if args.command == "object" and args.operation == "get":
        return client.request("GET", "/v1/object/pose", params={"object_id": args.object_id})
    if args.command == "object" and args.operation == "in-target":
        return client.request("GET", "/v1/object/in-target", params={"object_id": args.object_id})
    if args.command == "object":
        data = payload or ObjectCommand(object_id=args.object_id, position=required(args.position, "position"), quaternion=args.quaternion, settle_s=args.settle, timeout_s=args.timeout).model_dump()
        return client.request("POST", "/v1/object/pose", data)
    if args.command == "environment":
        data = payload or ResetCommand(settle_s=args.settle, timeout_s=args.timeout).model_dump()
        return client.request("POST", "/v1/environment/reset", data)
    raise ValueError("unsupported command")


def point_intrinsics(values):
    fx, fy, cx, cy, width, height = values
    if not width.is_integer() or not height.is_integer():
        raise ValueError("intrinsics width and height must be integers")
    return {"fx": fx, "fy": fy, "cx": cx, "cy": cy, "width": int(width), "height": int(height)}


def required(value, name):
    if value is None:
        raise ValueError(f"{name} is required")
    return value


def emit(value: dict) -> None:
    print(json.dumps(value, ensure_ascii=False), flush=True)


if __name__ == "__main__":
    sys.exit(main())
