#!/usr/bin/env python3
"""Seven-step pick/place demo using privileged object pose queries."""
from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from mujoco_robot_cli.client import ClientFailure, RobotClient
from mujoco_robot_cli.simulation.scene import rotate_vector
import numpy as np

# TCP +X is the approach direction: world +X/-Z at 45 degrees.
GRASP_QUATERNION = [float(np.cos(np.pi / 8)), 0.0, float(np.sin(np.pi / 8)), 0.0]


class DemoError(RuntimeError):
    pass


def run(client: RobotClient, *, target: list[float], grasp_offset: list[float],
        clearance: float, timeout_s: float) -> dict:
    """Use Cartesian waypoints; return to the joints recorded before opening."""
    initial_state = client.request("GET", "/v1/arm/joints")["data"]
    initial = initial_state["positions"]
    if "limits" in initial_state:
        # MuJoCo soft constraints can leave a measured joint a fraction outside its limit.
        initial = np.clip(initial, np.asarray(initial_state["limits"])[:, 0], np.asarray(initial_state["limits"])[:, 1]).tolist()

    def gripper(action: str) -> None:
        result = client.request("POST", "/v1/gripper", {"action": action, "timeout_s": timeout_s})
        if action == "open" and result["data"]["result"].get("status") != "reached":
            raise DemoError("gripper did not open completely")

    def move(position: list[float]) -> None:
        client.request("POST", "/v1/arm/pose", {
            "position": position, "quaternion": GRASP_QUATERNION,
            "frame": "world", "timeout_s": timeout_s,
        })

    print("1/7 open gripper", flush=True)
    gripper("open")
    print("2/7 query object pose", flush=True)
    obj = client.request("GET", "/v1/object/pose")["data"]["position"]
    # Offset is expressed in TCP axes, so it rotates with the grasp orientation.
    rotation = np.asarray(GRASP_QUATERNION)
    offset_world = rotate_vector(rotation, np.asarray(grasp_offset, dtype=float))
    grasp = (np.asarray(obj) + offset_world).tolist()
    release = (np.asarray(target) + offset_world).tolist()
    travel_z = max(grasp[2], release[2]) + clearance
    print("fixed grasp orientation wxyz", GRASP_QUATERNION, "TCP offset world", offset_world.tolist(), flush=True)
    print("3/7 approach object", grasp, flush=True)
    move([grasp[0], grasp[1], travel_z])
    move(grasp)
    print("4/7 close gripper", flush=True)
    gripper("close")
    print("5/7 lift and move to target", target, flush=True)
    move([grasp[0], grasp[1], travel_z])
    lifted = client.request("GET", "/v1/object/pose")["data"]["position"]
    print("lifted box", lifted, flush=True)
    if lifted[2] - obj[2] < clearance * 0.4:
        raise DemoError("box did not lift with the gripper; stopping before transfer")
    actual_tcp = client.request("GET", "/v1/arm/pose")["data"]["position"]
    carried_offset = np.asarray(lifted) - np.asarray(actual_tcp)
    release = (np.asarray(target) - carried_offset).tolist()
    move([release[0], release[1], travel_z])
    # Reobserve after transport to compensate for any small in-hand displacement.
    transported = client.request("GET", "/v1/object/pose")["data"]["position"]
    if transported[2] < obj[2] + clearance * 0.4:
        raise DemoError("box slipped during transport; stopping before release")
    actual_tcp = client.request("GET", "/v1/arm/pose")["data"]["position"]
    release = (np.asarray(actual_tcp) + np.asarray(target) - np.asarray(transported)).tolist()
    print("release TCP", release, "transported box", transported, flush=True)
    move(release)
    print("6/7 open gripper", flush=True)
    gripper("open")
    print("7/7 retreat and return to initial joints", flush=True)
    move([release[0], release[1], travel_z])
    client.request("POST", "/v1/arm/joints", {"positions": initial, "timeout_s": timeout_s})
    time.sleep(0.5)
    check = client.request("GET", "/v1/object/in-target")["data"]
    final = client.request("GET", "/v1/object/pose")["data"]
    # Require table height as well: the current privileged API also accepts boxes in the air.
    if not check["in_target"] or abs(final["position"][2] - 0.322) > 0.015:
        raise DemoError(f"box is not resting in target: {final['position']}")
    return {"ok": True, "object": final, "in_target": True}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--server", default="http://127.0.0.1:8000")
    parser.add_argument("--timeout", type=float, default=20)
    parser.add_argument("--target", nargs=3, type=float, default=[0.6, 0.0, 0.332], help="desired box centre at release, world metres")
    parser.add_argument("--grasp-offset", nargs=3, type=float, default=[0.015, 0.0, 0.0016], help="object-centre to TCP vector expressed in TCP axes (metres)")
    parser.add_argument("--clearance", type=float, default=0.12)
    args = parser.parse_args()
    if args.timeout <= 0 or args.clearance <= 0:
        parser.error("timeout and clearance must be positive")
    try:
        result = run(RobotClient(args.server, timeout_s=args.timeout + 12), target=args.target,
                     grasp_offset=args.grasp_offset,
                     clearance=args.clearance, timeout_s=args.timeout)
    except (ClientFailure, DemoError) as exc:
        print(json.dumps({"ok": False, "error": str(exc)}, ensure_ascii=False), flush=True)
        return 2
    print(json.dumps(result, ensure_ascii=False), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
