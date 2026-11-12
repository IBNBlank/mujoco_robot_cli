import base64
import io
import json
import threading
import time

import httpx
import pytest
import uvicorn

from mujoco_robot_cli.api import create_app
from mujoco_robot_cli.config import load_settings
from mujoco_robot_cli.simulation.worker import WorkerClient


@pytest.fixture(scope="module")
def server():
    settings = load_settings(overrides={"port": 8765, "startup_timeout_s": 30, "headless": True, "render_backend": "egl"})
    worker = WorkerClient(settings)
    worker.start()
    app = create_app(settings, worker)
    config = uvicorn.Config(app, host="127.0.0.1", port=8765, log_level="warning")
    instance = uvicorn.Server(config)
    thread = threading.Thread(target=instance.run, daemon=True)
    thread.start()
    deadline = time.monotonic() + 10
    while time.monotonic() < deadline:
        try:
            if httpx.get("http://127.0.0.1:8765/healthz", timeout=1).status_code == 200:
                break
        except httpx.HTTPError:
            time.sleep(0.1)
    yield "http://127.0.0.1:8765"
    instance.should_exit = True
    thread.join(5)
    worker.close()


def test_idle_server_keeps_publishing_without_image_readers(server):
    from concurrent.futures import ThreadPoolExecutor

    # Leave both one-slot queues unread for longer than the freshness limit.
    time.sleep(1)
    with httpx.Client(base_url=server, timeout=5) as client:
        first = client.get("/v1/arm/joints")
        assert first.status_code == 200, first.text
        first_step = first.json()["data"]["step"]
        time.sleep(0.5)
        with ThreadPoolExecutor(max_workers=8) as pool:
            replies = list(pool.map(lambda _: client.get("/v1/arm/joints"), range(24)))
        for reply in replies:
            assert reply.status_code == 200, reply.text
            assert reply.json()["data"]["step"] > first_step
        image = client.get("/v1/camera/image")
        assert image.status_code == 200, image.text


def test_queries_actions_and_errors(server):
    joints = httpx.post(f"{server}/v1/arm/joints", json={"positions": [0.1, -0.2, 0.4, -0.2, 0.1, 0.05], "timeout_s": 10}, timeout=20)
    assert joints.status_code == 200, joints.text
    pose = httpx.get(f"{server}/v1/arm/pose", timeout=5)
    assert pose.json()["ok"] and len(pose.json()["data"]["position"]) == 3
    image = httpx.get(f"{server}/v1/camera/image", timeout=5)
    assert image.status_code == 200 and image.content.startswith(b"\x89PNG")
    import io
    import numpy as np
    from PIL import Image
    rear_rgb = httpx.get(f"{server}/v1/camera/image?camera=left_rear&type=rgb", timeout=5)
    rear_depth = httpx.get(f"{server}/v1/camera/image?camera=left_rear&type=depth", timeout=5)
    assert rear_rgb.status_code == rear_depth.status_code == 200
    rgb = np.array(Image.open(io.BytesIO(rear_rgb.content)))
    depth = np.array(Image.open(io.BytesIO(rear_depth.content)))
    assert rgb.shape == (224, 224, 3)
    assert depth.shape == (224, 224) and depth.max() > 0
    assert rear_rgb.content != image.content
    assert httpx.get(f"{server}/v1/camera/image?camera=unknown", timeout=5).status_code == 422
    invalid = httpx.post(f"{server}/v1/arm/joints", json={"positions": [9, 0, 0, 0, 0, 0]}, timeout=5)
    assert invalid.status_code == 422 and invalid.json()["error"]["code"] == "INVALID_ARGUMENT"
    blocked = httpx.post(f"{server}/v1/object/pose", json={"position": [0.5, 0, -0.15]}, timeout=5)
    assert blocked.status_code == 422
    pose = httpx.get(f"{server}/v1/object/pose", timeout=5)
    assert pose.status_code == 200 and len(pose.json()["data"]["position"]) == 3
    outside = httpx.get(f"{server}/v1/object/in-target", timeout=5)
    assert outside.json()["data"]["in_target"] is False
    placed = httpx.post(f"{server}/v1/object/pose", json={"position": [0.6, 0.0, 0.07], "settle_s": 0.05}, timeout=10)
    assert placed.status_code == 200
    inside = httpx.get(f"{server}/v1/object/in-target", timeout=5)
    assert inside.json()["data"]["in_target"] is True
    reset = httpx.post(f"{server}/v1/environment/reset", json={"settle_s": 0.05}, timeout=10)
    assert reset.status_code == 200


def test_tcp_delta_motion_and_pixel_point(server):
    import numpy as np
    from PIL import Image
    from mujoco_robot_cli.simulation.scene import local_delta_pose, quaternion_error

    with httpx.Client(base_url=server, timeout=20) as client:
        moved = client.post("/v1/arm/joints", json={"positions": [.1, -.4, .8, -.4, .1, .05], "timeout_s": 10})
        assert moved.status_code == 200, moved.text
        before = client.get("/v1/arm/pose").json()["data"]
        delta = {"delta_position": [.005, 0, 0], "delta_rpy": [0, .01, 0], "timeout_s": 10}
        expected = local_delta_pose(np.array(before["position"]), np.array(before["quaternion"]), delta["delta_position"], delta["delta_rpy"])
        result = client.post("/v1/arm/pose/delta", json=delta)
        assert result.status_code == 200, result.text
        assert result.json()["data"]["result"]["status"] == "reached", result.text
        state = result.json()["data"]["state"]
        assert state["pose"]["frame"] == state["object"]["frame"] == "base"
        after = client.get("/v1/arm/pose").json()["data"]
        assert np.linalg.norm(np.array(after["position"]) - expected[0]) < .003
        assert quaternion_error(np.array(after["quaternion"]), expected[1]) < .03
        depth_frame = client.get("/v1/camera/image", params={"camera": "left_rear", "type": "depth"})
        depth_image = np.array(Image.open(io.BytesIO(depth_frame.content)))
        v, u = map(int, np.argwhere(depth_image > 0)[0])
        capture = {key: json.loads(base64.urlsafe_b64decode(depth_frame.headers[header])) for header, key in (
            ("x-tcp-position-base", "tcp_position_base"), ("x-tcp-quaternion-base", "tcp_quaternion_base"),
            ("x-camera-position-tcp", "camera_position_tcp"), ("x-camera-rotation-tcp", "camera_rotation_tcp"),
            ("x-intrinsics", "intrinsics"))}
        point = client.post("/v1/camera/point", json={
            "camera": "left_rear", "pixel": [u, v], "depth_m": float(depth_image[v, u]) / 1000,
            **capture, "epoch": int(depth_frame.headers["x-epoch"]), "step": int(depth_frame.headers["x-step"]),
        })
        assert point.status_code == 200, point.text
        assert point.json()["data"]["frame"] == "base"


def test_cli_disconnected_server():
    from mujoco_robot_cli.cli import main
    assert main(["--server", "http://127.0.0.1:9", "joints", "get"]) == 3
