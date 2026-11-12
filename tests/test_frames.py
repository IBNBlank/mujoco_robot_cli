"""Coordinate contracts, rendered depth and local motion composition."""
import base64
import json
import queue
import time

import mujoco
import numpy as np
import pytest
from fastapi.testclient import TestClient

from mujoco_robot_cli.api import create_app
from mujoco_robot_cli.config import Settings
from mujoco_robot_cli.simulation.scene import local_delta_pose, pixel_to_base, rotate_vector, transform
from mujoco_robot_cli.simulation.worker import Command, Simulation


@pytest.fixture
def simulation():
    sim = Simulation(Settings(headless=True, image_width=96, image_height=64))
    # A translated AND rotated base catches accidental world/base equivalence.
    parent = sim.model.body_parentid[sim.base]
    sim.model.body_pos[parent] = [0.12, -0.08, 0.25]
    sim.model.body_quat[parent] = [np.cos(.3), 0, 0, np.sin(.3)]
    sim.reset_state()
    yield sim
    sim.close()


def test_pose_snapshots_and_object_control_use_base(simulation):
    sim = simulation
    snap = sim.snapshot(time.monotonic())
    expected = transform(sim.data.xpos[sim.tcp], sim.data.xquat[sim.tcp], "world", "base", *sim.base_pose())
    assert snap["pose"]["frame"] == snap["object"]["frame"] == "base"
    np.testing.assert_allclose(snap["pose"]["position"], expected[0])
    sim._set_object({"position": [.4, .1, .2], "quaternion": [1, 0, 0, 0]})
    obj = sim.snapshot(time.monotonic())["object"]
    np.testing.assert_allclose(obj["position"], [.4, .1, .2], atol=1e-12)
    np.testing.assert_allclose(obj["quaternion"], [1, 0, 0, 0], atol=1e-12)
    with pytest.raises(ValueError, match="table"):
        sim._set_object({"position": [.4, .1, -.1]})


def test_camera_axis_and_rendered_pixel_match_geometry(simulation):
    sim = simulation
    sim.start()
    camera_rotation = sim.data.cam_xmat[sim.camera].reshape(3, 3)
    tcp_rotation = sim.data.xmat[sim.tcp].reshape(3, 3)
    np.testing.assert_allclose(-camera_rotation[:, 2], tcp_rotation[:, 0], atol=1e-12)
    np.testing.assert_allclose(camera_rotation[:, 1], tcp_rotation[:, 2], atol=1e-12)
    image = sim.image(time.monotonic())["left_rear"]
    u, v = 48, 32
    point = pixel_to_base(image, u, v)
    world_point = rotate_vector(sim.base_pose()[1], point) + sim.base_pose()[0]
    # Compare unprojection to an independent ray cast against scene geometry.
    origin = sim.data.cam_xpos[sim.model.camera("left_rear_camera").id]
    direction = world_point - origin
    distance = np.linalg.norm(direction)
    geom = np.zeros(1, dtype=np.int32)
    hit = mujoco.mj_ray(sim.model, sim.data, origin, direction / distance, None, 1, -1, geom)
    assert hit > 0
    assert distance == pytest.approx(hit, abs=.002)
    image["depth"][v, u] = 0
    with pytest.raises(ValueError, match="valid depth"):
        pixel_to_base(image, u, v)


def test_local_delta_composition_and_worker_execution(simulation, monkeypatch):
    # A 90 degree yaw turns local forward into base +Y.
    quat = np.array([np.cos(np.pi/4), 0, 0, np.sin(np.pi/4)])
    pos, orientation = local_delta_pose(np.zeros(3), quat, [.1, 0, 0], [np.pi/2, 0, 0])
    np.testing.assert_allclose(pos, [0, .1, 0], atol=1e-12)
    np.testing.assert_allclose(rotate_vector(orientation, np.array([0, 1, 0])), [0, 0, 1], atol=1e-12)
    sim = simulation
    before = sim.tcp_pose("base")
    desired = local_delta_pose(*before, [.001, 0, 0], [0, .002, 0])
    captured = {}

    def solve(*args):
        captured["target"] = args[-3:-1]
        return sim.joints()

    monkeypatch.setattr("mujoco_robot_cli.simulation.worker.solve_pose", solve)
    sim._begin(Command("delta", "pose_delta", {"delta_position": [.001, 0, 0], "delta_rpy": [0, .002, 0]}, time.monotonic()+10), queue.Queue(), time.monotonic())
    motion = sim.controller.motion
    assert motion is not None and motion.frame == "base"
    np.testing.assert_allclose(motion.target, desired[0])
    np.testing.assert_allclose(motion.quaternion, desired[1])
    world_target = transform(*desired, "base", "world", *sim.base_pose())
    for actual, expected in zip(captured["target"], world_target):
        np.testing.assert_allclose(actual, expected)


def test_api_coordinate_and_pixel_contract(simulation):
    sim = simulation
    sim.start()

    image_available = [True]

    class Worker:
        process = type("Process", (), {"is_alive": lambda self: True})()
        def latest(self):
            return sim.snapshot(time.monotonic())
        def latest_image(self, camera):
            if not image_available[0]:
                raise AssertionError("point conversion must not read a server image")
            return images[camera]
        def submit(self, kind, payload, timeout):
            submitted.append((kind, payload))
            return {"ok": True}

    images = sim.image(time.monotonic())
    submitted = []
    with TestClient(create_app(sim.settings, Worker())) as client:
        assert client.get("/v1/arm/pose").json()["data"]["frame"] == "base"
        for path in ("/v1/arm/pose", "/v1/object/pose"):
            assert client.get(path, params={"frame": "world"}).status_code == 422
            assert client.post(path, json={"position": [0, 0, .2], "frame": "world"}).status_code == 422
        assert client.post("/v1/arm/pose/delta", json={"delta_position": [.01, 0, 0], "delta_rpy": [0, 0, .1]}).status_code == 200
        assert submitted[-1][0] == "pose_delta"
        params = {"camera": "left_rear", "type": "depth"}
        frame_reply = client.get("/v1/camera/image", params=params)
        assert frame_reply.status_code == 200
        epoch, step = int(frame_reply.headers["x-epoch"]), int(frame_reply.headers["x-step"])
        captured = {key: json.loads(base64.urlsafe_b64decode(frame_reply.headers[header])) for header, key in (
            ("x-tcp-position-base", "tcp_position_base"), ("x-tcp-quaternion-base", "tcp_quaternion_base"),
            ("x-camera-position-tcp", "camera_position_tcp"), ("x-camera-rotation-tcp", "camera_rotation_tcp"),
            ("x-intrinsics", "intrinsics"))}
        image = images["left_rear"]
        valid = np.argwhere(image["depth"] > 0)
        v, u = map(int, valid[len(valid) // 2])
        expected = pixel_to_base(image, u, v)
        point_command = {"camera": "left_rear", "pixel": [u, v], "depth_m": float(image["depth"][v, u]) / 1000,
                         **captured, "epoch": epoch, "step": step}
        image_available[0] = False
        reply = client.post("/v1/camera/point", json=point_command)
        assert reply.status_code == 200, reply.text
        assert reply.json()["data"]["frame"] == "base"
        np.testing.assert_allclose(reply.json()["data"]["position"], expected, atol=1e-9)
        assert reply.json()["data"]["step"] == step
        assert client.post("/v1/camera/point", json=point_command | {"depth_m": 0}).status_code == 422
        assert client.post("/v1/camera/point", json=point_command | {"camera": "unknown"}).status_code == 422
