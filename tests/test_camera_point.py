"""Pixel conversion with server calibration and optional TCP pose."""
import time

import numpy as np
import pytest
from fastapi.testclient import TestClient

from mujoco_robot_cli.api import create_app
from mujoco_robot_cli.config import Settings


class CameraWorker:
    process = type("Process", (), {"is_alive": lambda self: True})()

    def __init__(self):
        self.position = [1., 2., 3.]
        self.quaternion = [1., 0., 0., 0.]
        self.stale = False

    def latest(self):
        return {"monotonic_s": time.monotonic(), "epoch": 1, "step": 20,
                "pose": {"position": self.position, "quaternion": self.quaternion}}

    def latest_image(self, camera):
        return {"monotonic_s": 0 if self.stale else time.monotonic(), "epoch": 1, "step": 10,
                "tcp_position_base": [0., 0., 0.], "tcp_quaternion_base": [1., 0., 0., 0.],
                "camera_position_tcp": [0.1, 0.2, 0.3] if camera == "end" else [4., 5., 6.],
                "camera_rotation_tcp": np.eye(3).tolist(),
                "intrinsics": {"fx": 100., "fy": 100., "cx": 50., "cy": 40., "width": 100, "height": 80}}


@pytest.mark.parametrize("camera", ["end", "opposite", "left"])
def test_current_and_explicit_pose(camera):
    worker = CameraWorker()
    command = {"camera": camera, "pixel": [60, 60], "depth_m": 2.}
    with TestClient(create_app(Settings(), worker)) as client:
        reply = client.post("/v1/camera/point", json=command)
        assert reply.status_code == 200, reply.text
        # Camera axes are right/up/back; v grows downwards.
        expected = [1.3, 1.8, 1.3] if camera == "end" else [4.2, 4.6, 4.]
        np.testing.assert_allclose(reply.json()["data"]["position"], expected)
        explicit = command | {"tcp_position_base": [10., 20., 30.],
                              "tcp_quaternion_base": [np.sqrt(.5), 0., 0., np.sqrt(.5)]}
        reply = client.post("/v1/camera/point", json=explicit)
        assert reply.status_code == 200, reply.text
        expected = [10.2, 20.3, 28.3] if camera == "end" else [4.2, 4.6, 4.]
        np.testing.assert_allclose(reply.json()["data"]["position"], expected)
        worker.position = [3., 4., 5.]
        reply = client.post("/v1/camera/point", json=command)
        expected = [3.3, 3.8, 3.3] if camera == "end" else [4.2, 4.6, 4.]
        np.testing.assert_allclose(reply.json()["data"]["position"], expected)
        for invalid in ({"pixel": [100, 0]}, {"depth_m": 0}, {"camera": "unknown"},
                        {"tcp_position_base": [1, 2, 3]}, {"camera_position_tcp": [0, 0, 0]}):
            assert client.post("/v1/camera/point", json=command | invalid).status_code == 422
        assert client.post("/v1/camera/point", json={"pixel": [0, 0], "depth_m": 1}).status_code == 422
        worker.stale = True
        assert client.post("/v1/camera/point", json=command).status_code == 503


def test_cli_point_minimal_and_json(monkeypatch, capsys):
    from mujoco_robot_cli.cli import main
    from mujoco_robot_cli.client import RobotClient

    sent = []
    def request(self, method, path, payload):
        sent.append((method, path, payload))
        return {"ok": True}
    monkeypatch.setattr(RobotClient, "request", request)
    assert main(["image", "point", "--camera", "opposite", "--pixel", "10", "20", "--depth-m", "0.5"]) == 0
    assert sent[-1][2] == {"camera": "opposite", "pixel": [10, 20], "depth_m": .5}
    assert main(["--request-json", '{"camera":"end","pixel":[1,2],"depth_m":1}', "image", "point"]) == 0
    assert sent[-1][2] == {"camera": "end", "pixel": [1, 2], "depth_m": 1}
