import importlib.util
from pathlib import Path

import pytest

spec = importlib.util.spec_from_file_location("pick_demo", Path(__file__).parents[1] / "scripts/pick_place_demo.py")
demo = importlib.util.module_from_spec(spec)
spec.loader.exec_module(demo)


class Client:
    def __init__(self, lifted=True):
        self.calls = []
        self.object_reads = 0
        self.lifted = lifted

    def request(self, method, path, payload=None):
        self.calls.append((method, path, payload))
        if method == "POST":
            return {"data": {"result": {"status": "reached"}}}
        if path == "/v1/arm/joints":
            return {"data": {"positions": [0, 0, 0, 0, 0, 0]}}
        if path == "/v1/arm/pose":
            commands = [body for method, endpoint, body in self.calls if method == "POST" and endpoint == path]
            return {"data": commands[-1]}
        if path == "/v1/object/pose":
            self.object_reads += 1
            positions = [[0.5, 0.0, 0.32], [0.5, 0.0, 0.45 if self.lifted else 0.32], [0.6, 0.0, 0.45], [0.6, 0.0, 0.322]]
            return {"data": {"position": positions[self.object_reads - 1]}}
        return {"data": {"in_target": True}}


def test_seven_step_demo(monkeypatch):
    monkeypatch.setattr(demo.time, "sleep", lambda _: None)
    client = Client()
    assert demo.run(client, target=[0.6, 0, 0.337], grasp_offset=[0, 0, 0.015], clearance=0.12, timeout_s=20)["ok"]
    posts = [(path, payload) for method, path, payload in client.calls if method == "POST"]
    assert [payload["action"] for path, payload in posts if path == "/v1/gripper"] == ["open", "close", "open"]
    assert posts[1][0] == posts[2][0] == "/v1/arm/pose"
    assert posts[2][1]["position"] == pytest.approx([0.51060660, 0, 0.33060660])
    assert all(body["quaternion"] == demo.GRASP_QUATERNION for path, body in posts if path == "/v1/arm/pose")
    assert posts[-1] == ("/v1/arm/joints", {"positions": [0] * 6, "timeout_s": 20})
    assert not any(path in {"/v1/object/pose", "/v1/environment/reset"} for path, _ in posts)


def test_rotated_grasp_offset(monkeypatch):
    import numpy as np
    monkeypatch.setattr(demo.time, "sleep", lambda _: None)
    client = Client()
    demo.run(client, target=[0.6, 0, 0.332], grasp_offset=[0.015, 0, 0.0016], clearance=0.12, timeout_s=20)
    poses = [body for method, endpoint, body in client.calls if method == "POST" and endpoint == "/v1/arm/pose"]
    assert poses[1]["position"] == pytest.approx([0.51173797, 0, 0.31052477], abs=1e-7)


def test_failed_grasp_stops_transfer():
    client = Client(lifted=False)
    with pytest.raises(demo.DemoError, match="did not lift"):
        demo.run(client, target=[0.6, 0, 0.337], grasp_offset=[0, 0, 0.015], clearance=0.12, timeout_s=20)
    assert not any(payload.get("position") == [0.6, 0, 0.337] for method, _, payload in client.calls if method == "POST")
