import numpy as np

from mujoco_robot_cli.config import Settings
from mujoco_robot_cli.simulation.controller import Controller, Motion


def advance(controller, now, grip):
    pose = (np.zeros(3), np.array([1., 0, 0, 0]))
    return controller._advance(np.zeros(6), np.zeros(6), grip, 0., pose, pose, now)


def test_contact_jitter_returns_blocked_and_keeps_grip_force():
    controller = Controller(Settings(), None)
    controller.motion = Motion("gripper", np.array([1.52]), deadline=20.)
    result = None
    for i in range(12):
        result = advance(controller, i * .1, 1.0 + .0002 * (-1) ** i)
        if result:
            break
    assert result["status"] == "blocked"
    assert controller.hold_grip == 1.52
    assert i * .1 <= 1.1


def test_slow_continuous_motion_is_not_blocked():
    controller = Controller(Settings(), None)
    controller.motion = Motion("gripper", np.array([1.52]), deadline=20.)
    for i in range(30):
        assert advance(controller, i * .1, 0.01 * i) is None


def test_timeout_is_not_reported_as_blocked():
    controller = Controller(Settings(), None)
    controller.motion = Motion("gripper", np.array([1.52]), deadline=.5)
    assert advance(controller, 0., 1.) is None
    result = advance(controller, .5, 1.)
    assert result["code"] == "TIMEOUT"
