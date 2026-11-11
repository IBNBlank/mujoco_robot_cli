from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from hex_util_robot import HexDynUtilY6, HexFricUtil

from .scene import GRIPPER_RATIO, normalize_quaternion, orientation_delta, quaternion_error, transform


@dataclass
class Motion:
    kind: str
    target: np.ndarray
    quaternion: np.ndarray | None = None
    frame: str = "world"
    deadline: float = 0.0
    settle_until: float = 0.0
    operation_id: str = ""
    joint_target: np.ndarray | None = None
    stalled_since: float | None = None
    stalled_state: np.ndarray | None = None
    stalled_min: np.ndarray | None = None
    stalled_max: np.ndarray | None = None


class Controller:
    def __init__(self, settings, dyn: HexDynUtilY6) -> None:
        self.settings = settings
        self.dyn = dyn
        self.arm = HexFricUtil(np.array([0.1, 0.1, 0.1, 0.1, 0, 0]), np.zeros(6), np.zeros(6), np.full(6, 200.0))
        self.grip = HexFricUtil(np.array([0.03]), np.zeros(1), np.zeros(1), np.array([200.0]))
        self.motion: Motion | None = None
        self.hold_arm: np.ndarray | None = None
        self.hold_grip: float | None = None
        self.command_arm: np.ndarray | None = None
        self.command_grip: float | None = None
        self.last_error = ""

    def clear(self) -> None:
        self.motion = None
        self.hold_arm = None
        self.hold_grip = None
        self.command_arm = None
        self.command_grip = None
        self.last_error = ""

    def compensation(self, joints: np.ndarray, velocity: np.ndarray, grip_velocity: float) -> tuple[np.ndarray, float]:
        return self.dyn.compensation(joints, velocity) + self.arm(velocity), float(self.grip(np.array([grip_velocity]))[0])

    def track(self, joints: np.ndarray, velocity: np.ndarray, grip: float, grip_velocity: float, tcp: tuple[np.ndarray, np.ndarray], base: tuple[np.ndarray, np.ndarray], now: float, jacobian=None, limits=None) -> tuple[np.ndarray, float, dict | None]:
        self._jacobian = jacobian
        self._limits = limits
        result = self._advance(joints, velocity, grip, grip_velocity, tcp, base, now) if self.motion is not None else None
        if self.command_arm is None:
            self.command_arm = joints.copy()
            self.command_grip = grip
        arm_target = self.hold_arm if self.hold_arm is not None else joints
        grip_target = self.hold_grip if self.hold_grip is not None else grip
        # Advance the position-servo command by at most one configured speed step.
        self.command_arm = self._limited(self.command_arm, arm_target, self.settings.joint_speed_limit / self.settings.sim_hz)
        self.command_grip = float(self._limited(np.array([self.command_grip]), np.array([grip_target]), self.settings.gripper_speed_limit / self.settings.sim_hz)[0])
        return self.command_arm, self.command_grip, result

    def _advance(self, joints, velocity, grip, grip_velocity, tcp, base, now) -> dict | None:
        motion = self.motion
        assert motion is not None
        if motion.kind == "joints":
            self.hold_arm = motion.target
            state = joints
            done = np.all(np.abs(joints - motion.target) <= self.settings.position_tolerance) and np.all(np.abs(velocity) <= self.settings.velocity_tolerance)
            error = float(np.max(np.abs(joints - motion.target)))
        elif motion.kind == "pose":
            state = np.concatenate((tcp[0], tcp[1]))
            target_q, failure = self._ik(motion, joints, tcp, base)
            if failure:
                self.motion = None
                self.hold_arm = joints.copy()
                return {"ok": False, "code": "IK_FAILED", "message": failure, "operation_id": motion.operation_id}
            self.hold_arm = target_q
            position_error = float(np.linalg.norm(tcp[0] - self._world_target(motion, base)[0]))
            orientation_error = quaternion_error(tcp[1], self._world_target(motion, base)[1])
            done = position_error <= self.settings.pose_position_tolerance and orientation_error <= self.settings.pose_orientation_tolerance and np.all(np.abs(velocity) <= self.settings.velocity_tolerance)
            error = position_error
        else:
            self.hold_grip = float(motion.target[0])
            state = np.array([grip])
            error = abs(grip - self.hold_grip)
            slow = abs(grip_velocity) <= self.settings.velocity_tolerance
            done = error <= self.settings.gripper_tolerance and slow
        if now >= motion.deadline and not done:
            self.motion = None
            self.hold_arm = joints.copy()
            self.hold_grip = grip
            return {"ok": False, "code": "TIMEOUT", "message": "motion timed out", "error": error, "velocity": float(np.max(np.abs(velocity))), "operation_id": motion.operation_id}
        if done:
            if motion.settle_until == 0:
                motion.settle_until = now + self.settings.settle_s
            elif now >= motion.settle_until:
                self.motion = None
                return {"ok": True, "status": "reached", "error": error, "operation_id": motion.operation_id}
        else:
            motion.settle_until = 0

        if not done:
            if motion.stalled_state is None:
                motion.stalled_state = state.copy()
                motion.stalled_min = state.copy()
                motion.stalled_max = state.copy()
                motion.stalled_since = now
            else:
                motion.stalled_min = np.minimum(motion.stalled_min, state)
                motion.stalled_max = np.maximum(motion.stalled_max, state)
                threshold = self.settings.gripper_blocked_delta if motion.kind == "gripper" else self.settings.blocked_delta
                if float(np.max(motion.stalled_max - motion.stalled_min)) > threshold:
                    motion.stalled_min = state.copy()
                    motion.stalled_max = state.copy()
                    motion.stalled_since = now
                elif motion.stalled_since is not None and now - motion.stalled_since >= self.settings.blocked_window_s:
                    self.motion = None
                    if motion.kind != "gripper":
                        self.hold_arm = joints.copy()
                        self.command_arm = joints.copy()
                    return {"ok": True, "status": "blocked", "error": error, "operation_id": motion.operation_id}
        else:
            motion.stalled_state = None
            motion.stalled_min = None
            motion.stalled_max = None
            motion.stalled_since = None
        return None

    def _ik(self, motion: Motion, joints: np.ndarray, tcp, base) -> tuple[np.ndarray, str]:
        if motion.joint_target is not None:
            return motion.joint_target, ""
        target_pos, target_quat = self._world_target(motion, base)
        position_error = target_pos - tcp[0]
        orientation_error = orientation_delta(tcp[1], target_quat)
        if self._jacobian is None:
            return joints, "Jacobian is unavailable"
        error = np.concatenate([position_error, orientation_error])
        if float(np.linalg.norm(error)) < 1e-4:
            return joints, ""
        damped = self._jacobian @ self._jacobian.T + (0.01 ** 2) * np.eye(6)
        delta = self._jacobian.T @ np.linalg.solve(damped, error)
        solved = np.clip(joints + np.clip(delta, -0.08, 0.08), self._limits[:, 0], self._limits[:, 1])
        if float(np.linalg.norm(solved - joints)) < 1e-5 and float(np.linalg.norm(position_error)) > 0.03:
            return joints, "target pose is unreachable"
        return solved, ""

    def _world_target(self, motion: Motion, base) -> tuple[np.ndarray, np.ndarray]:
        quat = motion.quaternion if motion.quaternion is not None else np.array([1.0, 0, 0, 0])
        return transform(motion.target, quat, motion.frame, "world", base[0], base[1])

    @staticmethod
    def _limited(current: np.ndarray, target: np.ndarray, step: float) -> np.ndarray:
        delta = target - current
        scale = np.max(np.abs(delta))
        if scale <= step:
            return target.copy()
        return current + delta / scale * step
