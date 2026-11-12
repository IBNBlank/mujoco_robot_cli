from __future__ import annotations

import multiprocessing as mp
import os
import time
import traceback
from dataclasses import dataclass
from .snapshot import LatestSnapshot

import mujoco
import numpy as np
from hex_util_robot import HexDynUtilY6
from hex_util_urdf import HEXARM_URDF_PATH_DICT

from ..config import Settings
from .kinematics import solve_pose
from .controller import Controller, Motion
from .scene import (
    BASE_BODY,
    CAMERA_NAME,
    CAMERAS,
    GRIPPER_JOINT,
    GRIPPER_RATIO,
    JOINT_NAMES,
    OBJECT_JOINTS,
    TCP_BODY,
    actuator_id,
    joint_address,
    name_id,
    local_delta_pose,
    normalize_quaternion,
    pose_of,
    scene_path,
    transform,
)


@dataclass
class Command:
    operation_id: str
    kind: str
    payload: dict
    deadline: float


def _worker(settings: Settings, commands: mp.Queue, results: mp.Queue, state: LatestSnapshot, images: LatestSnapshot, ready: mp.Event, stop: mp.Event) -> None:
    os.environ["MUJOCO_GL"] = settings.render_backend
    try:
        engine = Simulation(settings)
        engine.start()
        ready.set()
        last_state = last_image = 0.0
        while not stop.is_set():
            started = time.perf_counter()
            engine.step(commands, results, time.monotonic())
            now = time.monotonic()
            if now - last_state >= 1.0 / settings.state_hz:
                state.publish(engine.snapshot(now))
                last_state = now
            if now - last_image >= 1.0 / settings.camera_hz:
                images.publish(engine.image(now))
                last_image = now
            remaining = 1.0 / settings.sim_hz - (time.perf_counter() - started)
            if remaining > 0:
                time.sleep(remaining)
        engine.close()
    except Exception:
        results.put({"ok": False, "code": "INTERNAL_ERROR", "message": traceback.format_exc(), "operation_id": "startup"})
        ready.set()


class Simulation:
    def __init__(self, settings: Settings) -> None:
        self.settings = settings
        self.model = mujoco.MjModel.from_xml_path(scene_path())
        self.data = mujoco.MjData(self.model)
        self.model.opt.timestep = 1.0 / settings.sim_hz
        self.arm_q = [joint_address(self.model, name)[0] for name in JOINT_NAMES]
        self.arm_v = [joint_address(self.model, name)[1] for name in JOINT_NAMES]
        self.grip_q, self.grip_v = joint_address(self.model, GRIPPER_JOINT)
        self.grip_followers = [joint_address(self.model, name)[0] for name in ("grip_left_joint_2", "grip_right_joint_1", "grip_right_joint_2", "grip_left_help_joint", "grip_right_help_joint")]
        self.object_addresses = {object_id: joint_address(self.model, joint) for object_id, joint in OBJECT_JOINTS.items()}
        self.object_q, self.object_v = self.object_addresses["red"]
        self.arm_ctrl = [actuator_id(self.model, name) for name in JOINT_NAMES]
        self.grip_ctrl = actuator_id(self.model, GRIPPER_JOINT)
        self.tcp = name_id(self.model, mujoco.mjtObj.mjOBJ_BODY, TCP_BODY)
        self.base = name_id(self.model, mujoco.mjtObj.mjOBJ_BODY, BASE_BODY)
        self.camera = name_id(self.model, mujoco.mjtObj.mjOBJ_CAMERA, CAMERA_NAME)
        self.home = name_id(self.model, mujoco.mjtObj.mjOBJ_KEY, "home")
        self.limits = np.array([self.model.jnt_range[name_id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)] for name in JOINT_NAMES])
        self.grip_limit = self.model.jnt_range[name_id(self.model, mujoco.mjtObj.mjOBJ_JOINT, GRIPPER_JOINT)].copy()
        self.dyn = HexDynUtilY6(HEXARM_URDF_PATH_DICT["archer_y6_gp100_p050"], pose_end_in_flange=np.array([0.187, 0, 0, 1, 0, 0, 0]))
        self.controller = Controller(settings, self.dyn)
        self.renderer = None
        self.depth_renderer = None
        self.viewer = None
        self.epoch = 0
        self.pending: dict | None = None

    def start(self) -> None:
        self.reset_state()
        # Multisample depth resolves at a subpixel sample, which biases pixel unprojection.
        self.model.vis.quality.offsamples = 0
        self.renderer = mujoco.Renderer(self.model, self.settings.image_height, self.settings.image_width)
        self.depth_renderer = mujoco.Renderer(self.model, self.settings.image_height, self.settings.image_width)
        self.depth_renderer.enable_depth_rendering()
        if not self.settings.headless:
            from mujoco import viewer

            self.viewer = viewer.launch_passive(self.model, self.data)

    def close(self) -> None:
        for item in (self.renderer, self.depth_renderer, self.viewer):
            if item is not None:
                item.close()

    def reset_state(self) -> None:
        mujoco.mj_resetDataKeyframe(self.model, self.data, self.home)
        self.data.qvel[:] = 0
        self.data.ctrl[:] = 0
        self.data.qfrc_applied[:] = 0
        mujoco.mj_forward(self.model, self.data)
        self.controller.clear()
        self.controller.hold_arm = self.joints().copy()
        self.controller.hold_grip = self.gripper()
        self.epoch += 1

    def joints(self) -> np.ndarray:
        return np.array([self.data.qpos[i] for i in self.arm_q])

    def joint_velocity(self) -> np.ndarray:
        return np.array([self.data.qvel[i] for i in self.arm_v])

    def gripper(self) -> float:
        return float(self.data.qpos[self.grip_q])

    def gripper_velocity(self) -> float:
        return float(self.data.qvel[self.grip_v])

    def base_pose(self):
        return pose_of(self.data, self.base)

    def tcp_pose(self, frame="world"):
        pos, quat = pose_of(self.data, self.tcp)
        return transform(pos, quat, "world", frame, *self.base_pose())

    def step(self, commands: mp.Queue, results: mp.Queue, now: float) -> None:
        while not commands.empty():
            command = commands.get_nowait()
            if command.kind == "cancel":
                self._finish_timeout(results, command.payload.get("operation_id", command.operation_id))
                continue
            if self.pending or self.controller.motion:
                results.put({"ok": False, "code": "BUSY", "message": "another action is running", "operation_id": command.operation_id})
                continue
            self._begin(command, results, now)
        if self.pending and self.pending["kind"] == "settle" and now >= self.pending["until"]:
            results.put({"ok": True, "operation_id": self.pending["operation_id"], "status": "settled"})
            self.pending = None
        jacobian = np.zeros((6, self.model.nv))
        mujoco.mj_jacBody(self.model, self.data, jacobian[:3], jacobian[3:], self.tcp)
        arm_jacobian = jacobian[:, self.arm_v]
        arm_command, grip_command, result = self.controller.track(self.joints(), self.joint_velocity(), self.gripper(), self.gripper_velocity(), self.tcp_pose(), self.base_pose(), now, arm_jacobian, self.limits)
        if result:
            results.put(result)
        elif self.controller.motion is not None and now >= self.controller.motion.deadline:
            operation_id = self.controller.motion.operation_id
            self.controller.motion = None
            self.controller.hold_arm = self.joints().copy()
            self.controller.hold_grip = self.gripper()
            results.put({"ok": False, "code": "TIMEOUT", "message": "motion timed out", "operation_id": operation_id})
        for index, value in zip(self.arm_ctrl, arm_command):
            self.data.ctrl[index] = value
        self.data.ctrl[self.grip_ctrl] = grip_command
        mujoco.mj_step(self.model, self.data)
        if self.viewer is not None:
            self.viewer.sync()

    def _begin(self, command: Command, results: mp.Queue, now: float) -> None:
        try:
            if command.kind == "joints":
                target = np.asarray(command.payload["positions"], dtype=float)
                if np.any((target < self.limits[:, 0]) | (target > self.limits[:, 1])):
                    raise ValueError("joint target exceeds limits")
                self.controller.motion = Motion("joints", target, deadline=command.deadline, operation_id=command.operation_id)
            elif command.kind in {"pose", "pose_delta"}:
                if command.kind == "pose_delta":
                    target, quat = local_delta_pose(*self.tcp_pose("base"), command.payload["delta_position"], command.payload["delta_rpy"])
                else:
                    if command.payload.get("frame", "base") != "base":
                        raise ValueError("pose frame must be base")
                    target = np.asarray(command.payload["position"], dtype=float)
                    quat = self.tcp_pose("base")[1] if command.payload.get("quaternion") is None else normalize_quaternion(command.payload["quaternion"])
                if target.shape != (3,) or not np.all(np.isfinite(target)):
                    raise ValueError("position must contain three finite values")
                motion = Motion("pose", target, quat, "base", command.deadline, operation_id=command.operation_id)
                position, orientation = transform(motion.target, quat, motion.frame, "world", *self.base_pose())
                solved = solve_pose(self.model, self.data, self.arm_q, self.arm_v, self.tcp, self.limits, position, orientation, self.joints())
                if solved is None:
                    results.put({"ok": False, "code": "IK_FAILED", "message": "target pose is unreachable within joint limits", "operation_id": command.operation_id})
                    return
                motion.joint_target = solved
                self.controller.motion = motion
            elif command.kind == "gripper":
                ratio = 1.0 if command.payload.get("action") == "open" else 0.0 if command.payload.get("action") == "close" else float(command.payload["opening_ratio"])
                # Zero joint angle opens the fingers; increasing the angle closes them.
                target = self.grip_limit[0] + (1.0 - ratio) * (self.grip_limit[1] - self.grip_limit[0])
                self.controller.motion = Motion("gripper", np.array([target]), deadline=command.deadline, operation_id=command.operation_id)
            elif command.kind == "object":
                self._set_object(command.payload)
                self.pending = {"kind": "settle", "until": now + float(command.payload.get("settle_s", 0)), "operation_id": command.operation_id}
            elif command.kind == "reset":
                self.reset_state()
                self.pending = {"kind": "settle", "until": now + float(command.payload.get("settle_s", 0)), "operation_id": command.operation_id}
            else:
                raise ValueError(f"unknown command {command.kind}")
        except Exception as exc:
            code = "INVALID_ARGUMENT" if isinstance(exc, ValueError) else "INTERNAL_ERROR"
            results.put({"ok": False, "code": code, "message": str(exc), "operation_id": command.operation_id})

    def _set_object(self, payload: dict) -> None:
        if payload.get("frame", "base") != "base":
            raise ValueError("object frame must be base")
        quat = normalize_quaternion(payload.get("quaternion") or [1, 0, 0, 0])
        position, quat = transform(np.asarray(payload["position"], dtype=float), quat, "base", "world", *self.base_pose())
        if not np.all(np.isfinite(position)):
            raise ValueError("position must contain three finite values")
        if position[2] < self.settings.table_z + 0.02 - self.settings.object_clearance:
            raise ValueError("object penetrates the table")
        object_id = payload.get("object_id", "red")
        if object_id not in self.object_addresses:
            raise ValueError(f"unknown object_id: {object_id}")
        object_q, object_v = self.object_addresses[object_id]
        self.data.qpos[object_q:object_q + 3] = position
        self.data.qpos[object_q + 3:object_q + 7] = quat
        self.data.qvel[object_v:object_v + 6] = 0
        mujoco.mj_forward(self.model, self.data)

    def _finish_timeout(self, results: mp.Queue, operation_id: str) -> None:
        self.controller.motion = None
        self.controller.hold_arm = self.joints().copy()
        self.controller.hold_grip = self.gripper()
        self.pending = None
        results.put({"ok": False, "code": "TIMEOUT", "message": "request deadline reached", "operation_id": operation_id})

    def snapshot(self, now: float) -> dict:
        pos, quat = self.tcp_pose("base")
        base_pos, base_quat = self.base_pose()
        target_id = name_id(self.model, mujoco.mjtObj.mjOBJ_GEOM, "tar_plane")
        target_pos, target_quat = transform(self.data.geom_xpos[target_id], np.array([1., 0, 0, 0]), "world", "base", base_pos, base_quat)
        target_half_size = self.model.geom_size[target_id].copy()
        target = {"frame": "base", "center": target_pos.tolist(), "quaternion": target_quat.tolist(), "half_size": target_half_size.tolist()}
        objects = {}
        for object_id, (object_q, _) in self.object_addresses.items():
            obj = self.data.qpos[object_q:object_q + 7].copy()
            obj_pos, obj_quat = transform(obj[:3], obj[3:], "world", "base", base_pos, base_quat)
            inside_xy = np.all(np.abs(obj[:2] - self.data.geom_xpos[target_id, :2]) <= target_half_size[:2] + 0.02)
            on_table = abs(obj[2] - (self.settings.table_z + 0.022)) <= 0.015
            objects[object_id] = {"object_id": object_id, "frame": "base", "position": obj_pos.tolist(), "quaternion": obj_quat.tolist(), "in_target": bool(inside_xy and on_table), "target": target}
        ratio = (self.grip_limit[1] - self.data.qpos[self.grip_q]) / (self.grip_limit[1] - self.grip_limit[0])

        moving = abs(self.gripper_velocity()) > self.settings.velocity_tolerance
        return {
            "epoch": self.epoch,
            "step": int(self.data.time / self.model.opt.timestep),
            "sim_time_s": float(self.data.time),
            "captured_at_ns": time.time_ns(),
            "monotonic_s": now,
            "joints": {"names": JOINT_NAMES, "positions": self.joints().tolist(), "velocities": self.joint_velocity().tolist(), "limits": self.limits.tolist()},
            "pose": {"frame": "base", "position": pos.tolist(), "quaternion": quat.tolist()},
            "gripper": {"raw_position": float(self.data.qpos[self.grip_q]), "opening_ratio": float(np.clip(ratio, 0, 1)), "target": self.controller.hold_grip, "velocity": self.gripper_velocity(), "moving": moving},
            "object": objects["red"],
            "objects": objects,
        }

    def image(self, now: float) -> dict:
        frames = {}
        tcp_position_base, tcp_quaternion_base = self.tcp_pose("base")
        tcp_position_world, _ = pose_of(self.data, self.tcp)
        tcp_rotation_world = self.data.xmat[self.tcp].reshape(3, 3)
        for camera, model_name in CAMERAS.items():
            captured = time.monotonic()
            captured_at_ns = time.time_ns()
            self.renderer.update_scene(self.data, model_name)
            rgb = self.renderer.render().copy()
            self.depth_renderer.update_scene(self.data, model_name)
            depth_m = self.depth_renderer.render().copy()
            near = self.model.vis.map.znear * self.model.stat.extent
            far = self.model.vis.map.zfar * self.model.stat.extent
            valid = np.isfinite(depth_m) & (depth_m > near) & (depth_m < far * (1 - 1e-6)) & (depth_m < 65.535)
            depth = np.where(valid, np.clip(depth_m * 1000.0, 0, 65534), 0).astype(np.uint16)
            camera_id = name_id(self.model, mujoco.mjtObj.mjOBJ_CAMERA, model_name)
            base_rotation = self.data.xmat[self.base].reshape(3, 3)
            camera_position = base_rotation.T @ (self.data.cam_xpos[camera_id] - self.data.xpos[self.base])
            camera_rotation = base_rotation.T @ self.data.cam_xmat[camera_id].reshape(3, 3)
            height, width = depth.shape
            focal = height / (2 * np.tan(np.deg2rad(self.model.cam_fovy[camera_id]) / 2))
            camera_position_tcp = tcp_rotation_world.T @ (self.data.cam_xpos[camera_id] - tcp_position_world)
            camera_rotation_tcp = tcp_rotation_world.T @ self.data.cam_xmat[camera_id].reshape(3, 3)
            frames[camera] = {"camera": camera, "epoch": self.epoch, "step": int(self.data.time / self.model.opt.timestep), "captured_at_ns": captured_at_ns, "monotonic_s": captured, "rgb": rgb, "depth": depth, "width": width, "height": height,
                              "tcp_position_base": tcp_position_base.tolist(), "tcp_quaternion_base": tcp_quaternion_base.tolist(),
                              "camera_position_tcp": camera_position_tcp.tolist(), "camera_rotation_tcp": camera_rotation_tcp.tolist(),
                              "intrinsics": {"fx": float(focal), "fy": float(focal), "cx": (width - 1) / 2, "cy": (height - 1) / 2, "width": width, "height": height},
                              "camera_position_base": camera_position.tolist(), "camera_rotation_base": camera_rotation.tolist()}
        return frames


class WorkerClient:
    def __init__(self, settings: Settings) -> None:
        self.settings = settings
        ctx = mp.get_context("spawn")
        self.commands = ctx.Queue(8)
        self.results = ctx.Queue()
        self.state = LatestSnapshot(ctx, 64 * 1024)
        # RGB uint8 + depth uint16, with room for metadata and serialization.
        self.images = LatestSnapshot(ctx, len(CAMERAS) * settings.image_width * settings.image_height * 5 + 64 * 1024)
        self.ready = ctx.Event()
        self.stop = ctx.Event()
        self.process = ctx.Process(target=_worker, args=(settings, self.commands, self.results, self.state, self.images, self.ready, self.stop), daemon=True)
        self.action = None

    def start(self) -> None:
        self.process.start()
        if not self.ready.wait(self.settings.startup_timeout_s):
            self.close()
            raise TimeoutError("simulation did not become ready")
        if not self.results.empty():
            failure = self.results.get()
            self.close()
            raise RuntimeError(failure["message"])
        deadline = time.monotonic() + self.settings.startup_timeout_s
        while time.monotonic() < deadline and (self.latest() is None or self.latest_image() is None):
            time.sleep(0.01)
        if self.latest() is None or self.latest_image() is None:
            self.close()
            raise TimeoutError("initial state or image was not published")

    def close(self) -> None:
        self.stop.set()
        if self.process.is_alive():
            self.process.join(5)
        if self.process.is_alive():
            self.process.terminate()
            self.process.join(2)

    def latest(self):
        return self.state.read()

    def latest_image(self, camera: str = "end"):
        frames = self.images.read()
        return None if frames is None else frames.get(camera)

    def submit(self, kind: str, payload: dict, timeout_s: float) -> dict:
        if self.action is not None:
            return {"ok": False, "code": "BUSY", "message": "another action is running"}
        operation_id = f"{time.time_ns()}"
        deadline = time.monotonic() + timeout_s
        self.action = operation_id
        self.commands.put(Command(operation_id, kind, payload, deadline))
        while time.monotonic() < deadline + max(8.0, self.settings.settle_s + 2.0):
            self._drain_results()
            if self.action is None:
                return self._last_result
            if not self.process.is_alive():
                self.action = None
                return {"ok": False, "code": "INTERNAL_ERROR", "message": "simulation worker exited"}
            time.sleep(0.002)
        self.commands.put(Command(f"cancel-{operation_id}", "cancel", {"operation_id": operation_id}, deadline))
        end = time.monotonic() + 2
        while time.monotonic() < end:
            self._drain_results()
            if self.action is None:
                return self._last_result
            if not self.process.is_alive():
                self.action = None
                return {"ok": False, "code": "INTERNAL_ERROR", "message": getattr(self, "_unexpected", {"message": "worker exited"})["message"]}
            time.sleep(0.002)
        self.action = None
        return {"ok": False, "code": "TIMEOUT", "message": "request deadline reached"}

    def _drain_results(self) -> None:
        while not self.results.empty():
            result = self.results.get()
            if result.get("operation_id") == self.action or result.get("code") == "BUSY":
                self._last_result = result
                if result.get("operation_id") == self.action:
                    self.action = None
            else:
                self._unexpected = result
