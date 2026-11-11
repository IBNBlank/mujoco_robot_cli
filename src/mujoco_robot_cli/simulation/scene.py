from __future__ import annotations

from importlib.resources import files

import mujoco
import numpy as np


JOINT_NAMES = [f"joint_{i}" for i in range(1, 7)]
GRIPPER_JOINT = "grip_left_joint_1"
OBJECT_JOINT = "red_box_joint"
TCP_BODY = "ee_frame"
BASE_BODY = "base_link"
CAMERA_NAME = "end_camera"
CAMERAS = {"end": CAMERA_NAME, "left_rear": "left_rear_camera"}
# Reference driver exposes the gripper joint scaled by this ratio.
GRIPPER_RATIO = 1.33 / 1.52


def scene_path() -> str:
    return str(files("mujoco_robot_cli.assets.archer_y6").joinpath("scene.xml"))


def name_id(model: mujoco.MjModel, kind: int, name: str) -> int:
    index = mujoco.mj_name2id(model, kind, name)
    if index < 0:
        raise RuntimeError(f"model is missing {name}")
    return int(index)


def joint_address(model: mujoco.MjModel, name: str) -> tuple[int, int]:
    index = name_id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
    return int(model.jnt_qposadr[index]), int(model.jnt_dofadr[index])


def actuator_id(model: mujoco.MjModel, joint_name: str) -> int:
    for index in range(model.nu):
        joint = int(model.actuator_trnid[index, 0])
        if mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, joint) == joint_name:
            return index
    raise RuntimeError(f"no actuator for {joint_name}")


def normalize_quaternion(quat: np.ndarray) -> np.ndarray:
    value = np.asarray(quat, dtype=float)
    if not np.all(np.isfinite(value)) or value.shape != (4,):
        raise ValueError("quaternion must contain four finite values")
    norm = float(np.linalg.norm(value))
    if norm < 1e-8 or abs(norm - 1.0) > 1e-3:
        raise ValueError("quaternion must be non-zero and nearly normalized")
    if value[0] < 0:
        value = -value
    return value / norm


def orientation_delta(current: np.ndarray, target: np.ndarray) -> np.ndarray:
    cw, cx, cy, cz = current
    inverse = np.array([cw, -cx, -cy, -cz])
    tw, tx, ty, tz = target
    w = tw * inverse[0] - tx * inverse[1] - ty * inverse[2] - tz * inverse[3]
    x = tw * inverse[1] + tx * inverse[0] + ty * inverse[3] - tz * inverse[2]
    y = tw * inverse[2] - tx * inverse[3] + ty * inverse[0] + tz * inverse[1]
    z = tw * inverse[3] + tx * inverse[2] - ty * inverse[1] + tz * inverse[0]
    vector = np.array([x, y, z])
    if w < 0:
        vector = -vector
    norm = float(np.linalg.norm(vector))
    if norm < 1e-9:
        return np.zeros(3)
    return vector / norm * (2.0 * np.arctan2(norm, abs(w)))


def quaternion_error(current: np.ndarray, target: np.ndarray) -> float:
    dot = float(np.clip(abs(np.dot(current, target)), -1.0, 1.0))
    return float(2.0 * np.arccos(dot))


def pose_of(data: mujoco.MjData, body_id: int) -> tuple[np.ndarray, np.ndarray]:
    quat = np.zeros(4)
    mujoco.mju_mat2Quat(quat, data.xmat[body_id])
    return data.xpos[body_id].copy(), quat


def quaternion_multiply(left: np.ndarray, right: np.ndarray) -> np.ndarray:
    w1, x1, y1, z1 = left
    w2, x2, y2, z2 = right
    return np.array([
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    ])


def rotate_vector(quaternion: np.ndarray, vector: np.ndarray) -> np.ndarray:
    rotated = quaternion_multiply(quaternion_multiply(quaternion, np.array([0.0, *vector])), np.array([quaternion[0], *(-quaternion[1:])]))
    return rotated[1:]


def transform(position: np.ndarray, quaternion: np.ndarray, source: str, target: str, base_pos: np.ndarray, base_quat: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    if source == target:
        return position.copy(), quaternion.copy()
    inverse = np.array([base_quat[0], *(-base_quat[1:])])
    if source == "world" and target == "base":
        return rotate_vector(inverse, position - base_pos), normalize_quaternion(quaternion_multiply(inverse, quaternion))
    if source == "base" and target == "world":
        return rotate_vector(base_quat, position) + base_pos, normalize_quaternion(quaternion_multiply(base_quat, quaternion))
    raise ValueError(f"unsupported frame conversion {source}->{target}")
