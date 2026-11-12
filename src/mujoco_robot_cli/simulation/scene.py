from __future__ import annotations

from importlib.resources import files

import mujoco
import numpy as np


JOINT_NAMES = [f"joint_{i}" for i in range(1, 7)]
GRIPPER_JOINT = "grip_left_joint_1"
OBJECT_JOINTS = {color: f"{color}_box_joint" for color in ("red", "orange", "yellow")}
OBJECT_JOINT = OBJECT_JOINTS["red"]
TCP_BODY = "ee_frame"
BASE_BODY = "base_link"
CAMERA_NAME = "end_camera"
CAMERAS = {"end": CAMERA_NAME, "opposite": "opposite_camera", "left": "left_camera"}
FIXED_CAMERAS = frozenset({"opposite", "left"})
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


def local_delta_pose(position: np.ndarray, quaternion: np.ndarray, delta_position, delta_rpy) -> tuple[np.ndarray, np.ndarray]:
    """Compose T_base_tcp @ Trans(delta_position) @ Rz(yaw) @ Ry(pitch) @ Rx(roll)."""
    translation = np.asarray(delta_position, dtype=float)
    angles = np.asarray(delta_rpy, dtype=float)
    if translation.shape != (3,) or angles.shape != (3,) or not np.all(np.isfinite([translation, angles])):
        raise ValueError("deltas must each contain three finite values")
    roll, pitch, yaw = angles / 2.0
    qx = np.array([np.cos(roll), np.sin(roll), 0, 0])
    qy = np.array([np.cos(pitch), 0, np.sin(pitch), 0])
    qz = np.array([np.cos(yaw), 0, 0, np.sin(yaw)])
    delta_quat = quaternion_multiply(quaternion_multiply(qz, qy), qx)
    return position + rotate_vector(quaternion, translation), normalize_quaternion(quaternion_multiply(quaternion, delta_quat))


def pixel_to_base_from_capture(pixel, depth_m: float, tcp_position_base, tcp_quaternion_base,
                               camera_position_tcp, camera_rotation_tcp, intrinsics) -> np.ndarray:
    """Unproject client-supplied pixel depth and capture-time TCP pose."""
    u, v = pixel
    if type(u) is not int or type(v) is not int or u < 0 or v < 0:
        raise ValueError("pixel must contain two non-negative integers")
    depth = float(depth_m)
    tcp_position = np.asarray(tcp_position_base, dtype=float)
    tcp_quaternion = normalize_quaternion(np.asarray(tcp_quaternion_base, dtype=float))
    camera_offset = np.asarray(camera_position_tcp, dtype=float)
    camera_rotation = np.asarray(camera_rotation_tcp, dtype=float)
    calibration = intrinsics
    if (not np.isfinite(depth) or not 0 < depth < 65.535 or tcp_position.shape != (3,)
            or camera_offset.shape != (3,) or camera_rotation.shape != (3, 3)
            or not np.all(np.isfinite(tcp_position)) or not np.all(np.isfinite(camera_offset))
            or not np.all(np.isfinite(camera_rotation))):
        raise ValueError("depth and captured camera pose must be finite and valid")
    if not np.allclose(camera_rotation.T @ camera_rotation, np.eye(3), atol=1e-3) or np.linalg.det(camera_rotation) < 0.999:
        raise ValueError("camera_rotation_tcp must be a proper rotation matrix")
    if u >= calibration["width"] or v >= calibration["height"]:
        raise ValueError("pixel is outside the image")
    rotation_tcp = quaternion_matrix(tcp_quaternion)
    camera_pos_base = tcp_position + rotation_tcp @ camera_offset
    camera_rot_base = rotation_tcp @ camera_rotation
    point_camera = np.array([(u - calibration["cx"]) * depth / calibration["fx"],
                             -(v - calibration["cy"]) * depth / calibration["fy"], -depth])
    return camera_pos_base + camera_rot_base @ point_camera


def quaternion_matrix(quaternion: np.ndarray) -> np.ndarray:
    w, x, y, z = normalize_quaternion(quaternion)
    return np.array([
        [1 - 2 * (y*y + z*z), 2 * (x*y - z*w), 2 * (x*z + y*w)],
        [2 * (x*y + z*w), 1 - 2 * (x*x + z*z), 2 * (y*z - x*w)],
        [2 * (x*z - y*w), 2 * (y*z + x*w), 1 - 2 * (x*x + y*y)],
    ])


def pixel_to_base(image: dict, u: int, v: int) -> np.ndarray:
    """Unproject an RGB-D pixel using the calibration captured with that image."""
    if not (0 <= u < image["width"] and 0 <= v < image["height"]):
        raise ValueError("pixel is outside the image")
    depth_mm = float(image["depth"][v, u])
    if not np.isfinite(depth_mm) or not 0 < depth_mm < 65535:
        raise ValueError("pixel has no valid depth")
    calibration = image["intrinsics"]
    z = depth_mm / 1000.0
    # MuJoCo camera axes: +X right, +Y up, -Z forward; image v increases downwards.
    point = np.array([(u - calibration["cx"]) * z / calibration["fx"],
                      -(v - calibration["cy"]) * z / calibration["fy"], -z])
    return np.asarray(image["camera_position_base"]) + np.asarray(image["camera_rotation_base"]) @ point


def transform(position: np.ndarray, quaternion: np.ndarray, source: str, target: str, base_pos: np.ndarray, base_quat: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    if source == target:
        return position.copy(), quaternion.copy()
    inverse = np.array([base_quat[0], *(-base_quat[1:])])
    if source == "world" and target == "base":
        return rotate_vector(inverse, position - base_pos), normalize_quaternion(quaternion_multiply(inverse, quaternion))
    if source == "base" and target == "world":
        return rotate_vector(base_quat, position) + base_pos, normalize_quaternion(quaternion_multiply(base_quat, quaternion))
    raise ValueError(f"unsupported frame conversion {source}->{target}")
