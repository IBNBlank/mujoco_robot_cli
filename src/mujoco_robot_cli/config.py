from __future__ import annotations

from pathlib import Path
from typing import Any

import yaml
from pydantic import BaseModel, Field


class Settings(BaseModel):
    model: str = "archer_y6"
    host: str = "127.0.0.1"
    port: int = 8000
    headless: bool = False
    render_backend: str = "glfw"
    sim_hz: int = 1000
    state_hz: int = 500
    camera_hz: int = 30
    image_width: int = 224
    image_height: int = 224
    freshness_s: float = 0.25
    startup_timeout_s: float = 20
    default_timeout_s: float = 15
    max_timeout_s: float = 60
    joint_speed_limit: float = 3.0
    gripper_speed_limit: float = 2.0
    position_tolerance: float = 0.01
    velocity_tolerance: float = 0.1
    pose_position_tolerance: float = 0.01
    pose_orientation_tolerance: float = 0.05
    gripper_tolerance: float = 0.03
    settle_s: float = 0.1
    blocked_window_s: float = 1.0
    blocked_delta: float = 0.001
    gripper_blocked_delta: float = 0.005
    arm_kp: list[float] = Field(default_factory=lambda: [2000, 2000, 2500, 1000, 500, 500])
    arm_kd: list[float] = Field(default_factory=lambda: [80, 80, 80, 40, 20, 20])
    gripper_kp: float = 200.0
    gripper_kd: float = 10.0
    table_z: float = 0.30
    object_clearance: float = 0.005


def default_config_path() -> Path:
    return Path(__file__).resolve().parents[2] / "configs" / "default.yaml"


def load_settings(path: str | Path | None = None, overrides: dict[str, Any] | None = None) -> Settings:
    data: dict[str, Any] = {}
    if path is not None:
        data.update(yaml.safe_load(Path(path).read_text()) or {})
    elif default_config_path().is_file():
        data.update(yaml.safe_load(default_config_path().read_text()) or {})
    data.update(overrides or {})
    return Settings.model_validate(data)
