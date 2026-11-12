from __future__ import annotations

from typing import Any, Literal

from pydantic import BaseModel, Field, FiniteFloat, StrictInt, model_validator


class ErrorBody(BaseModel):
    code: str
    message: str
    details: dict[str, Any] = Field(default_factory=dict)


class Envelope(BaseModel):
    ok: bool
    request_id: str
    data: dict[str, Any] | None = None
    error: ErrorBody | None = None


class JointCommand(BaseModel):
    positions: list[float] = Field(min_length=6, max_length=6)
    timeout_s: float | None = Field(default=None, gt=0)


class PoseCommand(BaseModel):
    position: list[FiniteFloat] = Field(min_length=3, max_length=3)
    quaternion: list[FiniteFloat] | None = Field(default=None, min_length=4, max_length=4)
    frame: Literal["base"] = "base"
    timeout_s: float | None = Field(default=None, gt=0)


class DeltaPoseCommand(BaseModel):
    delta_position: list[FiniteFloat] = Field(default_factory=lambda: [0.0, 0.0, 0.0], min_length=3, max_length=3)
    delta_rpy: list[FiniteFloat] = Field(default_factory=lambda: [0.0, 0.0, 0.0], min_length=3, max_length=3)
    timeout_s: float | None = Field(default=None, gt=0)


class GripperCommand(BaseModel):
    action: Literal["open", "close"] | None = None
    opening_ratio: float | None = Field(default=None, ge=0, le=1)
    timeout_s: float | None = Field(default=None, gt=0)

    @model_validator(mode="after")
    def exactly_one(self) -> "GripperCommand":
        if (self.action is None) == (self.opening_ratio is None):
            raise ValueError("provide exactly one of action or opening_ratio")
        return self


class ObjectCommand(BaseModel):
    frame: Literal["base"] = "base"
    position: list[FiniteFloat] = Field(min_length=3, max_length=3)
    quaternion: list[float] | None = Field(default=None, min_length=4, max_length=4)
    settle_s: float = Field(default=0, ge=0)
    timeout_s: float | None = Field(default=None, gt=0)


class CameraIntrinsics(BaseModel):
    fx: FiniteFloat = Field(gt=0)
    fy: FiniteFloat = Field(gt=0)
    cx: FiniteFloat
    cy: FiniteFloat
    width: int = Field(gt=0)
    height: int = Field(gt=0)


class PixelPointCommand(BaseModel):
    camera: Literal["end", "left_rear"]
    pixel: list[StrictInt] = Field(min_length=2, max_length=2)
    depth_m: FiniteFloat = Field(gt=0, lt=65.535)
    tcp_position_base: list[FiniteFloat] | None = Field(default=None, min_length=3, max_length=3)
    tcp_quaternion_base: list[FiniteFloat] | None = Field(default=None, min_length=4, max_length=4)
    camera_position_tcp: list[FiniteFloat] | None = Field(default=None, min_length=3, max_length=3)
    camera_rotation_tcp: list[list[FiniteFloat]] | None = Field(default=None, min_length=3, max_length=3)
    intrinsics: CameraIntrinsics | None = None
    epoch: int | None = Field(default=None, ge=0)
    step: int | None = Field(default=None, ge=0)

    @model_validator(mode="after")
    def valid_vectors(self) -> "PixelPointCommand":
        if any(type(value) is not int or value < 0 for value in self.pixel):
            raise ValueError("pixel must contain two non-negative integers")
        if (self.tcp_position_base is None) != (self.tcp_quaternion_base is None):
            raise ValueError("provide both tcp_position_base and tcp_quaternion_base")
        calibration = (self.camera_position_tcp, self.camera_rotation_tcp, self.intrinsics)
        if any(value is not None for value in calibration) and not all(value is not None for value in calibration):
            raise ValueError("provide all camera extrinsics and intrinsics, or omit them all")
        if self.intrinsics is not None and self.tcp_position_base is None:
            raise ValueError("capture calibration requires the capture TCP pose")
        if self.camera_rotation_tcp is not None and any(len(row) != 3 for row in self.camera_rotation_tcp):
            raise ValueError("camera_rotation_tcp must be a 3x3 matrix")
        return self


class ResetCommand(BaseModel):
    settle_s: float = Field(default=0, ge=0)
    timeout_s: float | None = Field(default=None, gt=0)
