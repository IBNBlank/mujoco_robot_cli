from __future__ import annotations

from typing import Any, Literal

from pydantic import BaseModel, Field, model_validator


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
    position: list[float] = Field(min_length=3, max_length=3)
    quaternion: list[float] | None = Field(default=None, min_length=4, max_length=4)
    frame: Literal["world", "base"] = "world"
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
    position: list[float] = Field(min_length=3, max_length=3)
    quaternion: list[float] | None = Field(default=None, min_length=4, max_length=4)
    settle_s: float = Field(default=0, ge=0)
    timeout_s: float | None = Field(default=None, gt=0)


class ResetCommand(BaseModel):
    settle_s: float = Field(default=0, ge=0)
    timeout_s: float | None = Field(default=None, gt=0)
