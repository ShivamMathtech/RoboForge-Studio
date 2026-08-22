from enum import Enum

from pydantic import BaseModel, Field


class TrajectoryType(str, Enum):
    LINEAR = "linear"
    CUBIC = "cubic"
    QUINTIC = "quintic"
    MINIMUM_JERK = "minimum_jerk"


class TrajectoryRequest(BaseModel):
    start: list[float]
    goal: list[float]
    duration: float = Field(gt=0, le=300)
    sample_count: int = Field(default=101, ge=2, le=10000)
    type: TrajectoryType = TrajectoryType.QUINTIC


class TrajectoryResponse(BaseModel):
    time: list[float]
    position: list[list[float]]
    velocity: list[list[float]]
    acceleration: list[list[float]]
    jerk: list[list[float]]

