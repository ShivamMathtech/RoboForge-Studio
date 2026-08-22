from enum import Enum
from uuid import uuid4

from pydantic import BaseModel, Field, model_validator


class JointType(str, Enum):
    REVOLUTE = "revolute"
    PRISMATIC = "prismatic"
    FIXED = "fixed"
    CONTINUOUS = "continuous"


class DHConvention(str, Enum):
    STANDARD = "standard"
    MODIFIED = "modified"


class JointLimit(BaseModel):
    lower: float = -3.141592653589793
    upper: float = 3.141592653589793
    velocity: float = Field(default=2.0, gt=0)
    acceleration: float = Field(default=5.0, gt=0)
    torque: float = Field(default=100.0, gt=0)

    @model_validator(mode="after")
    def valid_range(self) -> "JointLimit":
        if self.lower >= self.upper:
            raise ValueError("joint lower limit must be smaller than upper limit")
        return self


class DHParameter(BaseModel):
    a: float = Field(description="Link length in metres")
    alpha: float = Field(description="Link twist in radians")
    d: float = Field(description="Link offset in metres")
    theta_offset: float = Field(default=0.0, description="Fixed angle offset in radians")


class Joint(BaseModel):
    id: str = Field(default_factory=lambda: str(uuid4()))
    name: str
    type: JointType = JointType.REVOLUTE
    axis: list[float] = Field(default_factory=lambda: [0.0, 0.0, 1.0], min_length=3, max_length=3)
    dh: DHParameter
    limit: JointLimit = Field(default_factory=JointLimit)
    damping: float = Field(default=0.05, ge=0)
    friction: float = Field(default=0.0, ge=0)
    mass: float = Field(default=1.0, gt=0)


class RobotModel(BaseModel):
    id: str = Field(default_factory=lambda: str(uuid4()))
    name: str
    description: str = ""
    convention: DHConvention = DHConvention.STANDARD
    joints: list[Joint] = Field(min_length=1, max_length=12)
    educational: bool = True
    payload_kg: float = Field(default=1.0, ge=0)

    @property
    def dof(self) -> int:
        return sum(j.type not in {JointType.FIXED} for j in self.joints)


class Pose(BaseModel):
    position: list[float] = Field(min_length=3, max_length=3)
    rotation_matrix: list[list[float]]
    quaternion_xyzw: list[float] = Field(min_length=4, max_length=4)
    rpy: list[float] = Field(min_length=3, max_length=3)
    transform: list[list[float]]


class FKRequest(BaseModel):
    robot: RobotModel
    q: list[float]


class FKResponse(BaseModel):
    pose: Pose
    frames: list[list[list[float]]]
    joint_transforms: list[list[list[float]]]


class IKTarget(BaseModel):
    position: list[float] = Field(min_length=3, max_length=3)
    rpy: list[float] | None = Field(default=None, min_length=3, max_length=3)


class IKRequest(BaseModel):
    robot: RobotModel
    target: IKTarget
    initial_q: list[float]
    solver: str = "damped_least_squares"
    max_iterations: int = Field(default=150, ge=1, le=2000)
    tolerance: float = Field(default=1e-5, gt=0)
    damping: float = Field(default=0.08, gt=0)


class IKResponse(BaseModel):
    q: list[float]
    converged: bool
    iterations: int
    position_error: float
    message: str
    path: list[list[float]]


class JacobianRequest(BaseModel):
    robot: RobotModel
    q: list[float]


class JacobianResponse(BaseModel):
    matrix: list[list[float]]
    singular_values: list[float]
    rank: int
    condition_number: float | None
    manipulability: float
    near_singular: bool

