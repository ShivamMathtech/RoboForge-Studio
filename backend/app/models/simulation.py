from enum import Enum

from pydantic import BaseModel, Field

from app.models.robot import RobotModel


class Integrator(str, Enum):
    EULER = "euler"
    SEMI_IMPLICIT_EULER = "semi_implicit_euler"
    RK4 = "rk4"


class ControllerParameters(BaseModel):
    kp: list[float] | float = 45.0
    ki: list[float] | float = 0.0
    kd: list[float] | float = 9.0
    output_limit: float = Field(default=100.0, gt=0)
    anti_windup: bool = True


class SimulationState(BaseModel):
    timestamp: float = 0.0
    q: list[float]
    qd: list[float]
    qdd: list[float]
    torque: list[float]
    target_q: list[float]
    error: list[float]
    end_effector_position: list[float]
    energy: float = 0.0
    controller_integral: list[float] = Field(default_factory=list)


class SimulationStepRequest(BaseModel):
    robot: RobotModel
    state: SimulationState
    controller: ControllerParameters = Field(default_factory=ControllerParameters)
    dt: float = Field(default=0.01, gt=0, le=0.1)
    integrator: Integrator = Integrator.SEMI_IMPLICIT_EULER
    steps: int = Field(default=1, ge=1, le=1000)


class SimulationStepResponse(BaseModel):
    state: SimulationState
    samples: list[SimulationState]
