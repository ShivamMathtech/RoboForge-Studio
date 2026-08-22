from copy import deepcopy
from uuid import uuid4

from fastapi import APIRouter, HTTPException

from app.kinematics.engine import (
    forward_kinematics,
    geometric_jacobian,
    inverse_kinematics,
    jacobian_metrics,
    pose_components,
)
from app.models.robot import (
    FKRequest,
    FKResponse,
    IKRequest,
    IKResponse,
    JacobianRequest,
    JacobianResponse,
    RobotModel,
)
from app.models.simulation import SimulationStepRequest, SimulationStepResponse
from app.models.trajectory import TrajectoryRequest, TrajectoryResponse
from app.robots.presets import PRESETS
from app.simulation.engine import SimulationEngine
from app.trajectory.generator import generate_trajectory


router = APIRouter(prefix="/api")
robot_store: dict[str, RobotModel] = {key: deepcopy(robot) for key, robot in PRESETS.items()}


@router.get("/health")
def health() -> dict[str, str]:
    return {"status": "healthy", "service": "roboforge-api", "version": "1.0.0"}


@router.get("/robots/presets", response_model=list[RobotModel])
def list_presets() -> list[RobotModel]:
    return list(PRESETS.values())


@router.get("/robots", response_model=list[RobotModel])
def list_robots() -> list[RobotModel]:
    return list(robot_store.values())


@router.get("/robots/{robot_id}", response_model=RobotModel)
def get_robot(robot_id: str) -> RobotModel:
    if robot_id not in robot_store:
        raise HTTPException(404, "Robot was not found")
    return robot_store[robot_id]


@router.post("/robots", response_model=RobotModel, status_code=201)
def create_robot(robot: RobotModel) -> RobotModel:
    created = robot.model_copy(update={"id": str(uuid4())})
    robot_store[created.id] = created
    return created


@router.put("/robots/{robot_id}", response_model=RobotModel)
def update_robot(robot_id: str, robot: RobotModel) -> RobotModel:
    if robot_id not in robot_store:
        raise HTTPException(404, "Robot was not found")
    updated = robot.model_copy(update={"id": robot_id})
    robot_store[robot_id] = updated
    return updated


@router.post("/kinematics/fk", response_model=FKResponse)
def calculate_fk(request: FKRequest) -> FKResponse:
    try:
        transform, frames, relative = forward_kinematics(request.robot, request.q)
        return FKResponse(
            pose=pose_components(transform),
            frames=[frame.tolist() for frame in frames],
            joint_transforms=[matrix.tolist() for matrix in relative],
        )
    except (ValueError, FloatingPointError) as error:
        raise HTTPException(422, str(error)) from error


@router.post("/kinematics/ik", response_model=IKResponse)
def calculate_ik(request: IKRequest) -> IKResponse:
    try:
        result = inverse_kinematics(
            request.robot,
            request.target.position,
            request.initial_q,
            max_iterations=request.max_iterations,
            tolerance=request.tolerance,
            damping=request.damping,
        )
        return IKResponse(
            q=result.q.tolist(),
            converged=result.converged,
            iterations=result.iterations,
            position_error=result.position_error,
            message=result.message,
            path=result.path,
        )
    except (ValueError, FloatingPointError) as error:
        raise HTTPException(422, str(error)) from error


@router.post("/jacobian", response_model=JacobianResponse)
def calculate_jacobian(request: JacobianRequest) -> JacobianResponse:
    try:
        matrix = geometric_jacobian(request.robot, request.q)
        return JacobianResponse(matrix=matrix.tolist(), **jacobian_metrics(matrix))
    except (ValueError, FloatingPointError) as error:
        raise HTTPException(422, str(error)) from error


@router.post("/trajectory/generate", response_model=TrajectoryResponse)
def trajectory(request: TrajectoryRequest) -> TrajectoryResponse:
    try:
        return TrajectoryResponse(**generate_trajectory(request))
    except ValueError as error:
        raise HTTPException(422, str(error)) from error


@router.post("/simulation/step", response_model=SimulationStepResponse)
def simulation_step(request: SimulationStepRequest) -> SimulationStepResponse:
    try:
        state, samples = SimulationEngine.run_request(request)
        return SimulationStepResponse(state=state, samples=samples)
    except (ValueError, FloatingPointError) as error:
        raise HTTPException(422, str(error)) from error

