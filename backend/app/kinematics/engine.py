from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from scipy.spatial.transform import Rotation

from app.models.robot import DHConvention, JointType, RobotModel


EPSILON = 1e-10


def _validate_q(robot: RobotModel, q: list[float] | np.ndarray) -> np.ndarray:
    values = np.asarray(q, dtype=float)
    if values.shape != (len(robot.joints),):
        raise ValueError(f"expected {len(robot.joints)} joint values, received {values.size}")
    if not np.all(np.isfinite(values)):
        raise ValueError("joint values must be finite")
    return values


def standard_dh(a: float, alpha: float, d: float, theta: float) -> np.ndarray:
    """Return the standard DH homogeneous transformation matrix."""
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)
    return np.array(
        [
            [ct, -st * ca, st * sa, a * ct],
            [st, ct * ca, -ct * sa, a * st],
            [0.0, sa, ca, d],
            [0.0, 0.0, 0.0, 1.0],
        ],
        dtype=float,
    )


def modified_dh(a: float, alpha: float, d: float, theta: float) -> np.ndarray:
    """Return the Craig modified-DH homogeneous transformation matrix."""
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)
    return np.array(
        [
            [ct, -st, 0.0, a],
            [st * ca, ct * ca, -sa, -d * sa],
            [st * sa, ct * sa, ca, d * ca],
            [0.0, 0.0, 0.0, 1.0],
        ],
        dtype=float,
    )


def joint_transform(robot: RobotModel, index: int, q_value: float) -> np.ndarray:
    joint = robot.joints[index]
    theta = joint.dh.theta_offset
    d = joint.dh.d
    if joint.type in {JointType.REVOLUTE, JointType.CONTINUOUS}:
        theta += q_value
    elif joint.type == JointType.PRISMATIC:
        d += q_value
    transform = modified_dh if robot.convention == DHConvention.MODIFIED else standard_dh
    return transform(joint.dh.a, joint.dh.alpha, d, theta)


def forward_kinematics(
    robot: RobotModel, q: list[float] | np.ndarray
) -> tuple[np.ndarray, list[np.ndarray], list[np.ndarray]]:
    """Calculate end-effector pose and every intermediate frame."""
    values = _validate_q(robot, q)
    world_to_current = np.eye(4)
    frames = [world_to_current.copy()]
    relative_transforms: list[np.ndarray] = []
    for index, value in enumerate(values):
        relative = joint_transform(robot, index, float(value))
        world_to_current = world_to_current @ relative
        if not np.all(np.isfinite(world_to_current)):
            raise FloatingPointError("non-finite value produced during forward kinematics")
        relative_transforms.append(relative)
        frames.append(world_to_current.copy())
    return world_to_current, frames, relative_transforms


def pose_components(transform: np.ndarray) -> dict[str, list]:
    rotation = transform[:3, :3]
    quaternion = Rotation.from_matrix(rotation).as_quat()
    rpy = Rotation.from_matrix(rotation).as_euler("xyz")
    return {
        "position": transform[:3, 3].tolist(),
        "rotation_matrix": rotation.tolist(),
        "quaternion_xyzw": quaternion.tolist(),
        "rpy": rpy.tolist(),
        "transform": transform.tolist(),
    }


def geometric_jacobian(robot: RobotModel, q: list[float] | np.ndarray) -> np.ndarray:
    """Build the 6×n spatial geometric Jacobian in the world frame."""
    values = _validate_q(robot, q)
    _, frames, _ = forward_kinematics(robot, values)
    end_position = frames[-1][:3, 3]
    jacobian = np.zeros((6, len(robot.joints)), dtype=float)

    for index, joint in enumerate(robot.joints):
        origin = frames[index][:3, 3]
        axis = frames[index][:3, :3] @ np.asarray(joint.axis, dtype=float)
        norm = np.linalg.norm(axis)
        if norm < EPSILON:
            raise ValueError(f"joint {index + 1} has a zero-length axis")
        axis /= norm
        if joint.type in {JointType.REVOLUTE, JointType.CONTINUOUS}:
            jacobian[:3, index] = np.cross(axis, end_position - origin)
            jacobian[3:, index] = axis
        elif joint.type == JointType.PRISMATIC:
            jacobian[:3, index] = axis
    return jacobian


def jacobian_metrics(jacobian: np.ndarray) -> dict[str, float | int | bool | list[float] | None]:
    singular_values = np.linalg.svd(jacobian, compute_uv=False)
    nonzero = singular_values[singular_values > EPSILON]
    rank = int(np.linalg.matrix_rank(jacobian, tol=1e-9))
    condition = float(nonzero.max() / nonzero.min()) if nonzero.size else None
    expected_rank = min(jacobian.shape)
    manipulability = float(np.prod(singular_values[:expected_rank])) if rank == expected_rank else 0.0
    near_singular = rank < expected_rank or bool(nonzero.size and nonzero.min() < 1e-4)
    return {
        "singular_values": singular_values.tolist(),
        "rank": rank,
        "condition_number": condition,
        "manipulability": manipulability,
        "near_singular": near_singular,
    }


def _clamp_joints(robot: RobotModel, q: np.ndarray) -> np.ndarray:
    clamped = q.copy()
    for index, joint in enumerate(robot.joints):
        if joint.type != JointType.CONTINUOUS:
            clamped[index] = np.clip(clamped[index], joint.limit.lower, joint.limit.upper)
    return clamped


@dataclass
class IKResult:
    q: np.ndarray
    converged: bool
    iterations: int
    position_error: float
    message: str
    path: list[list[float]]


def inverse_kinematics(
    robot: RobotModel,
    target_position: list[float] | np.ndarray,
    initial_q: list[float] | np.ndarray,
    *,
    max_iterations: int = 150,
    tolerance: float = 1e-5,
    damping: float = 0.08,
    step_limit: float = 0.25,
) -> IKResult:
    """Solve position IK with a joint-limited damped least-squares iteration."""
    target = np.asarray(target_position, dtype=float)
    if target.shape != (3,) or not np.all(np.isfinite(target)):
        raise ValueError("target position must contain three finite values")
    q = _clamp_joints(robot, _validate_q(robot, initial_q))
    path = [q.tolist()]

    for iteration in range(1, max_iterations + 1):
        transform, _, _ = forward_kinematics(robot, q)
        error_vector = target - transform[:3, 3]
        error_norm = float(np.linalg.norm(error_vector))
        if error_norm <= tolerance:
            return IKResult(q, True, iteration - 1, error_norm, "Target reached", path)

        position_jacobian = geometric_jacobian(robot, q)[:3]
        regularizer = (damping**2) * np.eye(3)
        delta = position_jacobian.T @ np.linalg.solve(
            position_jacobian @ position_jacobian.T + regularizer, error_vector
        )
        delta_norm = float(np.linalg.norm(delta))
        if delta_norm > step_limit:
            delta *= step_limit / delta_norm
        next_q = _clamp_joints(robot, q + delta)
        if not np.all(np.isfinite(next_q)):
            return IKResult(q, False, iteration, error_norm, "Numerical instability detected", path)
        q = next_q
        if iteration == 1 or iteration % 4 == 0:
            path.append(q.tolist())

    transform, _, _ = forward_kinematics(robot, q)
    final_error = float(np.linalg.norm(target - transform[:3, 3]))
    return IKResult(
        q,
        False,
        max_iterations,
        final_error,
        f"IK solver did not converge after {max_iterations} iterations",
        path,
    )
