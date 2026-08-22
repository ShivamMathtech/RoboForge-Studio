from math import pi

import numpy as np

from app.kinematics.engine import (
    forward_kinematics,
    geometric_jacobian,
    inverse_kinematics,
    standard_dh,
)
from app.robots.presets import PRESETS


def test_standard_dh_translation_at_zero_angle() -> None:
    transform = standard_dh(a=0.8, alpha=0.0, d=0.0, theta=0.0)
    np.testing.assert_allclose(transform, [[1, 0, 0, 0.8], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])


def test_planar_forward_kinematics_reference_pose() -> None:
    robot = PRESETS["planar-2dof"]
    transform, frames, _ = forward_kinematics(robot, [0.0, 0.0])
    np.testing.assert_allclose(transform[:3, 3], [1.6, 0.0, 0.0], atol=1e-12)
    assert len(frames) == 3


def test_planar_forward_kinematics_right_angle() -> None:
    robot = PRESETS["planar-2dof"]
    transform, _, _ = forward_kinematics(robot, [pi / 2, 0.0])
    np.testing.assert_allclose(transform[:3, 3], [0.0, 1.6, 0.0], atol=1e-12)


def test_planar_jacobian_matches_finite_difference() -> None:
    robot = PRESETS["planar-2dof"]
    q = np.asarray([0.35, -0.7])
    analytical = geometric_jacobian(robot, q)[:3]
    numerical = np.zeros_like(analytical)
    epsilon = 1e-7
    for index in range(2):
        shifted = q.copy()
        shifted[index] += epsilon
        plus, _, _ = forward_kinematics(robot, shifted)
        base, _, _ = forward_kinematics(robot, q)
        numerical[:, index] = (plus[:3, 3] - base[:3, 3]) / epsilon
    np.testing.assert_allclose(analytical, numerical, atol=2e-6)


def test_damped_least_squares_ik_reaches_planar_target() -> None:
    robot = PRESETS["planar-2dof"]
    result = inverse_kinematics(robot, [0.9, 0.8, 0.0], [0.2, 0.1], max_iterations=400)
    assert result.converged
    assert result.position_error < 1e-4
    reached, _, _ = forward_kinematics(robot, result.q)
    np.testing.assert_allclose(reached[:3, 3], [0.9, 0.8, 0.0], atol=1e-4)


def test_invalid_joint_vector_rejected() -> None:
    robot = PRESETS["planar-2dof"]
    try:
        forward_kinematics(robot, [0.0])
    except ValueError as error:
        assert "expected 2 joint values" in str(error)
    else:
        raise AssertionError("invalid joint vector was accepted")

