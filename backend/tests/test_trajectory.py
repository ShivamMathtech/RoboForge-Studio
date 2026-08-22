import numpy as np

from app.models.trajectory import TrajectoryRequest, TrajectoryType
from app.trajectory.generator import generate_trajectory


def test_quintic_trajectory_satisfies_boundary_conditions() -> None:
    request = TrajectoryRequest(start=[0.0, -1.0], goal=[1.0, 2.0], duration=2.0, type=TrajectoryType.QUINTIC)
    result = generate_trajectory(request)
    np.testing.assert_allclose(result["position"][0], request.start)
    np.testing.assert_allclose(result["position"][-1], request.goal)
    np.testing.assert_allclose(result["velocity"][0], [0.0, 0.0])
    np.testing.assert_allclose(result["velocity"][-1], [0.0, 0.0], atol=1e-12)
    np.testing.assert_allclose(result["acceleration"][0], [0.0, 0.0])
    np.testing.assert_allclose(result["acceleration"][-1], [0.0, 0.0], atol=1e-12)


def test_linear_trajectory_has_constant_velocity() -> None:
    request = TrajectoryRequest(start=[0.0], goal=[2.0], duration=4.0, sample_count=11, type=TrajectoryType.LINEAR)
    result = generate_trajectory(request)
    np.testing.assert_allclose(result["velocity"], np.full((11, 1), 0.5))

