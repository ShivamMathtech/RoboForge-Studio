import numpy as np

from app.controllers.pid import PIDController
from app.models.simulation import ControllerParameters, SimulationState, SimulationStepRequest
from app.robots.presets import PRESETS
from app.simulation.engine import SimulationEngine


def test_pid_output_and_limit() -> None:
    controller = PIDController(np.array([100.0]), np.array([0.0]), np.array([10.0]), output_limit=20.0)
    output = controller.compute_control(np.array([1.0]), np.array([0.0]), np.array([0.0]), 0.01)
    np.testing.assert_allclose(output, [20.0])


def test_simulation_time_is_deterministic_and_tracks_target() -> None:
    robot = PRESETS["planar-2dof"]
    state = SimulationState(
        q=[0.0, 0.0], qd=[0.0, 0.0], qdd=[0.0, 0.0], torque=[0.0, 0.0],
        target_q=[0.4, -0.2], error=[0.4, -0.2], end_effector_position=[1.6, 0.0, 0.0],
    )
    request = SimulationStepRequest(robot=robot, state=state, controller=ControllerParameters(kp=30, kd=8), dt=0.01, steps=100)
    final, samples = SimulationEngine.run_request(request)
    assert final.timestamp == 1.0000000000000007
    assert len(samples) == 100
    assert abs(final.error[0]) < abs(state.error[0])
    assert final.energy >= 0

