from __future__ import annotations

import numpy as np

from app.controllers.pid import PIDController
from app.kinematics.engine import forward_kinematics
from app.models.robot import RobotModel
from app.models.simulation import (
    ControllerParameters,
    Integrator,
    SimulationState,
    SimulationStepRequest,
)


def _vector(value: list[float] | float, length: int) -> np.ndarray:
    if isinstance(value, list):
        array = np.asarray(value, dtype=float)
        if array.shape != (length,):
            raise ValueError(f"controller gain vector must contain {length} values")
        return array
    return np.full(length, float(value))


class SimulationEngine:
    """Deterministic joint simulation with pluggable integration and control."""

    def __init__(
        self, robot: RobotModel, parameters: ControllerParameters, initial_state: SimulationState
    ) -> None:
        self.robot = robot
        self.state = initial_state
        n = len(robot.joints)
        self.inertia = np.asarray([joint.mass * max(joint.dh.a**2, 0.04) for joint in robot.joints])
        self.damping = np.asarray([joint.damping for joint in robot.joints])
        self.controller = PIDController(
            _vector(parameters.kp, n),
            _vector(parameters.ki, n),
            _vector(parameters.kd, n),
            parameters.output_limit,
            parameters.anti_windup,
        )
        if len(initial_state.controller_integral) == n:
            self.controller.integral = np.asarray(initial_state.controller_integral, dtype=float)

    def _acceleration(self, qd: np.ndarray, torque: np.ndarray) -> np.ndarray:
        # Educational decoupled rigid-body approximation: M(q)qdd + Dqd = tau.
        return (torque - self.damping * qd) / self.inertia

    def _integrate(
        self, q: np.ndarray, qd: np.ndarray, torque: np.ndarray, dt: float, method: Integrator
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        if method == Integrator.EULER:
            qdd = self._acceleration(qd, torque)
            return q + qd * dt, qd + qdd * dt, qdd
        if method == Integrator.RK4:
            def derivative(state: np.ndarray) -> np.ndarray:
                n = q.size
                return np.concatenate([state[n:], self._acceleration(state[n:], torque)])

            state = np.concatenate([q, qd])
            k1 = derivative(state)
            k2 = derivative(state + dt * k1 / 2)
            k3 = derivative(state + dt * k2 / 2)
            k4 = derivative(state + dt * k3)
            result = state + dt * (k1 + 2 * k2 + 2 * k3 + k4) / 6
            next_q, next_qd = np.split(result, 2)
            return next_q, next_qd, self._acceleration(next_qd, torque)

        qdd = self._acceleration(qd, torque)
        next_qd = qd + qdd * dt
        return q + next_qd * dt, next_qd, qdd

    def step(self, dt: float, method: Integrator) -> SimulationState:
        q = np.asarray(self.state.q, dtype=float)
        qd = np.asarray(self.state.qd, dtype=float)
        target = np.asarray(self.state.target_q, dtype=float)
        torque = self.controller.compute_control(target, q, qd, dt)
        q, qd, qdd = self._integrate(q, qd, torque, dt, method)

        for index, joint in enumerate(self.robot.joints):
            qd[index] = np.clip(qd[index], -joint.limit.velocity, joint.limit.velocity)
            if joint.type.value != "continuous":
                q[index] = np.clip(q[index], joint.limit.lower, joint.limit.upper)
            torque[index] = np.clip(torque[index], -joint.limit.torque, joint.limit.torque)

        if not all(np.all(np.isfinite(values)) for values in (q, qd, qdd, torque)):
            raise FloatingPointError("simulation produced NaN or infinite values")
        transform, _, _ = forward_kinematics(self.robot, q)
        energy = float(0.5 * np.sum(self.inertia * qd**2))
        self.state = SimulationState(
            timestamp=self.state.timestamp + dt,
            q=q.tolist(),
            qd=qd.tolist(),
            qdd=qdd.tolist(),
            torque=torque.tolist(),
            target_q=target.tolist(),
            error=(target - q).tolist(),
            end_effector_position=transform[:3, 3].tolist(),
            energy=energy,
            controller_integral=self.controller.integral.tolist(),
        )
        return self.state

    @classmethod
    def run_request(cls, request: SimulationStepRequest) -> tuple[SimulationState, list[SimulationState]]:
        engine = cls(request.robot, request.controller, request.state)
        samples = [engine.step(request.dt, request.integrator) for _ in range(request.steps)]
        return engine.state, samples
