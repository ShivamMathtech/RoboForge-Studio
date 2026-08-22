import numpy as np

from app.controllers.base import Controller


class PIDController(Controller):
    def __init__(
        self,
        kp: np.ndarray,
        ki: np.ndarray,
        kd: np.ndarray,
        output_limit: float = 100.0,
        anti_windup: bool = True,
    ) -> None:
        self.kp = np.asarray(kp, dtype=float)
        self.ki = np.asarray(ki, dtype=float)
        self.kd = np.asarray(kd, dtype=float)
        self.output_limit = float(output_limit)
        self.anti_windup = anti_windup
        self.integral = np.zeros_like(self.kp)

    def reset(self) -> None:
        self.integral.fill(0.0)

    def compute_control(
        self, target: np.ndarray, position: np.ndarray, velocity: np.ndarray, dt: float
    ) -> np.ndarray:
        if dt <= 0:
            raise ValueError("controller timestep must be positive")
        error = target - position
        candidate_integral = self.integral + error * dt
        raw = self.kp * error + self.ki * candidate_integral - self.kd * velocity
        output = np.clip(raw, -self.output_limit, self.output_limit)
        if not self.anti_windup or np.allclose(raw, output):
            self.integral = candidate_integral
        return output

    def get_parameters(self) -> dict[str, object]:
        return {
            "kp": self.kp.tolist(),
            "ki": self.ki.tolist(),
            "kd": self.kd.tolist(),
            "output_limit": self.output_limit,
            "anti_windup": self.anti_windup,
        }

