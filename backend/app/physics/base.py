from abc import ABC, abstractmethod
from typing import Any


class PhysicsEngine(ABC):
    """Backend-neutral physics contract for PyBullet/MuJoCo/Gazebo adapters."""

    @abstractmethod
    def initialize(self, robot: Any) -> None: ...

    @abstractmethod
    def step(self, dt: float) -> None: ...

    @abstractmethod
    def apply_force(self, body_id: str, force: list[float]) -> None: ...

    @abstractmethod
    def apply_torque(self, joint_id: str, torque: float) -> None: ...

    @abstractmethod
    def detect_collisions(self) -> list[dict[str, Any]]: ...

    @abstractmethod
    def get_state(self) -> dict[str, Any]: ...

