from abc import ABC, abstractmethod

import numpy as np


class Controller(ABC):
    """Common extension contract for every joint-space controller."""

    @abstractmethod
    def reset(self) -> None: ...

    @abstractmethod
    def compute_control(
        self, target: np.ndarray, position: np.ndarray, velocity: np.ndarray, dt: float
    ) -> np.ndarray: ...

    @abstractmethod
    def get_parameters(self) -> dict[str, object]: ...

