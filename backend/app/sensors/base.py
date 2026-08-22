from abc import ABC, abstractmethod
from dataclasses import dataclass

import numpy as np


@dataclass
class NoiseModel:
    standard_deviation: float = 0.0
    bias: float = 0.0
    drift_per_second: float = 0.0

    def apply(self, value: np.ndarray, time: float, rng: np.random.Generator) -> np.ndarray:
        return value + self.bias + self.drift_per_second * time + rng.normal(0, self.standard_deviation, value.shape)


class Sensor(ABC):
    @abstractmethod
    def sample(self, state: dict[str, object], timestamp: float) -> dict[str, object]: ...

