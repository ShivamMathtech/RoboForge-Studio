from abc import ABC, abstractmethod


class MotionPlanner(ABC):
    @abstractmethod
    def plan(
        self, start: list[float], goal: list[float], obstacles: list[dict[str, object]]
    ) -> list[list[float]]: ...

