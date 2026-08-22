import numpy as np

from app.models.trajectory import TrajectoryRequest, TrajectoryType


def _blend(kind: TrajectoryType, s: np.ndarray) -> tuple[np.ndarray, ...]:
    if kind == TrajectoryType.LINEAR:
        return s, np.ones_like(s), np.zeros_like(s), np.zeros_like(s)
    if kind == TrajectoryType.CUBIC:
        return 3 * s**2 - 2 * s**3, 6 * s - 6 * s**2, 6 - 12 * s, -12 * np.ones_like(s)
    # Quintic is also the classic minimum-jerk blend for rest-to-rest motion.
    p = 10 * s**3 - 15 * s**4 + 6 * s**5
    v = 30 * s**2 - 60 * s**3 + 30 * s**4
    a = 60 * s - 180 * s**2 + 120 * s**3
    j = 60 - 360 * s + 360 * s**2
    return p, v, a, j


def generate_trajectory(request: TrajectoryRequest) -> dict[str, list]:
    start = np.asarray(request.start, dtype=float)
    goal = np.asarray(request.goal, dtype=float)
    if start.shape != goal.shape or start.ndim != 1:
        raise ValueError("start and goal must be same-length vectors")
    if not np.all(np.isfinite(start)) or not np.all(np.isfinite(goal)):
        raise ValueError("trajectory endpoints must be finite")

    time = np.linspace(0.0, request.duration, request.sample_count)
    normalized_time = time / request.duration
    p, v, a, j = _blend(request.type, normalized_time)
    delta = goal - start
    position = start + p[:, None] * delta
    velocity = v[:, None] * delta / request.duration
    acceleration = a[:, None] * delta / request.duration**2
    jerk = j[:, None] * delta / request.duration**3
    return {
        "time": time.tolist(),
        "position": position.tolist(),
        "velocity": velocity.tolist(),
        "acceleration": acceleration.tolist(),
        "jerk": jerk.tolist(),
    }

