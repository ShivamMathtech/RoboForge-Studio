from fastapi.testclient import TestClient

from app.main import app
from app.robots.presets import PRESETS


client = TestClient(app)


def test_health() -> None:
    response = client.get("/api/health")
    assert response.status_code == 200
    assert response.json()["status"] == "healthy"


def test_fk_endpoint_returns_real_transform() -> None:
    robot = PRESETS["planar-2dof"]
    response = client.post("/api/kinematics/fk", json={"robot": robot.model_dump(mode="json"), "q": [0.0, 0.0]})
    assert response.status_code == 200
    assert response.json()["pose"]["position"] == [1.6, 0.0, 0.0]


def test_bad_fk_input_is_descriptive() -> None:
    robot = PRESETS["planar-2dof"]
    response = client.post("/api/kinematics/fk", json={"robot": robot.model_dump(mode="json"), "q": [0.0]})
    assert response.status_code == 422
    assert "expected 2 joint values" in response.json()["detail"]

