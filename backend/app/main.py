import asyncio

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware

from app.api.routes import router
from app.core.config import get_settings
from app.models.simulation import SimulationStepRequest
from app.simulation.engine import SimulationEngine


settings = get_settings()
app = FastAPI(
    title=settings.app_name,
    version=settings.app_version,
    description="Robotics kinematics, trajectory, control, and deterministic simulation API.",
)
app.add_middleware(
    CORSMiddleware,
    allow_origins=settings.cors_origins,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)
app.include_router(router)


@app.get("/")
def root() -> dict[str, str]:
    return {"name": settings.app_name, "documentation": "/docs", "health": "/api/health"}


@app.websocket("/ws/simulation")
async def simulation_socket(websocket: WebSocket) -> None:
    """Stream deterministic simulation samples from a client-supplied request."""
    await websocket.accept()
    try:
        payload = await websocket.receive_json()
        request = SimulationStepRequest.model_validate(payload)
        engine = SimulationEngine(request.robot, request.controller, request.state)
        while True:
            state = engine.step(request.dt, request.integrator)
            await websocket.send_json(state.model_dump())
            await asyncio.sleep(request.dt)
    except WebSocketDisconnect:
        return
    except Exception as error:
        await websocket.send_json({"error": str(error)})
        await websocket.close(code=1011)

