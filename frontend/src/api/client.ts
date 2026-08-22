import type { FKResponse, JacobianResponse, RobotModel, SimulationState } from "../types/robot";

const API_URL = import.meta.env.VITE_API_URL ?? "";

async function request<T>(path: string, init?: RequestInit): Promise<T> {
  const response = await fetch(`${API_URL}${path}`, {
    ...init,
    headers: { "Content-Type": "application/json", ...init?.headers },
  });
  if (!response.ok) {
    const body = await response.json().catch(() => ({}));
    throw new Error(body.detail ?? `Request failed (${response.status})`);
  }
  return response.json() as Promise<T>;
}

export const api = {
  presets: () => request<RobotModel[]>("/api/robots/presets"),
  fk: (robot: RobotModel, q: number[]) =>
    request<FKResponse>("/api/kinematics/fk", {
      method: "POST",
      body: JSON.stringify({ robot, q }),
    }),
  jacobian: (robot: RobotModel, q: number[]) =>
    request<JacobianResponse>("/api/jacobian", {
      method: "POST",
      body: JSON.stringify({ robot, q }),
    }),
  ik: (robot: RobotModel, q: number[], target: number[]) =>
    request<{ q: number[]; converged: boolean; iterations: number; position_error: number; message: string }>(
      "/api/kinematics/ik",
      {
        method: "POST",
        body: JSON.stringify({
          robot,
          initial_q: q,
          target: { position: target },
          solver: "damped_least_squares",
          max_iterations: 250,
          tolerance: 0.0001,
          damping: 0.06,
        }),
      },
    ),
  step: (
    robot: RobotModel,
    state: SimulationState,
    gains: { kp: number; ki: number; kd: number },
    steps = 2,
  ) =>
    request<{ state: SimulationState; samples: SimulationState[] }>("/api/simulation/step", {
      method: "POST",
      body: JSON.stringify({
        robot,
        state,
        controller: { ...gains, output_limit: 100, anti_windup: true },
        dt: 0.01,
        steps,
        integrator: "semi_implicit_euler",
      }),
    }),
};

