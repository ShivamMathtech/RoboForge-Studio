import {
  Activity,
  Box,
  Braces,
  ChevronDown,
  CircleGauge,
  Crosshair,
  Database,
  Focus,
  GitBranch,
  Grid3X3,
  Hexagon,
  Pause,
  Play,
  RotateCcw,
  Save,
  Settings,
  SkipForward,
  SlidersHorizontal,
  Target,
  TriangleAlert,
  Waypoints,
} from "lucide-react";
import { useCallback, useEffect, useRef, useState } from "react";

import { api } from "./api/client";
import { MatrixView } from "./components/MatrixView";
import { RobotViewport } from "./components/RobotViewport";
import { TelemetryChart } from "./components/TelemetryChart";
import type { FKResponse, JacobianResponse, RobotModel, Sample, SimulationState } from "./types/robot";

const identity = [
  [1, 0, 0, 0],
  [0, 1, 0, 0],
  [0, 0, 1, 0],
  [0, 0, 0, 1],
];

const initialState = (robot: RobotModel): SimulationState => {
  const zero = robot.joints.map(() => 0);
  return {
    timestamp: 0,
    q: [...zero], qd: [...zero], qdd: [...zero], torque: [...zero],
    target_q: [...zero], error: [...zero], end_effector_position: [0, 0, 0], energy: 0,
    controller_integral: [...zero],
  };
};

function format(value: number, digits = 3) {
  return Number.isFinite(value) ? value.toFixed(digits) : "—";
}

export default function App() {
  const [robots, setRobots] = useState<RobotModel[]>([]);
  const [robot, setRobot] = useState<RobotModel | null>(null);
  const [state, setState] = useState<SimulationState | null>(null);
  const stateRef = useRef<SimulationState | null>(null);
  const [fk, setFk] = useState<FKResponse | null>(null);
  const [jacobian, setJacobian] = useState<JacobianResponse | null>(null);
  const [target, setTarget] = useState([0.8, 0.4, 0.7]);
  const [gains, setGains] = useState({ kp: 45, ki: 0, kd: 9 });
  const [playing, setPlaying] = useState(false);
  const [speed, setSpeed] = useState(1);
  const [editMode, setEditMode] = useState<"pose" | "target">("pose");
  const [showFrames, setShowFrames] = useState(true);
  const [showTrail, setShowTrail] = useState(true);
  const [bottomTab, setBottomTab] = useState<"telemetry" | "matrix" | "log">("telemetry");
  const [samples, setSamples] = useState<Sample[]>([]);
  const [trail, setTrail] = useState<number[][]>([]);
  const [message, setMessage] = useState("Connecting to numerical engine…");
  const [error, setError] = useState<string | null>(null);
  const [solveStatus, setSolveStatus] = useState<string>("Target ready");
  const [mathTime, setMathTime] = useState(0);
  const busy = useRef(false);

  useEffect(() => {
    api.presets()
      .then((items) => {
        setRobots(items);
        const selected = items.find((item) => item.id === "industrial-6dof") ?? items[0];
        if (selected) {
          setRobot(selected);
          const created = initialState(selected);
          setState(created);
          stateRef.current = created;
          setMessage("Numerical engine online");
        }
      })
      .catch((reason: Error) => {
        setError(reason.message);
        setMessage("Backend offline");
      });
  }, []);

  useEffect(() => { stateRef.current = state; }, [state]);

  useEffect(() => {
    if (!robot || !state) return;
    const timer = window.setTimeout(() => {
      const started = performance.now();
      Promise.all([api.fk(robot, state.q), api.jacobian(robot, state.q)])
        .then(([fkResult, jacobianResult]) => {
          setFk(fkResult);
          setJacobian(jacobianResult);
          setMathTime(performance.now() - started);
          setError(null);
        })
        .catch((reason: Error) => setError(reason.message));
    }, 35);
    return () => window.clearTimeout(timer);
  }, [robot, state?.q]);

  const runStep = useCallback(async (stepCount = 2) => {
    const current = stateRef.current;
    if (!robot || !current || busy.current) return;
    busy.current = true;
    try {
      const result = await api.step(robot, current, gains, stepCount);
      setState(result.state);
      stateRef.current = result.state;
      setSamples((existing) => [
        ...existing,
        ...result.samples.map((sample) => ({
          time: sample.timestamp,
          value: sample.q[0] ?? 0,
          reference: sample.target_q[0] ?? 0,
        })),
      ].slice(-240));
      setTrail((existing) => [...existing, result.state.end_effector_position].slice(-300));
      setError(null);
    } catch (reason) {
      setPlaying(false);
      setError(reason instanceof Error ? reason.message : "Simulation step failed");
    } finally {
      busy.current = false;
    }
  }, [robot, gains]);

  useEffect(() => {
    if (!playing) return;
    const timer = window.setInterval(() => runStep(Math.max(1, Math.round(speed * 3))), 50);
    return () => window.clearInterval(timer);
  }, [playing, speed, runStep]);

  const reset = useCallback(() => {
    if (!robot) return;
    const created = initialState(robot);
    setPlaying(false);
    setState(created);
    stateRef.current = created;
    setSamples([]);
    setTrail([]);
    setSolveStatus("Target ready");
  }, [robot]);

  useEffect(() => {
    const onKey = (event: KeyboardEvent) => {
      if ((event.target as HTMLElement).tagName === "INPUT" || (event.target as HTMLElement).tagName === "SELECT") return;
      if (event.code === "Space") { event.preventDefault(); setPlaying((value) => !value); }
      if (event.key.toLowerCase() === "r") reset();
      if (event.key.toLowerCase() === "s") runStep(1);
      if (event.key.toLowerCase() === "a") setShowFrames((value) => !value);
      if (event.key.toLowerCase() === "t") setShowTrail((value) => !value);
    };
    window.addEventListener("keydown", onKey);
    return () => window.removeEventListener("keydown", onKey);
  }, [reset, runStep]);

  const selectRobot = (id: string) => {
    const selected = robots.find((item) => item.id === id);
    if (!selected) return;
    setRobot(selected);
    const created = initialState(selected);
    setState(created);
    stateRef.current = created;
    setSamples([]);
    setTrail([]);
    setPlaying(false);
  };

  const updateJoint = (index: number, value: number) => {
    setState((current) => {
      if (!current) return current;
      const next = { ...current, q: [...current.q], target_q: [...current.target_q] };
      next[editMode === "pose" ? "q" : "target_q"][index] = value;
      if (editMode === "pose") {
        next.qd = next.qd.map(() => 0);
        next.qdd = next.qdd.map(() => 0);
        next.error = next.target_q.map((targetValue, i) => targetValue - next.q[i]);
      }
      stateRef.current = next;
      return next;
    });
  };

  const solveIk = async () => {
    if (!robot || !state) return;
    setSolveStatus("Solving DLS IK…");
    try {
      const result = await api.ik(robot, state.q, target);
      setSolveStatus(`${result.converged ? "Target reached" : "Unreachable"} · ${result.iterations} iter · ${format(result.position_error * 1000, 2)} mm`);
      setState((current) => {
        if (!current) return current;
        const next = { ...current, q: result.q, target_q: result.q, qd: result.q.map(() => 0), qdd: result.q.map(() => 0) };
        stateRef.current = next;
        return next;
      });
    } catch (reason) {
      setSolveStatus("IK failed");
      setError(reason instanceof Error ? reason.message : "IK request failed");
    }
  };

  if (!robot || !state) {
    return <main className="loading"><Hexagon size={42} /><h1>RoboForge Studio</h1><p>{message}</p>{error && <code>{error}</code>}</main>;
  }

  const displayedJointValues = editMode === "pose" ? state.q : state.target_q;
  const positionError = fk ? Math.sqrt(fk.pose.position.reduce((sum, value, index) => sum + (target[index] - value) ** 2, 0)) : 0;
  const frames = fk?.frames ?? [identity];

  return (
    <div className="app-shell">
      <header className="topbar">
        <div className="brand"><Hexagon size={21} strokeWidth={2.4} /><div><strong>ROBOFORGE</strong><span>STUDIO</span></div></div>
        <div className="project-select">
          <span>ROBOT MODEL</span>
          <select value={robot.id} onChange={(event) => selectRobot(event.target.value)}>
            {robots.map((item) => <option key={item.id} value={item.id}>{item.name}</option>)}
          </select><ChevronDown size={13} />
        </div>
        <div className="status-cluster">
          <span className="online-dot" /> <span>{message}</span>
          <button title="Save experiment"><Save size={16} /></button>
          <button title="Settings"><Settings size={17} /></button>
          <div className="avatar">SS</div>
        </div>
      </header>

      <aside className="side-nav">
        <NavIcon icon={<Box />} label="Robot" active />
        <NavIcon icon={<Crosshair />} label="Kinematics" />
        <NavIcon icon={<GitBranch />} label="Jacobian" />
        <NavIcon icon={<CircleGauge />} label="Control" />
        <NavIcon icon={<Waypoints />} label="Trajectory" />
        <NavIcon icon={<Activity />} label="Plots" />
        <NavIcon icon={<Database />} label="Research" />
      </aside>

      <aside className="robot-tree panel">
        <div className="panel-title"><span>SCENE / ROBOT TREE</span><button>+</button></div>
        <div className="tree-root"><ChevronDown size={14} /><Hexagon size={15} /> <strong>{robot.name}</strong></div>
        <div className="tree-item"><span className="tree-line" /><Box size={14} /> Base</div>
        {robot.joints.map((joint, index) => (
          <div className="tree-item" key={joint.id}><span className="tree-line" /><Focus size={14} /> {joint.name}<em>J{index + 1}</em></div>
        ))}
        <div className="tree-item"><span className="tree-line" /><Target size={14} /> Tool center point</div>
        <div className="tree-summary">
          <span><b>{robot.joints.length}</b> DOF</span><span><b>{robot.payload_kg}</b> kg payload</span>
        </div>
        <div className="tree-section">ENVIRONMENT</div>
        <div className="tree-item"><Grid3X3 size={14} /> World grid</div>
        <div className="tree-item"><Target size={14} /> IK target <em className="amber">LIVE</em></div>
      </aside>

      <section className="workspace">
        <div className="viewport-toolbar">
          <div><span className="workspace-label">ROBOT STUDIO</span><b>{robot.name}</b></div>
          <div className="view-actions">
            <button className={showFrames ? "active" : ""} onClick={() => setShowFrames(!showFrames)}><Crosshair size={14} /> Frames</button>
            <button className={showTrail ? "active" : ""} onClick={() => setShowTrail(!showTrail)}><Waypoints size={14} /> Trail</button>
            <button><Grid3X3 size={14} /> Grid</button>
          </div>
        </div>
        <RobotViewport frames={frames} target={target} showFrames={showFrames} showTrail={showTrail} trail={trail} />
        <div className="viewport-stats">
          <span><i className="green" /> SIM {playing ? "RUNNING" : "PAUSED"}</span>
          <span>t = {format(state.timestamp, 2)} s</span>
          <span>dt = 0.010 s</span>
          <span>Math {format(mathTime, 1)} ms</span>
        </div>
      </section>

      <aside className="inspector panel">
        <div className="panel-title"><span>PARAMETER INSPECTOR</span><SlidersHorizontal size={15} /></div>
        <div className="tab-switch"><button className={editMode === "pose" ? "active" : ""} onClick={() => setEditMode("pose")}>Joint pose</button><button className={editMode === "target" ? "active" : ""} onClick={() => setEditMode("target")}>Control target</button></div>
        <div className="section-title">JOINT STATE <span>rad</span></div>
        <div className="joint-list">
          {robot.joints.map((joint, index) => {
            const value = displayedJointValues[index];
            return (
              <label className="joint-control" key={joint.id}>
                <span><b>J{index + 1}</b>{joint.name}<output>{format(value, 3)}</output></span>
                <input type="range" min={joint.limit.lower} max={joint.limit.upper} step="0.001" value={value} onChange={(event) => updateJoint(index, Number(event.target.value))} />
                <small>{format(joint.limit.lower, 2)} <i style={{ left: `${((value - joint.limit.lower) / (joint.limit.upper - joint.limit.lower)) * 100}%` }} /> {format(joint.limit.upper, 2)}</small>
              </label>
            );
          })}
        </div>
        <div className="section-title">IK TARGET <span>metres</span></div>
        <div className="xyz-inputs">
          {(["X", "Y", "Z"] as const).map((axis, index) => (
            <label key={axis}><span>{axis}</span><input type="number" step="0.05" value={target[index]} onChange={(event) => setTarget(target.map((value, i) => i === index ? Number(event.target.value) : value))} /></label>
          ))}
        </div>
        <button className="solve-button" onClick={solveIk}><Target size={15} /> Solve inverse kinematics</button>
        <p className={`solver-status ${solveStatus.startsWith("Unreachable") ? "warning" : ""}`}><span />{solveStatus}</p>
        <div className="section-title">PID GAINS <span>live tuning</span></div>
        <div className="gain-row">
          {(["kp", "ki", "kd"] as const).map((gain) => <label key={gain}>{gain.toUpperCase()}<input type="number" value={gains[gain]} onChange={(event) => setGains({ ...gains, [gain]: Number(event.target.value) })} /></label>)}
        </div>
      </aside>

      <section className="playback">
        <div className="playback-controls">
          <button className="transport secondary" onClick={reset} title="Reset (R)"><RotateCcw size={16} /></button>
          <button className="transport primary" onClick={() => setPlaying(!playing)} title="Play/Pause (Space)">{playing ? <Pause size={19} /> : <Play size={19} />}</button>
          <button className="transport secondary" onClick={() => runStep(1)} title="Single step (S)"><SkipForward size={16} /></button>
          <div className="speed"><span>SPEED</span>{[0.5, 1, 2, 5].map((item) => <button key={item} className={speed === item ? "active" : ""} onClick={() => setSpeed(item)}>{item}×</button>)}</div>
        </div>
        <div className="time-readout"><span>SIMULATION TIME</span><strong>{format(state.timestamp, 3)}<em>s</em></strong></div>
        <div className="error-readout"><span>POSITION ERROR</span><strong className={positionError < 0.01 ? "ok" : ""}>{format(positionError * 1000, 1)}<em>mm</em></strong></div>
      </section>

      <section className="bottom-panel panel">
        <div className="bottom-tabs">
          <button className={bottomTab === "telemetry" ? "active" : ""} onClick={() => setBottomTab("telemetry")}>Live telemetry</button>
          <button className={bottomTab === "matrix" ? "active" : ""} onClick={() => setBottomTab("matrix")}>Matrices</button>
          <button className={bottomTab === "log" ? "active" : ""} onClick={() => setBottomTab("log")}>Performance / logs</button>
          <span className="sample-count">{samples.length} SAMPLES</span>
        </div>
        <div className="bottom-content">
          {bottomTab === "telemetry" && <TelemetryChart samples={samples} />}
          {bottomTab === "matrix" && (
            <div className="matrix-grid">
              <div><h3>T₀ⁿ — END-EFFECTOR TRANSFORM</h3><MatrixView matrix={fk?.pose.transform ?? identity} /></div>
              <div><h3>J(q) — GEOMETRIC JACOBIAN</h3><MatrixView matrix={jacobian?.matrix ?? [[0]]} /></div>
              <div className="metric-stack"><h3>NUMERICAL HEALTH</h3><p><span>Rank</span><b>{jacobian?.rank ?? "—"}</b></p><p><span>Condition number</span><b>{jacobian?.condition_number ? format(jacobian.condition_number, 2) : "—"}</b></p><p><span>Manipulability</span><b>{format(jacobian?.manipulability ?? 0, 4)}</b></p></div>
            </div>
          )}
          {bottomTab === "log" && (
            <div className="log-view">
              <p><time>{format(state.timestamp, 3)}</time><span className="info">INFO</span> Deterministic simulation state synchronized</p>
              <p><time>{format(state.timestamp, 3)}</time><span className={jacobian?.near_singular ? "warn" : "success"}>{jacobian?.near_singular ? "WARN" : "PASS"}</span> Jacobian {jacobian?.near_singular ? "is near a singular configuration" : "numerical rank is healthy"}</p>
              <p><time>{format(state.timestamp, 3)}</time><span className="success">PASS</span> FK transform finite · position [{fk?.pose.position.map((value) => format(value, 3)).join(", ")}]</p>
              {error && <p><time>{format(state.timestamp, 3)}</time><span className="error">ERROR</span> {error}</p>}
            </div>
          )}
        </div>
      </section>

      {error && <div className="error-toast"><TriangleAlert size={17} /><div><b>Engineering warning</b><span>{error}</span></div><button onClick={() => setError(null)}>×</button></div>}
    </div>
  );
}

function NavIcon({ icon, label, active = false }: { icon: React.ReactElement; label: string; active?: boolean }) {
  return <button className={active ? "active" : ""} title={label}>{icon}<span>{label}</span></button>;
}
