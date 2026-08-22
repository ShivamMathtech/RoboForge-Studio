# Development roadmap

The original product vision is larger than a single safe release. The table below separates delivered functionality from the next engineering phases.

| Capability | Status in v1.0 | Next step |
| --- | --- | --- |
| Application shell and 3D viewport | Implemented | Add resizable/dockable panes |
| Robot presets and DH model | Implemented | Visual drag-and-drop model builder |
| FK and coordinate frames | Implemented | Symbolic matrix view with SymPy |
| DLS position IK | Implemented | Orientation task, analytical solvers, null-space objectives |
| Spatial Jacobian and metrics | Implemented | Interactive singularity map |
| PID control | Implemented | PD, computed torque, Cartesian, impedance |
| Deterministic loop and three integrators | Implemented | Coupled rigid-body dynamics |
| Polynomial trajectories | Implemented in API | Timeline/waypoint UI, trapezoidal and S-curve profiles |
| Controller, physics, sensor, planner contracts | Implemented | Production plugins |
| Collision/contact physics | Interface only | Broad/narrow phase and external engine adapter |
| Sensors | Noise model/interface only | Encoder, IMU, F/T, LiDAR, RGB-D outputs |
| Motion planning | Interface only | RRT, RRT*, PRM, A*, obstacle UI |
| Experiments/research sweeps | Planned | Persistent experiment schema and batch workers |
| Import/export | Planned | URDF first, then mesh validation and SDF |
| ROS 2 | Planned | Optional adapter for JointState, TF, sensors, actions |
| Database/authentication | Planned | PostgreSQL, migrations, JWT/OIDC, role/ownership checks |
| Multi-robot simulation | Planned | Independent state/controller instances and collision groups |

## Recommended implementation order

### Phase 3 — Control and trajectory UI

1. Add PD and computed-torque controllers through the common controller interface.
2. Add an endpoint for controller metadata and parameter schemas.
3. Build the trajectory editor around the existing generator.
4. Add reference-vs-actual plots and standard step-response metrics.

### Phase 4 — Coupled dynamics and collision

1. Define link inertial tensors and centers of mass.
2. Implement or integrate a tested rigid-body backend.
3. Add collision shapes and collision event contracts.
4. Verify energy and gravity-compensation results against known models.

### Phase 5 — Persistence and experiments

1. Add PostgreSQL with Alembic migrations.
2. Persist robots, controllers, trajectories, experiments, and runs.
3. Add authentication and role-based ownership.
4. Run parameter sweeps in background workers with reproducibility metadata.

### Phase 6 — Sensors, planning, and import

1. Implement encoder, IMU, and force/torque sensors first.
2. Add environment obstacles and RRT/PRM planning.
3. Import validated URDF models into the neutral `RobotModel` representation.
4. Add ROS 2 as an optional adapter, never a local-simulation requirement.

## Definition of done for every new algorithm

- Validated input/output contract
- Numerical implementation outside React
- UI that exposes real inputs and results
- Visualization based on computed values
- Finite-value and dimension safeguards
- Descriptive convergence/limit errors
- At least one known-reference test
- At least one numerical-regression or finite-difference test when applicable
- Documentation of assumptions and units

