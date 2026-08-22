# Mathematics and numerical safeguards

## Denavit–Hartenberg representation

For standard DH parameters $(a_i,\alpha_i,d_i,\theta_i)$,

$$
{}^{i-1}T_i=
\begin{bmatrix}
c_\theta & -s_\theta c_\alpha & s_\theta s_\alpha & a c_\theta\\
s_\theta & c_\theta c_\alpha & -c_\theta s_\alpha & a s_\theta\\
0 & s_\alpha & c_\alpha & d\\
0 & 0 & 0 & 1
\end{bmatrix}.
$$

For a revolute joint, the state value is added to $\theta_i$. For a prismatic joint, it is added to $d_i$. Fixed joints contribute a constant transform.

The engine also implements the Craig modified-DH convention. A robot declares its convention; conventions are never mixed silently.

## Forward kinematics

The engine begins with the identity world frame and left-multiplies each relative transform:

$$
{}^0T_i={}^0T_{i-1}{}^{i-1}T_i.
$$

It returns all frames, not only the end-effector result. Orientation is returned as a rotation matrix, XYZW quaternion, and XYZ roll/pitch/yaw angles.

Safeguards:

- Exact joint-vector dimension check
- Finite-value check before calculation
- Finite transform check after every multiplication
- Validated joint limits and positive velocity/acceleration/torque limits

## Geometric Jacobian

For a revolute joint,

$$
J_i=
\begin{bmatrix}
z_{i-1}\times(o_n-o_{i-1})\\
z_{i-1}
\end{bmatrix}.
$$

For a prismatic joint,

$$
J_i=
\begin{bmatrix}
z_{i-1}\\
0
\end{bmatrix}.
$$

The implementation reports singular values, numerical rank, condition number, and the product of the required singular values as a manipulability measure. A rank deficiency or a smallest singular value below the tolerance produces a near-singularity warning.

The automated suite compares the analytical translational Jacobian to a finite-difference derivative of FK.

## Inverse kinematics

This release solves end-effector position. The residual is

$$
e=x_d-f(q).
$$

Each iteration uses damped least squares:

$$
\Delta q=J_v^T(J_vJ_v^T+\lambda^2I)^{-1}e.
$$

This form is stable around many singular configurations because the damping term prevents inversion of a zero singular value. The solver also:

- Limits the norm of each update
- Clamps non-continuous joints after every update
- Stops at a configurable residual tolerance
- Stops after a configurable iteration limit
- Detects non-finite states
- Returns convergence status, iterations, final residual, and sampled solver path

Orientation-constrained and analytical IK are planned extensions; the API already accepts an optional target orientation so the contract can evolve compatibly.

## Controller

For target $q_d$, the error is $e=q_d-q$. The implemented controller is

$$
\tau=K_pe+K_i\int e\,dt-K_d\dot q.
$$

The integral state is returned inside `SimulationState`, so repeated stateless REST calls still preserve integral history. Output saturation respects the controller’s global limit and every joint’s torque limit. Conditional integration prevents simple saturation windup.

## Educational plant model

The included plant uses one effective inertia per joint:

$$
M_i=m_i\max(a_i^2,0.04).
$$

Acceleration is

$$
\ddot q=M^{-1}(\tau-D\dot q).
$$

This produces a real controller-dependent response, but it is not presented as a full coupled robot dynamics model. The physics abstraction exists so recursive Newton–Euler, articulated-body dynamics, contact, and external engines can replace it.

## Integrators

### Explicit Euler

$$
q_{k+1}=q_k+\dot q_k\Delta t,
\qquad
\dot q_{k+1}=\dot q_k+\ddot q_k\Delta t.
$$

### Semi-implicit Euler

$$
\dot q_{k+1}=\dot q_k+\ddot q_k\Delta t,
\qquad
q_{k+1}=q_k+\dot q_{k+1}\Delta t.
$$

### RK4

The state $y=[q,\dot q]^T$ is advanced with the classical four-slope weighted update. All methods enforce velocity, position, and torque limits and reject NaN/Inf.

## Trajectories

With normalized time $s=t/T$, the quintic rest-to-rest blend is

$$
h(s)=10s^3-15s^4+6s^5.
$$

Position is

$$
q(t)=q_0+h(t/T)(q_f-q_0).
$$

Velocity, acceleration, and jerk are computed analytically with the correct powers of $T$. Tests verify endpoint position and zero endpoint velocity/acceleration.

