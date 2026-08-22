export type JointType = "revolute" | "prismatic" | "fixed" | "continuous";

export interface JointLimit {
  lower: number;
  upper: number;
  velocity: number;
  acceleration: number;
  torque: number;
}

export interface Joint {
  id: string;
  name: string;
  type: JointType;
  axis: [number, number, number];
  dh: { a: number; alpha: number; d: number; theta_offset: number };
  limit: JointLimit;
  damping: number;
  friction: number;
  mass: number;
}

export interface RobotModel {
  id: string;
  name: string;
  description: string;
  convention: "standard" | "modified";
  joints: Joint[];
  educational: boolean;
  payload_kg: number;
}

export interface Pose {
  position: number[];
  rotation_matrix: number[][];
  quaternion_xyzw: number[];
  rpy: number[];
  transform: number[][];
}

export interface FKResponse {
  pose: Pose;
  frames: number[][][];
  joint_transforms: number[][][];
}

export interface JacobianResponse {
  matrix: number[][];
  singular_values: number[];
  rank: number;
  condition_number: number | null;
  manipulability: number;
  near_singular: boolean;
}

export interface SimulationState {
  timestamp: number;
  q: number[];
  qd: number[];
  qdd: number[];
  torque: number[];
  target_q: number[];
  error: number[];
  end_effector_position: number[];
  energy: number;
  controller_integral: number[];
}

export interface Sample {
  time: number;
  value: number;
  reference: number;
}
