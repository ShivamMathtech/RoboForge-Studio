import { Grid, Line, OrbitControls, PerspectiveCamera } from "@react-three/drei";
import { Canvas } from "@react-three/fiber";
import { useMemo } from "react";
import * as THREE from "three";

interface Props {
  frames: number[][][];
  target: number[];
  showFrames: boolean;
  showTrail: boolean;
  trail: number[][];
}

function LinkBody({ start, end, index }: { start: number[]; end: number[]; index: number }) {
  const { midpoint, quaternion, length } = useMemo(() => {
    const a = new THREE.Vector3(...(start as [number, number, number]));
    const b = new THREE.Vector3(...(end as [number, number, number]));
    const direction = b.clone().sub(a);
    const distance = Math.max(direction.length(), 0.001);
    return {
      midpoint: a.clone().add(b).multiplyScalar(0.5),
      quaternion: new THREE.Quaternion().setFromUnitVectors(
        new THREE.Vector3(0, 1, 0),
        direction.normalize(),
      ),
      length: distance,
    };
  }, [start, end]);

  return (
    <group>
      <mesh position={midpoint} quaternion={quaternion} castShadow>
        <cylinderGeometry args={[0.055, 0.055, length, 20]} />
        <meshStandardMaterial color={index % 2 ? "#2d8cff" : "#20d6a1"} metalness={0.55} roughness={0.28} />
      </mesh>
      <mesh position={end as [number, number, number]} castShadow>
        <sphereGeometry args={[0.085, 24, 16]} />
        <meshStandardMaterial color="#d9e3ef" metalness={0.65} roughness={0.22} />
      </mesh>
    </group>
  );
}

function CoordinateFrame({ matrix, scale = 0.18 }: { matrix: number[][]; scale?: number }) {
  const position = [matrix[0][3], matrix[1][3], matrix[2][3]] as [number, number, number];
  const rotation = new THREE.Matrix4().set(
    matrix[0][0], matrix[0][1], matrix[0][2], 0,
    matrix[1][0], matrix[1][1], matrix[1][2], 0,
    matrix[2][0], matrix[2][1], matrix[2][2], 0,
    0, 0, 0, 1,
  );
  const quaternion = new THREE.Quaternion().setFromRotationMatrix(rotation);
  return (
    <group position={position} quaternion={quaternion}>
      <axesHelper args={[scale]} />
    </group>
  );
}

function Scene({ frames, target, showFrames, showTrail, trail }: Props) {
  const points = frames.map((frame) => [frame[0][3], frame[1][3], frame[2][3]]);
  const end = points[points.length - 1] ?? [0, 0, 0];
  return (
    <>
      <color attach="background" args={["#070c12"]} />
      <fog attach="fog" args={["#070c12", 5, 13]} />
      <PerspectiveCamera makeDefault position={[3.1, 2.3, 3.4]} fov={42} />
      <OrbitControls makeDefault enableDamping dampingFactor={0.08} target={[0.35, 0.3, 0]} />
      <ambientLight intensity={0.7} />
      <directionalLight position={[4, 6, 3]} intensity={3.0} castShadow />
      <pointLight position={[-3, 2, -2]} intensity={2.0} color="#1e78ff" />

      <Grid
        args={[12, 12]}
        position={[0, -0.002, 0]}
        cellSize={0.25}
        cellThickness={0.5}
        cellColor="#17334a"
        sectionSize={1}
        sectionThickness={1}
        sectionColor="#245576"
        fadeDistance={8}
        infiniteGrid
      />
      <mesh position={[0, 0.04, 0]} receiveShadow>
        <cylinderGeometry args={[0.28, 0.34, 0.08, 40]} />
        <meshStandardMaterial color="#1b2732" metalness={0.7} roughness={0.25} />
      </mesh>

      {points.slice(1).map((point, index) => (
        <LinkBody key={index} start={points[index]} end={point} index={index} />
      ))}
      {showFrames && frames.map((frame, index) => <CoordinateFrame key={index} matrix={frame} />)}

      <mesh position={target as [number, number, number]}>
        <octahedronGeometry args={[0.11, 0]} />
        <meshStandardMaterial color="#ffb547" emissive="#ff8a00" emissiveIntensity={0.45} />
      </mesh>
      <Line
        points={[end as [number, number, number], target as [number, number, number]]}
        color="#ffb547"
        lineWidth={1}
        dashed
        dashSize={0.04}
        gapSize={0.035}
      />
      {showTrail && trail.length > 1 && (
        <Line points={trail as [number, number, number][]} color="#23e0aa" lineWidth={2} />
      )}
    </>
  );
}

export function RobotViewport(props: Props) {
  return (
    <div className="viewport-canvas">
      <Canvas shadows dpr={[1, 1.7]} gl={{ antialias: true }}>
        <Scene {...props} />
      </Canvas>
      <div className="axis-legend" aria-label="Coordinate axis legend">
        <span className="axis-x">X</span><span className="axis-y">Y</span><span className="axis-z">Z</span>
      </div>
    </div>
  );
}
