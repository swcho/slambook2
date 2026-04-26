// Minimal r3f / drei viewer for SLAM-style scenes.
// Phase D — used by Step 6 to show the initial KeyFrame frustum + the
// triangulated MapPoint cloud. Will be reused by Step 13 (full pipeline)
// for the running trajectory.

import { useMemo } from 'react';
import { Canvas } from '@react-three/fiber';
import { OrbitControls, Grid, Line } from '@react-three/drei';
import * as THREE from 'three';

export interface CameraFrustum {
  /** 4×4 row-major world→camera transform — frustum drawn at its inverse. */
  worldFromCamera: number[];
  /** Frustum side length in metres. */
  scale: number;
  /** RGB tuple in [0..1]. */
  color: [number, number, number];
  label?: string;
}

export interface PointCloudInput {
  /** Float32Array of XYZ triplets (length = 3 * N). */
  positions: Float32Array;
  /** Optional per-point color (Float32Array length = 3 * N). */
  colors?: Float32Array;
  /** Render size (metres at depth 1, with sizeAttenuation). */
  size?: number;
}

interface Scene3DProps {
  width: number;
  height: number;
  frustums: CameraFrustum[];
  pointCloud: PointCloudInput | null;
  /** Initial orbit camera distance (metres). */
  initialDistance?: number;
}

export function Scene3D({ width, height, frustums, pointCloud, initialDistance = 25 }: Scene3DProps) {
  return (
    <div
      style={{
        width,
        height,
        border: '1px solid #333',
        borderRadius: 4,
        background: '#0d1117',
        overflow: 'hidden',
      }}
    >
      <Canvas
        camera={{ position: [initialDistance, initialDistance * 0.6, initialDistance], fov: 50, near: 0.1, far: 1000 }}
        dpr={[1, 2]}
        gl={{ antialias: true }}
      >
        <color attach="background" args={['#0d1117']} />
        <Grid
          args={[40, 40]}
          position={[0, 0, 0]}
          cellColor="#222"
          sectionColor="#444"
          sectionSize={5}
          cellSize={1}
          fadeDistance={50}
          fadeStrength={1}
          infiniteGrid={false}
        />
        <axesHelper args={[2]} />
        {frustums.map((f, i) => (
          <FrustumLines key={i} frustum={f} />
        ))}
        {pointCloud && <PointCloud {...pointCloud} />}
        <OrbitControls enableDamping makeDefault target={[0, 0, 5]} />
      </Canvas>
    </div>
  );
}

function FrustumLines({ frustum }: { frustum: CameraFrustum }) {
  const points = useMemo(() => buildFrustumPoints(frustum), [frustum]);
  return (
    <Line
      points={points}
      color={new THREE.Color(frustum.color[0], frustum.color[1], frustum.color[2])}
      lineWidth={1.5}
      segments
    />
  );
}

function PointCloud({ positions, colors, size = 0.15 }: PointCloudInput) {
  const geo = useMemo(() => {
    const g = new THREE.BufferGeometry();
    g.setAttribute('position', new THREE.BufferAttribute(positions, 3));
    if (colors) g.setAttribute('color', new THREE.BufferAttribute(colors, 3));
    return g;
  }, [positions, colors]);
  return (
    <points geometry={geo}>
      <pointsMaterial
        size={size}
        sizeAttenuation
        vertexColors={!!colors}
        color={colors ? '#ffffff' : '#66ddaa'}
      />
    </points>
  );
}

function buildFrustumPoints(f: CameraFrustum): [number, number, number][] {
  const s = f.scale;
  const half = s * 0.45;
  const apex: [number, number, number] = [0, 0, 0];
  const tl: [number, number, number] = [-half, -half * 0.6, s];
  const tr: [number, number, number] = [half, -half * 0.6, s];
  const br: [number, number, number] = [half, half * 0.6, s];
  const bl: [number, number, number] = [-half, half * 0.6, s];
  // 8 segments: 4 from apex + 4 around the image plane.
  const segs: [number, number, number][] = [
    apex, tl,
    apex, tr,
    apex, br,
    apex, bl,
    tl, tr,
    tr, br,
    br, bl,
    bl, tl,
  ];
  // Apply world←camera (4×4 row-major) transform.
  const m = f.worldFromCamera;
  if (m.length !== 16) throw new Error('worldFromCamera must be 4×4 row-major');
  const M = new THREE.Matrix4().set(
    m[0], m[1], m[2], m[3],
    m[4], m[5], m[6], m[7],
    m[8], m[9], m[10], m[11],
    m[12], m[13], m[14], m[15],
  );
  const v = new THREE.Vector3();
  return segs.map((p) => {
    v.set(p[0], p[1], p[2]).applyMatrix4(M);
    return [v.x, v.y, v.z] as [number, number, number];
  });
}
