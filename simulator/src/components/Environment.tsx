import React, { useRef, useMemo } from 'react';
import { useFrame } from '@react-three/fiber';
import { GradientSky } from './GradientSky';
import {
  AdditiveBlending,
  BufferAttribute,
  BufferGeometry,
  Color,
  Line as ThreeLine,
  LineBasicMaterial,
  Mesh,
  Vector3,
} from 'three';
import { CHECKPOINTS, GATE_RADIUS } from '../utils/constants';

const TRAIL_POINTS = 90;

/** Shared by the sky gradient and the key light, so the shading agrees with
 *  where the sun is actually drawn. */
const SUN_POSITION: [number, number, number] = [-150, 34, -240];

/**
 * Face each gate along the track. A torus's hole points down +Z, so aligning
 * +Z with the direction from the previous gate to the next one leaves the drone
 * flying through the hoop rather than sliding past its edge.
 */
const GATE_HEADINGS: number[] = CHECKPOINTS.map((_, i) => {
  const n = CHECKPOINTS.length;
  const previous = CHECKPOINTS[(i - 1 + n) % n];
  const next = CHECKPOINTS[(i + 1) % n];
  return Math.atan2(next.x - previous.x, next.z - previous.z);
});

/**
 * Ribbon of recent positions behind the drone. Written straight into a
 * preallocated buffer from the render loop, so it costs no React renders.
 */
const FlightTrail: React.FC<{ trailRef: React.MutableRefObject<Vector3[]> }> = ({ trailRef }) => {
  const line = useMemo(() => {
    const geometry = new BufferGeometry();
    geometry.setAttribute('position', new BufferAttribute(new Float32Array(TRAIL_POINTS * 3), 3));
    geometry.setDrawRange(0, 0);
    const material = new LineBasicMaterial({
      color: new Color('#67e8f9'),
      transparent: true,
      opacity: 0.9,
      blending: AdditiveBlending,
      depthWrite: false,
    });
    const l = new ThreeLine(geometry, material);
    l.frustumCulled = false;
    return l;
  }, []);

  useFrame(() => {
    const points = trailRef.current;
    if (points.length < 2) return;

    const attribute = line.geometry.getAttribute('position') as BufferAttribute;
    const array = attribute.array as Float32Array;
    const count = Math.min(points.length, TRAIL_POINTS);
    const offset = points.length - count;

    for (let i = 0; i < count; i++) {
      const p = points[offset + i];
      array[i * 3] = p.x;
      array[i * 3 + 1] = p.y;
      array[i * 3 + 2] = p.z;
    }

    attribute.needsUpdate = true;
    line.geometry.setDrawRange(0, count);
  });

  return <primitive object={line} />;
};

interface GateProps {
  position: Vector3;
  /** Heading the gate faces, so the drone flies through the hole not past it. */
  yaw: number;
  state: 'passed' | 'current' | 'upcoming';
}

const Gate: React.FC<GateProps> = ({ position, yaw, state }) => {
  const ringRef = useRef<Mesh>(null);
  const glowRef = useRef<Mesh>(null);

  const palette = {
    // Emissive stays modest on purpose: push it past ~2 and the material
    // saturates to white and the colour coding stops reading.
    passed: { color: '#10b981', emissive: 0.9 },
    current: { color: '#22d3ee', emissive: 1.7 },
    upcoming: { color: '#a855f7', emissive: 0.8 },
  }[state];

  useFrame((frame) => {
    const t = frame.clock.elapsedTime;
    if (state === 'current') {
      const pulse = 1 + Math.sin(t * 4) * 0.045;
      if (ringRef.current) ringRef.current.scale.setScalar(pulse);
      if (glowRef.current) {
        glowRef.current.scale.setScalar(pulse * 1.06);
        (glowRef.current.material as any).opacity = 0.16 + Math.sin(t * 4) * 0.07;
      }
    }
  });

  return (
    <group position={position} rotation={[0, yaw, 0]}>
      {/* The gate itself, standing upright so the drone flies through it. */}
      <mesh ref={ringRef}>
        <torusGeometry args={[GATE_RADIUS, 0.22, 20, 64]} />
        <meshStandardMaterial
          color={palette.color}
          emissive={palette.color}
          emissiveIntensity={palette.emissive}
          roughness={0.25}
          metalness={0.4}
        />
      </mesh>

      {/* Soft disc so the gate reads as lit rather than as a wire hoop. */}
      <mesh ref={glowRef}>
        <circleGeometry args={[GATE_RADIUS, 48]} />
        <meshBasicMaterial
          color={palette.color}
          transparent
          opacity={state === 'current' ? 0.18 : 0.05}
          depthWrite={false}
        />
      </mesh>

      {/* Posts, so a gate reads as a structure rather than a floating hoop. */}
      {state !== 'passed' && (
        <>
          <mesh position={[GATE_RADIUS * 0.94, -GATE_RADIUS * 0.5, 0]}>
            <cylinderGeometry args={[0.13, 0.2, GATE_RADIUS, 6]} />
            <meshStandardMaterial color="#1e293b" roughness={0.8} />
          </mesh>
          <mesh position={[-GATE_RADIUS * 0.94, -GATE_RADIUS * 0.5, 0]}>
            <cylinderGeometry args={[0.13, 0.2, GATE_RADIUS, 6]} />
            <meshStandardMaterial color="#1e293b" roughness={0.8} />
          </mesh>
        </>
      )}
    </group>
  );
};

/** Trees, laid out once and never recomputed. */
const Forest: React.FC = React.memo(() => {
  const trees = useMemo(() => {
    const result: { x: number; z: number; scale: number; tone: string }[] = [];
    const tones = ['#14532d', '#166534', '#1a7f3c', '#0f3d22'];
    for (let i = 0; i < 150; i++) {
      // Deterministic scatter: a golden-angle spiral, so the layout is stable
      // across reloads and no two runs look different.
      const angle = i * 2.39996;
      const radius = 22 + Math.sqrt(i) * 9.5;
      const x = Math.cos(angle) * radius;
      const z = Math.sin(angle) * radius;
      result.push({
        x,
        z,
        scale: 0.7 + ((i * 37) % 100) / 140,
        tone: tones[i % tones.length],
      });
    }
    return result;
  }, []);

  return (
    <>
      {trees.map((tree, i) => (
        <group key={`tree-${i}`} position={[tree.x, 0, tree.z]} scale={tree.scale}>
          <mesh position={[0, 1.8, 0]} castShadow>
            <cylinderGeometry args={[0.35, 0.5, 3.6, 6]} />
            <meshStandardMaterial color="#2d1b12" roughness={1} />
          </mesh>
          <mesh position={[0, 6.2, 0]} castShadow>
            <coneGeometry args={[2.6, 7.5, 7]} />
            <meshStandardMaterial color={tree.tone} roughness={0.9} />
          </mesh>
          <mesh position={[0, 9.4, 0]} castShadow>
            <coneGeometry args={[1.7, 4.5, 7]} />
            <meshStandardMaterial color={tree.tone} roughness={0.9} />
          </mesh>
        </group>
      ))}
    </>
  );
});
Forest.displayName = 'Forest';

/** Low hills on the horizon so the world does not end in a flat line. */
const Horizon: React.FC = React.memo(() => {
  const hills = useMemo(() => {
    const result: { x: number; z: number; r: number; h: number }[] = [];
    for (let i = 0; i < 14; i++) {
      const angle = (i / 14) * Math.PI * 2 + 0.4;
      const radius = 260 + ((i * 53) % 90);
      result.push({
        x: Math.cos(angle) * radius,
        z: Math.sin(angle) * radius,
        r: 55 + ((i * 29) % 45),
        h: 45 + ((i * 41) % 60),
      });
    }
    return result;
  }, []);

  return (
    <>
      {hills.map((hill, i) => (
        <mesh key={`hill-${i}`} position={[hill.x, -2, hill.z]}>
          <coneGeometry args={[hill.r, hill.h, 6]} />
          <meshStandardMaterial color="#0f2a2e" roughness={1} flatShading />
        </mesh>
      ))}
    </>
  );
});
Horizon.displayName = 'Horizon';

interface Props {
  currentGate: number;
  trailRef: React.MutableRefObject<Vector3[]>;
}

export const WorldEnvironment: React.FC<Props> = ({ currentGate, trailRef }) => {
  return (
    <>
      <GradientSky sunPosition={SUN_POSITION} />
      <fog attach="fog" args={['#1b3347', 110, 520]} />

      <ambientLight intensity={0.4} color="#7fb4d6" />
      <hemisphereLight args={['#5aa9d6', '#04211c', 0.65]} />
      <directionalLight
        position={[-75, 42, -110]}
        intensity={2.4}
        color="#ffc178"
        castShadow
        shadow-mapSize={[2048, 2048]}
        shadow-camera-left={-90}
        shadow-camera-right={90}
        shadow-camera-top={90}
        shadow-camera-bottom={-90}
        shadow-camera-far={260}
      />
      {/* Cool fill from the opposite side so shadowed faces are not black. */}
      <directionalLight position={[60, 25, 60]} intensity={0.5} color="#67e8f9" />

      {/* Ground */}
      <mesh rotation={[-Math.PI / 2, 0, 0]} position={[0, -0.02, 0]} receiveShadow>
        <planeGeometry args={[1400, 1400]} />
        <meshStandardMaterial color="#14574a" roughness={0.95} />
      </mesh>
      <gridHelper args={[400, 80, '#17514c', '#0e3832']} position={[0, 0.02, 0]} />

      <Horizon />
      <Forest />

      {CHECKPOINTS.map((pos, i) => (
        <Gate
          key={i}
          position={pos}
          yaw={GATE_HEADINGS[i]}
          state={i === currentGate ? 'current' : i < currentGate ? 'passed' : 'upcoming'}
        />
      ))}

      <FlightTrail trailRef={trailRef} />
    </>
  );
};
