import React, { useRef } from 'react';
import { useFrame } from '@react-three/fiber';
import { Group, Mesh, MeshBasicMaterial } from 'three';
import { PHYSICS } from '../utils/constants';

interface Props {
  /**
   * Motor RPMs, read through a ref so prop speed changes never trigger a React
   * render. At 60 fps this is the difference between a smooth sim and a stuttery one.
   */
  motorSpeedsRef: React.MutableRefObject<[number, number, number, number]>;
}

const ARM_COLOR_FRONT = '#22d3ee';
const ARM_COLOR_REAR = '#f43f5e';

/** Centre-to-motor distance for an X frame with arms at (+-L, +-L). */
const ARM_SPAN = PHYSICS.armLength * Math.SQRT2;

/**
 * Yaw of each boom. A group rotated by theta about Y points its -Z axis at
 * (-sin theta, 0, -cos theta), which is how these angles were picked.
 */
const ARMS = [
  { key: 'fl', yaw: -Math.PI / 4, color: ARM_COLOR_FRONT },
  { key: 'fr', yaw: Math.PI / 4, color: ARM_COLOR_FRONT },
  { key: 'rl', yaw: (-3 * Math.PI) / 4, color: ARM_COLOR_REAR },
  { key: 'rr', yaw: (3 * Math.PI) / 4, color: ARM_COLOR_REAR },
];

/** One motor: a bell, a spinning prop disc and an LED under the boom. */
const MotorPod: React.FC<{
  position: [number, number, number];
  spin: 1 | -1;
  ledColor: string;
  index: number;
  motorSpeedsRef: Props['motorSpeedsRef'];
}> = ({ position, spin, ledColor, index, motorSpeedsRef }) => {
  const propRef = useRef<Group>(null);
  const discRef = useRef<Mesh>(null);

  useFrame((_, delta) => {
    const rpm = motorSpeedsRef.current[index] ?? 0;
    const radiansPerSecond = (rpm / 60) * Math.PI * 2;

    if (propRef.current) {
      // Deliberately under-sampled so the blades read as spinning rather than
      // strobing at 60 fps, the same way a camera shutter renders a real prop.
      propRef.current.rotation.y += spin * radiansPerSecond * delta * 0.02;
    }
    if (discRef.current) {
      // The blur disc fades in with RPM, so a spooled-down motor is visible.
      const material = discRef.current.material as MeshBasicMaterial;
      material.opacity = Math.min(0.28, (rpm / 11000) * 0.32);
    }
  });

  return (
    <group position={position}>
      {/* Motor bell */}
      <mesh castShadow>
        <cylinderGeometry args={[0.028, 0.032, 0.035, 12]} />
        <meshStandardMaterial color="#0f172a" metalness={0.85} roughness={0.3} />
      </mesh>

      {/* Two blades */}
      <group ref={propRef} position={[0, 0.03, 0]}>
        <mesh rotation={[0, 0, 0.14]}>
          <boxGeometry args={[0.27, 0.004, 0.03]} />
          <meshStandardMaterial color="#cbd5e1" roughness={0.45} />
        </mesh>
        <mesh rotation={[0, Math.PI / 2, 0.14]}>
          <boxGeometry args={[0.27, 0.004, 0.03]} />
          <meshStandardMaterial color="#cbd5e1" roughness={0.45} />
        </mesh>
      </group>

      {/* Motion-blur disc */}
      <mesh ref={discRef} position={[0, 0.03, 0]} rotation={[-Math.PI / 2, 0, 0]}>
        <circleGeometry args={[0.14, 28]} />
        <meshBasicMaterial color="#e2e8f0" transparent opacity={0.12} depthWrite={false} />
      </mesh>

      {/* Navigation LED. Emissive only, no point light: four coloured lights on
          a 0.45 m airframe wash the whole model out to a pale blur. */}
      <mesh position={[0, -0.028, 0]}>
        <sphereGeometry args={[0.028, 12, 12]} />
        <meshBasicMaterial color={ledColor} />
      </mesh>
    </group>
  );
};

export const DroneModel: React.FC<Props> = ({ motorSpeedsRef }) => {
  const groupRef = useRef<Group>(null);
  const L = PHYSICS.armLength;

  return (
    <group ref={groupRef}>
      {/* Central body */}
      <mesh castShadow>
        <boxGeometry args={[0.15, 0.06, 0.24]} />
        <meshStandardMaterial color="#334155" metalness={0.55} roughness={0.4} />
      </mesh>
      {/* Canopy, tapered toward -Z so the nose direction is obvious in flight */}
      <mesh position={[0, 0.03, -0.08]} rotation={[-Math.PI / 2, 0, Math.PI / 4]} castShadow>
        <coneGeometry args={[0.05, 0.12, 4]} />
        <meshStandardMaterial color="#0f172a" metalness={0.7} roughness={0.25} />
      </mesh>
      {/* Forward-facing camera pod */}
      <mesh position={[0, 0.012, -0.1]}>
        <sphereGeometry args={[0.018, 12, 12]} />
        <meshStandardMaterial color="#020617" metalness={0.9} roughness={0.1} />
      </mesh>

      {/* Arms, X configuration. Front pair cyan, rear pair red, matching the
          motor colour key in the telemetry panel.

          Each boom is a box elongated along -Z inside a group yawed to aim it
          at its motor. A bare cylinder would not work here: three.js cylinders
          run along +Y, so rotating one about Y leaves it standing upright. */}
      {ARMS.map((arm) => (
        <group key={arm.key} rotation={[0, arm.yaw, 0]}>
          <mesh position={[0, 0, -ARM_SPAN / 2]} castShadow>
            <boxGeometry args={[0.032, 0.018, ARM_SPAN]} />
            <meshStandardMaterial
              color={arm.color}
              emissive={arm.color}
              emissiveIntensity={0.5}
              roughness={0.5}
            />
          </mesh>
        </group>
      ))}

      {/* Landing skids */}
      <mesh position={[0.07, -0.058, 0]}>
        <boxGeometry args={[0.014, 0.03, 0.17]} />
        <meshStandardMaterial color="#0b1220" roughness={0.8} />
      </mesh>
      <mesh position={[-0.07, -0.058, 0]}>
        <boxGeometry args={[0.014, 0.03, 0.17]} />
        <meshStandardMaterial color="#0b1220" roughness={0.8} />
      </mesh>

      {/* FL, FR, RL, RR, the same order the motor mixer uses. */}
      <MotorPod position={[L, 0.02, -L]} spin={1} ledColor="#22d3ee" index={0} motorSpeedsRef={motorSpeedsRef} />
      <MotorPod position={[-L, 0.02, -L]} spin={-1} ledColor="#34d399" index={1} motorSpeedsRef={motorSpeedsRef} />
      <MotorPod position={[L, 0.02, L]} spin={-1} ledColor="#fbbf24" index={2} motorSpeedsRef={motorSpeedsRef} />
      <MotorPod position={[-L, 0.02, L]} spin={1} ledColor="#f43f5e" index={3} motorSpeedsRef={motorSpeedsRef} />
    </group>
  );
};
