import React, { useRef } from 'react';
import { useFrame } from '@react-three/fiber';
import { Group, Mesh } from 'three';
import { PHYSICS } from '../utils/constants';

interface Props {
  motorSpeeds: [number, number, number, number];
}

export const DroneModel: React.FC<Props> = ({ motorSpeeds }) => {
  const groupRef = useRef<Group>(null);
  
  const propFL = useRef<Mesh>(null);
  const propFR = useRef<Mesh>(null);
  const propRL = useRef<Mesh>(null);
  const propRR = useRef<Mesh>(null);

  useFrame((state, delta) => {
    if (propFL.current) propFL.current.rotation.y += motorSpeeds[0] * delta * 0.1;
    if (propFR.current) propFR.current.rotation.y -= motorSpeeds[1] * delta * 0.1;
    if (propRL.current) propRL.current.rotation.y -= motorSpeeds[2] * delta * 0.1;
    if (propRR.current) propRR.current.rotation.y += motorSpeeds[3] * delta * 0.1;
  });

  const L = PHYSICS.armLength;

  return (
    <group ref={groupRef}>
      {/* Central Body */}
      <mesh position={[0, 0, 0]}>
        <boxGeometry args={[0.1, 0.05, 0.15]} />
        <meshStandardMaterial color="#2d3748" />
      </mesh>
      
      {/* Arms (X config) */}
      <mesh position={[L/2, 0, -L/2]} rotation={[0, Math.PI/4, 0]}>
        <cylinderGeometry args={[0.01, 0.01, L * 1.4]} />
        <meshStandardMaterial color="#06b6d4" />
      </mesh>
      <mesh position={[-L/2, 0, -L/2]} rotation={[0, -Math.PI/4, 0]}>
        <cylinderGeometry args={[0.01, 0.01, L * 1.4]} />
        <meshStandardMaterial color="#06b6d4" />
      </mesh>
      <mesh position={[L/2, 0, L/2]} rotation={[0, -Math.PI/4, 0]}>
        <cylinderGeometry args={[0.01, 0.01, L * 1.4]} />
        <meshStandardMaterial color="#ef4444" />
      </mesh>
      <mesh position={[-L/2, 0, L/2]} rotation={[0, Math.PI/4, 0]}>
        <cylinderGeometry args={[0.01, 0.01, L * 1.4]} />
        <meshStandardMaterial color="#ef4444" />
      </mesh>

      {/* Motors and Props */}
      <group position={[L, 0.02, -L]}>
        <mesh><cylinderGeometry args={[0.02, 0.02, 0.03]} /><meshStandardMaterial color="#1f2937" /></mesh>
        <mesh ref={propFL} position={[0, 0.02, 0]}><cylinderGeometry args={[0.1, 0.1, 0.002]} /><meshStandardMaterial color="#ffffff" transparent opacity={0.6} /></mesh>
      </group>
      
      <group position={[-L, 0.02, -L]}>
        <mesh><cylinderGeometry args={[0.02, 0.02, 0.03]} /><meshStandardMaterial color="#1f2937" /></mesh>
        <mesh ref={propFR} position={[0, 0.02, 0]}><cylinderGeometry args={[0.1, 0.1, 0.002]} /><meshStandardMaterial color="#ffffff" transparent opacity={0.6} /></mesh>
      </group>

      <group position={[L, 0.02, L]}>
        <mesh><cylinderGeometry args={[0.02, 0.02, 0.03]} /><meshStandardMaterial color="#1f2937" /></mesh>
        <mesh ref={propRL} position={[0, 0.02, 0]}><cylinderGeometry args={[0.1, 0.1, 0.002]} /><meshStandardMaterial color="#ffffff" transparent opacity={0.6} /></mesh>
      </group>

      <group position={[-L, 0.02, L]}>
        <mesh><cylinderGeometry args={[0.02, 0.02, 0.03]} /><meshStandardMaterial color="#1f2937" /></mesh>
        <mesh ref={propRR} position={[0, 0.02, 0]}><cylinderGeometry args={[0.1, 0.1, 0.002]} /><meshStandardMaterial color="#ffffff" transparent opacity={0.6} /></mesh>
      </group>
    </group>
  );
};
