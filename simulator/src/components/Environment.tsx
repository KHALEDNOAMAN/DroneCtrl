import React, { useRef, useMemo } from 'react';
import { useFrame } from '@react-three/fiber';
import { Grid, Sky } from '@react-three/drei';
import { CHECKPOINTS } from '../utils/constants';

interface Props {
  currentCheckpoint: number;
}

export const WorldEnvironment: React.FC<Props> = ({ currentCheckpoint }) => {
  const ringsRef = useRef<any[]>([]);

  // Generate tree positions ONCE — stable across re-renders
  const trees = useMemo(() => {
    const result: { x: number; z: number }[] = [];
    // Use a seeded-style approach with fixed values
    for (let i = 0; i < 50; i++) {
      const angle = (i / 50) * Math.PI * 2 * 7.3; // spread in a pattern
      const radius = 20 + (i * 3.7) % 80;
      const x = Math.cos(angle) * radius;
      const z = Math.sin(angle) * radius;
      if (Math.abs(x) < 10 && Math.abs(z) < 10) continue; // keep center clear
      result.push({ x, z });
    }
    return result;
  }, []);

  useFrame((state) => {
    const t = state.clock.elapsedTime;
    if (ringsRef.current[currentCheckpoint]) {
      ringsRef.current[currentCheckpoint].scale.setScalar(1 + Math.sin(t * 5) * 0.05);
    }
  });

  return (
    <>
      <Sky sunPosition={[100, 20, 100]} turbidity={0.1} rayleigh={0.5} />
      <ambientLight intensity={0.4} />
      <directionalLight position={[10, 20, 10]} intensity={1.5} castShadow />
      
      {/* Ground */}
      <Grid infiniteGrid fadeDistance={200} cellColor="#1f2937" sectionColor="#374151" />
      <mesh rotation={[-Math.PI / 2, 0, 0]} position={[0, -0.01, 0]} receiveShadow>
        <planeGeometry args={[1000, 1000]} />
        <meshStandardMaterial color="#064e3b" />
      </mesh>

      {/* Checkpoints */}
      {CHECKPOINTS.map((pos, i) => {
        const isCurrent = i === currentCheckpoint;
        const isPassed = i < currentCheckpoint;
        
        return (
          <mesh 
            key={i} 
            position={pos} 
            rotation={[Math.PI/2, 0, 0]}
            ref={el => ringsRef.current[i] = el}
          >
            <torusGeometry args={[3, 0.2, 16, 32]} />
            <meshStandardMaterial 
              color={isPassed ? "#10b981" : (isCurrent ? "#06b6d4" : "#4b5563")}
              emissive={isPassed ? "#10b981" : (isCurrent ? "#06b6d4" : "#000000")}
              emissiveIntensity={isCurrent ? 2 : 0}
            />
          </mesh>
        );
      })}

      {/* Trees — positions are stable (computed once via useMemo) */}
      {trees.map((tree, i) => (
        <group key={`tree-${i}`} position={[tree.x, 0, tree.z]}>
          <mesh position={[0, 2, 0]}>
            <cylinderGeometry args={[0.5, 0.5, 4]} />
            <meshStandardMaterial color="#3e2723" />
          </mesh>
          <mesh position={[0, 6, 0]}>
            <coneGeometry args={[3, 8, 8]} />
            <meshStandardMaterial color="#1b5e20" />
          </mesh>
        </group>
      ))}
    </>
  );
};
