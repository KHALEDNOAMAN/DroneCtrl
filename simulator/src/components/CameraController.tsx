import React, { useRef } from 'react';
import { useFrame, useThree } from '@react-three/fiber';
import { Vector3, Euler } from 'three';
import { CAMERA_OFFSETS } from '../utils/constants';

interface Props {
  dronePos: Vector3;
  droneRot: Vector3;
  mode: 'chase' | 'top-down' | 'fpv';
}

export const CameraController: React.FC<Props> = ({ dronePos, droneRot, mode }) => {
  const { camera } = useThree();
  const currentTarget = useRef(new Vector3());

  useFrame((state, delta) => {
    let targetPos = new Vector3();
    let lookAtPos = dronePos.clone();

    if (mode === 'chase') {
      const euler = new Euler(0, droneRot.y, 0, 'YXZ');
      const offset = CAMERA_OFFSETS.chase.clone().applyEuler(euler);
      targetPos.copy(dronePos).add(offset);
      lookAtPos.y += 1;
    } else if (mode === 'fpv') {
      const euler = new Euler(droneRot.x, droneRot.y, droneRot.z, 'YXZ');
      const offset = CAMERA_OFFSETS.fpv.clone().applyEuler(euler);
      targetPos.copy(dronePos).add(offset);
      
      const fwd = new Vector3(0, 0, -10).applyEuler(euler);
      lookAtPos.copy(dronePos).add(fwd);
    } else if (mode === 'top-down') {
      targetPos.copy(dronePos).add(CAMERA_OFFSETS.topDown);
    }

    // Smooth camera movement — use tighter follow to reduce world jitter
    const lerpFactor = mode === 'fpv' ? 0.9 : Math.min(1, 6 * delta);
    camera.position.lerp(targetPos, lerpFactor);
    
    currentTarget.current.lerp(lookAtPos, lerpFactor);
    camera.lookAt(currentTarget.current);
  });

  return null;
};
