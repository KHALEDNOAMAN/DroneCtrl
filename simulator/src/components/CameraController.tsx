import React, { useRef } from 'react';
import { useFrame, useThree } from '@react-three/fiber';
import { Vector3, Euler } from 'three';
import { CAMERA_OFFSETS, CINEMATIC } from '../utils/constants';

interface Props {
  posRef: React.MutableRefObject<Vector3>;
  rotRef: React.MutableRefObject<Vector3>;
  mode: 'chase' | 'top-down' | 'fpv' | 'cinematic';
}

export const CameraController: React.FC<Props> = ({ posRef, rotRef, mode }) => {
  const { camera } = useThree();
  const smoothedTarget = useRef(new Vector3());
  const desired = useRef(new Vector3());
  const lookAt = useRef(new Vector3());

  useFrame((state, delta) => {
    const dronePos = posRef.current;
    const droneRot = rotRef.current;

    desired.current.set(0, 0, 0);
    lookAt.current.copy(dronePos);

    if (mode === 'chase') {
      // Follow the heading only. Copying roll and pitch into the camera makes
      // the horizon tumble and reads as a bug rather than as banking.
      const heading = new Euler(0, droneRot.y, 0, 'YXZ');
      desired.current.copy(CAMERA_OFFSETS.chase).applyEuler(heading).add(dronePos);
      lookAt.current.y += 0.25;
    } else if (mode === 'fpv') {
      const full = new Euler(droneRot.x, droneRot.y, droneRot.z, 'YXZ');
      desired.current.copy(CAMERA_OFFSETS.fpv).applyEuler(full).add(dronePos);
      lookAt.current.copy(dronePos).add(new Vector3(0, 0, -12).applyEuler(full));
    } else if (mode === 'top-down') {
      desired.current.copy(dronePos).add(CAMERA_OFFSETS.topDown);
    } else {
      // Cinematic: a slow orbit that keeps the drone centred while the world
      // sweeps behind it. This is the framing that shows the banking best.
      const angle = state.clock.elapsedTime * CINEMATIC.orbitSpeed;
      desired.current.set(
        dronePos.x + Math.sin(angle) * CINEMATIC.radius,
        dronePos.y + CINEMATIC.height,
        dronePos.z + Math.cos(angle) * CINEMATIC.radius,
      );
    }

    // FPV is rigidly mounted to the airframe; every other view is on a soft
    // gimbal, damped frame-rate independently so it behaves the same at 30 and
    // 144 fps.
    // The cinematic view gets its smoothness from the slow orbit, not from a
    // slack follow. Damping its position heavily just lets a 10 m/s drone
    // outrun the camera until it is a speck at the edge of frame.
    const positionResponse = mode === 'fpv' ? 40 : 9;

    // Aim faster still, so the subject stays centred through direction changes.
    const aimResponse = mode === 'fpv' ? 40 : 20;

    camera.position.lerp(desired.current, 1 - Math.exp(-positionResponse * delta));
    smoothedTarget.current.lerp(lookAt.current, 1 - Math.exp(-aimResponse * delta));
    camera.lookAt(smoothedTarget.current);
  });

  return null;
};
