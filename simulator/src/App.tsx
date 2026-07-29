import React, { useState, useEffect } from 'react';
import { Canvas, useFrame } from '@react-three/fiber';
import { DroneModel } from './components/DroneModel';
import { WorldEnvironment } from './components/Environment';
import { CameraController } from './components/CameraController';
import { TelemetryPanel } from './components/TelemetryPanel';
import { PIDTuningPanel } from './components/PIDTuningPanel';
import { ScoreDisplay } from './components/ScoreDisplay';
import { ControlsHelp } from './components/ControlsHelp';
import { DroneSimulator } from './engine/DronePhysics';
import { WindSystem } from './engine/WindSystem';
import { DroneState, TelemetryData, GameState, PIDGains } from './types';
import { DEFAULT_ROLL_PID, DEFAULT_PITCH_PID, DEFAULT_YAW_PID, CHECKPOINTS } from './utils/constants';
import { Vector3, Euler } from 'three';

const engine = new DroneSimulator(DEFAULT_ROLL_PID, DEFAULT_PITCH_PID, DEFAULT_YAW_PID);
const wind = new WindSystem();

const keys: Record<string, boolean> = {};

const Scene: React.FC<{
  updateState: (ds: DroneState, td: TelemetryData, gs: GameState) => void,
  gameState: GameState,
  setGameState: React.Dispatch<React.SetStateAction<GameState>>
}> = ({ updateState, gameState, setGameState }) => {
  const [localDs, setLocalDs] = useState<DroneState>({
    position: new Vector3(),
    rotation: new Vector3(),
    velocity: new Vector3(),
    motorSpeeds: [0,0,0,0],
    armed: false,
    mode: 'manual'
  });

  useFrame((state, delta) => {
    // Input mapping
    let throttle = 0;
    let roll = 0;
    let pitch = 0;
    let yaw = 0;

    if (keys[' ']) throttle += 6; // base hover roughly
    if (keys['Shift']) throttle -= 2;
    if (keys['w']) pitch -= 0.5;
    if (keys['s']) pitch += 0.5;
    if (keys['a']) roll += 0.5;
    if (keys['d']) roll -= 0.5;
    if (keys['q']) yaw += 1;
    if (keys['e']) yaw -= 1;

    // Wind
    const windForce = wind.getWindForce(state.clock.elapsedTime);

    // Update physics
    const { droneState, telemetry } = engine.update(delta, { throttle, roll, pitch, yaw }, windForce);
    
    setLocalDs(droneState);

    // Checkpoint logic
    let newGs = { ...gameState };
    if (newGs.checkpointsHit < CHECKPOINTS.length) {
      const cp = CHECKPOINTS[newGs.checkpointsHit];
      if (droneState.position.distanceTo(cp) < 3.5) {
        newGs.checkpointsHit += 1;
      }
    }

    // Crash logic
    if (droneState.position.y <= 0.15 && droneState.velocity.length() > 5) {
      newGs.crashCount += 1;
      engine.position.y = 0.2;
      engine.velocity.set(0,0,0);
    }

    if (newGs.checkpointsHit !== gameState.checkpointsHit || newGs.crashCount !== gameState.crashCount) {
      setGameState(newGs);
    }

    updateState(droneState, telemetry, newGs);
  });

  return (
    <>
      <WorldEnvironment currentCheckpoint={gameState.checkpointsHit} />
      <group position={localDs.position} rotation={new Euler(localDs.rotation.x, localDs.rotation.y, localDs.rotation.z, 'YXZ')}>
        <DroneModel motorSpeeds={localDs.motorSpeeds} />
      </group>
      <CameraController dronePos={localDs.position} droneRot={localDs.rotation} mode={gameState.cameraMode} />
    </>
  );
};

export default function App() {
  const [droneState, setDroneState] = useState<DroneState | null>(null);
  const [telemetry, setTelemetry] = useState<TelemetryData | null>(null);
  
  const [gameState, setGameState] = useState<GameState>({
    score: 0,
    checkpointsHit: 0,
    crashCount: 0,
    windEnabled: false,
    showTelemetry: true,
    showPID: false,
    cameraMode: 'chase'
  });

  const [rollPID, setRollPID] = useState<PIDGains>(DEFAULT_ROLL_PID);
  const [pitchPID, setPitchPID] = useState<PIDGains>(DEFAULT_PITCH_PID);
  const [yawPID, setYawPID] = useState<PIDGains>(DEFAULT_YAW_PID);

  useEffect(() => {
    const handleKeyDown = (e: KeyboardEvent) => {
      keys[e.key] = true;
      if (e.key === 'c') {
        setGameState(prev => ({
          ...prev, 
          cameraMode: prev.cameraMode === 'chase' ? 'fpv' : (prev.cameraMode === 'fpv' ? 'top-down' : 'chase')
        }));
      }
      if (e.key === 'r') {
        setGameState(prev => {
          const w = !prev.windEnabled;
          wind.setIntensity(w ? 1 : 0);
          return { ...prev, windEnabled: w };
        });
      }
      if (e.key === 'p') {
        setGameState(prev => ({ ...prev, showPID: !prev.showPID }));
      }
    };
    
    const handleKeyUp = (e: KeyboardEvent) => {
      keys[e.key] = false;
    };

    window.addEventListener('keydown', handleKeyDown);
    window.addEventListener('keyup', handleKeyUp);
    return () => {
      window.removeEventListener('keydown', handleKeyDown);
      window.removeEventListener('keyup', handleKeyUp);
    };
  }, []);

  const handlePIDChange = (axis: 'roll'|'pitch'|'yaw', gains: PIDGains) => {
    if (axis === 'roll') { setRollPID(gains); engine.rollPID.setGains(gains.kp, gains.ki, gains.kd); }
    if (axis === 'pitch') { setPitchPID(gains); engine.pitchPID.setGains(gains.kp, gains.ki, gains.kd); }
    if (axis === 'yaw') { setYawPID(gains); engine.yawPID.setGains(gains.kp, gains.ki, gains.kd); }
  };

  const resetPID = () => {
    setRollPID(DEFAULT_ROLL_PID); engine.rollPID.setGains(DEFAULT_ROLL_PID.kp, DEFAULT_ROLL_PID.ki, DEFAULT_ROLL_PID.kd);
    setPitchPID(DEFAULT_PITCH_PID); engine.pitchPID.setGains(DEFAULT_PITCH_PID.kp, DEFAULT_PITCH_PID.ki, DEFAULT_PITCH_PID.kd);
    setYawPID(DEFAULT_YAW_PID); engine.yawPID.setGains(DEFAULT_YAW_PID.kp, DEFAULT_YAW_PID.ki, DEFAULT_YAW_PID.kd);
  };

  return (
    <>
      <Canvas shadows camera={{ fov: 60 }}>
        <Scene 
          updateState={(ds, td, gs) => {
            setDroneState(ds);
            setTelemetry(td);
          }}
          gameState={gameState}
          setGameState={setGameState}
        />
      </Canvas>
      
      <div className="hud-overlay">
        {telemetry && gameState.showTelemetry && <TelemetryPanel telemetry={telemetry} />}
        {gameState.showPID && (
          <PIDTuningPanel 
            rollPID={rollPID} pitchPID={pitchPID} yawPID={yawPID} 
            onChange={handlePIDChange} onReset={resetPID} 
          />
        )}
        <ScoreDisplay gameState={gameState} />
        <ControlsHelp />
        
        <div style={{ position: 'absolute', top: 20, left: 20, display: 'flex', gap: 10, pointerEvents: 'auto' }}>
          <button className="reset-btn" onClick={() => setGameState(p => ({...p, showPID: !p.showPID}))} style={{ width: 'auto' }}>
            {gameState.showPID ? 'Hide PID' : 'Tune PID'}
          </button>
        </div>
      </div>
    </>
  );
}
