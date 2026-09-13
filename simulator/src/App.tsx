import React, { useState, useEffect, useRef, useCallback } from 'react';
import { Canvas, useFrame } from '@react-three/fiber';
import { Group, Vector3 } from 'three';
import { DroneModel } from './components/DroneModel';
import { WorldEnvironment } from './components/Environment';
import { CameraController } from './components/CameraController';
import { TelemetryPanel } from './components/TelemetryPanel';
import { PIDTuningPanel } from './components/PIDTuningPanel';
import { ScoreDisplay } from './components/ScoreDisplay';
import { ControlsHelp } from './components/ControlsHelp';
import { FlightModeBadge } from './components/FlightModeBadge';
import { AttitudeIndicator } from './components/AttitudeIndicator';
import { DroneSimulator } from './engine/DronePhysics';
import { Autopilot } from './engine/Autopilot';
import { WindSystem } from './engine/WindSystem';
import {
  AutopilotStatus,
  ControlInputs,
  DroneState,
  FlightMode,
  GameState,
  PIDGains,
  TelemetryData,
} from './types';
import {
  DEFAULT_ROLL_PID,
  DEFAULT_PITCH_PID,
  DEFAULT_YAW_PID,
  CHECKPOINTS,
  GATE_RADIUS,
  HOVER_THRUST_PER_MOTOR,
} from './utils/constants';

const engine = new DroneSimulator(DEFAULT_ROLL_PID, DEFAULT_PITCH_PID, DEFAULT_YAW_PID);
const autopilot = new Autopilot(CHECKPOINTS);
const wind = new WindSystem();

const keys: Record<string, boolean> = {};

/** Keys that mean "the pilot is flying", so the autopilot should stand down. */
const FLIGHT_KEYS = new Set([' ', 'shift', 'w', 'a', 's', 'd', 'q', 'e']);

const CAMERA_CYCLE: GameState['cameraMode'][] = ['chase', 'cinematic', 'fpv', 'top-down'];

/**
 * The control loops are tuned for this rate and go unstable if they are handed
 * a long frame, so physics runs on a fixed step and the renderer takes as many
 * substeps as the elapsed frame time pays for. Without this the drone flies
 * fine at 120 fps and falls out of the sky on a slow machine.
 */
const FIXED_DT = 1 / 120;
/** Never run more than this much simulated time in one frame, so a backgrounded
 *  tab does not come back and try to catch up on minutes of physics at once. */
const MAX_FRAME_TIME = 0.25;

interface SceneProps {
  gameStateRef: React.MutableRefObject<GameState>;
  cameraMode: GameState['cameraMode'];
  currentGate: number;
  onTelemetry: (t: TelemetryData, a: AutopilotStatus) => void;
  onGate: (index: number, lap: number) => void;
  onCrash: () => void;
}

/**
 * Memoised so the 14 Hz telemetry updates that re-render the HUD do not also
 * re-render the 3D tree. Its props are a stable ref plus stable callbacks, so
 * it only re-renders when the camera or the active gate actually changes.
 */
const Scene = React.memo<SceneProps>(({
  gameStateRef,
  cameraMode,
  currentGate,
  onTelemetry,
  onGate,
  onCrash,
}) => {
  const droneRef = useRef<Group>(null);
  const motorSpeedsRef = useRef<[number, number, number, number]>([0, 0, 0, 0]);
  const posRef = useRef(new Vector3(0, 0.1, 0));
  const rotRef = useRef(new Vector3(0, 0, 0));
  const trailRef = useRef<Vector3[]>([]);

  // HUD only needs to be readable, not frame-accurate. Pushing 60 React
  // renders a second through the whole tree is what made the old build stutter.
  const hudTimer = useRef(0);
  const trailTimer = useRef(0);
  const gateIndexRef = useRef(0);

  const accumulator = useRef(0);

  useFrame((state, delta) => {
    const gs = gameStateRef.current;

    accumulator.current = Math.min(accumulator.current + delta, MAX_FRAME_TIME);

    let droneState: DroneState | null = null;
    let telemetry: TelemetryData | null = null;

    while (accumulator.current >= FIXED_DT) {
      accumulator.current -= FIXED_DT;
      const stepped = step(gs, state.clock.elapsedTime);
      droneState = stepped.droneState;
      telemetry = stepped.telemetry;
    }

    if (!droneState || !telemetry) return;

    // Drive the 3D transform straight off the physics, no React round trip.
    posRef.current.copy(droneState.position);
    rotRef.current.copy(droneState.rotation);
    motorSpeedsRef.current = droneState.motorSpeeds;

    if (droneRef.current) {
      droneRef.current.position.copy(droneState.position);
      droneRef.current.rotation.set(
        droneState.rotation.x,
        droneState.rotation.y,
        droneState.rotation.z,
        'YXZ',
      );
    }

    // Gate capture. In AUTO the autopilot owns sequencing, so mirror its index
    // rather than running a second, slightly different test against the rings.
    if (gs.flightMode === 'AUTO') {
      const idx = autopilot.waypointIndex;
      if (idx !== gateIndexRef.current) {
        gateIndexRef.current = idx;
        onGate(idx, autopilot.lap);
      }
    } else {
      const idx = gateIndexRef.current;
      if (droneState.position.distanceTo(CHECKPOINTS[idx]) < GATE_RADIUS) {
        const next = (idx + 1) % CHECKPOINTS.length;
        gateIndexRef.current = next;
        onGate(next, 0);
      }
    }

    // Crash: hitting the ground with real vertical speed.
    if (droneState.position.y <= 0.15 && droneState.velocity.length() > 6) {
      onCrash();
      engine.position.y = 0.3;
      engine.velocity.multiplyScalar(0.2);
    }

    // Motion trail, sampled on a timer so its length is frame-rate independent.
    trailTimer.current += delta;
    if (trailTimer.current > 0.045) {
      trailTimer.current = 0;
      trailRef.current.push(droneState.position.clone());
      if (trailRef.current.length > 90) trailRef.current.shift();
    }

    hudTimer.current += delta;
    if (hudTimer.current > 0.07) {
      hudTimer.current = 0;
      onTelemetry(telemetry, autopilot.status);
    }
  });

  /** One fixed-size physics step: guidance or sticks, then the airframe. */
  function step(gs: GameState, elapsed: number) {
    let inputs: ControlInputs;

    if (gs.flightMode === 'AUTO') {
      inputs = autopilot.update(engine.position, engine.velocity, engine.rotation.y, FIXED_DT);
    } else {
      // Manual: sticks are attitude setpoints, throttle is per-motor thrust
      // biased around hover so releasing the keys holds roughly level flight.
      let throttle = HOVER_THRUST_PER_MOTOR;
      let roll = 0;
      let pitch = 0;
      let yaw = 0;

      if (keys[' ']) throttle += 2.2;
      if (keys['shift']) throttle -= 2.2;
      if (keys['w']) pitch -= 0.45;
      if (keys['s']) pitch += 0.45;
      if (keys['a']) roll += 0.45;
      if (keys['d']) roll -= 0.45;
      if (keys['q']) yaw += 1.5;
      if (keys['e']) yaw -= 1.5;

      inputs = { throttle, roll, pitch, yaw };
    }

    const windForce = wind.getWindForce(elapsed);
    return engine.update(FIXED_DT, inputs, windForce, gs.flightMode);
  }

  return (
    <>
      <WorldEnvironment currentGate={currentGate} trailRef={trailRef} />
      <group ref={droneRef}>
        <DroneModel motorSpeedsRef={motorSpeedsRef} />
      </group>
      <CameraController posRef={posRef} rotRef={rotRef} mode={cameraMode} />
    </>
  );
});
Scene.displayName = 'Scene';

export default function App() {
  const [telemetry, setTelemetry] = useState<TelemetryData | null>(null);
  const [apStatus, setApStatus] = useState<AutopilotStatus>(autopilot.status);

  const [gameState, setGameState] = useState<GameState>({
    score: 0,
    checkpointsHit: 0,
    lap: 0,
    crashCount: 0,
    windEnabled: false,
    showTelemetry: true,
    showPID: false,
    showHelp: true,
    flightMode: 'AUTO',
    cameraMode: 'chase',
  });

  // useFrame closes over whatever render it was created in, so the loop reads
  // game state through a ref instead of a stale captured value.
  const gameStateRef = useRef(gameState);
  gameStateRef.current = gameState;

  const [rollPID, setRollPID] = useState<PIDGains>(DEFAULT_ROLL_PID);
  const [pitchPID, setPitchPID] = useState<PIDGains>(DEFAULT_PITCH_PID);
  const [yawPID, setYawPID] = useState<PIDGains>(DEFAULT_YAW_PID);

  const handleTelemetry = useCallback((t: TelemetryData, a: AutopilotStatus) => {
    setTelemetry(t);
    setApStatus(a);
  }, []);

  const handleGate = useCallback((index: number, lap: number) => {
    setGameState(prev => ({
      ...prev,
      checkpointsHit: index,
      lap,
      score: prev.score + 100,
    }));
  }, []);

  const handleCrash = useCallback(() => {
    setGameState(prev => ({ ...prev, crashCount: prev.crashCount + 1 }));
  }, []);

  const setFlightMode = useCallback((mode: FlightMode) => {
    setGameState(prev => {
      if (prev.flightMode === mode) return prev;
      if (mode === 'AUTO') {
        autopilot.reset();
      } else {
        autopilot.disengage();
      }
      return { ...prev, flightMode: mode };
    });
  }, []);

  useEffect(() => {
    const handleKeyDown = (e: KeyboardEvent) => {
      const key = e.key.length === 1 ? e.key.toLowerCase() : e.key.toLowerCase();
      keys[key] = true;

      // Touching the sticks hands control back to the pilot, the way a real
      // ground station drops out of mission mode on manual input.
      if (FLIGHT_KEYS.has(key)) {
        e.preventDefault();
        setFlightMode('MANUAL');
      }

      if (key === 'c') {
        setGameState(prev => {
          const i = CAMERA_CYCLE.indexOf(prev.cameraMode);
          return { ...prev, cameraMode: CAMERA_CYCLE[(i + 1) % CAMERA_CYCLE.length] };
        });
      }
      if (key === 'm') {
        setGameState(prev => {
          const next: FlightMode = prev.flightMode === 'AUTO' ? 'MANUAL' : 'AUTO';
          if (next === 'AUTO') autopilot.reset();
          else autopilot.disengage();
          return { ...prev, flightMode: next };
        });
      }
      if (key === 'r') {
        setGameState(prev => {
          const w = !prev.windEnabled;
          wind.setIntensity(w ? 1 : 0);
          return { ...prev, windEnabled: w };
        });
      }
      if (key === 'p') {
        setGameState(prev => ({ ...prev, showPID: !prev.showPID }));
      }
      if (key === 'h' || key === '?' || key === '/') {
        setGameState(prev => ({ ...prev, showHelp: !prev.showHelp }));
      }
      if (key === 't') {
        setGameState(prev => ({ ...prev, showTelemetry: !prev.showTelemetry }));
      }
    };

    const handleKeyUp = (e: KeyboardEvent) => {
      keys[e.key.toLowerCase()] = false;
    };

    window.addEventListener('keydown', handleKeyDown);
    window.addEventListener('keyup', handleKeyUp);
    return () => {
      window.removeEventListener('keydown', handleKeyDown);
      window.removeEventListener('keyup', handleKeyUp);
    };
  }, [setFlightMode]);

  const handlePIDChange = (axis: 'roll' | 'pitch' | 'yaw', gains: PIDGains) => {
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
      <Canvas
        shadows
        camera={{ fov: 62, near: 0.1, far: 1000 }}
        gl={{ antialias: true, powerPreference: 'high-performance' }}
        dpr={[1, 2]}
      >
        <Scene
          gameStateRef={gameStateRef}
          cameraMode={gameState.cameraMode}
          currentGate={gameState.checkpointsHit}
          onTelemetry={handleTelemetry}
          onGate={handleGate}
          onCrash={handleCrash}
        />
      </Canvas>

      <div className="hud-overlay">
        <div className="hud-scanline" />

        <FlightModeBadge
          mode={gameState.flightMode}
          status={apStatus}
          windEnabled={gameState.windEnabled}
          cameraMode={gameState.cameraMode}
          onToggleMode={() => setFlightMode(gameState.flightMode === 'AUTO' ? 'MANUAL' : 'AUTO')}
          onTogglePID={() => setGameState(p => ({ ...p, showPID: !p.showPID }))}
          showPID={gameState.showPID}
        />

        {telemetry && gameState.showTelemetry && (
          <TelemetryPanel telemetry={telemetry} status={apStatus} mode={gameState.flightMode} />
        )}

        {telemetry && <AttitudeIndicator telemetry={telemetry} />}

        {gameState.showPID && (
          <PIDTuningPanel
            rollPID={rollPID} pitchPID={pitchPID} yawPID={yawPID}
            onChange={handlePIDChange} onReset={resetPID}
            telemetry={telemetry}
          />
        )}

        <ScoreDisplay gameState={gameState} status={apStatus} totalGates={CHECKPOINTS.length} />

        {gameState.showHelp && <ControlsHelp mode={gameState.flightMode} />}
      </div>
    </>
  );
}
