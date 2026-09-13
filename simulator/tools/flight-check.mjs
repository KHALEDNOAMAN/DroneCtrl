/**
 * Headless flight check.
 *
 * Runs the real DroneSimulator + Autopilot at a fixed timestep with no browser
 * and no renderer, and asserts the autopilot can actually fly the circuit.
 * Catches control-law regressions that are invisible until you watch the sim
 * for 30 seconds in a browser.
 *
 *   node tools/flight-check.mjs
 */
import { build } from 'esbuild';
import { Vector3 } from 'three';
import { unlinkSync } from 'node:fs';
import { fileURLToPath, pathToFileURL } from 'node:url';

const outfile = fileURLToPath(new URL('./.flight-check.bundle.mjs', import.meta.url));

await build({
  entryPoints: [fileURLToPath(new URL('../src/engine/index.ts', import.meta.url))],
  bundle: true,
  outfile,
  format: 'esm',
  platform: 'node',
});

const module = await import(pathToFileURL(outfile).href);
unlinkSync(outfile);

const {
  DroneSimulator,
  Autopilot,
  WindSystem,
  DEFAULT_ROLL_PID,
  DEFAULT_PITCH_PID,
  DEFAULT_YAW_PID,
  CHECKPOINTS,
} = module;

const DURATION = 120; // seconds of simulated flight
const zeroWind = new Vector3();

const wrap = (a) => {
  let x = a;
  while (x > Math.PI) x -= 2 * Math.PI;
  while (x < -Math.PI) x += 2 * Math.PI;
  return x;
};

function fly(dt) {
  const steps = Math.round(DURATION / dt);
  const engine = new DroneSimulator(DEFAULT_ROLL_PID, DEFAULT_PITCH_PID, DEFAULT_YAW_PID);
  const autopilot = new Autopilot(CHECKPOINTS);

  let groundContacts = 0;
  let maxTilt = 0;
  let maxSpeed = 0;
  let minAltitudeAirborne = Infinity;
  let headingErrorSum = 0;
  let headingSamples = 0;
  const gateTimes = [];
  let seenGates = 0;

  for (let step = 0; step < steps; step++) {
    const t = step * dt;
    const inputs = autopilot.update(engine.position, engine.velocity, engine.rotation.y, dt);
    engine.update(dt, inputs, zeroWind, 'AUTO');

    const totalGates = autopilot.lap * CHECKPOINTS.length + autopilot.waypointIndex;
    if (totalGates > seenGates) {
      seenGates = totalGates;
      gateTimes.push(t);
    }

    const tilt = Math.max(Math.abs(engine.rotation.x), Math.abs(engine.rotation.z));
    if (tilt > maxTilt) maxTilt = tilt;
    if (engine.velocity.length() > maxSpeed) maxSpeed = engine.velocity.length();

    if (t > 8) {
      if (engine.position.y < minAltitudeAirborne) minAltitudeAirborne = engine.position.y;
      if (engine.position.y <= 0.2) groundContacts++;

      // Is the nose actually pointing down the track, or is the drone
      // crabbing sideways to every gate with a frozen heading?
      const target = autopilot.target;
      const dx = target.x - engine.position.x;
      const dz = target.z - engine.position.z;
      if (Math.hypot(dx, dz) > 12) {
        headingErrorSum += Math.abs(wrap(Math.atan2(-dx, -dz) - engine.rotation.y));
        headingSamples++;
      }
    }
  }

  return {
    dt,
    gates: seenGates,
    laps: autopilot.lap,
    lapTime: autopilot.lap > 0 ? gateTimes[CHECKPOINTS.length - 1] : null,
    maxTiltDeg: (maxTilt * 180) / Math.PI,
    maxSpeed,
    minAltitude: minAltitudeAirborne,
    groundContacts,
    meanHeadingErrorDeg: headingSamples
      ? ((headingErrorSum / headingSamples) * 180) / Math.PI
      : NaN,
  };
}

const failures = [];

// Primary run at the rate the browser uses.
const main = fly(1 / 120);

console.log('--- DroneCtrl flight check ---');
console.log(`simulated        ${DURATION}s at 120 Hz`);
console.log(`gates captured   ${main.gates}`);
console.log(`laps completed   ${main.laps}`);
console.log(`first lap time   ${main.lapTime ? main.lapTime.toFixed(1) + 's' : 'n/a'}`);
console.log(`max tilt         ${main.maxTiltDeg.toFixed(1)} deg`);
console.log(`max speed        ${main.maxSpeed.toFixed(1)} m/s`);
console.log(`min altitude     ${main.minAltitude.toFixed(2)} m (after takeoff)`);
console.log(`ground contacts  ${main.groundContacts}`);
console.log(`mean hdg error   ${main.meanHeadingErrorDeg.toFixed(1)} deg`);

if (main.laps < 2) failures.push(`expected at least 2 laps in ${DURATION}s, flew ${main.laps}`);
if (main.groundContacts > 0) failures.push(`touched the ground ${main.groundContacts} times mid-mission`);
if (main.maxTiltDeg > 63) failures.push(`tilt exceeded 63 deg (${main.maxTiltDeg.toFixed(0)})`);
if (main.maxSpeed > 30) failures.push(`speed ran away to ${main.maxSpeed.toFixed(1)} m/s`);
if (!(main.meanHeadingErrorDeg < 30)) {
  failures.push(`nose is not tracking the route: mean heading error ${main.meanHeadingErrorDeg.toFixed(0)} deg`);
}

// The physics must not depend on the timestep. Halving it should not change
// the outcome in any way a pilot would notice.
const half = fly(1 / 240);
console.log(`\n240 Hz rerun     ${half.laps} laps, ${half.gates} gates, max tilt ${half.maxTiltDeg.toFixed(1)} deg`);
if (Math.abs(half.gates - main.gates) > 2) {
  failures.push(`timestep dependent: ${main.gates} gates at 120 Hz vs ${half.gates} at 240 Hz`);
}

if (failures.length) {
  console.error('\nFAIL');
  failures.forEach(f => console.error('  - ' + f));
  process.exit(1);
}

console.log('\nPASS');
