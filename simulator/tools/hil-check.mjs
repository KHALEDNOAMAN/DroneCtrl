/**
 * Headless hardware-in-the-loop check.
 *
 * Flies the simulator's HIL session (src/hil) against the firmware's own HIL
 * loop, with no board: firmware/sil/sil_flight --hil-board runs exactly what
 * the ESP32 runs in the esp32_hil build, with stdin and stdout in place of
 * the USB serial port. Every step crosses the pipe as MAVLink bytes, encoded
 * by TypeScript on one side and C++ on the other.
 *
 * What it asserts, in order:
 *   1. the arm gesture sent as RC_CHANNELS_OVERRIDE arms the board
 *   2. the board holds a level hover on the plant its gains were tuned for
 *   3. a 15 degree roll command is tracked, holding height through it
 *   4. when the sticks stop arriving, the board flies the airframe DOWN
 *      (FAILSAFE_LAND), rather than cutting the motors in the air
 *   5. no timeouts and no rejected frames in either direction
 *
 *   make -C ../firmware/sil sil_flight && node tools/hil-check.mjs
 */
import { build } from 'esbuild';
import { spawn } from 'node:child_process';
import { existsSync, unlinkSync } from 'node:fs';
import { fileURLToPath, pathToFileURL } from 'node:url';

const board = fileURLToPath(new URL('../../firmware/sil/sil_flight', import.meta.url));
if (!existsSync(board)) {
  console.error(`missing ${board}; run: make -C firmware/sil sil_flight`);
  process.exit(1);
}

const outfile = fileURLToPath(new URL('./.hil-check.bundle.mjs', import.meta.url));
await build({
  entryPoints: [fileURLToPath(new URL('../src/hil/index.ts', import.meta.url))],
  bundle: true,
  outfile,
  format: 'esm',
  platform: 'node',
});
const { HilSession } = await import(pathToFileURL(outfile).href);
unlinkSync(outfile);

const child = spawn(board, ['--hil-board'], { stdio: ['pipe', 'pipe', 'inherit'] });
const session = new HilSession({
  write: (bytes) => new Promise((resolve) => child.stdin.write(bytes, resolve)),
  onData: (cb) => child.stdout.on('data', (b) => cb(new Uint8Array(b))),
});

const deg = (r) => (r * 180) / Math.PI;
const failures = [];
const check = (ok, what) => {
  console.log(`  ${ok ? 'ok  ' : 'FAIL'} ${what}`);
  if (!ok) failures.push(what);
};

// Hover for this airframe: four motors at mg/4 is 0.409 of full thrust, which
// is 0.378 of the idle-to-max stick range (see sil_flight.cpp).
const HOVER = 0.378;
const BANKED_HOVER = (1000 + 409 / Math.cos((15 * Math.PI) / 180) - 1050) / 950;
let armedAt = null;
let worstHoverTilt = 0;
let rollReached = 0;
let lossAt = null;
let zAtLoss = 0;
let landSeenAt = null;
let zOneSecondAfter = null;
let peakDescent = 0;
const seenStates = new Set();

console.log('DroneCtrl HIL check: TypeScript simulator <-> firmware HIL loop over MAVLink\n');

const STEPS = Math.round(32 / 0.004);
for (let i = 0; i < STEPS; i++) {
  const t = session.stats.simTime;
  const sticks = { throttle: 0, roll: 0, pitch: 0, yaw: 0 };
  if (t < 2.5) {
    sticks.yaw = 1; // arm gesture: throttle down, yaw right, held two seconds
  } else if (t < 3.5) {
    sticks.throttle = 0.42; // climb
  } else {
    sticks.throttle = HOVER;
  }
  if (t >= 12 && t < 14) {
    // 15 degrees of bank, with the throttle raised by 1/cos(15) so the
    // airframe holds height through the turn instead of sinking.
    sticks.roll = 0.5;
    sticks.throttle = BANKED_HOVER;
  }
  if (t >= 18 && session.linkUp) {
    session.linkUp = false; // the receiver dies
    lossAt = t;
    zAtLoss = session.plant.z;
  }

  const ok = await session.step(sticks);
  if (!ok) break;

  seenStates.add(session.board.state);
  if (armedAt === null && session.board.armed) armedAt = t;
  if (t > 6 && t < 12) {
    worstHoverTilt = Math.max(worstHoverTilt, Math.abs(deg(session.plant.roll)), Math.abs(deg(session.plant.pitch)));
  }
  if (t > 13 && t < 14) rollReached = Math.max(rollReached, deg(session.plant.roll));
  if (lossAt !== null && landSeenAt === null && session.board.state === 'FAILSAFE_LAND') landSeenAt = t;
  if (lossAt !== null && zOneSecondAfter === null && t >= lossAt + 1.0) zOneSecondAfter = session.plant.z;
  if (lossAt !== null) peakDescent = Math.max(peakDescent, -session.plant.vz);
}
child.stdin.end();

const s = session.stats;
check(armedAt !== null && armedAt < 3.0, `board armed from the RC gesture (at ${armedAt?.toFixed(2)} s)`);
check(worstHoverTilt < 5, `level hover, worst tilt ${worstHoverTilt.toFixed(2)} deg`);
check(session.plant.z >= 0 && zAtLoss > 0.3, `airborne before the link loss (${zAtLoss.toFixed(2)} m)`);
check(Math.abs(rollReached - 15) < 3, `tracked a 15 deg roll command (reached ${rollReached.toFixed(1)} deg)`);
// The board declares the link lost after its 500 ms override timeout, and the
// state reaches us on the next 1 Hz HEARTBEAT, so up to 1.5 s is expected.
check(landSeenAt !== null && landSeenAt - lossAt <= 1.55,
  `link loss reported as FAILSAFE_LAND after ${landSeenAt === null ? 'never' : (landSeenAt - lossAt).toFixed(2) + ' s'}` +
  ' (500 ms timeout, then the next heartbeat)');
check(zOneSecondAfter !== null && zOneSecondAfter > 0.5 * zAtLoss,
  `still flying one second after the loss (${zOneSecondAfter?.toFixed(2)} m of ${zAtLoss.toFixed(2)} m), not cut`);
// The physical difference between a failsafe landing and a motor cut. A cut
// from this height free-falls to about 6.5 m/s. The landing ramp is open loop
// (a fixed throttle decrement, no barometer in the loop), so it still lands
// firmly, at a little over 4 m/s from 2.6 m; the threshold sits between the
// two. Closing the descent on altitude would bring this down, and when it does
// this threshold should come down with it.
check(peakDescent < 5.0, `failsafe descent, peak sink rate ${peakDescent.toFixed(2)} m/s (a motor cut reaches ~6.5)`);
check(session.plant.z < 0.05, `landed by the end (${session.plant.z.toFixed(2)} m)`);
check(s.timeouts === 0, `${s.steps} lockstep steps, ${s.timeouts} timeouts`);
check(s.crcErrors === 0, `${s.crcErrors} frames rejected by the simulator`);
check('loop_jit' in session.board.values, 'board telemetry (NAMED_VALUE_FLOAT) received');

console.log(`\n  states seen: ${[...seenStates].join(', ')}`);
console.log(`  mean round trip ${s.roundTripMs.toFixed(3)} ms per step over the pipe\n`);

if (failures.length) {
  console.log(`${failures.length} check(s) failed`);
  process.exit(1);
}
console.log('HIL check passed');
