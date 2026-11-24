import { HilSession, Sticks } from './HilSession';
import { SerialLink, openSerialLink } from './webserial';

/**
 * Runs a HIL session against a real board, paced to wall clock time, and
 * turns the keyboard into stick positions.
 *
 * Pacing: the session is lockstep, so it can run faster than real time over
 * a fast link. Here it steps only while simulated time is behind the wall
 * clock, so the flight on screen happens at the speed it would in the air.
 * Over a link too slow for 250 Hz, simulated time simply falls behind and
 * the panel says so.
 *
 * Keys (HIL mode only):
 *   Space / Shift   throttle stick up / down (it stays where you leave it)
 *   X (hold 2 s)    arm: throttle closed, yaw right, the firmware's gesture
 *   Z (hold 2 s)    disarm: throttle closed, yaw left
 *   W A S D         pitch and roll, up to 15 degrees
 *   Q E             yaw rate
 *   L               cut the RC link, to watch the failsafe fly it down
 */
export class HilController {
  session: HilSession | null = null;
  private link: SerialLink | null = null;
  private running = false;
  private throttle = 0;
  error: string | null = null;
  /** How far simulated time is behind the wall clock, s. Grows on a slow link. */
  lag = 0;

  constructor(private keys: Record<string, boolean>) {}

  get active(): boolean {
    return this.running && this.session !== null;
  }

  async connect(): Promise<void> {
    this.error = null;
    try {
      this.link = await openSerialLink();
      this.session = new HilSession(this.link);
      this.throttle = 0;
      this.running = true;
      void this.loop();
    } catch (e) {
      this.error = e instanceof Error ? e.message : String(e);
      this.running = false;
    }
  }

  async disconnect(): Promise<void> {
    this.running = false;
    const link = this.link;
    this.link = null;
    if (link) await link.close().catch(() => undefined);
  }

  toggleLink(): void {
    if (this.session) this.session.linkUp = !this.session.linkUp;
  }

  /** Sticks from the keyboard, sampled once per step. */
  sticks(dt: number): Sticks {
    const k = this.keys;
    if (k[' ']) this.throttle = Math.min(1, this.throttle + 0.3 * dt);
    if (k['shift']) this.throttle = Math.max(0, this.throttle - 0.3 * dt);
    const s: Sticks = { throttle: this.throttle, roll: 0, pitch: 0, yaw: 0 };
    if (k['x']) { s.throttle = 0; s.yaw = 1; this.throttle = 0; }
    if (k['z']) { s.throttle = 0; s.yaw = -1; this.throttle = 0; }
    // Firmware frame: positive roll right side down, positive pitch nose up.
    if (k['a']) s.roll -= 0.5;
    if (k['d']) s.roll += 0.5;
    if (k['w']) s.pitch -= 0.5;
    if (k['s']) s.pitch += 0.5;
    if (k['q']) s.yaw -= 0.4;
    if (k['e']) s.yaw += 0.4;
    return s;
  }

  private async loop(): Promise<void> {
    const start = performance.now();
    let lastYield = performance.now();
    while (this.running && this.session) {
      const wall = (performance.now() - start) / 1000;
      this.lag = wall - this.session.stats.simTime;
      if (this.session.stats.simTime < wall) {
        const ok = await this.session.step(this.sticks(0.004));
        if (!ok && this.session.stats.timeouts > 20) {
          this.error = 'board stopped answering';
          this.running = false;
          break;
        }
        // Give the renderer a turn now and then, however fast the link is.
        if (performance.now() - lastYield > 12) {
          await new Promise((r) => setTimeout(r, 0));
          lastYield = performance.now();
        }
      } else {
        await new Promise((r) => setTimeout(r, 1));
      }
    }
  }
}
