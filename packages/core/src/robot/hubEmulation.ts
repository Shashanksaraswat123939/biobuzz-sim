/**
 * "Feels like a Control Hub" lives here, separate from the true physics so it can be
 * switched off for pure-physics experiments.
 *
 *  - the hub's built-in velocity PIDF for RUN_USING_ENCODER
 *  - RUN_TO_POSITION as a position P feeding that velocity loop
 *  - encoder velocity quantised over a 20 ms window, like the hub's
 *
 * The /32767 scaling is the hub's own: it is why the stock F of about 12 gives full power
 * at a 312 RPM motor's top speed (12 * 2796 ticks/s / 32767 = 1.02).
 */
import { clamp } from '../units.js';
import type { MotorCmd, RobotSpec } from '../types.js';

const HUB_SCALE = 32767;

export class HubMotorLoop {
  private iErr = 0;
  private lastErr = 0;
  private tickHistory: { t: number; ticks: number }[] = [];
  /** Encoder value the brain sees, after STOP_AND_RESET_ENCODER offsets. */
  private offset = 0;
  reportedVel = 0;
  targetVel = 0;
  busy = false;

  constructor(
    private readonly hub: RobotSpec['hub'],
    private readonly maxTicksPerSec: number,
  ) {}

  /** Raw ticks -> the value the brain reads. */
  position(rawTicks: number): number {
    return rawTicks - this.offset;
  }

  /**
   * Turn one command into a motor duty in [-1, 1].
   * `rawTicks` and `velTicks` are ground truth from the physics.
   */
  duty(cmd: MotorCmd | undefined, rawTicks: number, velTicks: number, dt: number): number {
    if (!cmd) return 0;
    if (cmd.reset) {
      this.offset = rawTicks;
      this.iErr = 0;
    }
    if (cmd.mode === 'STOP_AND_RESET_ENCODER') {
      this.offset = rawTicks;
      this.iErr = 0;
      this.targetVel = 0;
      this.busy = false;
      return 0;
    }
    if (cmd.mode === 'RUN_WITHOUT_ENCODER') {
      this.iErr = 0;
      this.targetVel = 0;
      this.busy = false;
      return clamp(cmd.power ?? 0, -1, 1);
    }

    if (cmd.mode === 'RUN_TO_POSITION') {
      const err = (cmd.target ?? 0) - this.position(rawTicks);
      const cap = Math.abs(cmd.power ?? 1) * this.maxTicksPerSec;
      // The hub turns position error into a velocity target; 5 /s is close to its feel.
      this.targetVel = clamp(5 * err, -cap, cap);
      this.busy = Math.abs(err) > 10;
    } else {
      // RUN_USING_ENCODER: an explicit velocity, or a power read as a fraction of top speed.
      this.targetVel = cmd.velocity !== undefined ? cmd.velocity : clamp(cmd.power ?? 0, -1, 1) * this.maxTicksPerSec;
      this.busy = false;
    }

    const pid = this.hub.velocityPid;
    const err = this.targetVel - velTicks;
    const dErr = dt > 0 ? (err - this.lastErr) / dt : 0;
    this.lastErr = err;

    const iMax = HUB_SCALE / Math.max(pid.i, 1e-6);
    const candidate = clamp(this.iErr + err * dt, -iMax, iMax);
    const fixed = pid.p * err + pid.d * dErr + pid.f * this.targetVel;
    const out = (fixed + pid.i * candidate) / HUB_SCALE;

    // Conditional integration. Without it the integral winds up through the whole spin-up,
    // and the flywheel then sits 400 RPM above target for ten seconds -- long enough that
    // the readiness gate never opens and nothing can be fired.
    const saturated = out > 1 || out < -1;
    const unwinding = (out > 1 && err < 0) || (out < -1 && err > 0);
    if (!saturated || unwinding) this.iErr = candidate;

    return clamp(out, -1, 1);
  }

  /**
   * The hub reports velocity as ticks counted over a window, not an instantaneous value.
   * Code that PID-tunes against this sees the same staircase it will see on the robot.
   */
  updateReportedVelocity(t: number, rawTicks: number): void {
    // A QUADRATURE ENCODER COUNTS WHOLE TRANSITIONS. Storing the real-valued tick position
    // made the hub's velocity estimate about fifty times better than the hardware's: the
    // flywheel runs 28 ticks a rev direct-driven, so at 2800 rpm a 20 ms window sees 26
    // counts and one count is worth 107 rpm. That is the resolution a team actually has, and
    // without this the sim will happily certify a firing window of 15 rpm that no Control Hub
    // can measure. tools/apercheck.ts prices 107 rpm at 15 in of range.
    this.tickHistory.push({ t, ticks: Math.floor(rawTicks) });
    const window = this.hub.encoderVelocityWindowMs / 1000;
    while (this.tickHistory.length > 2 && t - this.tickHistory[0].t > window) this.tickHistory.shift();
    const first = this.tickHistory[0];
    const dt = t - first.t;
    if (dt > 1e-6) this.reportedVel = Math.round((rawTicks - first.ticks) / dt);
  }

  reset(): void {
    this.iErr = 0;
    this.lastErr = 0;
    this.offset = 0;
    this.targetVel = 0;
    this.reportedVel = 0;
    this.busy = false;
    this.tickHistory.length = 0;
  }
}
