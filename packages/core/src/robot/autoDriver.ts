/**
 * The data-collection driver.
 *
 * It does not shoot by magic: it produces a GamepadState and hands it to the same
 * `BuiltinTeleOp` a human drives through, so every shot it logs went through the real
 * aim solver, the real readiness gate and the real feed timing. If the collected data
 * says the robot misses, the robot misses -- there is no privileged path here.
 *
 * The sample pattern sweeps the usable range band instead of firing from one spot,
 * because "it lands from 70 in" and "it lands" are different claims and only the sweep
 * can tell them apart. Poses come from a seeded RNG, so a run is repeatable.
 */
import { emptyGamepad } from '../physics/world.js';
import { clamp, DEG, RAD, M_TO_IN, wrapPi } from '../units.js';
import type { GamepadState, SensorFrame } from '../types.js';

export type Phase = 'drive' | 'settle' | 'shoot' | 'done';

export interface AutoPlan {
  /** How many shots to collect before stopping. */
  shots: number;
  /** Range band to sample, inches. */
  range_in: [number, number];
  /** How far either side of dead-on to sample the bearing, degrees. */
  bearing_deg: number;
  /** Fire while still rolling. The whole point of the motion lead is that this works. */
  onTheMove: boolean;
  seed: number;
}

export const defaultPlan = (): AutoPlan => ({
  shots: 40, range_in: [40, 90], bearing_deg: 70, onTheMove: false, seed: 1,
});

/**
 * Half-extents of the A-frame footprint, field inches, from the CAD: the foot bars sit at
 * x = +-24.08 in and span z = +-19.45 in. A robot is 17 in half-width, so anything inside
 * this box plus that is a pose it cannot reach.
 */
const AFRAME_X_IN = 24.08 + 10;
const AFRAME_Z_IN = 19.45 + 10;

/** mulberry32: 32 bits of state, good enough for sampling poses and exactly repeatable. */
function rng(seed: number): () => number {
  let a = seed >>> 0;
  return () => {
    a = (a + 0x6d2b79f5) >>> 0;
    let t = Math.imul(a ^ (a >>> 15), 1 | a);
    t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t;
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}

export class AutoDriver {
  phase: Phase = 'drive';
  /** Sample index, so the UI can show "shot 12 of 40". */
  shotsAsked = 0;
  /** Where this sample wants the robot, field inches. */
  target: [number, number] = [0, 0];
  note = 'starting';

  private rand: () => number;
  private since = 0;
  private lastShotCount = 0;

  constructor(
    private readonly plan: AutoPlan,
    /** The CELL mouth in world metres. Poses are sampled around it. */
    private readonly mouth: [number, number],
    /** Half the field, metres: samples are kept inside the wall. */
    private readonly halfWidth_m: number,
  ) {
    this.rand = rng(plan.seed);
    this.pick();
  }

  /**
   * Choose the next pose: a range and a bearing around the CELL.
   *
   * The requested range has to be the range the robot actually ends up at, or the range
   * bins in the report are a lie. Clamping a pose into the wall quietly shortens it, so
   * instead this searches the bearing fan for a pose that is inside the wall, clear of the
   * A-frame footprint, and at the range asked for. If the whole fan is unreachable at that
   * range -- which happens at the long end, because the field is only 141 in across -- the
   * range is walked in until something fits, and the report shows the range it really used.
   */
  private pick(): void {
    const [lo, hi] = this.plan.range_in;
    // Stratified, not uniform: sweeping the band in order fills every range bin evenly,
    // and an evenly filled bin table is the difference between a measurement and an anecdote.
    const f = this.plan.shots > 1 ? (this.shotsAsked % this.plan.shots) / (this.plan.shots - 1) : 0;
    const lim = this.halfWidth_m * M_TO_IN - 14;
    const mx = this.mouth[0] * M_TO_IN;
    const mz = this.mouth[1] * M_TO_IN;
    const face = mz >= 0 ? 1 : -1;   // the CELL faces +-Z; stand in front of it

    const reachable = (range: number, bear: number): [number, number] | null => {
      const a = (90 + bear) * DEG;
      const x = mx + range * Math.cos(a) * face;
      const z = mz + range * Math.sin(a) * face;
      if (Math.abs(x) > lim || Math.abs(z) > lim) return null;                      // off the field
      if (Math.abs(x) < AFRAME_X_IN && Math.abs(z) < AFRAME_Z_IN) return null;      // under the frame
      return [x, z];
    };

    for (let shrink = 0; shrink < 12; shrink++) {
      const range = (lo + (hi - lo) * f) - shrink * 6;
      if (range < 18) break;
      const ok: [number, number][] = [];
      for (let i = 0; i <= 24; i++) {
        const bear = -this.plan.bearing_deg + (2 * this.plan.bearing_deg * i) / 24;
        const q = reachable(range, bear);
        if (q) ok.push(q);
      }
      if (ok.length) {
        this.target = ok[Math.floor(this.rand() * ok.length)];
        return;
      }
    }
    // Nothing fits at any range: stand off the mouth by whatever the wall allows.
    this.target = [mx, Math.max(-lim, Math.min(lim, mz + 30 * face))];
  }

  /**
   * One frame of driving. `shotsTaken` is the world's own shot counter, which is how the
   * driver knows a ball actually left rather than assuming the command worked.
   */
  update(s: SensorFrame, dt: number, shotsTaken: number): GamepadState {
    const g = emptyGamepad();
    this.since += dt;

    if (this.phase === 'done' || this.shotsAsked >= this.plan.shots) {
      this.phase = 'done';
      this.note = `collected ${shotsTaken} shots`;
      return g;
    }

    const ex = this.target[0] - s.localizer.x;
    const ey = this.target[1] - s.localizer.y;
    const dist = Math.hypot(ex, ey);

    if (this.phase === 'drive') {
      this.note = `driving to sample ${this.shotsAsked + 1}/${this.plan.shots} (${dist.toFixed(0)} in away)`;
      // Field-relative error rotated into the robot's own frame, then straight onto the
      // sticks: the drivetrain is holonomic, so there is nothing to turn toward.
      const c = Math.cos(s.imu.yaw * DEG);
      const sn = Math.sin(s.imu.yaw * DEG);
      const fwd = ex * c + ey * sn;
      const left = -ex * sn + ey * c;
      // Taper inside 24 in so it arrives stopped instead of overshooting and hunting.
      const gain = clamp(dist / 24, 0.18, 1) / Math.max(1, dist);
      g.left_stick_y = clamp(-fwd * gain, -1, 1);
      g.left_stick_x = clamp(-left * gain, -1, 1);
      // Face the CELL, so the turret is not asked to sit on its end stop all run.
      const want = Math.atan2(this.mouth[1] * M_TO_IN - s.localizer.y, this.mouth[0] * M_TO_IN - s.localizer.x) * RAD;
      const yawErr = wrapPi((want - s.imu.yaw) * DEG) * RAD;
      g.right_stick_x = clamp(-yawErr * 0.02, -0.6, 0.6);
      if (dist < 4 || this.since > 8) {
        this.phase = this.plan.onTheMove ? 'shoot' : 'settle';
        this.since = 0;
      }
      return g;
    }

    if (this.phase === 'settle') {
      // Stop dead before firing, so a shot's error is the shooter's and not the chassis'.
      this.note = 'settling';
      if (Math.hypot(s.localizer.vx, s.localizer.vy) > 2 && this.since < 1.5) return g;
      this.phase = 'shoot';
      this.since = 0;
      this.lastShotCount = shotsTaken;
      return g;
    }

    // shoot: latch the fire toggle on the first frame, and wait for the world to say a ball
    // actually left. The gate can hold for a second while the wheel comes up; that wait is
    // real data, not a stall.
    this.note = `firing sample ${this.shotsAsked + 1}/${this.plan.shots}`;
    if (this.since < dt * 1.5) g.right_bumper = true;   // edge: toggles the fire latch on
    if (this.plan.onTheMove) {
      g.left_stick_y = -0.45;   // keep rolling: this is what the motion lead is for
    }
    if (shotsTaken > this.lastShotCount) {
      g.right_bumper = true;    // edge again: latch back off
      this.shotsAsked++;
      this.pick();
      this.phase = 'drive';
      this.since = 0;
    } else if (this.since > 6) {
      // Nothing left the muzzle in six seconds. Empty hopper or a gate that never opened;
      // either way, move on rather than freeze the run.
      this.note = 'shot timed out — hopper empty or the gate never opened';
      g.right_bumper = true;
      this.shotsAsked++;
      this.pick();
      this.phase = 'drive';
      this.since = 0;
    }
    return g;
  }
}
