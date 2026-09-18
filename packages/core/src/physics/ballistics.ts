/**
 * The shot solver. Integrates a ball with the same drag/Magnus model the world uses
 * (aero.ts), so the table the robot aims by and the flight it actually gets come from one
 * place. PLAN.md §10.3: this is the artefact that makes the hub port cheap -- the Control
 * Hub only interpolates a CSV, it never solves anything.
 */
import { aeroForce } from './aero.js';
import { DEG, radSToRpm, rpmToRadS } from '../units.js';
import type { Params, Vec3 } from '../types.js';

export interface ShotInput {
  /** Muzzle position, world metres. */
  from: Vec3;
  /** Horizontal bearing of the shot (world yaw, radians). */
  azimuth: number;
  elevation: number;
  speed: number;
  radius: number;
  mass: number;
  /** Backspin magnitude, rad/s. 0 for a dual-wheel shooter. */
  spin: number;
}

export interface Trajectory {
  points: Vec3[];
  apex: number;
  /** Height at the moment the ball reached the requested horizontal range. */
  heightAtRange: number;
  /** Downward flight-path angle there, radians. Steeper drops into the pocket better. */
  descentAngle: number;
  /**
   * Speed at that moment, m/s. Threading the mouth is not the same as staying in it: a ball
   * that arrives fast keeps most of that speed after the first bounce off the pocket floor
   * and can come back out. This is what makes a flat fast shot and a steep slow one score
   * differently even when both pass through the same hole.
   */
  arrivalSpeed: number;
  reachedRange: boolean;
  flightTime: number;
  /** Height at each range passed in `gates`, NaN if never reached. */
  gateHeights: number[];
}

const MAX_T = 4.0;

export function simulateShot(params: Params, s: ShotInput, horizRange: number, dt = 1 / 480, gates: number[] = []): Trajectory {
  const ca = Math.cos(s.elevation);
  const dir: Vec3 = [Math.sin(s.azimuth) * ca, Math.sin(s.elevation), Math.cos(s.azimuth) * ca];
  const p: Vec3 = [...s.from];
  const v: Vec3 = [dir[0] * s.speed, dir[1] * s.speed, dir[2] * s.speed];
  const n = Math.hypot(dir[0], dir[2]) || 1;
  const hx = dir[0] / n;
  const hz = dir[2] / n;
  const w: Vec3 = [-hz * s.spin, 0, hx * s.spin];

  const points: Vec3[] = [[...p] as Vec3];
  const gateHeights: number[] = gates.map(() => NaN);
  let apex = p[1];
  let travelled = 0;
  let heightAtRange = NaN;
  let descentAngle = 0;
  let arrivalSpeed = 0;
  let reachedRange = false;
  let t = 0;

  for (; t < MAX_T; t += dt) {
    const { force, spinScale } = aeroForce(params.ball, params.env.rho, s.radius, v, w, dt);
    const ax = force[0] / s.mass;
    const ay = force[1] / s.mass - params.env.g;
    const az = force[2] / s.mass;
    const prev: Vec3 = [...p];
    v[0] += ax * dt;
    v[1] += ay * dt;
    v[2] += az * dt;
    p[0] += v[0] * dt;
    p[1] += v[1] * dt;
    p[2] += v[2] * dt;
    w[0] *= spinScale;
    w[1] *= spinScale;
    w[2] *= spinScale;

    // Downrange distance is the SIGNED projection onto the shot's initial heading, not the
    // horizontal path length. At steep hood angles the Magnus force points backwards along
    // the flight and actually reverses the horizontal motion; path length keeps growing
    // through that, so gates fired while the ball was still climbing far above the target.
    const before = travelled;
    travelled = (p[0] - s.from[0]) * hx + (p[2] - s.from[2]) * hz;
    const step = travelled - before;
    if (!reachedRange && travelled >= horizRange && step > 0) {
      const f = step > 1e-9 ? (horizRange - before) / step : 0;
      heightAtRange = prev[1] + f * (p[1] - prev[1]);
      descentAngle = Math.atan2(-v[1], Math.hypot(v[0], v[2]));
      arrivalSpeed = Math.hypot(v[0], v[1], v[2]);
      reachedRange = true;
    }
    for (let gi = 0; gi < gates.length; gi++) {
      if (Number.isNaN(gateHeights[gi]) && travelled >= gates[gi] && step > 0) {
        const gf = step > 1e-9 ? (gates[gi] - before) / step : 0;
        gateHeights[gi] = prev[1] + gf * (p[1] - prev[1]);
      }
    }
    apex = Math.max(apex, p[1]);
    if (points.length < 400) points.push([...p] as Vec3);
    if (p[1] < 0 && v[1] < 0) break;
  }

  return { points, apex, heightAtRange, descentAngle, arrivalSpeed, reachedRange, flightTime: t, gateHeights };
}

/** The CELL mouth as the aperture it is: a near lip to clear and a far lip to stay under. */
export interface Aperture {
  /** Horizontal distance from the muzzle to the near lip, and the height to clear there. */
  nearRange: number;
  nearHeight: number;
  /** Horizontal distance to the far lip, and the height to be under by then. */
  farRange: number;
  farHeight: number;
}

/**
 * For one hood angle, the band of exit speeds that actually put a ball through the mouth.
 * Aiming at the mouth's centre point is not enough: a ball can pass through that point
 * while still climbing and clip the near lip, which is exactly what it did before this
 * existed. The band's width is the margin -- how much RPM error the shot tolerates.
 */
export function speedBand(
  params: Params,
  base: Omit<ShotInput, 'speed' | 'spin'>,
  ap: Aperture,
  spinPerSpeed: number,
  lo = 3,
  hi = 30,
  steps = 216,
): { lo: number; hi: number } | null {
  const ok = (speed: number): boolean => {
    const t = simulateShot(params, { ...base, speed, spin: spinPerSpeed * speed }, ap.farRange, 1 / 480, [ap.nearRange, ap.farRange]);
    const [hNear, hFar] = t.gateHeights;
    if (!Number.isFinite(hNear) || !Number.isFinite(hFar)) return false;
    return hNear > ap.nearHeight && hFar < ap.farHeight;
  };

  let first = -1;
  let last = -1;
  for (let i = 0; i <= steps; i++) {
    const v = lo + ((hi - lo) * i) / steps;
    if (ok(v)) {
      if (first < 0) first = v;
      last = v;
    } else if (first >= 0) {
      break; // the feasible set is one contiguous band
    }
  }
  return first < 0 ? null : { lo: first, hi: last };
}

export interface ShotTableRow {
  /**
   * Half-width of the mouth ACROSS the shot line, m, less the ball radius. Field geometry,
   * not a solver output, so it is filled in by the table builder -- but it travels in the
   * table because the hub reads nothing else.
   */
  halfLat_m?: number;
  range_in: number;
  hoodPos: number;
  hoodDeg: number;
  rpm: number;
  /** Fractional speed error still landing inside the mouth. Bigger is more forgiving. */
  margin: number;
  descentDeg: number;
  /** Highest point of the arc above the floor, m. The thing "how tall is the lob" means. */
  apex_m: number;
  /** Muzzle to mouth, seconds. A long lob is a slow cycle and a big wind target. */
  flight_s: number;
  /** Speed the ball is doing when it reaches the mouth, m/s. */
  arrival_mps: number;
  /** The exit-speed band that threads the mouth, m/s. `margin` is this, normalised. */
  speedLo: number;
  speedHi: number;
  /**
   * One sigma of exit-speed error from ALL launch scatter, m/s -- the speed scatter itself
   * plus the elevation scatter converted into the speed error that would move the ball the
   * same distance. Having it here is what lets the robot turn a band into a PROBABILITY at
   * run time instead of guessing from a margin figure.
   */
  sigmaSpeed: number;
  /**
   * Measured fraction of balls arriving like this that STAY in the CELL (tools/entrycheck).
   * 1 when no entry model was supplied, which is the old behaviour.
   */
  pStay: number;
}

/** Exit speed -> flywheel RPM, using the same k and radius the world launches with. */
export const speedToRpm = (speed: number, k: number, rFly: number) => radSToRpm(speed / (k * rFly));
export const rpmToSpeed = (rpm: number, k: number, rFly: number) => k * rFly * rpmToRadS(rpm);

/** Try every hood angle at one range and keep the one with the widest speed band. */
export function bestShot(
  params: Params,
  opts: {
    muzzle: Vec3;
    azimuth: number;
    aperture: Aperture;
    radius: number;
    mass: number;
    hoodRange: [number, number];
    hoodSteps: number;
    spinPerSpeed: number;
    k: number;
    rFly: number;
    maxRpm: number;
    range_in: number;
    /**
     * Reject solutions that arrive flatter than this (degrees below horizontal).
     * A ball that threads the mouth while still climbing hits the far wall and rebounds
     * straight back out; only a descending one drops in and stays. It also keeps the table
     * on one branch, so interpolating between two rows means something.
     */
    minDescentDeg?: number;
    /**
     * Hood position chosen at the previous range. Several hood angles usually tie on
     * margin; preferring the neighbouring one keeps the table smooth, which matters
     * because the robot interpolates between rows.
     */
    preferHoodPos?: number;
    /**
     * STAY ON ONE BRANCH. A lob and a flat drive can both thread the mouth, and picking the
     * best-scoring one at each range independently put 60 deg / 2570 rpm at 74 in next to
     * 70.7 deg / 2901 rpm at 78 in, then 60 at 94, 66.7 at 98 and 50.7 at 102. The robot
     * INTERPOLATES between rows, so crossing 74-78 in it was handed 65 deg at 2735 rpm --
     * halfway between two solutions, which is not a solution. `preferHoodPos` was meant to
     * hold the branch and could not: 0.04 per unit of hood travel against a score of order 1.
     *
     * With this set, a row whose hood is more than this many degrees from the previous
     * row's is only taken when nothing nearer threads at all. The walk anchors on the first
     * range, the way tools/hoodtable.ts already does for the fixed-speed table.
     */
    maxHoodJumpDeg?: number;
    /** Reject arcs that go higher than this (m). Venue ceilings and sanity. */
    maxApex_m?: number;
    /** Reject arcs that take longer than this (s). A 2.5 s lob is useless at a 1.5 s cycle. */
    maxFlight_s?: number;
    /**
     * Fraction of balls arriving at (speed, descent) that stay in the CELL. Measured by
     * tools/entrycheck.ts.
     *
     * Without this, `bestShot` scores a candidate purely on how much speed error still
     * THREADS the mouth -- and threading the mouth is not scoring. That blind spot is why
     * it picked 81 degree lobs arriving at 6.2 m/s, an arrival the measurement says stays
     * in only 25% of the time, over flatter ones that stay in 80-96% of the time.
     */
    entryRate?: (arrival_mps: number, descentDeg: number) => number;
    /** 1 sigma of exit-speed scatter as a fraction, and of elevation scatter in degrees. */
    scatter?: { speedFrac: number; angle_deg: number };
    /**
     * What to maximise.
     *
     * 'margin' is the original: the speed band's half-width divided by the band centre.
     * 'pLand' is P(threads the mouth) x P(stays in). Kept selectable so the two can be
     * compared honestly -- leaving 'margin' out entirely made the A/B in tools/gatecal.ts
     * compare pThread against pThread x pStay and report a 0.01 m difference in apex,
     * because BOTH arms had already dropped the thing that made the old table tall.
     */
    objective?: 'margin' | 'pLand';
  },
): ShotTableRow | null {
  let best: ShotTableRow | null = null;
  let bestScore = -Infinity;
  const minDescent = opts.minDescentDeg ?? -90;
  const prevHoodDeg = opts.preferHoodPos === undefined
    ? undefined
    : opts.hoodRange[0] + opts.preferHoodPos * (opts.hoodRange[1] - opts.hoodRange[0]);
  const jump = opts.maxHoodJumpDeg;

  for (let i = 0; i < opts.hoodSteps; i++) {
    const frac = opts.hoodSteps === 1 ? 0.5 : i / (opts.hoodSteps - 1);
    const hoodDeg = opts.hoodRange[0] + frac * (opts.hoodRange[1] - opts.hoodRange[0]);
    // Out of reach of the branch we are on: only worth having if the branch has run out,
    // which is settled after the loop by the fallback below.
    if (jump !== undefined && prevHoodDeg !== undefined && Math.abs(hoodDeg - prevHoodDeg) > jump) continue;
    const base = { from: opts.muzzle, azimuth: opts.azimuth, elevation: hoodDeg * DEG, radius: opts.radius, mass: opts.mass };
    const band = speedBand(params, base, opts.aperture, opts.spinPerSpeed);
    if (!band) continue;

    const mid = (band.lo + band.hi) / 2;
    const rpm = speedToRpm(mid, opts.k, opts.rFly);
    if (rpm > opts.maxRpm) continue;
    const margin = (band.hi - band.lo) / 2 / mid;

    const traj = simulateShot(params, { ...base, speed: mid, spin: opts.spinPerSpeed * mid }, opts.aperture.farRange);
    const descentDeg = (traj.descentAngle * 180) / Math.PI;
    if (descentDeg < minDescent) continue;
    if (opts.maxApex_m !== undefined && traj.apex > opts.maxApex_m) continue;
    if (opts.maxFlight_s !== undefined && traj.flightTime > opts.maxFlight_s) continue;

    // How much exit-speed error all the launch scatter is worth, together.
    //
    // Elevation scatter does not act on the same axis as speed scatter, so it cannot simply
    // be added: it is converted into the speed error that would shift the ball the same
    // way, by re-solving the band one sigma off in elevation and seeing how far the band
    // moved. That keeps everything on one axis, which is what makes a run-time probability
    // a single normal integral rather than a two-dimensional one.
    const sc = opts.scatter ?? { speedFrac: 0, angle_deg: 0 };
    let sigmaSpeed = sc.speedFrac * mid;
    if (sc.angle_deg > 0) {
      const tilted = speedBand(
        params,
        { ...base, elevation: (hoodDeg + sc.angle_deg) * DEG },
        opts.aperture,
        opts.spinPerSpeed,
      );
      if (tilted) {
        const shift = Math.abs((tilted.lo + tilted.hi) / 2 - mid);
        sigmaSpeed = Math.hypot(sigmaSpeed, shift);
      }
    }

    const pStay = opts.entryRate ? opts.entryRate(traj.arrivalSpeed, descentDeg) : 1;

    const row: ShotTableRow = {
      range_in: opts.range_in, hoodPos: frac, hoodDeg, rpm, margin, descentDeg,
      apex_m: traj.apex, flight_s: traj.flightTime,
      arrival_mps: traj.arrivalSpeed,
      speedLo: band.lo, speedHi: band.hi, sigmaSpeed, pStay,
    };
    const continuity = opts.preferHoodPos === undefined ? 0 : Math.abs(frac - opts.preferHoodPos) * 0.04;
    // THE OBJECTIVE. Probability of threading the mouth given the real scatter, times the
    // measured probability of staying in once it does. Previously this was `margin` alone,
    // which is the first factor expressed in the wrong units and the second ignored.
    const score = (opts.objective === 'margin'
      ? margin
      : pThread(band.lo, band.hi, mid, sigmaSpeed) * pStay) - continuity;
    if (score > bestScore) {
      bestScore = score;
      best = row;
    }
  }
  // The branch ran out at this range: take whatever threads, and the next row re-anchors
  // on it. A gap in the table would be honest too, but a row the robot can use beats a
  // hole it cannot, and the jump is reported by the table printer.
  if (!best && jump !== undefined && prevHoodDeg !== undefined) {
    return bestShot(params, { ...opts, maxHoodJumpDeg: undefined, preferHoodPos: undefined });
  }
  return best;
}


/** Standard normal CDF, Abramowitz & Stegun 26.2.17. Accurate to about 1e-7. */
export function normalCdf(z: number): number {
  const t = 1 / (1 + 0.2316419 * Math.abs(z));
  const d = 0.3989422804014327 * Math.exp(-0.5 * z * z);
  const p = d * t * (0.319381530 + t * (-0.356563782 + t * (1.781477937 + t * (-1.821255978 + t * 1.330274429))));
  return z >= 0 ? 1 - p : p;
}

/**
 * Probability that the exit speed lands inside the band that threads the mouth, given the
 * shot is AIMED at `mean` with one sigma of `sigma`.
 *
 * `mean` is the speed the wheel is actually doing, which is not the same as the speed the
 * table asked for -- that difference is exactly what the readiness gate used to test with a
 * flat RPM tolerance, and expressing it as a probability instead is what lets the robot ask
 * "will this one land?" rather than "is the wheel close enough?".
 */
export function pThread(lo: number, hi: number, mean: number, sigma: number): number {
  if (!(sigma > 0)) return mean >= lo && mean <= hi ? 1 : 0;
  return Math.max(0, normalCdf((hi - mean) / sigma) - normalCdf((lo - mean) / sigma));
}
