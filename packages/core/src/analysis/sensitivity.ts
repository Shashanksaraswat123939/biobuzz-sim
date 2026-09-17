/**
 * What actually moves the shot, and by how much.
 *
 * Every number in `config/` is either measured or a guess, and a guess with a big
 * derivative is a shot that misses. This walks each knob, nudges it, re-integrates the
 * SAME ballistic model the world launches with, and reports inches of movement at the
 * target. Nothing here is estimated analytically -- it is the real integrator both times,
 * so the answer cannot drift from what the simulator does.
 *
 * The error budget on top of it is the point: each knob carries a 1-sigma uncertainty, so
 * (derivative x sigma) is the inches of scatter that knob is responsible for. Square them,
 * add, and you have a predicted group size plus the share each knob owns. That tells you
 * which number to go and measure next.
 */
import { simulateShot, type ShotInput } from '../physics/ballistics.js';
import { M_TO_IN, DEG, RAD } from '../units.js';
import type { Params } from '../types.js';

export interface Knob {
  key: string;
  label: string;
  unit: string;
  /** Nudge used for the derivative, in the knob's own unit. */
  step: number;
  /** 1-sigma uncertainty, same unit. Where it comes from is in `why`. */
  sigma: number;
  why: string;
  /** Read the knob's current value out of the shot being analysed. */
  get: (p: Params, s: ShotInput) => number;
  /** Write a new value. Both arguments are private clones. */
  set: (p: Params, s: ShotInput, v: number) => void;
}

export interface Row extends Knob {
  value: number;
  /** Inches the ball moves downrange per `step` of this knob. */
  perStep_in: number;
  /** Inches of 1-sigma scatter this knob contributes. */
  contrib_in: number;
  /** Fraction of the predicted variance this knob owns, 0..1. */
  share: number;
}

export interface Prediction {
  /** Where the shot lands now, inches downrange of the muzzle. */
  reach_in: number;
  /** Where it needs to land. */
  target_in: number;
  /** Signed: positive is long. */
  bias_in: number;
  /** Predicted 1-sigma group size, inches, from the knobs below. */
  spread_in: number;
  rows: Row[];
}

/**
 * Horizontal distance at which the ball comes back DOWN through `targetY`.
 * That is where it arrives, which is the only thing the CELL cares about. A ball still
 * climbing through that height has not arrived; it is on its way to the far wall.
 */
export function reachAt(params: Params, shot: ShotInput, targetY: number): number {
  // Sampled through the integrator's GATES, not its point list: simulateShot caps `points`
  // at 400 for the renderer's sake, which at 1/480 is only 0.8 s -- a lofted shot has not
  // even reached apex by then, so walking the points never finds the descent. Gates are
  // recorded for the whole flight regardless of that cap.
  const N = 160;
  const maxR = 6.5; // metres; the field diagonal is 5.0
  const gates = Array.from({ length: N }, (_, i) => ((i + 1) * maxR) / N);
  const h = simulateShot(params, shot, maxR, 1 / 480, gates).gateHeights;
  for (let i = 1; i < N; i++) {
    if (!Number.isFinite(h[i - 1]) || !Number.isFinite(h[i])) continue;
    if (h[i - 1] >= targetY && h[i] < targetY) {
      const f = (h[i - 1] - targetY) / (h[i - 1] - h[i]);
      return (gates[i - 1] + (gates[i] - gates[i - 1]) * f) * M_TO_IN;
    }
  }
  return NaN;
}

export interface KnobSpec {
  tolRpm: number;
  hoodSteps: number;
  scatterDeg: number;
  scatterSpeedFrac: number;
  k: number;
  rFly: number;
}

/**
 * The knobs, in the order they are worth arguing about.
 *
 * `sigma` is where the honesty lives. Flywheel speed, hood angle and launch scatter have
 * sigmas we can defend from `config/robot.json` (the controller's own tolerance, the
 * servo's resolution, the measured scatter). The air and ball constants are labelled
 * guesses in `config/params.json`, so their sigma is a stated fraction and `why` says so
 * -- a big share on one of those means "go and measure it", not "the shot is broken".
 */
export function knobs(spec: KnobSpec): Knob[] {
  const toSpeed = (rpm: number) => spec.k * spec.rFly * ((rpm * 2 * Math.PI) / 60);
  const toRpm = (speed: number) => speed / (spec.k * spec.rFly) / ((2 * Math.PI) / 60);
  return [
    {
      key: 'rpm', label: 'Flywheel speed', unit: 'rpm', step: 100,
      sigma: spec.tolRpm,
      why: 'robot.json flywheel.tolRpm: the band the hub velocity loop is allowed to sit in.',
      get: (_p, s) => toRpm(s.speed),
      set: (_p, s, v) => { s.speed = toSpeed(v); },
    },
    {
      key: 'hood', label: 'Hood angle', unit: 'deg', step: 1,
      sigma: 90 / spec.hoodSteps / 2,
      why: 'half a servo step across the hood’s usable travel.',
      get: (_p, s) => s.elevation * RAD,
      set: (_p, s, v) => { s.elevation = v * DEG; },
    },
    {
      key: 'scatterElev', label: 'Launch scatter, elevation', unit: 'deg', step: 1,
      sigma: spec.scatterDeg,
      why: 'robot.json flywheel.scatter.angle_deg: compression varies ball to ball.',
      get: () => 0,
      set: (_p, s, v) => { s.elevation += v * DEG; },
    },
    {
      key: 'scatterSpeed', label: 'Launch scatter, speed', unit: '%', step: 1,
      sigma: spec.scatterSpeedFrac * 100,
      why: 'robot.json flywheel.scatter.speedFrac: grip on the ball varies shot to shot.',
      get: () => 0,
      set: (_p, s, v) => { s.speed *= 1 + v / 100; },
    },
    {
      key: 'mass', label: 'Ball mass', unit: 'g', step: 2,
      sigma: 3,
      why: 'ball.pollen.m_kg: about 3 g of spread across a bag of game balls.',
      get: (_p, s) => s.mass * 1000,
      set: (_p, s, v) => { s.mass = v / 1000; },
    },
    {
      key: 'Cd', label: 'Drag coefficient', unit: '', step: 0.05,
      sigma: 0.1,
      why: 'params.json ball.Cd is a GUESS. A 26-hole sphere is not a smooth one.',
      get: (p) => p.ball.Cd,
      set: (p, _s, v) => { p.ball.Cd = v; },
    },
    {
      key: 'clSlope', label: 'Magnus slope', unit: '', step: 0.05,
      sigma: 0.1,
      why: 'params.json ball.clSlope is a GUESS. Backspin lift on a holed ball is unmeasured.',
      get: (p) => p.ball.clSlope,
      set: (p, _s, v) => { p.ball.clSlope = v; },
    },
    {
      key: 'rho', label: 'Air density', unit: 'kg/m3', step: 0.02,
      sigma: 0.04,
      why: 'params.json env.rho: venue altitude and temperature move this a few percent.',
      get: (p) => p.env.rho,
      set: (p, _s, v) => { p.env.rho = v; },
    },
    {
      key: 'muzzleY', label: 'Muzzle height', unit: 'in', step: 0.5,
      sigma: 0.5,
      why: 'how far the chassis squats on its wheels as the battery sags and the hopper fills.',
      get: (_p, s) => s.from[1] * M_TO_IN,
      set: (_p, s, v) => { s.from[1] = v / M_TO_IN; },
    },
  ];
}

/**
 * Nudge every knob, re-integrate, and rank them.
 * `target_in` is the horizontal distance the ball has to cover to reach the CELL mouth.
 */
export function predict(params: Params, shot: ShotInput, targetY_m: number, target_in: number, ks: Knob[]): Prediction {
  const base = reachAt(params, shot, targetY_m);
  const rows: Row[] = [];
  for (const k of ks) {
    // Central difference: one-sided is wrong by half the curvature, and a lofted shot is
    // very curved near its solution.
    const at = (d: number): number => {
      const p = structuredClone(params);
      const s: ShotInput = { ...shot, from: [shot.from[0], shot.from[1], shot.from[2]] };
      k.set(p, s, k.get(p, s) + d);
      return reachAt(p, s, targetY_m);
    };
    const up = at(k.step / 2);
    const dn = at(-k.step / 2);
    const per = Number.isFinite(up) && Number.isFinite(dn) ? up - dn : NaN;
    const contrib = Number.isFinite(per) ? Math.abs((per / k.step) * k.sigma) : 0;
    rows.push({ ...k, value: k.get(params, shot), perStep_in: per, contrib_in: contrib, share: 0 });
  }
  const variance = rows.reduce((a, r) => a + r.contrib_in ** 2, 0);
  for (const r of rows) r.share = variance > 0 ? r.contrib_in ** 2 / variance : 0;
  rows.sort((a, b) => b.contrib_in - a.contrib_in);
  return { reach_in: base, target_in, bias_in: base - target_in, spread_in: Math.sqrt(variance), rows };
}
