/**
 * The shot table for a FIXED-SPEED shooter: hood angle against (range, radial velocity).
 *
 *   npm run tool -- tools/hoodtable.ts [--rpm 3400] [--step 4]
 *
 * The old table solves for the exit SPEED at each range, which puts the flywheel in the
 * control loop: it has to reach a new speed for every shot, it lags while it does, and its
 * speed can only be measured to about 107 rpm a count. tools/slew.ts and tools/apercheck.ts
 * price both -- 1.2 m/s^2 of trackable acceleration, and a +-60 rpm firing window worth
 * +-21 cm of range into a 22.5 cm pocket.
 *
 * This table holds the wheel at ONE speed for the whole match and solves for the HOOD, which
 * is a servo: commanded to a position, nothing to measure, nothing to chase. The robot's own
 * velocity stops being something to cancel and becomes part of the launch --
 *
 *     horizontal  S*cos(theta) + v        vertical  S*sin(theta)
 *
 * -- so the table gains an axis rather than a correction. For each (range, radial velocity) it
 * records the BAND of hood angles that thread the mouth, because the band's width is the
 * tolerance the robot actually has and its midpoint is the angle to command.
 */
import { writeFileSync, mkdirSync } from 'node:fs';
import params from '../config/params.json' with { type: 'json' };
import robotJson from '../config/robot.json' with { type: 'json' };
import { simulateShot, type Aperture } from '../packages/core/src/physics/ballistics.js';
import { DEG, inches, rpmToRadS } from '../packages/core/src/units.js';
import { loadEntry, mouthLips } from './shottable.js';
import type { Params, RobotSpec } from '../packages/core/src/types.js';

export interface HoodRow {
  range_in: number;
  v_mps: number;
  /** Band of hood angles that thread the mouth, and the one to command. */
  lo: number;
  mid: number;
  hi: number;
  /**
   * One sigma of hood-angle error from ALL of the launch scatter: the elevation scatter
   * directly, plus the wheel's speed scatter converted into the hood shift that moves the
   * ball the same way. One axis, so the run-time probability is one normal integral.
   */
  sigmaHood: number;
  /** Arrival, for the entry model and for reporting. */
  descentDeg: number;
  arrival_mps: number;
  pStay: number;
}

/** One trajectory at a given wheel speed, hood angle and robot radial velocity. */
function trialAt(
  S: number, hoodDeg: number, v: number, ap: Aperture, ballR: number, spinPerSpeed: number,
  p: Params, spec: RobotSpec,
): boolean {
  const L = launch(S, hoodDeg, v);
  if (!L) return false;
  const t = simulateShot(p, {
    from: [0, spec.turret.muzzleHeight_m, 0],
    azimuth: 0,
    elevation: L.elevDeg * DEG,
    speed: L.speed,
    radius: ballR,
    mass: p.ball.pollen.m_kg,
    spin: spinPerSpeed * S,
  }, ap.farRange, 1 / 480, [ap.nearRange, ap.farRange]);
  return t.gateHeights[0] > ap.nearHeight
    && Number.isFinite(t.gateHeights[1]) && t.gateHeights[1] < ap.farHeight
    && t.descentAngle > 10 * DEG;
}

/** Launch condition once the robot's own radial velocity is included. */
function launch(S: number, hoodDeg: number, v: number): { speed: number; elevDeg: number } | null {
  const hx = S * Math.cos(hoodDeg * DEG) + v;
  const hy = S * Math.sin(hoodDeg * DEG);
  if (hx <= 0.05) return null;                    // driving at the target faster than the ball
  return { speed: Math.hypot(hx, hy), elevDeg: (Math.atan2(hy, hx) * 180) / Math.PI };
}

export function buildHoodTable(p: Params, spec: RobotSpec, rpm: number, step = 4, vels: number[] = []): HoodRow[] {
  const f = spec.flywheel;
  const S = f.k * f.r_fly_m * rpmToRadS(rpm);
  const ballR = p.ball.pollen.d_m / 2;
  const spinPerSpeed = f.type === 'single' ? 1 / ballR : 0;
  const lips = mouthLips(p);
  const entry = loadEntry();
  const [hoodLo, hoodHi] = spec.hood.angleRange_deg;
  const V = vels.length ? vels : [-1.6, -1.2, -0.8, -0.4, 0, 0.4, 0.8, 1.2, 1.6];

  const rows: HoodRow[] = [];
  for (let range_in = 30; range_in <= 150; range_in += step) {
    const centreZ = (lips.near.z + lips.far.z) / 2 + inches(range_in);
    const muzzleZ = centreZ - spec.turret.muzzleOffset_m;
    const ap: Aperture = {
      nearRange: muzzleZ - lips.near.z,
      nearHeight: lips.near.y + ballR * (p.hive.lipClearanceFrac ?? 1),
      farRange: muzzleZ - lips.far.z,
      farHeight: lips.far.y - ballR * (p.hive.lipClearanceFrac ?? 1),
    };
    if (ap.nearRange <= 0) continue;

    // Bands per velocity first; the CHOICE between them comes after, because it depends on
    // the neighbours.
    const perV = new Map<number, { lo: number; hi: number; descentDeg: number; arrival_mps: number }[]>();
    for (const v of V) {
      // Walk the hood and collect EVERY contiguous band that threads, then choose between
      // them. There are usually two: a high lob and a flat drive, and taking whichever came
      // first put a 40 deg cell next to a 63 deg one in the same row. Interpolating across
      // that discontinuity would command an angle that solves nothing.
      const bands: { lo: number; hi: number; descentDeg: number; arrival_mps: number }[] = [];
      let lo = NaN;
      let hi = NaN;
      let best: { descentDeg: number; arrival_mps: number } | null = null;
      const closeBand = () => {
        if (!Number.isNaN(lo) && best) bands.push({ lo, hi, ...best });
        lo = NaN;
        hi = NaN;
        best = null;
      };
      // The band is measured at the NOMINAL speed. Requiring it to hold at S and S +- sigma
      // as well -- the intersection of three bands -- cut the mean width from 3.3 deg to 1.4
      // and is the wrong shape anyway: a shot one sigma off is unlikely, not forbidden. The
      // speed scatter is folded in below instead, as the amount it MOVES the band, which is
      // how the old solver handled elevation scatter and keeps everything on one axis.
      const trial = (hood: number, scale: number) => {
        const L = launch(S * scale, hood, v);
        if (!L) return null;
        return simulateShot(p, {
          from: [0, spec.turret.muzzleHeight_m, 0],
          azimuth: 0,
          elevation: L.elevDeg * DEG,
          speed: L.speed,
          radius: ballR,
          mass: p.ball.pollen.m_kg,
          // Spin comes from the wheel, which never changes speed -- so it is constant too.
          spin: spinPerSpeed * S * scale,
        }, ap.farRange, 1 / 480, [ap.nearRange, ap.farRange]);
      };
      for (let hood = hoodLo; hood <= hoodHi; hood += 0.25) {
        const t = trial(hood, 1);
        if (!t) continue;
        const ok = t.gateHeights[0] > ap.nearHeight
          && Number.isFinite(t.gateHeights[1]) && t.gateHeights[1] < ap.farHeight
          && t.descentAngle > 10 * DEG;
        if (ok) {
          if (Number.isNaN(lo)) lo = hood;
          hi = hood;
          best = { descentDeg: (t.descentAngle * 180) / Math.PI, arrival_mps: t.arrivalSpeed };
        } else {
          closeBand();
        }
      }
      closeBand();
      if (bands.length) perV.set(v, bands);
    }

    // STAY ON ONE BRANCH. A lob and a flat drive can both thread the mouth, and picking the
    // widest band per cell independently put a 40 deg cell next to a 63 deg one -- a surface
    // that cannot be interpolated, because halfway between two solutions is not a solution.
    // So anchor on the stationary shot, which is the one the robot takes most, and walk out
    // in both directions choosing the band closest to the neighbour already chosen.
    const chosen = new Map<number, { lo: number; hi: number; descentDeg: number; arrival_mps: number }>();
    const anchor = perV.get(0);
    if (!anchor?.length) continue;
    chosen.set(0, anchor.reduce((a, b) => (b.hi - b.lo > a.hi - a.lo ? b : a)));
    const walk = (order: number[]) => {
      let prevMid = (chosen.get(0)!.lo + chosen.get(0)!.hi) / 2;
      for (const v of order) {
        const bands = perV.get(v);
        if (!bands?.length) break;                 // a gap ends the run; do not jump over it
        const near = bands.reduce((a, b) =>
          Math.abs((b.lo + b.hi) / 2 - prevMid) < Math.abs((a.lo + a.hi) / 2 - prevMid) ? b : a);
        const mid = (near.lo + near.hi) / 2;
        // A step this large is a different branch, not the same shot moving.
        if (Math.abs(mid - prevMid) > 12) break;
        chosen.set(v, near);
        prevMid = mid;
      }
    };
    walk(V.filter((v) => v > 0).sort((a, b) => a - b));
    walk(V.filter((v) => v < 0).sort((a, b) => b - a));

    for (const [v, band] of [...chosen.entries()].sort((a, b) => a[0] - b[0])) {
      // How much a one-sigma wheel-speed error shifts the band, expressed in hood degrees,
      // so the run-time model is a single normal integral over ONE axis. Combined in
      // quadrature with the launch elevation scatter, which acts on the same axis directly.
      const mid0 = (band.lo + band.hi) / 2;
      let shift = 0;
      for (let hood = hoodLo; hood <= hoodHi; hood += 0.25) {
        const x = trialAt(S * (1 + f.scatter.speedFrac), hood, v, ap, ballR, spinPerSpeed, p, spec);
        if (x) { shift = Math.abs(hood - mid0); break; }
      }
      rows.push({
        range_in,
        v_mps: v,
        lo: band.lo,
        mid: mid0,
        hi: band.hi,
        sigmaHood: Math.hypot(f.scatter.angle_deg, Math.min(shift, 10)),
        descentDeg: band.descentDeg,
        arrival_mps: band.arrival_mps,
        pStay: entry ? entry.lookup(band.arrival_mps, band.descentDeg) : 1,
      });
    }
  }
  return rows;
}

export function toCsv(rows: HoodRow[], rpm: number): string {
  const head = `# fixed_rpm=${rpm}\nrange_in,v_mps,hoodLo,hoodMid,hoodHi,sigmaHood,descentDeg,arrival_mps,pStay`;
  return [head, ...rows.map((r) => [
    r.range_in.toFixed(1), r.v_mps.toFixed(2), r.lo.toFixed(2), r.mid.toFixed(2), r.hi.toFixed(2), r.sigmaHood.toFixed(3),
    r.descentDeg.toFixed(2), r.arrival_mps.toFixed(3), r.pStay.toFixed(4),
  ].join(','))].join('\n') + '\n';
}

export async function main(argv: string[] = []): Promise<void> {
  const num = (k: string, d: number) => {
    const i = argv.indexOf(`--${k}`);
    return i >= 0 ? Number(argv[i + 1]) : d;
  };
  const rpm = num('rpm', 3400);
  const p = params as unknown as Params;
  const spec = robotJson as unknown as RobotSpec;
  const rows = buildHoodTable(p, spec, rpm, num('step', 4));

  const dir = new URL('../java/teamcode/assets/', import.meta.url);
  mkdirSync(dir, { recursive: true });
  writeFileSync(new URL('hoodtable.csv', dir), toCsv(rows, rpm));

  const S = spec.flywheel.k * spec.flywheel.r_fly_m * rpmToRadS(rpm);
  console.log(`FIXED-SPEED HOOD TABLE — ${rpm} rpm = ${S.toFixed(2)} m/s exit, constant all match`);
  console.log('');
  const vels = [...new Set(rows.map((r) => r.v_mps))].sort((a, b) => a - b);
  const ranges = [...new Set(rows.map((r) => r.range_in))].sort((a, b) => a - b);
  console.log('  hood MIDPOINT, deg. Columns are radial velocity, + closing.');
  console.log(`    range  ${vels.map((v) => `${v > 0 ? '+' : ''}${v.toFixed(1)}`.padStart(6)).join('')}`);
  for (const r of ranges.filter((_, i) => i % 3 === 0)) {
    const cells = vels.map((v) => {
      const row = rows.find((x) => x.range_in === r && x.v_mps === v);
      return row ? row.mid.toFixed(0).padStart(6) : '    --';
    });
    console.log(`    ${(r * 0.0254).toFixed(2)} m${cells.join('')}`);
  }
  console.log('');
  const widths = rows.map((r) => r.hi - r.lo);
  const mean = (a: number[]) => a.reduce((x, y) => x + y, 0) / a.length;
  console.log(`  ${rows.length} solutions across ${ranges.length} ranges x ${vels.length} velocities`);
  console.log(`  hood band width: ${Math.min(...widths).toFixed(1)} to ${Math.max(...widths).toFixed(1)} deg, mean ${mean(widths).toFixed(1)}`);
  console.log(`  sigma from all launch scatter: mean ${mean(rows.map((r) => r.sigmaHood)).toFixed(2)} deg`);
  console.log(`  -- the band IS the tolerance. Launch elevation scatter is ${spec.flywheel.scatter.angle_deg} deg 1 sigma,`);
  console.log('     so a band several times that is a shot that survives its own noise.');
  console.log('');
  console.log('  wrote java/teamcode/assets/hoodtable.csv');
}
