/**
 * The shooter lands 10 inches long into a 6 inch hole. Whose fault?
 *
 *   npm run tool -- tools/aimbias.ts [--shots 12] [--seeds 4]
 *
 * tools/missmix.ts found that misses are not bounce-outs -- 29% of all shots arrive BEYOND
 * the far lip, with a mean overshoot of 5 to 13 inches against an opening only +-3 inches
 * deep. Something between "the table says 2900 rpm at 34 degrees" and "the ball lands"
 * disagrees by a fifth of the range, and there are five candidates:
 *
 *   a. the wheel is not at the rpm the table asked for
 *   b. the exit speed is not k*r*omega
 *   c. the ball's flight in the world is not the flight the solver integrated
 *   d. the range the table was looked up at is not the range to the mouth
 *   e. the muzzle is not where the table thinks it is
 *
 * This forks them with one measurement. For every shot fired, it re-runs the SOLVER at the
 * conditions that shot ACTUALLY left with -- recorded rpm, recorded hood angle, recorded
 * exit speed -- and asks where the solver says that ball lands.
 *
 *   solver says LONG too  ->  the exit conditions were wrong for the range. (a), (b) or (d).
 *   solver says ON TARGET ->  the exit conditions were right and the flight differs. (c)/(e).
 *
 * One number tells you which half of the machine to open up.
 */
import params from '../config/params.json' with { type: 'json' };
import robotJson from '../config/robot.json' with { type: 'json' };
import { simulateShot } from '../packages/core/src/physics/ballistics.js';
import { DEG, M_TO_IN, inches, rpmToRadS } from '../packages/core/src/units.js';
import { landRate } from './landrate.js';
import { mouthLips } from './shottable.js';
import type { Params, RobotSpec, ShotRecord } from '../packages/core/src/types.js';

const RANGES = [40, 55, 70];

/** Where the solver puts this shot down, as a horizontal range from the muzzle. */
function solverRange_m(p: Params, spec: RobotSpec, rec: ShotRecord, mouthY: number): number {
  const ballR = p.ball.pollen.d_m / 2;
  const spinPerSpeed = spec.flywheel.type === 'single' ? 1 / ballR : 0;
  const base = inches(rec.rangeIn) - spec.turret.muzzleOffset_m;
  // Gates every inch from well short to well long; the landing range is the first descending
  // crossing of the mouth's centre height.
  const gates: number[] = [];
  for (let g = base - inches(24); g <= base + inches(48); g += inches(1)) gates.push(g);
  const t = simulateShot(p, {
    from: [0, spec.turret.muzzleHeight_m, 0],
    azimuth: 0,
    elevation: rec.hoodDeg * DEG,
    speed: rec.exitSpeed,
    radius: ballR,
    mass: p.ball.pollen.m_kg,
    spin: spinPerSpeed * rec.exitSpeed,
  }, base, 1 / 480, gates);
  for (let i = 1; i < gates.length; i++) {
    const a = t.gateHeights[i - 1];
    const b = t.gateHeights[i];
    if (Number.isFinite(a) && Number.isFinite(b) && a >= mouthY && b < mouthY) {
      return gates[i - 1] + (gates[i] - gates[i - 1]) * ((a - mouthY) / (a - b || 1));
    }
  }
  return NaN;
}

export async function main(argv: string[] = []): Promise<void> {
  const num = (k: string, d: number) => {
    const i = argv.indexOf(`--${k}`);
    return i >= 0 ? Number(argv[i + 1]) : d;
  };
  const shots = num('shots', 12);
  const seeds = Array.from({ length: num('seeds', 4) }, (_, i) => 11 + i * 18);

  const p = params as unknown as Params;
  const spec = robotJson as unknown as RobotSpec;
  const lips = mouthLips(p);
  const mouthY = (lips.near.y + lips.far.y) / 2;
  const f = spec.flywheel;

  console.log('AIM BIAS — is the shot leaving wrong, or flying wrong?');
  console.log(`  mouth centre ${(mouthY * M_TO_IN).toFixed(1)} in up, muzzle ${(spec.turret.muzzleHeight_m * M_TO_IN).toFixed(1)} in`);
  console.log('');

  const before = f.minLandProb;
  f.minLandProb = 0;
  const all: { range: number; rec: ShotRecord; predLong: number }[] = [];
  try {
    for (const range of RANGES) {
      for (const seed of seeds) {
        const res = await landRate(range, shots, {}, seed);
        if (!res.placed) continue;
        for (const rec of res.log) {
          if (rec.result === 'flight' || !Number.isFinite(rec.long_in)) continue;
          const sr = solverRange_m(p, spec, rec, mouthY);
          const want = inches(rec.rangeIn) - spec.turret.muzzleOffset_m;
          all.push({ range, rec, predLong: Number.isFinite(sr) ? (sr - want) * M_TO_IN : NaN });
        }
      }
    }
  } finally {
    f.minLandProb = before;
  }

  const mean = (a: number[]) => (a.length ? a.reduce((x, y) => x + y, 0) / a.length : NaN);
  const sd = (a: number[]) => {
    if (a.length < 2) return NaN;
    const m = mean(a);
    return Math.sqrt(a.reduce((s, v) => s + (v - m) ** 2, 0) / (a.length - 1));
  };

  console.log(`  the model assumes bearing error is N(0, ${f.scatter.yaw_deg} deg). The 'aim err' column is what it`);
  console.log('  actually was, measured at the muzzle: launch azimuth minus the true bearing to the mouth.');
  console.log('');
  console.log('  range      n   rpm vs target   exit vs k*r*w   sensed range   ACTUAL long   SOLVER says     aim err');
  for (const r of RANGES) {
    const g = all.filter((x) => x.range === r);
    if (!g.length) continue;
    const rpmErr = g.map((x) => x.rec.rpm - x.rec.targetRpm);
    const vErr = g.map((x) => x.rec.exitSpeed - f.k * f.r_fly_m * rpmToRadS(x.rec.rpm));
    const sensed = g.map((x) => x.rec.rangeIn - r);
    const act = g.map((x) => x.rec.long_in);
    const pred = g.map((x) => x.predLong).filter(Number.isFinite);
    const aim = g.map((x) => x.rec.aimErrDeg).filter(Number.isFinite);
    console.log(
      `  ${String(r).padStart(5)}  ${String(g.length).padStart(5)}   ${mean(rpmErr).toFixed(0).padStart(6)} rpm     ` +
      `${mean(vErr).toFixed(3).padStart(6)} m/s   ${mean(sensed).toFixed(1).padStart(7)} in   ` +
      `${mean(act).toFixed(1).padStart(6)} in   ${mean(pred).toFixed(1).padStart(6)} in   ` +
      `${mean(aim).toFixed(2).padStart(5)}+-${sd(aim).toFixed(2)} deg`,
    );
  }
  const act = all.map((x) => x.rec.long_in);
  const pred = all.map((x) => x.predLong).filter(Number.isFinite);
  console.log('');
  // WHERE DO THE ONES THAT SCORE ARRIVE?
  //
  // CAREFUL: this statistic is survivorship, not a target. It first read 5.8 in for the
  // shots that landed against 8.8 in for all of them, and the tempting conclusion -- that
  // the aim point is really 5.8 in long and the mouth centre is the wrong reference -- is
  // WRONG. If the group is centred at +8.8 with a 10 in spread, the ones that go in are
  // simply the ones that happened to be least long, so their mean is pulled toward the
  // target without ever reaching it. tools/apercheck.ts settled it off-line (the table's own
  // solution crosses within 1.5 in of the mouth centre) and tools/trimsweep.ts settled it in
  // the world (shooting 6 in shorter took the land rate from 51% to 84%). Read this as a
  // consistency check on the spread, never as the aim point.
  const inCell = all.filter((x) => x.rec.result === 'cell').map((x) => x.rec.long_in);
  const out = all.filter((x) => x.rec.result !== 'cell').map((x) => x.rec.long_in);
  console.log(`  landed (n=${inCell.length}): ${mean(inCell).toFixed(1)} +- ${sd(inCell).toFixed(1)} in along the shot line`);
  console.log(`  missed (n=${out.length}): ${mean(out).toFixed(1)} +- ${sd(out).toFixed(1)} in`);
  if (mean(inCell) > 3 && mean(out) > mean(inCell)) {
    console.log('  -> both groups are long and the landed ones are less long: that is what a group');
    console.log('     centred PAST the target looks like through the filter of what went in.');
  } else if (mean(inCell) > 3) {
    console.log('  -> the shots that score arrive long and the misses do not, which no aiming error');
    console.log('     explains. Check the aperture geometry with tools/apercheck.ts before trimming.');
  } else {
    console.log('  -> the shots that score arrive near the mouth centre and the misses do not, so the');
    console.log('     long bias is real and is costing shots.');
  }
  console.log('');
  console.log(`  ALL    ${String(all.length).padStart(5)}   actual ${mean(act).toFixed(1)} +- ${sd(act).toFixed(1)} in,  solver ${mean(pred).toFixed(1)} +- ${sd(pred).toFixed(1)} in  (n=${pred.length})`);
  console.log('');
  const aimAll = all.map((x) => x.rec.aimErrDeg).filter(Number.isFinite);
  const yawSigma = f.scatter.yaw_deg;
  console.log(`  bearing error over all shots: ${mean(aimAll).toFixed(2)} +- ${sd(aimAll).toFixed(2)} deg, against a model sigma of ${yawSigma} deg`);
  if (sd(aimAll) > yawSigma * 1.5) {
    console.log(`  -> the model is UNDERSTATING the pointing error by ${(sd(aimAll) / yawSigma).toFixed(1)}x. Its bearing term will be`);
    console.log('     optimistic by that much, and so will anything calibrated on top of it.');
  }
  console.log('');
  console.log('VERDICT');
  const gap = mean(act) - mean(pred);
  console.log(Math.abs(mean(pred)) > 3
    ? `  THE SHOT LEAVES WRONG. Fired at its own recorded exit conditions the solver puts the ball\n` +
      `  ${mean(pred).toFixed(1)} in long, so the flight is not the problem -- the rpm, the exit-speed model or the\n` +
      `  range the table was read at is. Check the three columns above: whichever is not zero.`
    : `  THE SHOT FLIES WRONG. The solver says these exit conditions should land within\n` +
      `  ${mean(pred).toFixed(1)} in, and the world puts them ${mean(act).toFixed(1)} in out -- a ${gap.toFixed(1)} in disagreement between the\n` +
      `  solver's integration and the world's. Same aero code, so look at the launch geometry:\n` +
      `  muzzle height and offset, hood angle actually applied, and the turret's exit point.`);
}
