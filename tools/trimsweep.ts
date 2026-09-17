/**
 * The two field trims exist, are documented, and are both zero. Are they zero because the
 * shooter is centred, or because nobody has measured them?
 *
 *   npm run tool -- tools/trimsweep.ts [--shots 10] [--seeds 3]
 *
 * tools/aimbias.ts says the group is not centred: shots arrive a mean 8.8 in beyond the
 * mouth centre and, at 55 and 70 in, about 4 in to one side. The obvious move is the recipe
 * in config/robot.json -- rangeTrim = mean signed downrange error -- and it is WRONG here.
 * The shots that actually SCORE also arrive long (5.8 +- 3.6 in), because the mouth is a
 * slot tilted about 49 deg and "where the ball crosses the mouth's centre height" is not
 * the same point as "the middle of the hole". Centring that statistic would walk the group
 * off the target it is currently hitting.
 *
 * So do not compute a trim from a proxy. Sweep it against the only thing that settles it:
 * balls in the CELL.
 */
import robotJson from '../config/robot.json' with { type: 'json' };
import { landRate } from './landrate.js';
import type { RobotSpec } from '../packages/core/src/types.js';

const RANGES = [40, 55, 70];

interface Point { value: number; fired: number; landed: number; rate: number; se: number }

async function sweep(
  name: string,
  values: number[],
  set: (v: number) => void,
  seeds: number[],
  shots: number,
): Promise<Point[]> {
  const out: Point[] = [];
  for (const v of values) {
    set(v);
    let fired = 0;
    let landed = 0;
    for (const seed of seeds) {
      for (const r of RANGES) {
        const res = await landRate(r, shots, {}, seed);
        if (!res.placed) continue;
        fired += res.fired;
        landed += res.landed;
      }
    }
    const rate = fired ? landed / fired : 0;
    out.push({ value: v, fired, landed, rate, se: fired ? Math.sqrt((rate * (1 - rate)) / fired) : 0 });
    const p = out[out.length - 1];
    console.log(
      `  ${name} ${v >= 0 ? '+' : ''}${v.toFixed(1).padStart(5)}   ${String(p.fired).padStart(5)}   ${String(p.landed).padStart(6)}   ` +
      `${(p.rate * 100).toFixed(1).padStart(6)}%  +-${(p.se * 100).toFixed(1)}`,
    );
  }
  return out;
}

export async function main(argv: string[] = []): Promise<void> {
  const num = (k: string, d: number) => {
    const i = argv.indexOf(`--${k}`);
    return i >= 0 ? Number(argv[i + 1]) : d;
  };
  const shots = num('shots', 10);
  const seeds = Array.from({ length: num('seeds', 3) }, (_, i) => 11 + i * 18);
  const spec = robotJson as unknown as RobotSpec;
  const cal = spec.calibration ?? { rangeTrim_in: 0, turretTrim_deg: 0 };
  spec.calibration = cal;
  const before = { ...cal };
  const gate = spec.flywheel.minLandProb;

  console.log('FIELD TRIM SWEEP — measured against balls in the CELL, not against a proxy');
  console.log(`  ${seeds.length} seeds x ${RANGES.length} ranges x ${shots} shots per point, gate open`);
  console.log('');

  const tolBefore = spec.flywheel.tolRpm;
  const winBefore = spec.hub.encoderVelocityWindowMs;
  const filtBefore = spec.flywheel.rpmFilterFrames;
  try {
    spec.flywheel.minLandProb = 0;
    // FIRST OF ALL, because it bounds everything below it. The brain reads the hub's velocity
    // estimate, not the true omega, and that estimate is counts over a window: 28 ticks a rev
    // direct-driven means 26 counts in 20 ms at 2800 rpm, so ONE COUNT is 107 rpm -- about 15
    // in of range. Averaging over a longer window buys resolution and costs lag, which is the
    // trade a team makes in their own code when they filter getVelocity(). Sweeping the
    // window is the fair way to price it.
    console.log('  filter (loops)    fired   landed     rate');
    const v = await sweep('filter  ', [1, 3, 6, 12], (n) => { spec.flywheel.rpmFilterFrames = n; }, seeds, shots);
    const bestV = v.reduce((a, b) => (b.rate > a.rate ? b : a));
    spec.flywheel.rpmFilterFrames = bestV.value;
    console.log('');
    console.log(`  best filter ${bestV.value} loops = ${(bestV.value * 16.7).toFixed(0)} ms of lag, ${(bestV.rate * 100).toFixed(1)}%`);
    console.log('');
    // FIRST, because the other two depend on it. tools/apercheck.ts prices the flywheel's
    // speed error in inches: 10 rpm is 1.4 in of range and the +-120 rpm window the robot
    // fires inside is +-14.9 in -- against a hole 8.9 in deep. A trim fitted at tolRpm 120
    // is a trim fitted to a shooter that cannot hit anything anyway.
    console.log('  tolRpm (rpm)      fired   landed     rate');
    const w = await sweep('tolRpm  ', [120, 60, 30, 15], (x) => { spec.flywheel.tolRpm = x; }, seeds, shots);
    const bestW = w.reduce((a, b) => (b.rate > a.rate ? b : a));
    spec.flywheel.tolRpm = bestW.value;
    console.log('');
    console.log(`  best window ${bestW.value} rpm at ${(bestW.rate * 100).toFixed(1)}% (${bestW.fired} shots taken; a tighter window costs shots)`);
    console.log('');
    console.log('  range trim (cm)   fired   landed     rate');
    // BOTH SIGNS. tools/flywheeltune.ts --scan measures the closed loop parking about 45 rpm
    // BELOW target across the whole band the table uses, which is 6 in of range SHORT -- and
    // a sweep that only looks at positive trims cannot see the correction for it.
    const r = await sweep('rangeTrim', [-3, 0, 3], (v) => { cal.rangeTrim_in = v; }, seeds, shots);
    const bestR = r.reduce((a, b) => (b.rate > a.rate ? b : a));
    cal.rangeTrim_in = bestR.value;
    console.log('');
    console.log(`  best range trim ${bestR.value >= 0 ? '+' : ''}${(bestR.value * 2.54).toFixed(1)} cm at ${(bestR.rate * 100).toFixed(1)}%; held there for the turret sweep`);
    console.log('');
    console.log('  turret trim (deg) fired   landed     rate');
    const t = await sweep('turretTrim', [-1.5, 0, 1.5], (v) => { cal.turretTrim_deg = v; }, seeds, shots);
    const bestT = t.reduce((a, b) => (b.rate > a.rate ? b : a));
    console.log('');

    const base = r.find((p) => p.value === 0)!;
    console.log(`  window ${bestW.value} rpm, from ${w[0].fired} shots at ${(w[0].rate * 100).toFixed(1)}% to ${bestW.fired} at ${(bestW.rate * 100).toFixed(1)}%`);
    const gain = bestT.rate - base.rate;
    const se = Math.hypot(base.se, bestT.se);
    console.log('VERDICT');
    console.log(`  zero trim: ${(base.rate * 100).toFixed(1)}%.  best found: rangeTrim ${(bestR.value * 2.54).toFixed(1)} cm, turretTrim ${bestT.value} deg -> ${(bestT.rate * 100).toFixed(1)}%`);
    console.log(gain > 1.96 * se
      ? `  A gain of ${(gain * 100).toFixed(1)} points, ${(gain / se).toFixed(1)} standard errors. Worth setting.`
      : `  A gain of ${(gain * 100).toFixed(1)} points, only ${(gain / Math.max(se, 1e-9)).toFixed(1)} standard errors -- inside the noise.\n` +
        '  Leave the trims at zero and say so, rather than baking a number this run cannot see.');
  } finally {
    cal.rangeTrim_in = before.rangeTrim_in;
    cal.turretTrim_deg = before.turretTrim_deg;
    spec.flywheel.minLandProb = gate;
    spec.flywheel.tolRpm = tolBefore;
    spec.hub.encoderVelocityWindowMs = winBefore;
    spec.flywheel.rpmFilterFrames = filtBefore;
  }
}
