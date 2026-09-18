/**
 * Make "90% sure" mean 90%.
 *
 *   npm run tool -- tools/landcal.ts [--shots 12] [--seeds 4]
 *
 * The robot computes P(land) = P(exit speed threads the mouth) x P(a ball arriving like that
 * stays in). Both factors are real and both are measured, and the product is still wrong:
 * tools/gatecal.ts swept the threshold and found the gate OVERCONFIDENT by about 20 points
 * at every setting. Asking for 90% got 71%. A threshold that does not mean what it says is
 * worse than no threshold, because it invites you to trust it.
 *
 * The gap is not a mystery, it is the modelling that was left out:
 *
 *   - P(thread) tests a two-dimensional slot -- above the near lip, below the far one. The
 *     real pocket has ribs, churros and a back skin to hit, none of which are in that test.
 *   - P(stay) was measured with balls injected at the mouth CENTRE, jittered by one ball
 *     radius. A real shot arrives spread out, and one that only just clears the near lip
 *     behaves nothing like a centred one.
 *
 * Rather than invent terms for those, this measures the mapping directly: fire a lot of
 * shots, record what the model PREDICTED for each one and whether it actually landed, and
 * fit observed frequency against prediction. That curve is the calibration, and after it is
 * applied the threshold is a real probability rather than a score with a percent sign.
 */
import { readFileSync, writeFileSync } from 'node:fs';
import params from '../config/params.json' with { type: 'json' };
import robotJson from '../config/robot.json' with { type: 'json' };
import { World, initPhysics, emptyGamepad } from '../packages/core/src/physics/world.js';
import { BuiltinTeleOp, ShotTable } from '../packages/core/src/robot/builtinTeleOp.js';
import { inches } from '../packages/core/src/units.js';
import type { GamepadState, Params, RobotSpec, Vec3 } from '../packages/core/src/types.js';

const table = ShotTable.fromCsv(readFileSync(new URL('../java/teamcode/assets/shottable.csv', import.meta.url), 'utf8'));
// Three ranges, not five. The two longest ones mostly failed to place on the field or ran
// out of firing window, so they cost a full run each and contributed almost nothing.
const RANGES = [40, 55, 70];

export interface Sample {
  predicted: number; landed: boolean; range_in: number; moving: boolean;
  /** The three factors the prediction is the product of, at the frame the ball left. */
  pSpeed: number; pStay: number; pAim: number;
}

/**
 * How the robot is driving while it samples. THE APP SHOOTS ON THE MOVE and the calibration
 * is what turns the model's score into the probability the gate compares against, so a curve
 * fitted from a robot standing still is a curve that does not describe most of a match. It
 * used to be stationary only, and every moving shot in the app was then gated on a number
 * measured under conditions it was not taken in.
 */
const DRIVING: { name: string; stick: [number, number]; wobble: number }[] = [
  { name: 'still', stick: [0, 0], wobble: 0 },
  { name: 'strafing', stick: [0.45, 0], wobble: 0 },
  { name: 'shuttling', stick: [0, 0], wobble: 0.35 },
  // A HARD case on purpose. The curve is used to gate shots the robot takes while being
  // driven properly, and the previous fit's lowest bin was a score of 0.49 -- nothing worse
  // than that was ever sampled, so the whole bottom half of the curve was extrapolation.
  // This is the FAST wobble case from tools/movingtune.ts, which is where the gate now
  // refuses about a third of the loops, so it is exactly the region that needs data.
  { name: 'fast wobble', stick: [0, 0], wobble: 0.7 },
];

/**
 * Fire at one range with the gate WIDE OPEN and record (prediction, outcome) per shot.
 * The gate must be open or the sample only ever covers the high-prediction end, which is
 * exactly the region a calibration curve cannot be fitted from.
 */
async function sampleAt(range_in: number, shots: number, seed: number, drive = DRIVING[0]): Promise<Sample[]> {
  await initPhysics();
  const p = structuredClone(params) as unknown as Params;
  const spec = structuredClone(robotJson) as unknown as RobotSpec;
  spec.flywheel.minLandProb = 0;
  const staging = Array.from({ length: shots + 4 }, () => ({ kind: 'pollen' as const, pos: [0, -5, 0] as Vec3 }));
  const world = new World({ params: p, robot: spec, staging, alliance: 'red', seed });
  for (const b of world.balls.balls) world.balls.park(b);

  const mouth = world.hives.red.upCellMouthWorld();
  const limit = world.geom.halfWidth_m - 0.35;
  const d = inches(range_in);
  let spot: Vec3 | null = null;
  for (let deg = 0; deg <= 90 && !spot; deg += 2) {
    for (const sign of [1, -1]) {
      const a = (deg * Math.PI) / 180;
      const c: Vec3 = [mouth[0] + sign * d * Math.sin(a), spec.chassis.height_m / 2 + spec.chassis.clearance_m, mouth[2] + d * Math.cos(a)];
      if (Math.abs(c[0]) < limit && Math.abs(c[2]) < limit) { spot = c; break; }
    }
  }
  if (!spot) return [];
  world.robot.place(spot, (Math.atan2(mouth[0] - spot[0], mouth[2] - spot[2]) * 180) / Math.PI);

  const brain = new BuiltinTeleOp(spec, table);
  const step = (g = emptyGamepad()) => {
    world.setGamepads(g, emptyGamepad());
    world.step(brain.update(world.sensors(), g, world.seq));
  };
  step();
  const press = emptyGamepad();
  press.dpad_up = true;
  step(press);
  // Spin up STANDING STILL, then drive. Holding the stick through a 4 s spin-up walks the
  // robot most of the way across the field before the first sample.
  for (let f = 0; f < 240; f++) step();
  const t1 = world.t;
  const wall = world.geom.halfWidth_m - 0.32;
  const held = (): GamepadState => {
    const g = emptyGamepad();
    g.left_stick_x = drive.stick[0];
    g.left_stick_y = -Math.max(-1, Math.min(1, drive.stick[1] + drive.wobble * Math.sin(2 * Math.PI * 0.5 * (world.t - t1))));
    return g;
  };

  const predicted: number[] = [];
  const factors: { pSpeed: number; pStay: number; pAim: number }[] = [];
  let last = 0;
  let loaded = 0;
  for (let f = 0; f < 60 * (shots * 4 + 12) && world.robot.shots < shots; f++) {
    while (world.robot.hopper.length < 4 && loaded < staging.length) {
      if (!world.robot.preload(world.balls, world.balls.balls[loaded])) break;
      loaded++;
    }
    // Capture the prediction on the frame the shot leaves, not after -- the wheel dips on
    // firing, so reading it a frame later records a number the shot was never taken at.
    const before = brain.state.pLand;
    const parts = { pSpeed: brain.state.pSpeed, pStay: brain.state.pStayNow, pAim: brain.state.pAim };
    step({ ...held(), right_bumper: true });
    if (world.robot.shots > last) {
      predicted.push(before);
      factors.push(parts);
      last = world.robot.shots;
    }
    // A pass that drives into a wall stops being a moving sample and silently becomes a
    // stationary one, which is the fault this whole change is about.
    const r = world.robot.pos;
    if (Math.abs(r[0]) > wall || Math.abs(r[2]) > wall) break;
  }
  // Let everything settle. A shot still in the air is recorded as 'flight' and thrown away,
  // and at 6 s a good fraction of them were -- which is how 240 attempts became 39 samples.
  for (let f = 0; f < 60 * 12; f++) step();

  const log = world.snapshot().shots;
  const out: Sample[] = [];
  for (let i = 0; i < predicted.length && i < log.length; i++) {
    if (log[i].result === 'flight') continue;      // never settled; no outcome to learn from
    out.push({ predicted: predicted[i], landed: log[i].result === 'cell', range_in, moving: drive.wobble > 0 || drive.stick[0] !== 0 || drive.stick[1] !== 0, ...factors[i] });
  }
  return out;
}

export async function main(argv: string[] = []): Promise<void> {
  const num = (k: string, d: number) => {
    const i = argv.indexOf(`--${k}`);
    return i >= 0 ? Number(argv[i + 1]) : d;
  };
  const shots = num('shots', 12);
  const seeds = Array.from({ length: num('seeds', 4) }, (_, i) => 11 + i * 18);

  console.log('LAND-PROBABILITY CALIBRATION');
  console.log(`  ${seeds.length} seeds x ${RANGES.length} ranges x up to ${shots} shots, gate open so the whole range of`);
  console.log('  predictions gets sampled and not just the confident end.');
  console.log('');

  const all: Sample[] = [];
  for (const seed of seeds) {
    for (const r of RANGES) for (const d of DRIVING) all.push(...await sampleAt(r, shots, seed, d));
  }
  const flightless = all.length;
  console.log(`  ${flightless} shots with a settled outcome`);
  const byRange = RANGES.map((r) => `${r} in: ${all.filter((s2) => s2.range_in === r).length}`).join(', ');
  console.log(`  per range: ${byRange}`);
  const mv = all.filter((s2) => s2.moving);
  const st = all.filter((s2) => !s2.moving);
  const pct = (a: Sample[]) => (a.length ? `${((a.filter((x) => x.landed).length / a.length) * 100).toFixed(0)}% of ${a.length}` : 'none');
  console.log(`  standing still ${pct(st)};  on the move ${pct(mv)}`);
  console.log(`  predictions span ${(Math.min(...all.map((s2) => s2.predicted)) * 100).toFixed(0)}% to ${(Math.max(...all.map((s2) => s2.predicted)) * 100).toFixed(0)}%`);
  if (all.length < 40) {
    console.log('  NOT ENOUGH DATA to fit anything. Raise --shots or --seeds.');
    return;
  }

  // EQUAL-COUNT bins, not fixed edges. Fixed edges gave the top bin 14 samples out of 340,
  // and 12 of those 14 landed -- an 86% "ceiling" with a standard error of 9 points, which
  // the gate sweep then failed to reproduce anywhere (the strictest setting it could satisfy
  // landed 65% over 87 shots). A bin thin enough to be noise should not be allowed to set
  // the number the whole policy is compared against.
  const sorted = [...all].sort((a, b) => a.predicted - b.predicted);
  // MORE DATA HAS TO BUY RESOLUTION. This was capped at six bins whatever the sample size, so
  // a longer run only made the bins fatter -- and six bins over a 331-shot run is why the top
  // four all read the same 0.964 and the curve could not tell 85% from 96% anywhere in the
  // region the robot actually shoots from. About 80 samples a bin keeps the standard error
  // near 5 points, which is the resolution worth having; the cap is there so a short run
  // still produces something rather than a bin per shot.
  const nBins = Math.max(2, Math.min(12, Math.round(sorted.length / 80)));
  const per = Math.ceil(sorted.length / nBins);
  console.log('');
  console.log('  predicted band        n   landed   observed   +-1 se   model error');
  const bins: { mid: number; observed: number; n: number }[] = [];
  for (let i = 0; i < sorted.length; i += per) {
    const inBin = sorted.slice(i, i + per);
    const landed = inBin.filter((s) => s.landed).length;
    const obs = landed / inBin.length;
    const mid = inBin.reduce((a, s) => a + s.predicted, 0) / inBin.length;
    const se = Math.sqrt((obs * (1 - obs)) / inBin.length);
    bins.push({ mid, observed: obs, n: inBin.length });
    console.log(
      `  ${inBin[0].predicted.toFixed(2)}-${inBin[inBin.length - 1].predicted.toFixed(2)}   ${String(inBin.length).padStart(7)}   ${String(landed).padStart(6)}   ` +
      `${(obs * 100).toFixed(0).padStart(7)}%   ${(se * 100).toFixed(0).padStart(5)}   ${((obs - mid) * 100).toFixed(0).padStart(7)} pts`,
    );
  }
  console.log('');

  // WHICH FACTOR IS THE DEAD ONE? `predicted` is pSpeed x pStay x pAim, and the fit says the
  // product carries no information. Split the samples at each factor's own median and compare
  // the land rate either side: a factor that predicts shows a gap, one that does not shows
  // none. This is the whole diagnosis and it costs nothing to print.
  // PER RANGE, which is the confound behind the two inverted factors: pStay and pAim both
  // rise as range falls, so if CLOSE shots land worse than far ones both will read backwards
  // for one reason rather than two.
  console.log('  land rate by range, and what the model thought:');
  for (const r of RANGES) {
    const a = all.filter((x) => x.range_in === r);
    if (!a.length) continue;
    const rate = (a.filter((x) => x.landed).length / a.length) * 100;
    const mp = (f: (x: Sample) => number) => (a.reduce((t, x) => t + f(x), 0) / a.length) * 100;
    console.log(`    ${String(r).padStart(3)} in   n=${String(a.length).padStart(3)}   landed ${rate.toFixed(0).padStart(3)}%`
      + `   model: pSpeed ${mp((x) => x.pSpeed).toFixed(0).padStart(3)}%  pStay ${mp((x) => x.pStay).toFixed(0).padStart(3)}%`
      + `  pAim ${mp((x) => x.pAim).toFixed(0).padStart(3)}%  product ${mp((x) => x.predicted).toFixed(0).padStart(3)}%`);
  }
  console.log('');
  // Dump the raw samples so the next question does not cost another ten-minute run.
  writeFileSync(new URL('../config/landcal-samples.json', import.meta.url), JSON.stringify(all));
  console.log(`  wrote config/landcal-samples.json (${all.length} raw samples)`);
  console.log('');

  console.log('  is each FACTOR predictive? land rate below vs above its own median:');
  for (const [name, get] of [
    ['pSpeed (threads the mouth)', (x: Sample) => x.pSpeed],
    ['pStay  (stays in once there)', (x: Sample) => x.pStay],
    ['pAim   (lateral)', (x: Sample) => x.pAim],
    ['pLand  (the product)', (x: Sample) => x.predicted],
  ] as [string, (x: Sample) => number][]) {
    const ok = all.filter((x) => Number.isFinite(get(x)) && get(x) >= 0);
    if (ok.length < 20) { console.log(`    ${name.padEnd(30)} not recorded`); continue; }
    const med = [...ok].sort((a, b) => get(a) - get(b))[Math.floor(ok.length / 2)];
    const lo = ok.filter((x) => get(x) < get(med));
    const hi = ok.filter((x) => get(x) >= get(med));
    const r = (a: Sample[]) => (a.length ? (a.filter((x) => x.landed).length / a.length) * 100 : NaN);
    const gap = r(hi) - r(lo);
    console.log(`    ${name.padEnd(30)} ${r(lo).toFixed(0).padStart(3)}% low  ->  ${r(hi).toFixed(0).padStart(3)}% high`
      + `   gap ${gap > 0 ? '+' : ''}${gap.toFixed(0)} pts${Math.abs(gap) < 5 ? '   <- carries no information' : ''}`);
  }
  console.log('');

  // Enforce monotonicity: a higher prediction must never map to a lower observed rate. With
  // this little data a bin can invert by pure noise, and a calibration curve that goes
  // backwards would let the robot prefer a shot it rates WORSE.
  const mono = [...bins].sort((a, b) => a.mid - b.mid);
  for (let i = 1; i < mono.length; i++) mono[i].observed = Math.max(mono[i].observed, mono[i - 1].observed);

  const ceiling = Math.max(...mono.map((b) => b.observed));
  console.log(`  CEILING: the best-predicted shots land ${(ceiling * 100).toFixed(0)}% of the time.`);
  console.log('  No threshold above that can ever be satisfied honestly -- setting one just stops');
  console.log('  the robot shooting. That is a fact about the shooter, not about the gate.');
  console.log('');

  writeFileSync(new URL('../config/landcal.json', import.meta.url), JSON.stringify({
    _about: 'MEASURED by tools/landcal.ts: the mapping from the model\'s raw P(land) score to the frequency with which shots at that score ACTUALLY land. BuiltinTeleOp applies it so that flywheel.minLandProb is a real probability instead of a score.',
    _method: `${all.length} settled shots over ${seeds.length} seeds, ${RANGES.length} ranges and ${DRIVING.length} driving states (${DRIVING.map((d) => d.name).join(', ')}) with the gate open. Bins forced monotone. The app shoots on the move, so the curve has to have been fitted on the move.`,
    generated: new Date().toISOString(),
    samples: all.length,
    ceiling,
    points: mono.map((b) => ({ score: b.mid, observed: b.observed, n: b.n })),
  }, null, 1) + String.fromCharCode(10));
  console.log('  wrote config/landcal.json');
}
