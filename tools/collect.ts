/**
 * Headless data collection: the Data mode of the app, without the browser.
 *
 *   npm run tool -- tools/collect.ts                       # 40 shots, 50-115 in
 *   npm run tool -- tools/collect.ts --shots 200 --moving  # fire on the roll
 *   npm run tool -- tools/collect.ts --csv out.csv
 *
 * Same AutoDriver, same TeleOp, same world as the app, so the numbers are the same numbers
 * -- it just runs as fast as the CPU allows instead of at wall-clock speed. This is the one
 * to use when you are actually tuning: 200 shots here is a couple of minutes, and 200 shots
 * in the browser is most of an hour.
 */
import { readFileSync, writeFileSync } from 'node:fs';
import { World, initPhysics } from '../packages/core/src/physics/world.js';
import { BuiltinTeleOp, ShotTable } from '../packages/core/src/robot/builtinTeleOp.js';
import { loadLandCal } from '../packages/core/src/robot/loadCal.js';
import { AutoDriver, defaultPlan } from '../packages/core/src/robot/autoDriver.js';
import { analyse, toCsv, type Report } from '../packages/core/src/analysis/report.js';
import { knobs, predict } from '../packages/core/src/analysis/sensitivity.js';
import { rpmToSpeed } from '../packages/core/src/physics/ballistics.js';
import { inches, M_TO_IN, DEG } from '../packages/core/src/units.js';
import params from '../config/params.json' with { type: 'json' };
import robotJson from '../config/robot.json' with { type: 'json' };
import staging from '../assets/staging.json' with { type: 'json' };
import type { BallKind, Params, RobotSpec, Vec3 } from '../packages/core/src/types.js';

// tools/run.mjs hands args through, and argv is there too when run directly.
const ARGV = process.argv.slice(2);
const arg = (name: string, dflt: number): number => {
  const i = ARGV.indexOf(`--${name}`);
  return i >= 0 ? Number(ARGV[i + 1]) : dflt;
};
const flag = (name: string) => ARGV.includes(`--${name}`);
const inch = (v: number) => `${v >= 0 ? '+' : ''}${v.toFixed(1)}"`.padStart(8);
const str = (name: string): string | null => {
  const i = ARGV.indexOf(`--${name}`);
  return i >= 0 ? ARGV[i + 1] : null;
};

export async function main(): Promise<void> {
  await initPhysics();
  const p = structuredClone(params) as unknown as Params;
  const spec = structuredClone(robotJson) as unknown as RobotSpec;
  const table = ShotTable.fromCsv(readFileSync(new URL('../java/teamcode/assets/shottable.csv', import.meta.url), 'utf8'));
  const stage = (staging.balls as { kind: string; pos: number[] }[]).map((b) => ({ kind: b.kind as BallKind, pos: b.pos as Vec3 }));

  const plan = { ...defaultPlan(), shots: arg('shots', 40), onTheMove: flag('moving'), seed: arg('seed', 1) };
  if (str('range')) {
    const [lo, hi] = str('range')!.split('-').map(Number);
    plan.range_in = [lo, hi];
  }

  /** One collection run against the CURRENT spec, trims included. */
  let mouthY = 0;
  const run = (seed: number) => {
    const world = new World({ params: p, robot: spec, staging: stage, alliance: 'red', seed: p.sim.seed, preload: 4 });
    const brain = new BuiltinTeleOp(spec, table, loadLandCal());
    const mouth = world.hives.red.upCellMouthWorld();
    const auto = new AutoDriver({ ...plan, seed }, [mouth[0], mouth[2]], world.geom.halfWidth_m);
    world.clock.startTeleOp();

    const dt = p.sim.dt * p.sim.substepsPerFrame;
    const budget = plan.shots * 20;    // seconds of sim time; 20 s per sample is generous
    let printed = 0;
    console.log(`collecting ${plan.shots} shots, ${plan.range_in[0]}-${plan.range_in[1]} in, ${plan.onTheMove ? 'on the move' : 'stopped'}`);

    for (let t = 0; t < budget && auto.phase !== 'done'; t += dt) {
      const g = auto.update(world.sensors(), dt, world.robot.shots);
      world.setGamepads(g, { ...g, left_stick_x: 0, left_stick_y: 0, right_stick_x: 0 });
      world.step(brain.update(world.sensors(), g, world.seq, dt));
      // Keep it fed: the robot holds six and the sweep needs forty.
      if (world.seq % 20 === 0) topUp(world, spec);
      if (world.shotLog.length > printed) {
        printed = world.shotLog.length;
        process.stdout.write(`\r  ${printed}/${plan.shots}`);
      }
    }
    // Let the last shots land and settle before the statistics are read.
    const idle = { seq: 0, motors: {}, servos: {} };
    for (let t = 0; t < 8; t += dt) world.step(idle);
    process.stdout.write('\n');
    mouthY = mouth[1];
    return { rep: analyse(world.shotLog, spec.flywheel.tolRpm), log: [...world.shotLog] };
  };

  let { rep, log } = run(plan.seed);

  // --calibrate N: collect, apply the trim that run implies, collect again -- N times,
  // each with a different seed so a fall in bias cannot be the same sample twice. If the
  // model is right the bias converges toward zero and the SPREAD does not move, because a
  // trim corrects where the group is and can do nothing about how big it is.
  const rounds = arg('calibrate', 0);
  if (rounds > 0) {
    console.log('');
    console.log('  round    n      bias    spread   lateral   in CELL   trim applied before this round');
    const line = (i: number | string, r: Report, applied: string) =>
      console.log(`  ${String(i).padEnd(6)} ${String(r.n).padStart(3)}  ${inch(r.bias_in)}  ${inch(r.sd_in)}  ${inch(r.latBias_in)}   ${String(r.landed).padStart(3)}/${String(r.n).padEnd(3)}   ${applied}`);
    line(0, rep, '-');
    for (let i = 1; i <= rounds; i++) {
      const applied = rep.fix.worthIt
        ? `range ${rep.fix.rangeTrim_in >= 0 ? '+' : ''}${rep.fix.rangeTrim_in.toFixed(1)} in, turret ${rep.fix.turretTrim_deg >= 0 ? '+' : ''}${rep.fix.turretTrim_deg.toFixed(2)} deg`
        : 'none (inside the run noise)';
      if (rep.fix.worthIt) {
        // Trims accumulate: each run measures what is LEFT after the last one.
        spec.calibration.rangeTrim_in += rep.fix.rangeTrim_in;
        spec.calibration.turretTrim_deg += rep.fix.turretTrim_deg;
      }
      ({ rep, log } = run(plan.seed + i));
      line(i, rep, applied);
    }
    console.log('');
    console.log('  final trims for config/robot.json -> calibration:');
    console.log(`    "rangeTrim_in": ${spec.calibration.rangeTrim_in.toFixed(2)},`);
    console.log(`    "turretTrim_deg": ${spec.calibration.turretTrim_deg.toFixed(3)}`);
  }

  console.log('');
  console.log(`  shots landed        ${rep.n}`);
  console.log(`  into the CELL       ${rep.landed}  (${(rep.landRate * 100).toFixed(0)}%)`);
  console.log(`  downrange bias      ${rep.bias_in >= 0 ? '+' : ''}${rep.bias_in.toFixed(1)} in`);
  console.log(`  downrange spread    +-${rep.sd_in.toFixed(1)} in  (1 sigma)`);
  console.log(`  lateral bias        ${rep.latBias_in >= 0 ? '+' : ''}${rep.latBias_in.toFixed(1)} in`);
  console.log(`  lateral spread      +-${rep.latSd_in.toFixed(1)} in`);
  console.log(`  rpm error at fire   ${rep.rpmErr_rpm.toFixed(0)} rpm  (${rep.firedOffSpeed} shots out of band)`);
  console.log('');
  console.log(`  ${rep.verdict}`);
  console.log(`  ${rep.fix.why}`);
  if (rep.faults.length) {
    console.log('');
    for (const f of rep.faults) console.log(`  - ${f}`);
  }

  console.log('');
  console.log('  by range');
  for (const b of rep.byRange) {
    console.log(`    ${String(b.range_in).padStart(4)} in  ${String(b.landed).padStart(2)}/${String(b.n).padEnd(2)} in CELL   mean miss ${b.meanMiss_in.toFixed(0)} in`);
  }

  // The error budget at the middle of the sampled band: which knob to go and measure.
  const mid = (plan.range_in[0] + plan.range_in[1]) / 2;
  const rowT = table.lookup(mid);
  const hood = spec.hood.enabled
    ? spec.hood.angleRange_deg[0] + rowT.hoodPos * (spec.hood.angleRange_deg[1] - spec.hood.angleRange_deg[0])
    : spec.hood.fixedAngle_deg;
  const speed = rpmToSpeed(rowT.rpm, spec.flywheel.k, spec.flywheel.r_fly_m);
  const radius = p.ball.pollen.d_m / 2;
  const pred = predict(
    p,
    {
      from: [0, spec.turret.muzzleHeight_m, 0], azimuth: 0, elevation: hood * DEG, speed, radius,
      mass: p.ball.pollen.m_kg,
      spin: spec.flywheel.type === 'single' ? speed / radius : 0,
    },
    mouthY,
    mid,
    knobs({
      tolRpm: spec.flywheel.tolRpm, hoodSteps: 200,
      scatterDeg: spec.flywheel.scatter.angle_deg,
      scatterSpeedFrac: spec.flywheel.scatter.speedFrac,
      k: spec.flywheel.k, rFly: spec.flywheel.r_fly_m,
    }),
  );
  console.log('');
  console.log(`  error budget at ${mid.toFixed(0)} in  (predicted spread +-${pred.spread_in.toFixed(1)} in)`);
  for (const r of pred.rows.slice(0, 5)) {
    console.log(`    ${r.label.padEnd(26)} ${r.contrib_in.toFixed(1).padStart(5)} in   ${(r.share * 100).toFixed(0)}%`);
  }

  const out = str('csv');
  if (out) {
    writeFileSync(out, toCsv(log));
    console.log(`\n  wrote ${out}  (${log.length} shots)`);
  }
}

/** Same practice aid the app uses: pick the nearest loose POLLEN off the floor. */
function topUp(world: World, spec: RobotSpec): void {
  const r = world.robot;
  if (r.hopper.length >= spec.hopper.capacity) return;
  let best: (typeof world.balls.balls)[number] | null = null;
  let bestD = Infinity;
  for (const b of world.balls.balls) {
    if (b.kind !== 'pollen' || b.state !== 'free') continue;
    const q = world.balls.pos(b);
    if (q[1] > inches(8)) continue;
    const d = Math.hypot(q[0] - r.pos[0], q[2] - r.pos[2]);
    if (d < bestD) { bestD = d; best = b; }
  }
  if (best) r.preload(world.balls, best);
}

void M_TO_IN;
