/**
 * Does every knob actually reach the ball, or are some of them decorative?
 *
 *   npm run tool -- tools/varcheck.ts [--shots 14]
 *
 * The Predictor tab computes what each variable is worth by re-running the ballistic
 * integrator with it nudged. That says what the MODEL thinks. It does not prove the running
 * simulation reads the same number -- a knob that is copied into a rebuilt world, or read once
 * at construction, or shadowed by a hard-coded constant, would show a confident contribution
 * in the Predictor and do nothing to a real shot.
 *
 * So this changes each variable in the actual world, fires real balls with the same seed, and
 * measures where they land. Same seed matters: the launch scatter draws are then identical, so
 * any difference in the group is the VARIABLE and not the dice.
 *
 * A knob that moves nothing is either dead or genuinely irrelevant, and the two need telling
 * apart -- which is why the expected direction is written next to each one.
 */
import params from '../config/params.json' with { type: 'json' };
import robotSpec from '../config/robot.json' with { type: 'json' };
import { readFileSync } from 'node:fs';
import { World, initPhysics, emptyGamepad } from '../packages/core/src/physics/world.js';
import { BuiltinTeleOp, ShotTable } from '../packages/core/src/robot/builtinTeleOp.js';
import { inches } from '../packages/core/src/units.js';
import type { Params, RobotSpec, Vec3 } from '../packages/core/src/types.js';

const shotCsv = readFileSync(new URL('../java/teamcode/assets/shottable.csv', import.meta.url), 'utf8');

export interface Group { n: number; landed: number; bias_cm: number; spread_cm: number; exit_mps: number }

/** Fire `shots` from one spot with a given (already mutated) params/spec pair. */
export async function group(p: Params, spec: RobotSpec, shots: number, seed: number, table?: ShotTable): Promise<Group> {
  await initPhysics();
  const staging = Array.from({ length: shots + 8 }, () => ({ kind: 'pollen' as const, pos: [0, -5, 0] as Vec3 }));
  const world = new World({ params: p, robot: spec, staging, alliance: 'red', seed });
  for (const b of world.balls.balls) world.balls.park(b);
  const mouth = world.hives.red.upCellMouthWorld();
  const limit = world.geom.halfWidth_m - 0.35;
  world.robot.place(
    [mouth[0], spec.chassis.height_m / 2 + spec.chassis.clearance_m, Math.min(mouth[2] + inches(55), limit)],
    180,
  );
  // OPEN THE GATE. Half these knobs change the exit speed, which is exactly what the
  // land-probability model watches -- so with the gate at its shipped 0.90 the robot
  // correctly refuses to shoot and the run returns NaN. That is the policy working, and it
  // hides the mechanism, which is what this tool is asking about.
  spec.flywheel.minLandProb = 0;
  const brain = new BuiltinTeleOp(spec, table ?? ShotTable.fromCsv(shotCsv));
  let loaded = 0;
  const step = (g = emptyGamepad()) => {
    while (world.robot.heldBalls().length < 7 && loaded < staging.length) {
      if (!world.robot.preload(world.balls, world.balls.balls[loaded])) break;
      loaded++;
    }
    world.setGamepads(g, emptyGamepad());
    world.step(brain.update(world.sensors(), g, world.seq));
  };
  step();
  const a = emptyGamepad();
  a.dpad_up = true;
  step(a);
  const fire = emptyGamepad();
  fire.right_bumper = true;
  for (let f = 0; f < 60 * (shots * 3 + 14) && world.robot.shots < shots; f++) step(fire);
  for (let f = 0; f < 60 * 6; f++) step();

  const log = world.snapshot().shots.filter((s) => s.result !== 'flight' && Number.isFinite(s.long_in));
  const longs = log.map((s) => s.long_in * 2.54);
  const mean = (x: number[]) => (x.length ? x.reduce((u, v) => u + v, 0) / x.length : NaN);
  const sd = (x: number[]) => {
    if (x.length < 2) return NaN;
    const m = mean(x);
    return Math.sqrt(x.reduce((u, v) => u + (v - m) ** 2, 0) / (x.length - 1));
  };
  return {
    n: longs.length,
    landed: log.filter((s) => s.result === 'cell').length,
    bias_cm: mean(longs),
    spread_cm: sd(longs),
    exit_mps: mean(log.map((s) => s.exitSpeed)),
  };
}

interface Knob {
  label: string;
  /** What it should do to the group if it is wired up. */
  expect: string;
  lo: (p: Params, r: RobotSpec) => void;
  hi: (p: Params, r: RobotSpec) => void;
}

const KNOBS: Knob[] = [
  {
    label: 'flywheel.scatter.speedFrac',
    expect: 'SPREAD grows',
    lo: (_p, r) => { r.flywheel.scatter.speedFrac = 0; },
    hi: (_p, r) => { r.flywheel.scatter.speedFrac = 0.04; },
  },
  {
    label: 'flywheel.scatter.angle_deg',
    expect: 'SPREAD grows',
    lo: (_p, r) => { r.flywheel.scatter.angle_deg = 0; },
    hi: (_p, r) => { r.flywheel.scatter.angle_deg = 3; },
  },
  {
    label: 'flywheel.k (exit efficiency)',
    expect: 'BIAS goes long',
    lo: (_p, r) => { r.flywheel.k = 0.42; },
    hi: (_p, r) => { r.flywheel.k = 0.48; },
  },
  {
    label: 'flywheel.r_fly_m',
    expect: 'BIAS goes long',
    lo: (_p, r) => { r.flywheel.r_fly_m = 0.045; },
    hi: (_p, r) => { r.flywheel.r_fly_m = 0.051; },
  },
  {
    label: 'ball.pollen.m_kg',
    expect: 'BIAS shifts (drag per mass)',
    lo: (p) => { p.ball.pollen.m_kg = 0.0224; },
    hi: (p) => { p.ball.pollen.m_kg = 0.0274; },
  },
  {
    label: 'ball.Cd (drag coefficient)',
    expect: 'BIAS goes short',
    lo: (p) => { p.ball.Cd = 0.3; },
    hi: (p) => { p.ball.Cd = 0.7; },
  },
  {
    label: 'env.rho (air density)',
    expect: 'BIAS goes short',
    lo: (p) => { p.env.rho = 0.9; },
    hi: (p) => { p.env.rho = 1.5; },
  },
  {
    label: 'env.g (gravity)',
    expect: 'BIAS goes short',
    lo: (p) => { p.env.g = 9.0; },
    hi: (p) => { p.env.g = 10.6; },
  },
  {
    label: 'turret.muzzleHeight_m',
    expect: 'BIAS goes long',
    lo: (_p, r) => { r.turret.muzzleHeight_m = 0.24; },
    hi: (_p, r) => { r.turret.muzzleHeight_m = 0.34; },
  },
];

export async function main(argv: string[] = []): Promise<void> {
  const i = argv.indexOf('--shots');
  const shots = i >= 0 ? Number(argv[i + 1]) : 14;
  const seed = 11;

  console.log('DOES EACH KNOB REACH THE BALL?');
  console.log(`  ${shots} shots a side from 1.4 m, SAME SEED both sides, so the scatter draws match`);
  console.log('  and any difference is the variable. Bias and spread are downrange, at the mouth.');
  console.log('');
  console.log('  variable                       expected        low -> high              verdict');

  for (const k of KNOBS) {
    const pLo = structuredClone(params) as unknown as Params;
    const rLo = structuredClone(robotSpec) as unknown as RobotSpec;
    k.lo(pLo, rLo);
    const pHi = structuredClone(params) as unknown as Params;
    const rHi = structuredClone(robotSpec) as unknown as RobotSpec;
    k.hi(pHi, rHi);

    const A = await group(pLo, rLo, shots, seed);
    const B = await group(pHi, rHi, shots, seed);
    const dBias = B.bias_cm - A.bias_cm;
    const dSpread = B.spread_cm - A.spread_cm;
    const dExit = B.exit_mps - A.exit_mps;
    // DEAD means EXACTLY nothing moved. The first version called a 1 cm shift dead, which
    // confuses "not wired up" with "wired up and genuinely small" -- a 10% mass change really
    // is worth about a centimetre here, because drag is a small term next to gravity.
    const still = Math.abs(dBias) < 0.05 && Math.abs(dSpread) < 0.05 && Math.abs(dExit) < 1e-6;
    const noData = !Number.isFinite(dBias);
    console.log(
      `  ${k.label.padEnd(30)} ${k.expect.padEnd(15)} ` +
      `bias ${dBias.toFixed(1).padStart(6)} cm  spread ${dSpread.toFixed(1).padStart(5)} cm  ` +
      `exit ${dExit.toFixed(3).padStart(6)} m/s   ` +
      `${noData ? 'NO SHOTS' : still ? 'DEAD — changes nothing' : 'live'}`,
    );
  }
  console.log('');
  console.log('  A knob that changes nothing is either not wired to the physics or genuinely');
  console.log('  irrelevant, and those need telling apart -- the expected column is the test.');
  console.log('');

  // WHAT IS A CONSISTENT SHOOTER WORTH? Everything in this project is downstream of the
  // exit-speed scatter, and the scatter is the one config block with no _source note at all.
  // Before buying consistency it is worth knowing the exchange rate.
  console.log('WHAT THE EXIT-SPEED SCATTER IS WORTH');
  console.log('  same spot, same seeds, gate OPEN so this measures the shooter and not the policy');
  console.log('');
  console.log('  speedFrac   landed of fired   group bias   group spread');
  for (const frac of [0, 0.005, 0.01, 0.015, 0.02, 0.03]) {
    let fired = 0;
    let landed = 0;
    const biases: number[] = [];
    const spreads: number[] = [];
    for (const sd2 of [11, 29, 47]) {
      const pp = structuredClone(params) as unknown as Params;
      const rr = structuredClone(robotSpec) as unknown as RobotSpec;
      rr.flywheel.scatter.speedFrac = frac;
      const g = await group(pp, rr, shots, sd2);
      fired += g.n;
      landed += g.landed;
      if (Number.isFinite(g.bias_cm)) biases.push(g.bias_cm);
      if (Number.isFinite(g.spread_cm)) spreads.push(g.spread_cm);
    }
    const mean = (x: number[]) => (x.length ? x.reduce((u, v) => u + v, 0) / x.length : NaN);
    console.log(
      `  ${(frac * 100).toFixed(1).padStart(8)}%   ${String(landed).padStart(6)} of ${String(fired).padStart(3)}   ` +
      `${mean(biases).toFixed(1).padStart(8)} cm   ${mean(spreads).toFixed(1).padStart(10)} cm` +
      `${frac === 0.015 ? '   <- shipped, and a guess' : ''}`,
    );
  }
}
