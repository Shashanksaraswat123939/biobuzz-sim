/**
 * Why does the HIVE-tip scenario fail with the flatter table? Two suspects, separated.
 *
 *   npm run tool -- tools/retention.ts [--seconds 70]
 *
 * The tip test fires for 70 s at 75 in and needs 12 POLLEN sitting in the up CELL. With the
 * flatter shot table it gets about 6, and there are two completely different explanations:
 *
 *   SHOT RATE   it is not firing often enough. 13 shots in 70 s is 5.4 s a shot against a
 *               1.5 s configured cycle, and 13 shots at any plausible land rate cannot
 *               reach 12. If this is the cause, the shot table is innocent.
 *
 *   RETENTION   it fires plenty but the balls do not stay. tools/entrycheck.ts measured
 *               retention over FOUR SECONDS in an EMPTY pocket, which is not the quantity
 *               that matters -- a ball has to still be there at the buzzer, in a pocket
 *               with other balls in it. A flat shot lands nearer the mouth than a steep
 *               one, so it has further to roll back down and more chance of being knocked
 *               out by the next arrival.
 *
 * These have opposite fixes, so guessing between them is how a day gets wasted. This runs
 * the same scenario on both tables and reports shots fired AND the CELL population over
 * time, which tells them apart directly: if both tables fire alike and only the new one
 * leaks, it is retention; if the new one simply fires less, it is not.
 */
import { readFileSync, writeFileSync } from 'node:fs';
import params from '../config/params.json' with { type: 'json' };
import robotSpec from '../config/robot.json' with { type: 'json' };
import { World, initPhysics, emptyGamepad } from '../packages/core/src/physics/world.js';
import { BuiltinTeleOp, ShotTable } from '../packages/core/src/robot/builtinTeleOp.js';
import { buildTable, toCsv, loadEntry } from './shottable.js';
import { inches } from '../packages/core/src/units.js';
import type { Params, RobotSpec, Vec3 } from '../packages/core/src/types.js';

export interface RunResult {
  name: string;
  shots: number;
  finalInCell: number;
  peakInCell: number;
  /** Balls that left the CELL without a tip explaining it. */
  lost: number;
  /** Tips. A tip empties the CELL on purpose and must not be counted as leakage. */
  tips: number;
  /** CELL population sampled every 10 s. */
  trace: number[];
  meanArrival_mps: number;
  meanDescent_deg: number;
}

async function run(name: string, table: ShotTable, seconds: number, range_in = 75): Promise<RunResult> {
  await initPhysics();
  const p = structuredClone(params) as unknown as Params;
  const spec = structuredClone(robotSpec) as unknown as RobotSpec;
  // Gate open: this is a question about the mechanism and the pocket, not about the policy.
  spec.flywheel.minLandProb = 0;
  const staging = Array.from({ length: 60 }, () => ({ kind: 'pollen' as const, pos: [0, -5, 0] as Vec3 }));
  const world = new World({ params: p, robot: spec, staging, alliance: 'red', seed: 5 });
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
  if (!spot) throw new Error('no on-field spot');
  world.robot.place(spot, (Math.atan2(mouth[0] - spot[0], mouth[2] - spot[2]) * 180) / Math.PI);
  for (let i = 0; i < 6; i++) world.robot.preload(world.balls, world.balls.balls[i]);

  const brain = new BuiltinTeleOp(spec, table);
  const step = (g = emptyGamepad()) => {
    world.setGamepads(g, emptyGamepad());
    world.step(brain.update(world.sensors(), g, world.seq));
  };
  step();
  const press = emptyGamepad();
  press.a = true;
  step(press);

  const fire = emptyGamepad();
  fire.right_bumper = true;
  let loaded = 6;
  let peak = 0;
  let lost = 0;
  let tipsSeen = 0;
  let prev = 0;
  const trace: number[] = [];
  const frames = Math.round(seconds * 60);
  for (let f = 0; f < frames; f++) {
    while (world.robot.heldBalls().length < 6 && loaded < staging.length) {
      if (!world.robot.preload(world.balls, world.balls.balls[loaded])) break;
      loaded++;
    }
    step(fire);
    const n = world.hives.red.ballsInUpCell;
    // A TIP empties the CELL, and that is the goal, not a loss. Counting every decrease as
    // a ball lost scored the best possible outcome as the worst -- the first run of this
    // reported 19 "lost" for a table that was tipping the hive repeatedly.
    if (n < prev && world.hives.red.tips === tipsSeen) lost += prev - n;
    tipsSeen = world.hives.red.tips;
    prev = n;
    peak = Math.max(peak, n);
    if (f % 600 === 0) trace.push(n);
  }
  for (let f = 0; f < 60 * 8; f++) step();

  const log = world.snapshot().shots.filter((s) => s.result !== 'flight');
  const mean = (a: number[]) => (a.length ? a.reduce((x, y) => x + y, 0) / a.length : 0);
  return {
    name,
    shots: world.robot.shots,
    finalInCell: world.hives.red.ballsInUpCell,
    peakInCell: peak,
    lost,
    tips: world.hives.red.tips,
    trace,
    meanArrival_mps: 0,
    meanDescent_deg: mean(log.map((s) => s.hoodDeg)),
  };
}

export async function main(argv: string[] = []): Promise<void> {
  const i = argv.indexOf('--seconds');
  const seconds = i >= 0 ? Number(argv[i + 1]) : 70;

  const p = params as unknown as Params;
  const spec = robotSpec as unknown as RobotSpec;
  const entry = loadEntry();
  if (!entry) throw new Error('config/entry.json missing');

  const flatRows = buildTable(p, spec, 30, 150, 4, entry, 'pLand');
  const tallRows = buildTable(p, spec, 30, 150, 4, null, 'margin');
  const tablePath = new URL('../java/teamcode/assets/shottable.csv', import.meta.url);
  const original = readFileSync(tablePath, 'utf8');

  console.log('SHOT RATE vs RETENTION — which one is failing the tip scenario?');
  console.log(`  ${seconds} s at 75 in, hopper topped up, gate open. Both tables, same seed.`);
  console.log('');

  const out: RunResult[] = [];
  try {
    for (const [nm, rows] of [['steep (margin)', tallRows], ['flat (margin x entry)', flatRows]] as const) {
      writeFileSync(tablePath, toCsv(rows));
      out.push(await run(nm, ShotTable.fromCsv(toCsv(rows)), seconds));
    }
  } finally {
    writeFileSync(tablePath, original);
  }

  console.log('  table                  shots   s/shot   peak in CELL   tips   lost (not via a tip)');
  for (const r of out) {
    console.log(
      `  ${r.name.padEnd(22)} ${String(r.shots).padStart(5)}   ${(seconds / Math.max(1, r.shots)).toFixed(1).padStart(6)}   ` +
      `${String(r.peakInCell).padStart(12)}   ${String(r.tips).padStart(4)}   ${String(r.lost).padStart(20)}`,
    );
  }
  console.log('');
  console.log('  CELL population every 10 s:');
  for (const r of out) console.log(`  ${r.name.padEnd(22)} ${r.trace.join(' -> ')}`);
  console.log('');

  const [tall, flat] = out;
  const rateGap = Math.abs(tall.shots - flat.shots) / Math.max(tall.shots, flat.shots);
  console.log('VERDICT');
  if (rateGap > 0.25) {
    console.log(`  SHOT RATE. The two tables fire ${tall.shots} and ${flat.shots} times -- a ${(rateGap * 100).toFixed(0)}% difference.`);
    console.log('  Retention cannot be blamed for a gap that opens before the ball is in the air.');
  } else if (flat.lost > tall.lost + 2) {
    console.log(`  RETENTION. Both fire about the same (${tall.shots} vs ${flat.shots}) but the flat table`);
    console.log(`  loses ${flat.lost} balls out of the CELL against ${tall.lost}. The entry model measures the`);
    console.log('  wrong quantity: 4 s in an empty pocket, when what matters is still being there at the end.');
  } else {
    console.log(`  NEITHER cleanly. shots ${tall.shots} vs ${flat.shots}, lost ${tall.lost} vs ${flat.lost},`);
    console.log(`  peak ${tall.peakInCell} vs ${flat.peakInCell}. The scenario may simply not fit 12 balls in ${seconds} s`);
    console.log('  at this shot rate whatever the table does, in which case the test is the thing to fix.');
  }
}
