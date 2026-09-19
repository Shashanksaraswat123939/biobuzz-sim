/**
 * Does the autonomous routine actually score? One line per seed.
 *
 *   npm run tool -- tools/autocheck.ts [--seeds 5]
 *
 * Runs `AutoRoutine` through `BuiltinTeleOp` for a full 30 s AUTO period, exactly as the UI's
 * Auto mode does, and reports the three things AUTO is scored on: LEAVE, what landed in the up
 * CELL, and PARK. Nothing here drives the robot except the routine's own gamepad frames.
 *
 * It exists because the UI cannot answer this question quickly: the browser only steps the
 * world while it is being painted, so watching a 30 s routine takes 30 s of staring at it, and
 * one run of one seed is an anecdote anyway.
 */
import params from '../config/params.json' with { type: 'json' };
import robotSpec from '../config/robot.json' with { type: 'json' };
import staging from '../assets/staging.json' with { type: 'json' };
import { readFileSync } from 'node:fs';
import { World, initPhysics } from '../packages/core/src/physics/world.js';
import { BuiltinTeleOp, ShotTable } from '../packages/core/src/robot/builtinTeleOp.js';
import { AutoRoutine } from '../packages/core/src/robot/autoRoutine.js';
import { loadLandCal } from '../packages/core/src/robot/loadCal.js';
import type { BallKind, Params, RobotSpec, Vec3 } from '../packages/core/src/types.js';

const table = ShotTable.fromCsv(readFileSync(new URL('../java/teamcode/assets/shottable.csv', import.meta.url), 'utf8'));
const balls = (staging.balls as { kind: string; pos: number[] }[]).map((b) => ({ kind: b.kind as BallKind, pos: b.pos as Vec3 }));

interface Run { seed: number; fired: number; landed: number; left: boolean; parked: boolean; phase: string; note: string; score: number }

async function one(seed: number): Promise<Run> {
  const p = structuredClone(params) as unknown as Params;
  const spec = structuredClone(robotSpec) as unknown as RobotSpec;
  const world = new World({ params: p, robot: spec, staging: balls, alliance: 'red', seed, preload: spec.hopper.capacity });
  const brain = new BuiltinTeleOp(spec, table, loadLandCal());
  const hive = world.hives.red;
  const zone = world.geom.zones.find((z) => z.name === 'LOADING' && z.alliance === 'red')!;
  const ranges = table.rows.map((r) => r.range_in);
  const routine = new AutoRoutine({
    mouth: hive.upCellMouthWorld(),
    mouthNormal: hive.upCellMouthNormalWorld(),
    loading: [zone.min[0], zone.min[2], zone.max[0], zone.max[2]],
    halfWidth_m: world.geom.halfWidth_m,
    band_in: [Math.min(...ranges), Math.max(...ranges)],
  });

  // Whatever is already in the CELL at the buzzer -- the manual stages NECTAR there
  // (10.3.1 B.i) and the routine did not put it there.
  //
  // READ, NOT ASSUMED. tools/landrate.ts hardcoded this as 3 against a rig that staged none,
  // and the constant three-shot error read as a believable rate that drifted with the sample
  // size: 4 shots scored 1 (25%), 12 scored 8 (73%), same robot and same spot. Here the
  // field IS staged, so 3 is probably right -- but a number that is probably right is worth
  // one line to make certainly right.
  const staged = world.landedInUpCell('red');
  world.clock.start();
  const dt = p.sim.dt * p.sim.substepsPerFrame;
  while (world.clock.period === 'AUTO') {
    const s = world.sensors();
    const g = routine.update(s, dt, world.robot.shots, world.clock.remaining, brain.target());
    world.setGamepads(g, { ...g, a: false, b: false, right_bumper: false });
    world.step(brain.update(s, g, world.seq, dt));
  }
  for (let f = 0; f < 120; f++) world.step({ seq: 0, motors: {}, servos: {} });   // let the last shot settle

  const sc = world.scorer.state.red;
  return {
    seed,
    fired: world.robot.shots,
    landed: Math.max(0, world.landedInUpCell('red') - staged),
    left: sc.leave,
    parked: sc.park,
    phase: routine.phase,
    note: routine.note,
    score: sc.total,
  };
}

export async function main(argv: string[] = []): Promise<void> {
  await initPhysics();
  const i = argv.indexOf('--seeds');
  const n = i >= 0 ? Number(argv[i + 1]) : 5;
  console.log('\nTHE AUTONOMOUS ROUTINE, one full 30 s AUTO period per seed.\n');
  console.log('  seed   fired  landed   LEAVE   PARK   points   ended');
  const runs: Run[] = [];
  for (let k = 0; k < n; k++) {
    const r = await one(7 + k * 18);
    runs.push(r);
    console.log(
      `  ${String(r.seed).padStart(4)}   ${String(r.fired).padStart(5)}  ${String(r.landed).padStart(6)}` +
      `   ${(r.left ? 'yes' : 'NO ').padStart(5)}  ${(r.parked ? 'yes' : 'NO ').padStart(5)}` +
      `   ${String(r.score).padStart(6)}   ${r.phase}`,
    );
  }
  const mean = (a: number[]) => a.reduce((x, y) => x + y, 0) / (a.length || 1);
  console.log('');
  console.log(`  mean: ${mean(runs.map((r) => r.fired)).toFixed(1)} fired, ${mean(runs.map((r) => r.landed)).toFixed(1)} landed, ` +
    `${mean(runs.map((r) => r.score)).toFixed(1)} points`);
  console.log(`  LEAVE ${runs.filter((r) => r.left).length}/${runs.length}, PARK ${runs.filter((r) => r.parked).length}/${runs.length}`);
  console.log('');
  console.log('  `points` is LEAVE + PARK only. Balls in the CELL are counted at the BUZZER, not');
  console.log('  at the end of AUTO -- Scorer.finalise is what credits them -- so they show up in');
  console.log('  `landed` here and in the score only if the match is played out.');
  console.log(`  last note: ${runs[runs.length - 1]?.note ?? '-'}\n`);
}
