/**
 * Phase 4 acceptance: land rate versus range. Fire N POLLEN from each range and count how
 * many are still in the up CELL afterwards.
 *
 *   npm run tool -- tools/landrate.ts [--shots 8] [--epoly 0.45]
 */
import { readFileSync } from 'node:fs';
import params from '../config/params.json' with { type: 'json' };
import robotSpec from '../config/robot.json' with { type: 'json' };
import { World, initPhysics, emptyGamepad } from '../packages/core/src/physics/world.js';
import { BuiltinTeleOp, ShotTable } from '../packages/core/src/robot/builtinTeleOp.js';
import { loadLandCal } from '../packages/core/src/robot/loadCal.js';
import { inches } from '../packages/core/src/units.js';
import type { Params, RobotSpec, ShotRecord, Vec3 } from '../packages/core/src/types.js';

/**
 * The table on disk, read ONCE at module load. Fine for a normal run, a trap for a sweep
 * that rewrites the file per variant -- every variant would silently reuse the first one's
 * table. Callers that change the table pass it in.
 */
const diskTable = ShotTable.fromCsv(readFileSync(new URL('../java/teamcode/assets/shottable.csv', import.meta.url), 'utf8'));

/** Put the robot `range_in` from the up CELL somewhere that is actually on the field. */
function placeAt(world: World, spec: RobotSpec, range_in: number): boolean {
  const mouth = world.hives.red.upCellMouthWorld();
  const limit = world.geom.halfWidth_m - 0.35;
  const d = inches(range_in);
  // Up to and INCLUDING 90: the spot that exists at long range is the one square across the
  // field from the mouth, and stopping at 88 gave up one step before the only answer.
  for (let deg = 0; deg <= 90; deg += 2) {
    for (const sign of [1, -1]) {
      const a = (deg * Math.PI) / 180;
      const c: Vec3 = [mouth[0] + sign * d * Math.sin(a), spec.chassis.height_m / 2 + spec.chassis.clearance_m, mouth[2] + d * Math.cos(a)];
      if (Math.abs(c[0]) < limit && Math.abs(c[2]) < limit) {
        world.robot.place(c, (Math.atan2(mouth[0] - c[0], mouth[2] - c[2]) * 180) / Math.PI);
        return true;
      }
    }
  }
  return false;
}

export async function landRate(
  range_in: number,
  shots: number,
  over: Partial<Params['ball']> = {},
  seed = 11,
  table: ShotTable = diskTable,
): Promise<{ fired: number; landed: number; tips: number; placed: boolean; log: ShotRecord[] }> {
  await initPhysics();
  const p = structuredClone(params) as unknown as Params;
  Object.assign(p.ball, over);
  const spec = robotSpec as unknown as RobotSpec;
  const staging = Array.from({ length: shots + 2 }, () => ({ kind: 'pollen' as const, pos: [0, -5, 0] as Vec3 }));
  const world = new World({ params: p, robot: spec, staging, alliance: 'red', seed });
  for (const b of world.balls.balls) world.balls.park(b);
  if (!placeAt(world, spec, range_in)) return { fired: 0, landed: 0, tips: 0, placed: false, log: [] };

  const brain = new BuiltinTeleOp(spec, table, loadLandCal());
  const step = (g = emptyGamepad()) => {
    world.setGamepads(g, emptyGamepad());
    world.step(brain.update(world.sensors(), g, world.seq));
  };
  step();
  const press = emptyGamepad();
  press.a = true;
  step(press);
  for (let f = 0; f < 300; f++) step();

  const fire = emptyGamepad();
  fire.right_bumper = true;
  let loaded = 0;
  // WHAT SCORED, which is neither "what is left at the end" nor "what ever went in".
  //
  // Two wrong versions came before this one, in both directions:
  //
  //   `inCell + tips * 12` capped at shots fired SATURATED. A tip empties the CELL so it had
  //   to be credited, but a flat 12 plus the cap returns the shot count whatever happened --
  //   the sweep that caught it printed 100.0% for three different turret trims in a row.
  //
  //   Summing the RISES in the census fixed the saturation and OVERCOUNTED instead: it
  //   credits a ball that drops in, bounces straight back out and ends up on the floor. It
  //   read 92.7% on a robot whose settled per-shot rate (tools/landcal.ts, an independent
  //   measurement) was 61%.
  //
  // What scores in the game is what is sitting in the CELL when it matters: at the end, plus
  // whatever was in there at the instant of each tip, since a tip is the CELL doing its job
  // and dumping its contents. Both are counted, neither is assumed, and a ball that only
  // visits is counted by neither.
  // STILL NOT TRUSTWORTHY, and the disagreement is how you can tell. tools/gatecal.ts run
  // straight after tools/landcal.ts reported 1.0% for the same robot and the same table that
  // landcal measured at 66% settled per shot. Both cannot be right, and each is wrong in its
  // own direction:
  //
  //   HERE: `ballsInUpCell` counts one CELL of one rocker. The rocker ROCKS -- balls landing
  //   in it rotate it -- and a rotation that does not reach a scored tip carries them into
  //   the down CELL, where this reads zero. A run can score all afternoon and report nothing.
  //
  //   THERE: the shot log's `result === 'cell'` is set by world.trackBallStates, which tests
  //   `pointInCell` over EVERY cell of BOTH hives. It counts the down CELL, and it counts the
  //   opponent's hive.
  //
  // Neither is "balls in our up CELL at the buzzer". Fix that before believing any land rate
  // in this repo, including the ones the threshold in config/robot.json was set from.
  let prevInCell = world.hives.red.ballsInUpCell;
  let tipsSeen = world.hives.red.tips;
  let dumped = 0;
  const census = () => {
    if (world.hives.red.tips > tipsSeen) {
      dumped += prevInCell;   // the census from BEFORE the dump
      tipsSeen = world.hives.red.tips;
    }
    prevInCell = world.hives.red.ballsInUpCell;
  };
  for (let f = 0; f < 60 * (shots * 3 + 10) && world.robot.shots < shots; f++) {
    while (world.robot.hopper.length < 4 && loaded < shots) {
      if (!world.robot.preload(world.balls, world.balls.balls[loaded])) break;
      loaded++;
    }
    step(fire);
    census();
  }
  for (let f = 0; f < 60 * 5; f++) { step(); census(); }
  const landed = Math.min(world.robot.shots, dumped + world.hives.red.ballsInUpCell);
  return { fired: world.robot.shots, landed, tips: world.hives.red.tips, placed: true, log: world.snapshot().shots };
}

export async function main(argv: string[] = []): Promise<void> {
  const num = (k: string, d: number) => {
    const i = argv.indexOf(`--${k}`);
    return i >= 0 ? Number(argv[i + 1]) : d;
  };
  const shots = num('shots', 8);
  // Open the gate for a pure land-rate measurement. With it closed this tool reports the
  // rate of the shots the POLICY allowed, which is a different question and was being
  // printed under the same heading.
  const gate = num('gate', 0);
  const epoly = argv.includes('--epoly') ? num('epoly', 0.45) : undefined;
  console.log(`${shots} shots per range${epoly !== undefined ? `, e_poly=${epoly}` : ''}`);
  console.log('range_in  fired  landed  rate   tips');
  const only = argv.indexOf('--range');
  const ranges = only >= 0 ? [Number(argv[only + 1])] : [40, 55, 70, 85, 100, 115];
  for (const r of ranges) {
    const before = (robotSpec as unknown as RobotSpec).flywheel.minLandProb;
    (robotSpec as unknown as RobotSpec).flywheel.minLandProb = gate;
    let res;
    try {
      res = await landRate(r, shots, epoly !== undefined ? { e_poly: epoly } : {});
    } finally {
      (robotSpec as unknown as RobotSpec).flywheel.minLandProb = before;
    }
    if (!res.placed) {
      console.log(`${String(r).padStart(7)}   (no on-field spot at this range)`);
      continue;
    }
    if (!res.fired) {
      console.log(`${String(r).padStart(7)}   (placed, but the gate declined every shot)`);
      continue;
    }
    console.log(`${String(r).padStart(7)}  ${String(res.fired).padStart(5)}  ${String(res.landed).padStart(6)}  ${((res.landed / res.fired) * 100).toFixed(0).padStart(4)}%  ${String(res.tips).padStart(4)}`);
  }
}
