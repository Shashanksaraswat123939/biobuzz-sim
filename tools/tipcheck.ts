/**
 * WHAT DOES THE ROBOT DO WHEN THE HIVE GOES OVER?
 *
 *   npm run tool -- tools/tipcheck.ts
 *
 * A TIP swaps which CELL is up, and the new one opens the other way -- so a robot that was
 * square onto the goal is suddenly behind it. It has exactly one legitimate way to find
 * out: each CELL carries its own AprilTag, so the ID in view changes. The catch is that
 * from the old shooting position the new tag usually CANNOT BE SEEN at all, so what the
 * robot really gets is not "tag 2 now" but "no tag".
 *
 * This fires until the rocker goes over and then reports, second by second, what the robot
 * believed and whether it kept shooting.
 */
import { readFileSync } from 'node:fs';
import params from '../config/params.json' with { type: 'json' };
import robotSpec from '../config/robot.json' with { type: 'json' };
import { World, initPhysics, emptyGamepad } from '../packages/core/src/physics/world.js';
import { BuiltinTeleOp, ShotTable } from '../packages/core/src/robot/builtinTeleOp.js';
import { loadLandCal } from '../packages/core/src/robot/loadCal.js';
import { RAD, inches } from '../packages/core/src/units.js';
import type { Params, RobotSpec, Vec3 } from '../packages/core/src/types.js';

const table = ShotTable.fromCsv(readFileSync(new URL('../java/teamcode/assets/shottable.csv', import.meta.url), 'utf8'));

export async function main(): Promise<void> {
  await initPhysics();
  const p = structuredClone(params) as unknown as Params;
  const spec = structuredClone(robotSpec) as unknown as RobotSpec;
  const pool = Array.from({ length: 200 }, () => ({ kind: 'pollen' as const, pos: [0, -5, 0] as Vec3 }));
  const w = new World({ params: p, robot: spec, staging: pool, alliance: 'red', seed: 5 });
  for (const b of w.balls.balls) w.balls.park(b);
  const mouth = w.hives.red.upCellMouthWorld();
  const z = mouth[2] + inches(40);
  w.robot.place([mouth[0], spec.chassis.height_m / 2 + spec.chassis.clearance_m, z], Math.atan2(0, -1) * RAD);
  const brain = new BuiltinTeleOp(spec, table, loadLandCal());
  brain.state.firing = true;
  let loaded = 0;
  let tips = 0, shotsAtTip = -1, creditedAtTip = -1;
  console.log('\n     t   tips   shots   credited   tag   aimErr   hold');
  for (let i = 0; i < 60 * 45; i++) {
    while (w.robot.heldBalls().length < spec.hopper.capacity && loaded < pool.length) {
      if (!w.robot.preload(w.balls, w.balls.balls[loaded])) break;
      loaded++;
    }
    const s = w.sensors();
    w.setGamepads(emptyGamepad(), emptyGamepad());
    w.step(brain.update(s, emptyGamepad(), w.seq, 1 / 60));
    if (w.hives.red.tips > tips) {
      tips = w.hives.red.tips;
      shotsAtTip = w.robot.shots;
      creditedAtTip = w.landedInUpCell('red');
      console.log(`  ---- THE HIVE WENT OVER at t=${(i / 60).toFixed(1)} s: ${shotsAtTip} fired, ${creditedAtTip} credited ----`);
    }
    if (i % 60 === 0 || (tips > 0 && i % 30 === 0 && i < 60 * 45)) {
      const st = brain.state;
      console.log(`  ${(i / 60).toFixed(0).padStart(4)}   ${String(tips).padStart(4)}   ${String(w.robot.shots).padStart(5)}   ${String(w.landedInUpCell('red')).padStart(8)}   ${s.tag ? String(s.tag.id) : '- '}   ${st.turretAimErrDeg.toFixed(1).padStart(6)}   ${st.hold || 'CLEAR TO FIRE'}`);
    }
  }
  const after = w.robot.shots - shotsAtTip;
  const gained = w.landedInUpCell('red') - creditedAtTip;
  console.log(`\n  AFTER THE TIP: ${after} more balls fired, ${gained} of them credited.`);
  console.log('  Every ball fired after a tip that is not credited went into the back of a');
  console.log('  goal the robot could no longer score in.\n');
}
