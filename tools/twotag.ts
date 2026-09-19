/**
 * IS THE OTHER TAG LOOKING AT US? Both CELL panels, measured while the robot scores.
 *
 *   npm run tool -- tools/twotag.ts
 *
 * The robot blinds itself by scoring: each ball rocks the rocker, and the AprilTag panel on
 * the up CELL swings to about 85 deg of incidence, past what a camera can decode (93600a8).
 *
 * But the HIVE has TWO cells and TWO panels, bolted to the same rocker at different angles.
 * If one is rotating away, the other should be rotating TOWARD the camera. The simulator only
 * ever offers the up cell's tag (World.tagTruthFor), so the robot has never been allowed to
 * look at the other one. This measures whether there is anything to look at.
 */
import { readFileSync } from 'node:fs';
import params from '../config/params.json' with { type: 'json' };
import robotSpec from '../config/robot.json' with { type: 'json' };
import { World, initPhysics, emptyGamepad } from '../packages/core/src/physics/world.js';
import { BuiltinTeleOp, ShotTable } from '../packages/core/src/robot/builtinTeleOp.js';
import { loadLandCal } from '../packages/core/src/robot/loadCal.js';
import { RAD, inches } from '../packages/core/src/units.js';
import type { GamepadState, Params, RobotSpec, Vec3 } from '../packages/core/src/types.js';

const table = ShotTable.fromCsv(readFileSync(new URL('../java/teamcode/assets/shottable.csv', import.meta.url), 'utf8'));

export async function main(): Promise<void> {
  await initPhysics();
  const p = structuredClone(params) as unknown as Params;
  const spec = structuredClone(robotSpec) as unknown as RobotSpec;
  const pool = Array.from({ length: 80 }, () => ({ kind: 'pollen' as const, pos: [0, -5, 0] as Vec3 }));
  const w = new World({ params: p, robot: spec, staging: pool, alliance: 'red', seed: 5 });
  for (const b of w.balls.balls) w.balls.park(b);
  const hive = w.hives.red;
  const mouth = hive.upCellMouthWorld();
  const n0 = hive.upCellMouthNormalWorld();
  const hn = Math.hypot(n0[0], n0[2]) || 1;
  const at: Vec3 = [mouth[0] + (n0[0] / hn) * inches(45), spec.chassis.height_m / 2 + spec.chassis.clearance_m, mouth[2] + (n0[2] / hn) * inches(45)];
  w.robot.place(at, Math.atan2(mouth[0] - at[0], mouth[2] - at[2]) * RAD);
  const brain = new BuiltinTeleOp(spec, table, loadLandCal());
  brain.state.firing = true;

  // Incidence of a panel, measured from the panel to the robot, exactly as TagCamera does.
  const inc = (tagW: Vec3, nrm: Vec3): number => {
    const dx = tagW[0] - w.robot.pos[0], dz = tagW[2] - w.robot.pos[2];
    const d = Math.hypot(dx, dz) || 1;
    const nn = Math.hypot(nrm[0], nrm[2]) || 1;
    return Math.acos(Math.max(-1, Math.min(1, (-dx * nrm[0] + -dz * nrm[2]) / (d * nn)))) * RAD;
  };
  const cap = spec.sensors.tag.maxIncidence_deg;
  console.log(`\nBOTH PANELS WHILE SCORING, from 45 in square on. Decode limit is ${cap} deg.\n`);
  console.log('   time   in CELL   up-cell tag   down-cell tag   either readable?');
  let loaded = 0;
  for (let i = 0; i < 60 * 25; i++) {
    while (w.robot.heldBalls().length < spec.hopper.capacity && loaded < pool.length) {
      if (!w.robot.preload(w.balls, w.balls.balls[loaded])) break;
      loaded++;
    }
    const g: GamepadState = emptyGamepad();
    w.setGamepads(g, emptyGamepad());
    w.step(brain.update(w.sensors(), g, w.seq, 1 / 60));
    if (i % 150 === 0) {
      const up = inc(hive.upCellTagWorld(), hive.upCellTagNormalWorld());
      const dn = inc(hive.downCellTagWorld(), hive.downCellTagNormalWorld());
      const ok = Math.min(up, dn) <= cap;
      console.log(`  ${(i / 60).toFixed(1).padStart(5)} s   ${String(w.landedInUpCell('red')).padStart(7)}   ${up.toFixed(0).padStart(9)}°   ${dn.toFixed(0).padStart(11)}°   ${ok ? 'YES' : 'no'}`);
    }
  }
  console.log('');
}
