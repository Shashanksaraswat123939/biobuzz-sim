/**
 * SQUARE ON TO THE MOUTH, AT SPEED. Why does it not shoot?
 *
 *   npm run tool -- tools/frontcheck.ts [--speed 1.7]
 *
 * tools/obliquity.ts answers "why not from the SIDE" -- past 55 deg the CELL has no opening
 * left to throw at. That says nothing about standing DIRECTLY IN FRONT and moving fast, where
 * the aperture is wide open and the robot still refuses. This is that case: the robot on the
 * mouth's own normal, driven at a chosen speed along, across and around it, reporting what the
 * gate objected to for every frame it was up to speed and inside the table.
 *
 * The normal comes from hive.upCellMouthNormalWorld() rather than an axis guess -- the HIVE
 * sits at a corner and its mouth faces neither X nor Z, so "straight out in +z" is 40 deg off
 * the opening and measures the obliquity case all over again.
 */
import { readFileSync } from 'node:fs';
import params from '../config/params.json' with { type: 'json' };
import robotSpec from '../config/robot.json' with { type: 'json' };
import { World, initPhysics, emptyGamepad } from '../packages/core/src/physics/world.js';
import { BuiltinTeleOp, ShotTable } from '../packages/core/src/robot/builtinTeleOp.js';
import { loadLandCal } from '../packages/core/src/robot/loadCal.js';
import { RAD, M_TO_IN, inches } from '../packages/core/src/units.js';
import type { GamepadState, Params, RobotSpec, Vec3 } from '../packages/core/src/types.js';

const table = ShotTable.fromCsv(readFileSync(new URL('../java/teamcode/assets/shottable.csv', import.meta.url), 'utf8'));

/**
 * THE ANSWER AS A MAP: charge the goal at a given speed and see where the gate opens.
 *
 * Closing on the mouth, the ball's horizontal speed has to beat the robot's own by enough
 * that the launch still fits under the hood's 80 deg stop. The ball's horizontal is fixed by
 * the range (tools/flatbranch.ts: 1.43 m/s at 30 in rising to 3.89 at 94), so every chassis
 * speed has a range inside which no launch exists. This finds that range by driving it.
 */
async function sweepIn(want: number, seed: number, hoodTop?: number) {
  await initPhysics();
  const p = structuredClone(params) as unknown as Params;
  const spec = structuredClone(robotSpec) as unknown as RobotSpec;
  if (hoodTop !== undefined) spec.hood.angleRange_deg = [spec.hood.angleRange_deg[0], hoodTop];
  const pool = Array.from({ length: 200 }, () => ({ kind: 'pollen' as const, pos: [0, -5, 0] as Vec3 }));
  const w = new World({ params: p, robot: spec, staging: pool, alliance: 'red', seed });
  for (const b of w.balls.balls) w.balls.park(b);
  const hive = w.hives.red;
  const mouth = hive.upCellMouthWorld();
  const n = hive.upCellMouthNormalWorld();
  const start: Vec3 = [mouth[0] + n[0] * inches(132), spec.chassis.height_m / 2 + spec.chassis.clearance_m, mouth[2] + n[2] * inches(132)];
  w.robot.place(start, Math.atan2(mouth[0] - start[0], mouth[2] - start[2]) * RAD);
  const brain = new BuiltinTeleOp(spec, table, loadLandCal());
  brain.state.firing = true;
  let loaded = 0;
  // 10 in buckets from 30 to 130.
  const open = new Array(10).fill(0), tot = new Array(10).fill(0), why: Record<number, Record<string, number>> = {};
  for (let i = 0; i < 60 * 20; i++) {
    while (w.robot.heldBalls().length < spec.hopper.capacity && loaded < pool.length) {
      if (!w.robot.preload(w.balls, w.balls.balls[loaded])) break;
      loaded++;
    }
    const g: GamepadState = emptyGamepad();
    g.left_stick_y = -1;   // forward, at the mouth
    // Hold the requested speed rather than flooring it, so the label means something.
    const v0 = w.robot.body.linvel();
    if (Math.hypot(v0.x, v0.z) > want) g.left_stick_y = 0;
    w.setGamepads(g, emptyGamepad());
    w.step(brain.update(w.sensors(), g, w.seq, 1 / 60));
    const c = w.robot.pos;
    const rng = Math.hypot(mouth[0] - c[0], mouth[2] - c[2]) * M_TO_IN;
    if (rng <= 34) break;
    const v = w.robot.body.linvel();
    if (Math.hypot(v.x, v.z) < want * 0.75) continue;
    const b = Math.floor((rng - 30) / 10);
    if (b < 0 || b > 9) continue;
    tot[b]++;
    if (!brain.state.hold) open[b]++;
    else {
      why[b] = why[b] ?? {};
      const k = brain.state.hold.replace(/-?[\d.]+/g, 'N');
      why[b][k] = (why[b][k] ?? 0) + 1;
    }
  }
  const cells = tot.map((t, b) => (t < 4 ? '  -' : `${Math.round((open[b] / t) * 100)}%`.padStart(3)));
  console.log(`  ${want.toFixed(1)} m/s${hoodTop === undefined ? '    ' : ` h${hoodTop}`}   ${cells.join('   ')}    ${String(w.robot.shots).padStart(3)} fired, ${String(w.landedInUpCell('red')).padStart(3)} in`);
  // The reason it shut, at the closest bucket that had one.
  for (let b = 0; b < 10; b++) {
    if (tot[b] >= 4 && open[b] / tot[b] < 0.5 && why[b]) {
      const top = Object.entries(why[b]).sort((x, y) => y[1] - x[1])[0];
      console.log(`             closed at ${30 + b * 10}-${40 + b * 10} in: ${top[0]}`);
      break;
    }
  }
}

export async function main(): Promise<void> {
  console.log('\nCHARGING THE GOAL SQUARE ON. Share of loops the gate is OPEN, by range.\n');
  const head = '           30-40 40-50 50-60 60-70 70-80 80-90 90-00 00-10 10-20 20-30 in';
  console.log(head);
  for (const v of [1.0, 1.4, 1.7]) await sweepIn(v, 900 + Math.round(v * 10));
  console.log('\n  The hood stops at 80 deg. Closing faster than the ball\'s own horizontal speed needs');
  console.log('  a launch thrown BACKWARDS over the shoulder -- which the +-270 deg turret can do and');
  console.log('  the hood cannot. Same runs with a taller hood:\n');
  console.log(head);
  for (const top of [85, 88]) {
    for (const v of [1.4, 1.7]) await sweepIn(v, 900 + Math.round(v * 10), top);
  }
  console.log('');
}
