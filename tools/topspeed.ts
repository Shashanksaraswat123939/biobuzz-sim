/**
 * HOW FAST DOES IT ACTUALLY GO, and does the speed cap hold?
 *
 *   npm run tool -- tools/topspeed.ts
 *
 * The drive gear (Y/A) is a POWER fraction, and power is not speed: half power on a mecanum
 * against rolling resistance is not half the ground speed. `drivetrain.maxSpeed_mps` is the
 * cap in the units a driver actually thinks in, so it has to be measured against the thing it
 * claims to limit rather than assumed to work.
 *
 * Free speed is measured down the long axis of an empty field, and again sideways. Both are
 * reported because a mecanum is usually slower strafing than driving; this model is not (2.11
 * against 2.09), so the cap does not need to know which way the robot is pointed. The cap is
 * then swept, and both the PEAK and the speed it settles at are reported -- a limiter that
 * only holds in the steady state still lets the robot lunge past it on every launch.
 */
import params from '../config/params.json' with { type: 'json' };
import robotSpec from '../config/robot.json' with { type: 'json' };
import { World, initPhysics, emptyGamepad } from '../packages/core/src/physics/world.js';
import { BuiltinTeleOp, ShotTable } from '../packages/core/src/robot/builtinTeleOp.js';
import { loadLandCal } from '../packages/core/src/robot/loadCal.js';
import { readFileSync } from 'node:fs';
import type { GamepadState, Params, RobotSpec, Vec3 } from '../packages/core/src/types.js';

const table = ShotTable.fromCsv(readFileSync(new URL('../java/teamcode/assets/shottable.csv', import.meta.url), 'utf8'));

async function drive(dir: 'forward' | 'sideways', cap: number | null, seconds = 8): Promise<{ peak: number; held: number }> {
  await initPhysics();
  const p = structuredClone(params) as unknown as Params;
  const spec = structuredClone(robotSpec) as unknown as RobotSpec;
  if (cap !== null) spec.drivetrain.maxSpeed_mps = cap;
  const w = new World({ params: p, robot: spec, staging: [], alliance: 'red', seed: 7 });
  for (const b of w.balls.balls) w.balls.park(b);
  // A clear run: one corner of the field, pointed down it, nothing in the way.
  const half = w.geom.halfWidth_m - 0.5;
  const y = spec.chassis.height_m / 2 + spec.chassis.clearance_m;
  // NO DIAGONAL RUN. It looks like 4.6 m of runway against the straight line's 2.7 and it is
  // not clear: corner to corner goes through the middle of the field, where both HIVEs are.
  // Measured that way free speed reads 1.69 m/s -- LOWER than the short straight run, which
  // is the collision and not the drivetrain. 2.7 m down the middle is the longest clear line
  // this field has, and the robot is still accelerating at the end of it, so what this
  // reports is a LOWER BOUND on free speed and is labelled as one.
  const start: Vec3 = dir === 'forward' ? [0, y, -half] : [-half, y, 0];
  w.robot.place(start, 0);
  const brain = new BuiltinTeleOp(spec, table, loadLandCal());
  const run_m = 2 * half - 0.7;
  let peak = 0, tail = 0, nTail = 0;
  for (let i = 0; i < 60 * seconds; i++) {
    // Stop short of the far wall: past that the robot is decelerating into a collision, and
    // averaging that in reports a limiter working when what is happening is a crash. By
    // distance travelled rather than by coordinate, so it does not depend on which way the
    // stick maps onto the world.
    const pos = w.robot.pos;
    if (Math.hypot(pos[0] - start[0], pos[2] - start[2]) > run_m) break;
    const g: GamepadState = emptyGamepad();
    if (dir === 'sideways') g.left_stick_x = -1; else g.left_stick_y = -1;
    w.setGamepads(g, emptyGamepad());
    w.step(brain.update(w.sensors(), g, w.seq, 1 / 60));
    const v = w.robot.body.linvel();
    const sp = Math.hypot(v.x, v.z);
    peak = Math.max(peak, sp);
    if (i > 45) { tail += sp; nTail++; }   // past the launch: what it settles at
  }
  return { peak, held: nTail ? tail / nTail : 0 };
}

export async function main(): Promise<void> {
  console.log('\nHOW FAST IT ACTUALLY GOES. Full stick, clear field, peak ground speed.\n');
  const fwd = await drive('forward', null);
  const side = await drive('sideways', null);
  console.log(`  driving forward, 2.7 m of clear run   ${fwd.peak.toFixed(2)} m/s peak, ${fwd.held.toFixed(2)} mean`);
  console.log(`  strafing, same run                    ${side.peak.toFixed(2)} m/s peak, ${side.held.toFixed(2)} mean`);
  console.log(`
  Still accelerating at the boards, so free speed is AT LEAST ${Math.max(fwd.peak, side.peak).toFixed(2)} m/s.`);
  console.log('  The field has no longer clear straight: the diagonal goes through both HIVEs.');
  console.log('\n  With drivetrain.maxSpeed_mps set. Asked for / actually reached, driving forward:\n');
  for (const cap of [0.8, 1.2, 1.5, 1.7, 2.0]) {
    const got = await drive('forward', cap);
    const over = got.peak - cap;
    console.log(`  ${cap.toFixed(2)} asked  ->  peak ${got.peak.toFixed(2)}, settles ${got.held.toFixed(2)}   ${over > 0.02 ? `OVER by ${over.toFixed(2)}` : 'holds'}   delivers ${((got.held / cap) * 100).toFixed(0)}%`);
  }
  console.log('');
}
