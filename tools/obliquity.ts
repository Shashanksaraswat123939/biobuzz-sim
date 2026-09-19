/**
 * IS "OFF THE OPENING" A POLICY OR A WALL? Measured by firing, at every bearing.
 *
 *   npm run tool -- tools/obliquity.ts [--shots 10]
 *
 * THIS TOOL USED TO LIE, and it is worth saying how, because it cost 20 squares of shot zone.
 * It computed
 *
 *     clear = width*cos(beta) - depth*sin(beta)
 *
 * printed the table, and concluded that past 55 deg the CELL has no opening at all. It never
 * fired a ball. On the strength of it the same subtraction went into BuiltinTeleOp, into
 * LandProbability.java and into tools/shotzone.ts (84525e6), and the green zone fell from 52
 * squares to 32.
 *
 * The formula is the clear straight line THROUGH a slot -- what a ball would need if it had
 * to reach the back wall untouched. A ball does not: it crosses the MOUTH and stays in the
 * pocket, and the depth is behind the mouth. Arriving against the far wall is a ball in the
 * CELL. Measured, the deleted squares land 98% (tools/lostzone.ts).
 *
 * So this now stands the robot at a real off-axis angle and shoots, with the probability gate
 * and the open-angle cap forced out of the way, and prints what lands against what cos(beta)
 * predicts. A model in a tool that never fires is an opinion.
 */
import params from '../config/params.json' with { type: 'json' };
import robotSpec from '../config/robot.json' with { type: 'json' };
import { readFileSync } from 'node:fs';
import { World, initPhysics, emptyGamepad } from '../packages/core/src/physics/world.js';
import { BuiltinTeleOp, ShotTable } from '../packages/core/src/robot/builtinTeleOp.js';
import { loadLandCal } from '../packages/core/src/robot/loadCal.js';
import { RAD, DEG, inches } from '../packages/core/src/units.js';
import type { GamepadState, Params, RobotSpec, Vec3 } from '../packages/core/src/types.js';

const table = ShotTable.fromCsv(readFileSync(new URL('../java/teamcode/assets/shottable.csv', import.meta.url), 'utf8'));

async function atAngle(betaDeg: number, range_in: number, shots: number, seed: number) {
  await initPhysics();
  const p = structuredClone(params) as unknown as Params;
  const spec = structuredClone(robotSpec) as unknown as RobotSpec;
  // The gate's opinion is the thing under test, so it does not get a vote. The scatter, the
  // rocker, the lip and the balls already in the pocket all stay real.
  spec.flywheel.minLandProb = 0;
  spec.turret.fireOpenCap_deg = 180;
  const pool = Array.from({ length: shots + 4 }, () => ({ kind: 'pollen' as const, pos: [0, -5, 0] as Vec3 }));
  const w = new World({ params: p, robot: spec, staging: pool, alliance: 'red', seed });
  for (const b of w.balls.balls) w.balls.park(b);
  const hive = w.hives.red;
  const mouth = hive.upCellMouthWorld();
  const n = hive.upCellMouthNormalWorld();
  // Round the mouth's normal by beta, at a fixed range.
  const a = Math.atan2(n[0], n[2]) + betaDeg * (Math.PI / 180);
  const d = inches(range_in);
  const at: Vec3 = [mouth[0] + Math.sin(a) * d, spec.chassis.height_m / 2 + spec.chassis.clearance_m, mouth[2] + Math.cos(a) * d];
  if (Math.abs(at[0]) > w.geom.halfWidth_m - 0.3 || Math.abs(at[2]) > w.geom.halfWidth_m - 0.3) return null;
  w.robot.place(at, Math.atan2(mouth[0] - at[0], mouth[2] - at[2]) * RAD);
  const brain = new BuiltinTeleOp(spec, table, loadLandCal());
  brain.state.firing = true;
  let loaded = 0;
  const step = () => {
    while (w.robot.heldBalls().length < 2 && loaded < pool.length) {
      if (!w.robot.preload(w.balls, w.balls.balls[loaded])) break;
      loaded++;
    }
    w.setGamepads(emptyGamepad() as GamepadState, emptyGamepad());
    w.step(brain.update(w.sensors(), emptyGamepad(), w.seq, 1 / 60));
  };
  let f = 0;
  // The hold as it was WHILE trying, not after: reading brain.state.hold at the end of the
  // settle loop reports an idle robot and prints "no reason given".
  const why: Record<string, number> = {};
  while (w.robot.shots < shots && f++ < 60 * 20) {
    step();
    const h = brain.state.hold;
    if (h) why[h.replace(/-?[\d.]+/g, 'N')] = (why[h.replace(/-?[\d.]+/g, 'N')] ?? 0) + 1;
  }
  const topHold = Object.entries(why).sort((a, b) => b[1] - a[1])[0]?.[0] ?? '';
  brain.state.firing = false;
  for (let k = 0; k < 60 * 4; k++) step();
  const dx = mouth[0] - w.robot.pos[0], dz = mouth[2] - w.robot.pos[2];
  const dd = Math.hypot(dx, dz) || 1;
  const trueOff = Math.acos(Math.max(-1, Math.min(1, -(dx * n[0] + dz * n[2]) / dd))) * RAD;
  return { shots: w.robot.shots, landed: w.landedInUpCell('red'), trueOff, hold: topHold };
}

export async function main(argv: string[] = []): Promise<void> {
  const i = argv.indexOf('--shots');
  const shots = i >= 0 ? Number(argv[i + 1]) : 10;
  const spec = robotSpec as unknown as RobotSpec;
  const cap = spec.turret.fireOpenCap_deg ?? 75;
  const range_in = 55;
  console.log(`\nTHE CELL SEEN FROM AN ANGLE, MEASURED. ${shots} balls a bearing at ${range_in} in, gate forced open.`);
  console.log(`The shipping gate refuses past turret.fireOpenCap_deg = ${cap}.\n`);
  console.log('  off-axis   cos(beta)   balls in   verdict');
  let lastGood = 0;
  for (let b = 0; b <= 80; b += 10) {
    const r = await atAngle(b, range_in, shots, 700 + b);
    if (!r) { console.log(`  ${String(b).padStart(6)}°    off the field`); continue; }
    const rate = r.shots ? r.landed / r.shots : 0;
    if (r.shots >= 3 && rate >= 0.7) lastGood = b;
    // NO SHOTS IS NOT A ZERO LANDING RATE. With the aperture gate forced open a bearing
    // that still fires nothing is being refused by something else, and printing a bare
    // 0/0 0% reads as "the mouth is shut" when the truth may be "it never took the shot".
    const why = r.shots === 0 ? `  never fired: ${r.hold || 'no reason given'}` : b > cap ? '  (the shipping gate refuses here)' : '';
    const got = r.shots === 0 ? '   -  ' : `${String(r.landed).padStart(4)}/${String(r.shots).padEnd(3)}  ${(rate * 100).toFixed(0).padStart(3)}%`;
    console.log(`  ${String(b).padStart(6)}°   ${Math.cos(b * DEG).toFixed(2).padStart(7)}   ${got}${why}`);
  }
  console.log(`\n  Balls still go in at ${lastGood} deg off the opening; the cap is ${cap} deg.`);
  console.log(`  Past ${spec.sensors.tag.maxIncidence_deg} deg the TAG stops decoding (sensors.tag.maxIncidence_deg), the fix goes`);
  console.log('  stale and the robot holds -- so the wall out there is the CAMERA, not the aperture.');
  console.log('  The aperture is halfLat*cos(beta), with NO depth term. See the header.\n');
}
