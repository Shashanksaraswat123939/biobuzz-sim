/**
 * Where is the muzzle actually POINTING while somebody drives?
 *
 *   npm run tool -- tools/aimwatch.ts [--travel 185]
 *
 * tools/turretcheck.ts answers this for a chassis spinning flat out for 12 s, which is not
 * what a driver does and not what the report is about. The report is "it aims at the other
 * hive while I am just driving around" -- so this drives the way a driver drives: translate
 * across the shot, tap the turn buttons, occasionally hold one, all with auto-aim on.
 *
 * It reports the share of loops the muzzle sits nearer the OPPONENT's hive than ours, which
 * is the complaint stated as a number, and splits that share by cause -- there is no point
 * fixing the unwind if the answer is the camera.
 */
import params from '../config/params.json' with { type: 'json' };
import robotSpec from '../config/robot.json' with { type: 'json' };
import { readFileSync } from 'node:fs';
import { World, initPhysics, emptyGamepad } from '../packages/core/src/physics/world.js';
import { BuiltinTeleOp, ShotTable } from '../packages/core/src/robot/builtinTeleOp.js';
import { loadLandCal } from '../packages/core/src/robot/loadCal.js';
import { wrapPi, RAD, DEG } from '../packages/core/src/units.js';
import type { Params, RobotSpec } from '../packages/core/src/types.js';

const table = ShotTable.fromCsv(readFileSync(new URL('../java/teamcode/assets/shottable.csv', import.meta.url), 'utf8'));

/** A minute of plausible driving: cross the shot, tap to correct, sometimes hold a turn. */
function driverInput(t: number) {
  const g = emptyGamepad();
  g.left_stick_y = -0.8 * Math.sin(t * 0.6);      // shuttle up and down the shot
  g.left_stick_x = 0.5 * Math.sin(t * 0.37 + 1);  // and drift across it
  // Turn: mostly short taps, with a long hold every 11 s -- the hold is what forces an unwind.
  const phase = t % 11;
  g.right_stick_x = phase < 2.5 ? 1 : (t % 1.7 < 0.18 ? -1 : 0);
  return g;
}

function run(travelDeg: number, seed: number, secs = 60, turretDps?: number, yawCap?: number) {
  const p = structuredClone(params) as unknown as Params;
  const spec = structuredClone(robotSpec) as unknown as RobotSpec;
  spec.turret.range_deg = [-travelDeg, travelDeg];
  if (turretDps) spec.turret.speed_dps = turretDps;
  const w = new World({ params: p, robot: spec, staging: [], alliance: 'red', seed, preload: spec.hopper.capacity });
  const brain = new BuiltinTeleOp(spec, table, loadLandCal());
  w.clock.start();
  const dt = 1 / 60;
  let n = 0, nearOpp = 0, wild = 0, offSum = 0, atStop = 0, traversing = 0, tagSeen = 0;
  let prevTurret = w.robot.turretAngle;
  for (let i = 0; i < secs * 60; i++) {
    const g = driverInput(i * dt);
    if (yawCap) g.right_stick_x *= yawCap;
    const s = w.sensors();
    if (s.game.tag) tagSeen++;
    w.step(brain.update(s, g, i, dt));
    const pos = w.robot.pos;
    const muzzle = w.robot.yaw * RAD + w.robot.turretAngle;
    const to = (a: 'red' | 'blue') => {
      const m = w.hives[a].upCellMouthWorld();
      return Math.atan2(m[0] - pos[0], m[2] - pos[2]) * RAD;
    };
    const offRed = Math.abs(wrapPi((muzzle - to('red')) * DEG) * RAD);
    const offBlue = Math.abs(wrapPi((muzzle - to('blue')) * DEG) * RAD);
    n++;
    offSum += offRed;
    if (offBlue < offRed) nearOpp++;
    if (offRed > 45) wild++;
    if (Math.abs(Math.abs(w.robot.turretAngle) - travelDeg) < 1) atStop++;
    // A traverse is the axis moving fast in one direction while the CHASSIS is not.
    if (Math.abs(w.robot.turretAngle - prevTurret) / dt > 150) traversing++;
    prevTurret = w.robot.turretAngle;
  }
  return {
    nearOpp: (nearOpp / n) * 100, wild: (wild / n) * 100, meanOff: offSum / n,
    atStop: (atStop / n) * 100, traversing: (traversing / n) * 100, tag: (tagSeen / n) * 100,
  };
}

export async function main(argv: string[] = []): Promise<void> {
  await initPhysics();
  const i = argv.indexOf('--travel');
  const travels = i >= 0 ? [Number(argv[i + 1])] : [185, 270, 370, 540];
  console.log('\nWHERE THE MUZZLE POINTS WHILE SOMEBODY DRIVES. 60 s per travel, 3 seeds, auto-aim on.\n');
  console.log('  turret travel   nearer the WRONG hive   >45 deg off ours   mean off   on its stop   traversing   tag in view');
  for (const tr of travels) {
    const rs = [0, 1, 2].map((k) => run(tr, 3 + k * 13));
    const m = (f: (r: typeof rs[0]) => number) => rs.reduce((a, r) => a + f(r), 0) / rs.length;
    console.log(`  +-${String(tr).padEnd(3)} deg    ${m((r) => r.nearOpp).toFixed(1).padStart(18)}%   ${m((r) => r.wild).toFixed(1).padStart(15)}%   ${m((r) => r.meanOff).toFixed(0).padStart(6)} deg   ${m((r) => r.atStop).toFixed(1).padStart(10)}%   ${m((r) => r.traversing).toFixed(1).padStart(9)}%   ${m((r) => r.tag).toFixed(0).padStart(10)}%`);
  }
  console.log('');
  console.log('  Same drive, travel held at the value above, but changing what the TURRET and the CHASSIS can do:');
  console.log('  turret slew   chassis turn   nearer the WRONG hive   >45 deg off ours   mean off');
  for (const [dps, cap] of [[261, 1], [500, 1], [900, 1], [261, 0.5], [261, 0.25], [500, 0.5]] as [number, number][]) {
    const rs = [0, 1, 2].map((k) => run(travels[0], 3 + k * 13, 60, dps, cap));
    const m = (f: (r: typeof rs[0]) => number) => rs.reduce((a, r) => a + f(r), 0) / rs.length;
    console.log(`  ${String(dps).padStart(8)} dps   ${(cap * 100).toFixed(0).padStart(9)}%   ${m((r) => r.nearOpp).toFixed(1).padStart(18)}%   ${m((r) => r.wild).toFixed(1).padStart(15)}%   ${m((r) => r.meanOff).toFixed(0).padStart(6)} deg`);
  }
  console.log('');
}
