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
import staging from '../assets/staging.json' with { type: 'json' };
import robotSpec from '../config/robot.json' with { type: 'json' };
import { readFileSync } from 'node:fs';
import { World, initPhysics, emptyGamepad } from '../packages/core/src/physics/world.js';
import { BuiltinTeleOp, ShotTable } from '../packages/core/src/robot/builtinTeleOp.js';
import { loadLandCal } from '../packages/core/src/robot/loadCal.js';
import { wrapPi, RAD, DEG } from '../packages/core/src/units.js';
import type { BallKind, GamepadState, Params, RobotSpec, Vec3 } from '../packages/core/src/types.js';

const balls = (staging.balls as { kind: string; pos: number[] }[]).map((b) => ({ kind: b.kind as BallKind, pos: b.pos as Vec3 }));
/** A pool under the floor for the shooting runs: the question is the shooter, not the fetch. */
const pool = Array.from({ length: 200 }, () => ({ kind: 'pollen' as BallKind, pos: [0, -5, 0] as Vec3 }));
const table = ShotTable.fromCsv(readFileSync(new URL('../java/teamcode/assets/shottable.csv', import.meta.url), 'utf8'));

/**
 * Two different drivers, because the two questions are different.
 *
 * `roam` is someone crossing the field: full stick, long turns, no regard for the goal. That
 * is the right input for "where does the muzzle end up", and the wrong one for "does it
 * shoot", because it spends most of its time behind the hive where no shot exists.
 *
 * `working` is someone parked in the shot zone taking shots: small corrections, short turn
 * taps to re-square, one longer turn every few seconds. That is the input the shooting
 * numbers have to come from, or they measure the test pattern instead of the robot.
 */
export function driverInput(t: number, working = false): GamepadState {
  const g = emptyGamepad();
  const a = working ? 0.16 : 0.8;
  g.left_stick_y = -a * Math.sin(t * 0.6);
  g.left_stick_x = (working ? 0.12 : 0.5) * Math.sin(t * 0.37 + 1);
  const phase = t % 11;
  const holdFor = working ? 0.6 : 2.5;
  g.right_stick_x = phase < holdFor ? 1 : (t % 1.7 < 0.18 ? -1 : 0);
  return g;
}

/** 50 in out on the side the mouth opens to, facing it: the spot the shot map is green at. */
function placeInTheZone(w: World, spec: RobotSpec): void {
  const mouth = w.hives.red.upCellMouthWorld();
  const lim = w.geom.halfWidth_m - 0.40;
  const R = 50 * 0.0254;
  const dz = Math.min(R, lim - mouth[2]);
  const dx = Math.sqrt(Math.max(0, R * R - dz * dz));
  const x = Math.min(mouth[0] + dx, lim);
  const z = mouth[2] + dz;
  w.robot.place([x, spec.chassis.height_m / 2 + spec.chassis.clearance_m, z],
    Math.atan2(mouth[0] - x, mouth[2] - z) * RAD);
}

function run(travelDeg: number, seed: number, secs = 60, turretDps?: number, yawCap?: number, inZone = false) {
  const p = structuredClone(params) as unknown as Params;
  const spec = structuredClone(robotSpec) as unknown as RobotSpec;
  spec.turret.range_deg = [-travelDeg, travelDeg];
  if (turretDps) spec.turret.speed_dps = turretDps;
  const w = new World({ params: p, robot: spec, staging: inZone ? pool : balls, alliance: 'red', seed, preload: spec.hopper.capacity });
  const brain = new BuiltinTeleOp(spec, table, loadLandCal());
  brain.state.firing = true;   // auto-fire latched, which is how the report was driven
  if (inZone) placeInTheZone(w, spec);
  w.clock.start();
  const dt = 1 / 60;
  let n = 0, nearOpp = 0, wild = 0, offSum = 0, atStop = 0, traversing = 0, tagSeen = 0;
  let prevTurret = w.robot.turretAngle;
  const why: Record<string, number> = {};
  const cell0 = { red: w.landedInUpCell('red'), blue: w.landedInUpCell('blue') };
  for (let i = 0; i < secs * 60; i++) {
    const g = driverInput(i * dt, inZone);
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
    // Keep the magazine fed, the way the app's auto-load does. Without it the run measures
    // how fast four balls run out, which is not a question about aim.
    if (inZone) for (const b of w.balls.balls) {
      if (w.robot.heldBalls().length >= spec.hopper.capacity) break;
      if (b.state === 'free' && w.balls.pos(b)[1] < -1) w.robot.preload(w.balls, b);
    }
    const h = brain.state.hold;
    const k = h ? h.replace(/-?[\d.]+/g, 'N') : brain.state.hold === '' && w.sensors().game.hopper === 0 ? 'HOPPER EMPTY' : 'clear to fire';
    why[k] = (why[k] ?? 0) + 1;
  }
  return {
    nearOpp: (nearOpp / n) * 100, wild: (wild / n) * 100, meanOff: offSum / n,
    atStop: (atStop / n) * 100, traversing: (traversing / n) * 100, tag: (tagSeen / n) * 100,
    // THE QUESTION STATED IN BALLS. Everything above is where the muzzle pointed; this is
    // where the balls ended up, and it is the only version of the complaint that scores.
    ours: w.landedInUpCell('red') - cell0.red, theirs: w.landedInUpCell('blue') - cell0.blue, shots: w.robot.shots, why, n,
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
  console.log('  AND WHERE THE BALLS WENT -- same driving, but STARTED IN THE SHOT ZONE,');
  console.log('  50 in out on the side the mouth opens to, with auto-fire latched for 30 s.');
  console.log('  The staging sits balls in BOTH up CELLs before a shot is taken, so only the');
  console.log('  CHANGE over the run is ours.');
  console.log('');
  console.log('  turret travel    shots   into OUR CELL   into THEIRS   landed   fed to the wrong CELL');
  for (const tr of [185, 270]) {
    const rs = [0, 1, 2].map((k) => run(tr, 3 + k * 13, 30, undefined, undefined, true));
    const sum = (f: (r: typeof rs[0]) => number) => rs.reduce((a, r) => a + f(r), 0);
    const ours = sum((r) => r.ours), theirs = sum((r) => r.theirs), sh = sum((r) => r.shots);
    console.log(`  +-${String(tr).padEnd(3)} deg    ${String(sh).padStart(7)}   ${String(ours).padStart(13)}   ${String(theirs).padStart(11)}   ${((ours + theirs) / Math.max(1, sh) * 100).toFixed(0).padStart(5)}%   ${(theirs / Math.max(1, ours + theirs) * 100).toFixed(0).padStart(21)}%`);
    const agg: Record<string, number> = {};
    let tot = 0;
    for (const r of rs) for (const [k, v] of Object.entries(r.why)) { agg[k] = (agg[k] ?? 0) + v; tot += v; }
    for (const [k, v] of Object.entries(agg).sort((a, b) => b[1] - a[1]).slice(0, 4)) {
      console.log(`                 ${((v / tot) * 100).toFixed(0).padStart(3)}%  ${k}`);
    }
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
