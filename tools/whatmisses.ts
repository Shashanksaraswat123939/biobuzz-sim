/**
 * WHAT ACTUALLY PREDICTS A MISS? Every shot, split by each feature.
 *
 *   npm run tool -- tools/whatmisses.ts [--seeds 14]
 *
 * WHY THIS EXISTS. flywheel.minLandProb is supposed to be the robot deciding whether a shot
 * is worth taking, and it is inert: anything from 0.5 to 0.90 fires exactly the same shots,
 * and 0.92 fires none at all. The cause is that the score is pushed through
 * config/landcal.json, whose observed rates are 0.911 for five of six bins and 0.937 for the
 * last -- so the calibrated P(land) only ever takes two values and a threshold cannot
 * discriminate between two shots. The model computes a probability that does not vary.
 *
 * That means the robot cannot currently tell a good shot from a bad one, which is the real
 * reason the land rate sits at 76-80% and refusing more does not raise it. Before any gate
 * can help, something has to actually predict. This takes a large sample of real shots and,
 * for each recorded feature, splits them into quartiles and prints the land rate of each. A
 * feature whose quartiles all land the same is useless; one that spreads is a gate worth
 * having.
 */
import { readFileSync } from 'node:fs';
import params from '../config/params.json' with { type: 'json' };
import robotSpec from '../config/robot.json' with { type: 'json' };
import { World, initPhysics, emptyGamepad } from '../packages/core/src/physics/world.js';
import { BuiltinTeleOp, ShotTable } from '../packages/core/src/robot/builtinTeleOp.js';
import { loadLandCal } from '../packages/core/src/robot/loadCal.js';
import { RAD, inches } from '../packages/core/src/units.js';
import type { GamepadState, Params, RobotSpec, ShotRecord, Vec3 } from '../packages/core/src/types.js';

const table = ShotTable.fromCsv(readFileSync(new URL('../java/teamcode/assets/shottable.csv', import.meta.url), 'utf8'));

async function gather(speed: number, standoff_in: number, seed: number): Promise<ShotRecord[]> {
  await initPhysics();
  const p = structuredClone(params) as unknown as Params;
  const spec = structuredClone(robotSpec) as unknown as RobotSpec;
  spec.drivetrain.maxSpeed_mps = speed;
  const pool = Array.from({ length: 400 }, () => ({ kind: 'pollen' as const, pos: [0, -5, 0] as Vec3 }));
  const w = new World({ params: p, robot: spec, staging: pool, alliance: 'red', seed });
  for (const b of w.balls.balls) w.balls.park(b);
  const hive = w.hives.red;
  const mouth = hive.upCellMouthWorld();
  const n0 = hive.upCellMouthNormalWorld();
  const hn = Math.hypot(n0[0], n0[2]) || 1;
  const nrm: [number, number] = [n0[0] / hn, n0[2] / hn];
  const side: [number, number] = [-nrm[1], nrm[0]];
  const y = spec.chassis.height_m / 2 + spec.chassis.clearance_m;
  const at = (lat: number): Vec3 => [mouth[0] + nrm[0] * inches(standoff_in) + side[0] * inches(lat), y,
    mouth[2] + nrm[1] * inches(standoff_in) + side[1] * inches(lat)];
  const reach = standoff_in * 0.8;
  const start = at(-reach);
  w.robot.place(start, Math.atan2(mouth[0] - start[0], mouth[2] - start[2]) * RAD);
  const brain = new BuiltinTeleOp(spec, table, loadLandCal());
  brain.state.firing = true;
  let loaded = 0, dir = 1;
  const tips0 = hive.tips;
  for (let i = 0; i < 60 * 60; i++) {
    while (w.robot.heldBalls().length < spec.hopper.capacity && loaded < pool.length) {
      if (!w.robot.preload(w.balls, w.balls.balls[loaded])) break;
      loaded++;
    }
    const c = w.robot.pos;
    const lat = ((c[0] - mouth[0]) * side[0] + (c[2] - mouth[2]) * side[1]) / 0.0254;
    if (lat > reach) dir = -1; else if (lat < -reach) dir = 1;
    const g: GamepadState = emptyGamepad();
    const yawErr = ((Math.atan2(mouth[0] - c[0], mouth[2] - c[2]) * RAD - w.robot.yaw * RAD) + 540) % 360 - 180;
    g.right_stick_x = -Math.max(-0.2, Math.min(0.2, yawErr / 45));
    g.left_stick_x = -dir;
    const rangeNow = Math.hypot(mouth[0] - c[0], mouth[2] - c[2]) / 0.0254;
    g.left_stick_y = -Math.max(-0.4, Math.min(0.4, (rangeNow - standoff_in) / 20));
    w.setGamepads(g, emptyGamepad());
    w.step(brain.update(w.sensors(), g, w.seq, 1 / 60));
    if (hive.tips > tips0) break;   // after a tip the goal faces away; different question
  }
  for (let k = 0; k < 60 * 3; k++) w.step(brain.update(w.sensors(), emptyGamepad(), w.seq, 1 / 60));
  return w.snapshot().shots.filter((x) => x.result !== 'flight');
}

/** Land rate per quartile of `f`. A flat row means the feature predicts nothing. */
function split(name: string, rows: ShotRecord[], f: (r: ShotRecord) => number, unit = ''): void {
  const ok = rows.filter((r) => Number.isFinite(f(r)));
  if (ok.length < 20) { console.log(`  ${name.padEnd(22)} too few samples`); return; }
  const sorted = [...ok].sort((a, b) => f(a) - f(b));
  const q = Math.floor(sorted.length / 4);
  const parts = [sorted.slice(0, q), sorted.slice(q, 2 * q), sorted.slice(2 * q, 3 * q), sorted.slice(3 * q)];
  const rates = parts.map((pt) => (pt.filter((r) => r.result === 'cell').length / Math.max(1, pt.length)) * 100);
  const edges = parts.map((pt) => f(pt[0]));
  const spread = Math.max(...rates) - Math.min(...rates);
  console.log(`  ${name.padEnd(22)} ${rates.map((r, i) => `${edges[i].toFixed(1)}${unit}: ${r.toFixed(0)}%`).join('   ')}    spread ${spread.toFixed(0)} pts${spread > 12 ? '  <- PREDICTS' : ''}`);
}

export async function main(argv: string[] = []): Promise<void> {
  const i = argv.indexOf('--seeds');
  const seeds = i >= 0 ? Number(argv[i + 1]) : 14;
  const all: ShotRecord[] = [];
  for (const speed of [0.6, 0.93, 1.25]) {
    for (let s = 0; s < seeds; s++) all.push(...await gather(speed, 40, 700 + s * 13));
  }
  const landed = all.filter((r) => r.result === 'cell').length;
  console.log(`\n${all.length} settled shots, ${landed} in (${((landed / all.length) * 100).toFixed(0)}%), 40 in stand-off, three speeds.\n`);
  console.log('Land rate by quartile of each feature. Flat = the feature tells you nothing.\n');
  split('range', all, (r) => r.rangeIn, ' in');
  split('|aim error| at launch', all, (r) => Math.abs(r.aimErrDeg), ' deg');
  split('|rpm - target|', all, (r) => Math.abs(r.rpm - r.targetRpm), '');
  split('rpm - target (signed)', all, (r) => r.rpm - r.targetRpm, '');
  split('hood angle', all, (r) => r.hoodDeg, ' deg');
  split('exit speed', all, (r) => r.exitSpeed, ' m/s');
  split('|bearing|', all, (r) => Math.abs(r.bearingDeg), ' deg');
  console.log('');
}
