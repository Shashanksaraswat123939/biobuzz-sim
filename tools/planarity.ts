/**
 * IS A SHOT ACTUALLY FLAT? How far the flown ball leaves the plane it was predicted in.
 *
 *   npm run tool -- tools/planarity.ts
 *
 * simulateShot() is a 3-D integrator, but it is HANDED one direction (azimuth, elevation,
 * speed) and it orients the backspin axis square to that direction. Drag is anti-parallel to
 * v, gravity is vertical, and Magnus is w_hat x v with w_hat perpendicular to the plane -- so
 * every force lies in the plane and the predicted arc is exactly planar. A tilted 2-D curve
 * drawn in 3-D space.
 *
 * The REAL ball is not. Robot.launch() gives it dir*vExit + v_muzzle, and sets the spin axis
 * square to the LAUNCH direction rather than to that total. Once the chassis is moving the two
 * differ, w_hat is no longer perpendicular to the ball's ground velocity, and w_hat x v has a
 * component out of the plane: the real shot banks.
 *
 * This measures that gap -- the thing the yellow arc on the website cannot show -- by firing
 * while driving and reporting how far the flown path leaves the plane of its own launch.
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

async function fireWhileDriving(vx: number, vz: number, seed: number) {
  await initPhysics();
  const p = structuredClone(params) as unknown as Params;
  const spec = structuredClone(robotSpec) as unknown as RobotSpec;
  // EVERY GATE OUT OF THE WAY. This measures the FLIGHT, not the aim: whether the shot was
  // a good idea is irrelevant to how far the ball banks out of its own launch plane, and the
  // gates otherwise refuse to fire at all while the chassis is moving (tools/frontcheck.ts).
  spec.flywheel.minLandProb = 0;
  spec.flywheel.readySteps = 1;
  spec.flywheel.tolRpm = 5000;
  spec.turret.fireOpenCap_deg = 180;
  spec.turret.fireLeadCap_deg = 180;
  spec.turret.fireYawCap_dps = 9999;
  spec.sensors.tag.target.maxFireAgeS = 999;
  spec.sensors.tag.target.maxStateAgeS = 999;
  const pool = Array.from({ length: 6 }, () => ({ kind: 'pollen' as const, pos: [0, -5, 0] as Vec3 }));
  const w = new World({ params: p, robot: spec, staging: pool, alliance: 'red', seed });
  for (const b of w.balls.balls) w.balls.park(b);
  const hive = w.hives.red;
  const mouth = hive.upCellMouthWorld();
  const n = hive.upCellMouthNormalWorld();
  const d = inches(60);
  const at: Vec3 = [mouth[0] + n[0] * d, spec.chassis.height_m / 2 + spec.chassis.clearance_m, mouth[2] + n[2] * d];
  w.robot.place(at, Math.atan2(mouth[0] - at[0], mouth[2] - at[2]) * RAD);
  const brain = new BuiltinTeleOp(spec, table, loadLandCal());
  let loaded = 0;
  while (w.robot.heldBalls().length < 2 && loaded < pool.length) {
    if (!w.robot.preload(w.balls, w.balls.balls[loaded])) break;
    loaded++;
  }
  brain.state.firing = true;

  let shotId = -1;
  let launchV: Vec3 | null = null;
  let launchP: Vec3 | null = null;
  const path: Vec3[] = [];
  let speedAtShot = 0;
  let dbgPeak = 0;
  for (let i = 0; i < 60 * 10; i++) {
    const g: GamepadState = emptyGamepad();
    // THE CHASSIS VELOCITY IS IMPOSED, not driven. Driving it there needs a runway, the
    // gates then refuse the shot while it is moving, and neither has anything to do with
    // what this measures -- how a ball behaves once it has left. Holding the body at a known
    // velocity makes the muzzle term exact and the answer attributable.
    w.robot.body.setLinvel({ x: vx, y: w.robot.body.linvel().y, z: vz }, true);
    w.setGamepads(g, emptyGamepad());
    w.step(brain.update(w.sensors(), g, w.seq, 1 / 60));
    const rvNow = w.robot.body.linvel();
    dbgPeak = Math.max(dbgPeak, Math.hypot(rvNow.x, rvNow.z));
    if (shotId < 0 && w.robot.shots > 0 && w.robot.lastShotBallId >= 0) {
      shotId = w.robot.lastShotBallId;
      const b = w.balls.balls.find((x) => x.id === shotId);
      if (b) {
        const v = b.body.linvel();
        launchV = [v.x, v.y, v.z];
        launchP = w.balls.pos(b);
        // The IMPOSED velocity, not a reading taken after the step has damped it.
        speedAtShot = Math.hypot(vx, vz);
      }
    } else if (shotId >= 0) {
      const b = w.balls.balls.find((x) => x.id === shotId);
      if (b && b.state === 'flight') path.push(w.balls.pos(b));
      else if (path.length) break;
    }
  }
  if (!launchV || !launchP || path.length < 5) {
    console.log(`        [debug] armed=${brain.state.firing} shots=${w.robot.shots} peak=${dbgPeak.toFixed(2)} held=${w.robot.heldBalls().length} hold="${brain.state.hold}" pathLen=${path.length}`);
    return null;
  }
  // The plane the PREDICTOR would fly: vertical, containing the launch velocity. Its normal
  // is horizontal and square to the ground track.
  const h = Math.hypot(launchV[0], launchV[2]) || 1;
  const nx = -launchV[2] / h, nz = launchV[0] / h;
  let worst = 0;
  for (const q of path) {
    const off = (q[0] - launchP[0]) * nx + (q[2] - launchP[2]) * nz;
    if (Math.abs(off) > Math.abs(worst)) worst = off;
  }
  const flown = Math.hypot(path[path.length - 1][0] - launchP[0], path[path.length - 1][2] - launchP[2]);
  return { worst_in: worst * M_TO_IN, flown_in: flown * M_TO_IN, speed: speedAtShot };
}

export async function main(): Promise<void> {
  console.log('\nHOW FAR THE REAL BALL LEAVES ITS PREDICTED PLANE.');
  console.log('Fired from 60 in. The yellow arc on the site is planar by construction; this is the gap.\n');
  console.log('  chassis velocity       imposed     out of plane   over a ground run of');
  // Directions relative to the MOUTH's own normal, so "across" means across the opening.
  await initPhysics();
  const probe = new World({ params: params as unknown as Params, robot: robotSpec as unknown as RobotSpec, staging: [], alliance: 'red', seed: 1 });
  const nrm0 = probe.hives.red.upCellMouthNormalWorld();
  // HORIZONTAL UNIT VECTORS. The mouth normal tilts up, so its raw x,z components are shorter
  // than 1 and asking for 1.7 along them quietly imposed 1.47.
  const hn = Math.hypot(nrm0[0], nrm0[2]) || 1;
  const nrm: [number, number] = [nrm0[0] / hn, nrm0[2] / hn];
  const side: [number, number] = [-nrm[1], nrm[0]];
  const hv = 1.7;
  const cases: [string, number, number][] = [
    ['standing still       ', 0, 0],
    ['1.7 across the mouth ', side[0] * hv, side[1] * hv],
    ['1.7 the other way    ', -side[0] * hv, -side[1] * hv],
    ['1.7 backing away     ', nrm[0] * hv, nrm[1] * hv],
    ['1.7 diagonal         ', (nrm[0] + side[0]) * hv * 0.707, (nrm[1] + side[1]) * hv * 0.707],
  ];
  for (const [name, vx, vz] of cases) {
    const r = await fireWhileDriving(vx, vz, 900);
    if (!r) { console.log(`  ${name}    no shot`); continue; }
    console.log(`  ${name}   ${r.speed.toFixed(2)} m/s   ${r.worst_in >= 0 ? ' ' : ''}${r.worst_in.toFixed(2)} in      ${r.flown_in.toFixed(0)} in`);
  }
  console.log('\n  A POLLEN is 2.80 in across and the CELL mouth is 20 in wide.\n');
}
