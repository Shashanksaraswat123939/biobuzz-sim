/** One 20 s run from the shot zone, one line a second: what the shooter is actually doing. */
import params from '../config/params.json' with { type: 'json' };
import robotSpec from '../config/robot.json' with { type: 'json' };
import staging from '../assets/staging.json' with { type: 'json' };
import { readFileSync } from 'node:fs';
import { World, initPhysics, emptyGamepad } from '../packages/core/src/physics/world.js';
import { BuiltinTeleOp, ShotTable } from '../packages/core/src/robot/builtinTeleOp.js';
import { loadLandCal } from '../packages/core/src/robot/loadCal.js';
import { RAD, M_TO_IN } from '../packages/core/src/units.js';
import { driverInput } from './aimwatch.js';
import type { BallKind, Params, RobotSpec, Vec3 } from '../packages/core/src/types.js';

const balls = (staging.balls as { kind: string; pos: number[] }[]).map((b) => ({ kind: b.kind as BallKind, pos: b.pos as Vec3 }));
const table = ShotTable.fromCsv(readFileSync(new URL('../java/teamcode/assets/shottable.csv', import.meta.url), 'utf8'));

export async function main(argv: string[] = []): Promise<void> {
  await initPhysics();
  const still = argv.includes('--still');
  const p = structuredClone(params) as unknown as Params;
  const spec = structuredClone(robotSpec) as unknown as RobotSpec;
  // A POOL UNDER THE FLOOR, which is what movingfire.ts uses. `preload` draws its balls from
  // the staging list, so a bare list gives a robot with an empty hopper; the real field
  // staging gives it four and then Robot.place teleports the chassis out from under them,
  // because place moves one rigid body and the held balls are their own. Neither answers
  // "does a stationary robot with balls in it shoot".
  const pool = Array.from({ length: 80 }, () => ({ kind: 'pollen' as BallKind, pos: [0, -5, 0] as Vec3 }));
  const w = new World({ params: p, robot: spec, staging: argv.includes('--field') ? balls : pool, alliance: 'red', seed: 3, preload: spec.hopper.capacity });
  const brain = new BuiltinTeleOp(spec, table, loadLandCal());
  brain.state.firing = !argv.includes('--nofire');
  const mouth = w.hives.red.upCellMouthWorld();
  const lim = w.geom.halfWidth_m - 0.40;
  const ri = argv.indexOf('--range');
  const R = (ri >= 0 ? Number(argv[ri + 1]) : 50) * 0.0254;
  const dz = Math.min(R, lim - mouth[2]);
  const dx = Math.sqrt(Math.max(0, R * R - dz * dz));
  const x = Math.min(mouth[0] + dx, lim);
  const z = mouth[2] + dz;
  w.robot.place([x, spec.chassis.height_m / 2 + spec.chassis.clearance_m, z], Math.atan2(mouth[0] - x, mouth[2] - z) * RAD);
  w.clock.start();
  const dt = 1 / 60;
  console.log(`\n  placed at (${(x * M_TO_IN).toFixed(0)}, ${(z * M_TO_IN).toFixed(0)}) in, mouth at (${(mouth[0] * M_TO_IN).toFixed(0)}, ${(mouth[2] * M_TO_IN).toFixed(0)})\n`);
  console.log('     t   hop  shots   rng   open   rpm   aimErr   pLand   pSpeed  exit m/s  band            hold');
  for (let i = 0; i < 20 * 60; i++) {
    const s = w.sensors();
    w.step(brain.update(s, still ? emptyGamepad() : driverInput(i * dt, true), i, dt));
    if (i % 60 === 0) {
      const st = brain.state;
      console.log(`  ${(i * dt).toFixed(0).padStart(4)}   ${String(s.game.hopper).padStart(3)}  ${String(w.robot.shots).padStart(5)}  ${s.game.truth.upCellRangeIn.toFixed(0).padStart(4)}  ${s.game.truth.upCellOpenDeg.toFixed(0).padStart(5)}  ${s.game.flywheelRpm.toFixed(0).padStart(4)}  ${st.turretAimErrDeg.toFixed(1).padStart(6)}  ${st.pLand.toFixed(2).padStart(6)}  ${st.pSpeed.toFixed(2).padStart(6)}  ${(spec.flywheel.k * spec.flywheel.r_fly_m * s.game.flywheelRpm * Math.PI / 30).toFixed(3).padStart(7)}   ${(() => { const r = table.lookup(s.game.truth.upCellRangeIn - (spec.calibration?.rangeTrim_in ?? 0)); return `${(r.speedLo ?? 0).toFixed(3)}-${(r.speedHi ?? 0).toFixed(3)}`; })()}   ${st.hold || '-'}`);
    }
  }
  console.log(`\n  ended: ${w.robot.shots} shots, our CELL ${w.landedInUpCell('red')}, theirs ${w.landedInUpCell('blue')}\n`);
}
