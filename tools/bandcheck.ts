/**
 * HOW WIDE IS THE MOUTH REALLY, in exit speed? The band, measured instead of solved.
 *
 *   npm run tool -- tools/bandcheck.ts [--shots 4]
 *
 * The table's speedLo..speedHi is geometric: the exit speeds whose nominal trajectory clears
 * the near lip by a ball radius and passes under the far lip by one. tools/landcal.ts says
 * the threading factor built on that band reads 59% at 80 in while 88% of shots land, so
 * either the band is narrower than the pocket or the wheel is not where the brain thinks.
 *
 * This fires a stationary robot with scatter off while the WORLD's exit-speed constant is
 * scaled and the brain's is not, so every shot leaves at a known fraction of what the brain
 * aimed for. The fraction of those that land, against that error, is the band as the pocket
 * actually enforces it.
 */
import params from '../config/params.json' with { type: 'json' };
import robotSpec from '../config/robot.json' with { type: 'json' };
import { readFileSync } from 'node:fs';
import { World, initPhysics, emptyGamepad } from '../packages/core/src/physics/world.js';
import { BuiltinTeleOp, ShotTable } from '../packages/core/src/robot/builtinTeleOp.js';
import { RAD, inches, rpmToRadS } from '../packages/core/src/units.js';
import type { GamepadState, Params, RobotSpec, Vec3 } from '../packages/core/src/types.js';

const table = ShotTable.fromCsv(readFileSync(new URL('../java/teamcode/assets/shottable.csv', import.meta.url), 'utf8'));

async function landed(range_in: number, kScale: number, shots: number): Promise<number> {
  await initPhysics();
  const p = structuredClone(params) as unknown as Params;
  const spec = structuredClone(robotSpec) as unknown as RobotSpec;
  spec.flywheel.scatter = { angle_deg: 0, yaw_deg: 0, speedFrac: 0 };
  spec.flywheel.minLandProb = 0;
  p.ball.pollen.dVar = 0;
  p.ball.pollen.mVar = 0;
  // The world's wheel throws harder or softer than the brain's model of it.
  const specWorld = structuredClone(spec);
  specWorld.flywheel.k = spec.flywheel.k * kScale;
  const staging = Array.from({ length: 40 }, () => ({ kind: 'pollen' as const, pos: [0, -5, 0] as Vec3 }));
  const w = new World({ params: p, robot: specWorld, staging, alliance: 'red', seed: 3 });
  for (const b of w.balls.balls) w.balls.park(b);
  const mouth = w.hives.red.upCellMouthWorld();
  const lim = w.geom.halfWidth_m - 0.40;
  const R = inches(range_in);
  const dz = Math.min(R, lim - mouth[2]);
  const dx = Math.sqrt(Math.max(0, R * R - dz * dz));
  const x = mouth[0] + dx, z = mouth[2] + dz;
  w.robot.place([x, spec.chassis.height_m / 2 + spec.chassis.clearance_m, z], Math.atan2(mouth[0] - x, mouth[2] - z) * RAD);
  const brain = new BuiltinTeleOp(spec, table);
  let loaded = 0;
  const step = (g: GamepadState) => {
    while (w.robot.heldBalls().length < 2 && loaded < staging.length) {
      if (!w.robot.preload(w.balls, w.balls.balls[loaded])) break;
      loaded++;
    }
    w.setGamepads(g, emptyGamepad());
    w.step(brain.update(w.sensors(), g, w.seq));
  };
  const arm = emptyGamepad(); step(arm); arm.dpad_up = true; step(arm);
  for (let f = 0; f < 150; f++) step(emptyGamepad());
  let f = 0;
  // The latch is a toggle: hold R1 until the count is reached, then switch it OFF, or the
  // settling tail keeps firing into a pocket that is filling -- eight balls in it read 50%
  // at the nominal speed and 88% either side, which is the pile, not the band.
  while (w.robot.shots < shots && f++ < 60 * 20) step({ ...emptyGamepad(), right_bumper: true });
  brain.state.firing = false;
  for (let k = 0; k < 60 * 4; k++) step(emptyGamepad());
  return w.landedInUpCell('red') / Math.max(1, w.robot.shots);
}

export async function main(argv: string[] = []): Promise<void> {
  const i = argv.indexOf('--shots');
  const shots = i >= 0 ? Number(argv[i + 1]) : 4;
  const f = robotSpec.flywheel;
  console.log('\nTHE BAND AS THE POCKET ENFORCES IT. Stationary, scatter off, the world\'s wheel scaled against the brain\'s.\n');
  const scales = [0.90, 0.92, 0.94, 0.95, 0.96, 0.97, 0.98, 0.99, 1.00, 1.01, 1.02, 1.03, 1.04, 1.05, 1.06, 1.08, 1.10];
  for (const range of [40, 55, 70]) {
    const row = table.lookup(range - (robotSpec.calibration?.rangeTrim_in ?? 0));
    const nominal = f.k * f.r_fly_m * rpmToRadS(row.rpm);
    const lo = (row.speedLo as number) / nominal, hi = (row.speedHi as number) / nominal;
    console.log(`  ${range} in: the table's band is ${((lo - 1) * 100).toFixed(1)}% .. +${((hi - 1) * 100).toFixed(1)}% of ${nominal.toFixed(2)} m/s`);
    const line: string[] = [];
    let first = NaN, last = NaN;
    for (const sc of scales) {
      const r = await landed(range, sc, shots);
      line.push(`${((sc - 1) * 100).toFixed(0).padStart(3)}%:${(r * 100).toFixed(0).padStart(3)}`);
      if (r >= 0.75) { if (Number.isNaN(first)) first = sc; last = sc; }
    }
    console.log(`    ${line.join('  ')}`);
    console.log(`    lands 3 of 4 or better from ${((first - 1) * 100).toFixed(0)}% to +${((last - 1) * 100).toFixed(0)}%   (table: ${((lo - 1) * 100).toFixed(1)} .. +${((hi - 1) * 100).toFixed(1)})\n`);
  }
}
