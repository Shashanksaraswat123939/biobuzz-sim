/**
 * CAN A ROBOT PUT A BALL IN A FLOWER AT ALL? Before anything is scripted to try.
 *
 *   npm run tool -- tools/flowerprobe.ts [--dist 14]
 *
 * The opponent scores 0.0 from FLOWERs and so does the autonomous routine, and the obvious
 * reading is that nobody has written the behaviour. That is only worth writing if the
 * MECHANISM works: the retrieval opening is 3.55 in tall with a 2.80 in POLLEN, the intake
 * reversed is the only way out of the robot, and nothing has ever checked that those two
 * facts meet. This stands a loaded robot in front of each flower's opening at a few
 * stand-offs, reverses the intake, and counts what ends up in the tube.
 */
import params from '../config/params.json' with { type: 'json' };
import robotSpec from '../config/robot.json' with { type: 'json' };
import { World, initPhysics, emptyGamepad } from '../packages/core/src/physics/world.js';
import { RAD, inches } from '../packages/core/src/units.js';
import { inFlowerScoringVolume } from '../packages/core/src/rules/scoring.js';
import type { GamepadState, Params, RobotSpec, Vec3 } from '../packages/core/src/types.js';

async function one(flower: number, dist_in: number, seed: number) {
  await initPhysics();
  const p = structuredClone(params) as unknown as Params;
  const spec = structuredClone(robotSpec) as unknown as RobotSpec;
  const pool = Array.from({ length: 12 }, () => ({ kind: 'pollen' as const, pos: [0, -5, 0] as Vec3 }));
  const w = new World({ params: p, robot: spec, staging: pool, alliance: 'red', seed });
  for (const b of w.balls.balls) w.balls.park(b);
  const f = w.geom.flowers[flower];
  // Stand off along the opening's own direction, facing the tube.
  // openingDir is [x, z] -- a 2-tuple, not a Vec3. Indexing [2] put the robot at NaN.
  const dir = f.openingDir as unknown as [number, number];
  const x = f.x_m + dir[0] * inches(dist_in);
  const z = f.z_m + dir[1] * inches(dist_in);
  w.robot.place([x, spec.chassis.height_m / 2 + spec.chassis.clearance_m, z], Math.atan2(f.x_m - x, f.z_m - z) * RAD);
  let loaded = 0;
  while (w.robot.heldBalls().length < 4 && loaded < pool.length) {
    if (!w.robot.preload(w.balls, w.balls.balls[loaded])) break;
    loaded++;
  }
  const held0 = w.robot.heldBalls().length;
  const step = (g: GamepadState) => { w.setGamepads(g, emptyGamepad()); w.step({ seq: w.seq, motors: { intake: { mode: 'RUN_WITHOUT_ENCODER', power: g.left_trigger > 0.1 ? -1 : 1 } }, servos: {} }); };
  for (let i = 0; i < 30; i++) step(emptyGamepad());
  const spit = emptyGamepad(); spit.left_trigger = 1;
  for (let i = 0; i < 60 * 6; i++) step(spit);
  for (let i = 0; i < 60 * 3; i++) step(emptyGamepad());
  const inTube = w.balls.balls.filter((b) => b.body.isEnabled()
    && Math.hypot(w.balls.pos(b)[0] - f.x_m, w.balls.pos(b)[2] - f.z_m) < f.openingR_m + b.radius
    && w.balls.pos(b)[1] > inches(0.2)).length;
  const scoring = w.balls.balls.filter((b) => b.body.isEnabled() && inFlowerScoringVolume(w.balls.pos(b), f, b.radius)).length;
  return { held0, left: w.robot.heldBalls().length, inTube, scoring };
}

export async function main(argv: string[] = []): Promise<void> {
  if (argv.includes('--lob')) { await lob(); return; }
  const i = argv.indexOf('--dist');
  const dists = i >= 0 ? [Number(argv[i + 1])] : [10, 13, 16, 20];
  console.log('\nSPITTING INTO A FLOWER. 4 POLLEN aboard, intake reversed for 6 s.\n');
  console.log('  flower  stand-off   held before/after   in the tube   in the scoring volume');
  for (const fl of [0, 1, 2, 3]) {
    for (const d of dists) {
      const r = await one(fl, d, 5 + fl);
      console.log(`  ${String(fl).padStart(6)}  ${String(d).padStart(6)} in   ${String(r.held0).padStart(8)} / ${String(r.left).padEnd(6)}   ${String(r.inTube).padStart(11)}   ${String(r.scoring).padStart(20)}`);
    }
  }
  console.log('');
  console.log('  The retrieval opening is 3.55 in tall and a POLLEN is 2.80. If nothing goes in at');
  console.log('  any stand-off, the behaviour is not what is missing -- the mechanism is.');
  console.log('');
}

/**
 * THE TROUGH LOB (PHYSICS_AND_SIMULATION.md 7.2): a soft, steep shot into the tube's top
 * rather than through the retrieval opening. The doc derives 2.4-2.6 m/s at 55-62 deg from
 * about 9 in away; this checks that against the flower geometry this simulator actually has,
 * whose tube top is a 4.0 in hole at 22.6 in with no funnel around it.
 *
 *   npm run tool -- tools/flowerprobe.ts --lob
 */
export async function lob(): Promise<void> {
  await initPhysics();
  const p = params as unknown as Params;
  const spec = robotSpec as unknown as RobotSpec;
  const { buildFieldGeometry } = await import('../packages/core/src/field/geometry.js');
  const { simulateShot } = await import('../packages/core/src/physics/ballistics.js');
  const { DEG } = await import('../packages/core/src/units.js');
  const g = buildFieldGeometry(p);
  const f = g.flowers[0];
  const r = p.ball.pollen.d_m / 2;
  console.log('\nTHE TROUGH LOB into the tube top: a %s in hole at %s in.\n',
    (f.openingR_m * 2 * 39.37).toFixed(1), (f.topY_m * 39.37).toFixed(1));
  console.log('  stand-off  hood   m/s    rpm   apex in   height at the tube   in the hole?');
  const k = spec.flywheel.k * spec.flywheel.r_fly_m;
  for (const d_in of [9, 12, 16]) {
    for (const hood of [55, 62, 70, 78]) {
      let best = '';
      for (const v of [2.0, 2.2, 2.4, 2.6, 2.8, 3.0, 3.4]) {
        const t = simulateShot(p, {
          from: [0, spec.turret.muzzleHeight_m, 0], azimuth: 0, elevation: hood * DEG, speed: v,
          radius: r, mass: p.ball.pollen.m_kg, spin: v / r,
        }, d_in * 0.0254, 1 / 480, [d_in * 0.0254]);
        const h = t.gateHeights[0];
        if (!Number.isFinite(h)) continue;
        // In the hole: above the rim, and still descending when it gets there.
        const ok = h > f.topY_m && h < f.topY_m + 0.15 && t.descentAngle > 20 * DEG;
        const rpm = (v / k) * 60 / (2 * Math.PI);
        const line = `  ${String(d_in).padStart(8)} in  ${String(hood).padStart(3)}  ${v.toFixed(1)}  ${rpm.toFixed(0).padStart(5)}  ${(t.apex * 39.37).toFixed(1).padStart(7)}  ${(h * 39.37).toFixed(1).padStart(17)} in   ${ok ? 'YES' : ''}`;
        if (ok && !best) best = line;
        if (ok) console.log(line);
      }
      void best;
    }
  }
  console.log('');
  console.log('  A ball has to arrive ABOVE 22.6 in and still be coming down. If no row says YES,');
  console.log('  this shooter cannot reach the tube top and the FLOWER needs its own mechanism.');
  console.log('');
}
