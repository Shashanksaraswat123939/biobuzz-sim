/**
 * DOES THE FLOWER LOB ACTUALLY SCORE? The table, flown by the real robot.
 *
 *   npm run tool -- tools/flowercheck.ts [--shots 6]
 *
 * tools/flowertable.ts solves the lob on paper; this stands the robot at each solved
 * stand-off in FLOWER mode, lets its own gate decide, and counts what ends up in the tube's
 * scoring volume. The paper answer has 0.6 in of room in a 4.0 in hole, so it is exactly the
 * kind of shot that looks fine in the solver and misses in the world.
 */
import { readFileSync } from 'node:fs';
import params from '../config/params.json' with { type: 'json' };
import robotSpec from '../config/robot.json' with { type: 'json' };
import { World, initPhysics, emptyGamepad } from '../packages/core/src/physics/world.js';
import { BuiltinTeleOp, ShotTable } from '../packages/core/src/robot/builtinTeleOp.js';
import { FlowerTable } from '../packages/core/src/robot/flowerTable.js';
import { loadLandCal } from '../packages/core/src/robot/loadCal.js';
import { inFlowerScoringVolume } from '../packages/core/src/rules/scoring.js';
import { RAD, inches } from '../packages/core/src/units.js';
import type { GamepadState, Params, RobotSpec, Vec3 } from '../packages/core/src/types.js';

const table = ShotTable.fromCsv(readFileSync(new URL('../java/teamcode/assets/shottable.csv', import.meta.url), 'utf8'));
const flowers = FlowerTable.fromCsv(readFileSync(new URL('../java/teamcode/assets/flowertable.csv', import.meta.url), 'utf8'));

async function one(flowerIx: number, dist_in: number, shots: number, seed: number) {
  await initPhysics();
  const p = structuredClone(params) as unknown as Params;
  const spec = structuredClone(robotSpec) as unknown as RobotSpec;
  const pool = Array.from({ length: 40 }, () => ({ kind: 'pollen' as const, pos: [0, -5, 0] as Vec3 }));
  const w = new World({ params: p, robot: spec, staging: pool, alliance: 'red', seed });
  for (const b of w.balls.balls) w.balls.park(b);
  const f = w.geom.flowers[flowerIx];
  const dir = f.openingDir as unknown as [number, number];
  const x = f.x_m + dir[0] * inches(dist_in);
  const z = f.z_m + dir[1] * inches(dist_in);
  w.robot.place([x, spec.chassis.height_m / 2 + spec.chassis.clearance_m, z], Math.atan2(f.x_m - x, f.z_m - z) * RAD);
  const brain = new BuiltinTeleOp(spec, table, loadLandCal(), null, 'red', flowers);
  brain.state.flowerMode = true;
  brain.state.firing = true;
  let loaded = 0;
  const why: Record<string, number> = {};
  const step = (g: GamepadState) => {
    while (w.robot.heldBalls().length < spec.hopper.capacity && loaded < pool.length) {
      if (!w.robot.preload(w.balls, w.balls.balls[loaded])) break;
      loaded++;
    }
    w.setGamepads(g, emptyGamepad());
    w.step(brain.update(w.sensors(), g, w.seq, 1 / 60));
    const h = brain.state.hold;
    why[h || 'clear'] = (why[h || 'clear'] ?? 0) + 1;
  };
  const arm = emptyGamepad(); step(arm); arm.dpad_up = true; step(arm);
  for (let i = 0; i < 150; i++) step(emptyGamepad());
  let k = 0;
  while (w.robot.shots < shots && k++ < 60 * 25) step(emptyGamepad());
  brain.state.firing = false;
  for (let i = 0; i < 60 * 4; i++) step(emptyGamepad());
  const inVol = w.balls.balls.filter((b) => b.body.isEnabled() && inFlowerScoringVolume(w.balls.pos(b), f, b.radius)).length;
  const top = Object.entries(why).sort((a, b) => b[1] - a[1])[0]?.[0] ?? '';
  return { shots: w.robot.shots, inVol, top };
}

export async function main(argv: string[] = []): Promise<void> {
  const i = argv.indexOf('--shots');
  const shots = i >= 0 ? Number(argv[i + 1]) : 6;
  console.log('\nTHE FLOWER LOB, flown. FLOWER mode on, the robot\'s own gate deciding.\n');
  console.log('  flower  stand-off   shots   in the tube   rate   mostly held on');
  let S = 0, V = 0;
  for (const fl of [0, 1, 2, 3]) {
    for (const d of [10, 14, 18, 22]) {
      const r = await one(fl, d, shots, 20 + fl * 7 + d);
      S += r.shots; V += r.inVol;
      console.log(`  ${String(fl).padStart(6)}  ${String(d).padStart(6)} in   ${String(r.shots).padStart(5)}   ${String(r.inVol).padStart(11)}   ${(r.shots ? (r.inVol / r.shots) * 100 : 0).toFixed(0).padStart(3)}%   ${r.top.slice(0, 44)}`);
    }
  }
  console.log(`\n  TOTAL ${S} lobbed, ${V} in a tube, ${(S ? (V / S) * 100 : 0).toFixed(0)}%.\n`);
}
