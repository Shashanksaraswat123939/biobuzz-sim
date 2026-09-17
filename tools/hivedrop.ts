/**
 * Phase 1: put game elements into the up CELL one at a time until the HIVE tips.
 * This is the number the Competition Manual does not give and the season turns on.
 *
 *   npm run tool -- tools/hivedrop.ts [--kind pollen|nectar] [--mass 2.4] [--friction 0.05] [--json]
 *
 * Balls are placed gently just inside the mouth and allowed to roll to wherever the pocket
 * geometry puts them -- the settling place is an output, not an input.
 */
import params from '../config/params.json' with { type: 'json' };
import robotSpec from '../config/robot.json' with { type: 'json' };
import { World, initPhysics } from '../packages/core/src/physics/world.js';
import { M_TO_IN, inches, RAD } from '../packages/core/src/units.js';
import { fromCellLocal } from '../packages/core/src/field/geometry.js';
import type { Params, RobotSpec, Vec3, BallKind } from '../packages/core/src/types.js';

export interface DropResult {
  ballsToTip: number | null;
  placed: number;
  settledInCell: number;
  meanLever_in: number;
  ballTorque_Nm: number;
  gravityTorque_Nm: number;
  tipDuration_s: number;
  levers_in: number[];
}

const idle = { seq: 0, motors: {}, servos: {} };

export async function dropTest(over: Partial<Params['hive']> = {}, kind: BallKind = 'pollen', maxBalls = 40, trace = false): Promise<DropResult> {
  await initPhysics();
  const p: Params = structuredClone(params) as unknown as Params;
  Object.assign(p.hive, over);

  const staging = Array.from({ length: maxBalls }, () => ({ kind, pos: [0, -5, 0] as Vec3 }));
  const world = new World({ params: p, robot: robotSpec as unknown as RobotSpec, staging, alliance: 'red', seed: 7 });
  const hive = world.hives.red;
  for (const b of world.balls.balls) world.balls.park(b); // nothing in play until placed
  const cell = hive.upCell;

  let placed = 0;
  let tipAt: number | null = null;
  let tipStart = 0;
  let tipEnd = 0;
  let col = 0;

  /** Pocket-local (x, u, t) -> world. */
  const at = (x: number, u: number, t: number): Vec3 =>
    hive.toWorld([
      x,
      fromCellLocal(cell, 0, u, t)[1],
      fromCellLocal(cell, 0, u, t)[2],
    ]);

  const r0 = world.balls.balls[0].radius;
  const hu = cell.halfInterior[1];
  const ht = cell.halfInterior[2];

  for (let frame = 0; tipAt === null && frame < 60 * 240; frame++) {
    // Place one element every 0.5 s, gently, at the next free slot in the pocket: columns
    // across the 20 in width, rows up the downhill wall, then layers. Gently on purpose --
    // this measures the HIVE, not how well a shot stays in (that is the shooter's problem).
    if (frame % 30 === 0 && placed < maxBalls) {
      const b = world.balls.balls[placed];
      const c6 = col % 6;
      const row = Math.floor(col / 6) % 3;
      const layer = Math.floor(col / 18);
      const spawn = at(inches(-8 + c6 * 3.2), -hu + r0 + layer * inches(2.5) + 0.01, ht - r0 - row * inches(2.9));
      const clear = world.balls.balls.every(
        (o) => o === b || o.state === 'free' || Math.hypot(...(world.balls.pos(o).map((v, i) => v - spawn[i]) as Vec3)) > r0 * 2.1,
      );
      if (clear) {
        world.balls.release(b, spawn, [0, 0, 0], [0, 0, 0], 'cell');
        placed++;
      }
      col++;
    }
    world.step(idle);
    if (tipStart === 0 && Math.abs(hive.angle) < world.geom.restAngle_rad * 0.9) tipStart = world.t;
    if (hive.tips > 0 && tipAt === null) {
      tipAt = placed;
      tipEnd = world.t;
    }
    if (trace && frame % 60 === 0) {
      const lost = world.balls.balls
        .slice(0, placed)
        .filter((b) => b.state !== 'cell')
        .map((b) => `#${b.id}:${world.balls.pos(b).map((v) => (v * 39.3701).toFixed(0)).join('/')}(${b.state})`);
      console.log(`  f=${frame} placed=${placed} inUp=${hive.ballsInUpCell} lost=${lost.length} ${lost.slice(0, 6).join(' ')}`);
    }
    if (placed >= maxBalls && frame > 60 * 60 && Math.abs(hive.omega) < 1e-4) break;
  }

  const levers = hive.perBallTorque.map((b) => b.lever_in);
  return {
    ballsToTip: tipAt,
    placed,
    settledInCell: hive.ballsInUpCell,
    meanLever_in: levers.length ? levers.reduce((a, b) => a + Math.abs(b), 0) / levers.length : NaN,
    ballTorque_Nm: hive.ballTorque,
    gravityTorque_Nm: hive.gravityTorque,
    tipDuration_s: tipStart && tipEnd > tipStart ? tipEnd - tipStart : NaN,
    levers_in: levers,
  };
}

export async function main(argv: string[] = []): Promise<void> {
  const arg = (k: string) => {
    const i = argv.indexOf(`--${k}`);
    return i >= 0 ? argv[i + 1] : undefined;
  };
  const over: Partial<Params['hive']> = {};
  if (arg('mass')) over.massKg = Number(arg('mass'));
  if (arg('friction')) over.frictionTorque_Nm = Number(arg('friction'));
  const kind = (arg('kind') as BallKind) ?? 'pollen';

  const r = await dropTest(over, kind, 40, argv.includes('--trace'));
  if (argv.includes('--json')) {
    console.log(JSON.stringify(r, null, 1));
    return;
  }
  console.log(`kind=${kind}  mass=${over.massKg ?? params.hive.massKg} kg  pivot friction=${over.frictionTorque_Nm ?? params.hive.frictionTorque_Nm} N.m`);
  console.log(`balls to tip  : ${r.ballsToTip ?? `did not tip after ${r.placed}`}`);
  console.log(`in the CELL   : ${r.settledInCell} of ${r.placed} placed`);
  console.log(`mean lever    : ${r.meanLever_in.toFixed(2)} in from the pivot axis`);
  console.log(`ball torque   : ${r.ballTorque_Nm.toFixed(3)} N.m  vs gravity ${r.gravityTorque_Nm.toFixed(3)} N.m`);
  console.log(`tip duration  : ${r.tipDuration_s.toFixed(3)} s`);
  console.log(`levers (in)   : ${r.levers_in.map((s) => s.toFixed(1)).join(', ')}`);
}

export const _rad = RAD;
export const _m2in = M_TO_IN;
