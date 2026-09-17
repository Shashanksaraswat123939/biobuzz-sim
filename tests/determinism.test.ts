import { describe, it, expect, beforeAll } from 'vitest';
import params from '../config/params.json';
import robotSpec from '../config/robot.json';
import staging from '../assets/staging.json';
import { World, initPhysics, emptyGamepad } from '../packages/core/src/physics/world.js';
import { BuiltinTeleOp, ShotTable } from '../packages/core/src/robot/builtinTeleOp.js';
import type { BallKind, GamepadState, Params, RobotSpec, Vec3 } from '../packages/core/src/types.js';

const balls = (staging.balls as { kind: string; pos: number[] }[]).map((b) => ({ kind: b.kind as BallKind, pos: b.pos as Vec3 }));

beforeAll(async () => {
  await initPhysics();
});

/** One scripted run: same seed, same command log, so the hashes must match exactly. */
function run(seed: number, frames: number): string[] {
  const p = structuredClone(params) as unknown as Params;
  const spec = robotSpec as unknown as RobotSpec;
  const world = new World({ params: p, robot: spec, staging: balls, alliance: 'red', seed });
  const brain = new BuiltinTeleOp(spec, new ShotTable([{ range_in: 60, hoodPos: 0.8, rpm: 3500, margin: 0.05 }]));
  world.clock.start();

  const hashes: string[] = [];
  for (let f = 0; f < frames; f++) {
    // A fixed command log: drive, then intake, then spin up and fire.
    const g: GamepadState = emptyGamepad();
    g.left_stick_y = f < 60 ? -0.8 : 0;
    g.right_stick_x = f >= 60 && f < 90 ? 0.5 : 0;
    g.right_trigger = f >= 90 && f < 150 ? 1 : 0;
    g.a = f === 150;
    g.right_bumper = f > 200;

    world.setGamepads(g, emptyGamepad());
    world.step(brain.update(world.sensors(), g, world.seq));
    if (f === 99 || f === 299 || f === frames - 1) hashes.push(world.createSnapshotHash());
  }
  return hashes;
}

describe('determinism (PLAN.md section 12)', () => {
  it('the same seed and command log give bit-identical snapshots, three runs', () => {
    const a = run(1, 360);
    const b = run(1, 360);
    const c = run(1, 360);
    expect(a).toEqual(b);
    expect(b).toEqual(c);
    expect(a.every((h) => /^[0-9a-f]{8}$/.test(h))).toBe(true);
  }, 60000);

  it('a different seed gives a different run', () => {
    expect(run(1, 360)).not.toEqual(run(9, 360));
  }, 60000);

  it('reset puts the match, the score and every ball back where they started', () => {
    // Not bit-identical on purpose: Rapier warm-starts its contact solver, so only building
    // a fresh World reproduces a hash exactly (which is what the UI's reset button does).
    // What reset must guarantee is the game state.
    const p = structuredClone(params) as unknown as Params;
    const world = new World({ params: p, robot: robotSpec as unknown as RobotSpec, staging: balls, alliance: 'red', seed: 3 });
    const idle = { seq: 0, motors: {}, servos: {} };
    const home = balls.map((b) => b.pos);

    world.clock.start();
    for (let f = 0; f < 240; f++) world.step(idle);
    world.hives.red.extraTorque = 5;
    for (let f = 0; f < 120; f++) world.step(idle);
    expect(world.hives.red.tips).toBeGreaterThan(0);

    world.hives.red.extraTorque = 0;
    world.reset();

    expect(world.t).toBe(0);
    expect(world.clock.period).toBe('STAGING');
    expect(world.scorer.state.red.total).toBe(0);
    expect(world.hives.red.tips).toBe(0);
    expect(world.robot.shots).toBe(0);
    expect(world.battery.soc).toBe(1);
    world.balls.balls.forEach((b, i) => {
      const q = world.balls.pos(b);
      for (let k = 0; k < 3; k++) expect(q[k]).toBeCloseTo(home[i][k], 6);
    });
  }, 60000);
});
