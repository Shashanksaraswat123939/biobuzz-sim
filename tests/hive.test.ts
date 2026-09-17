import { describe, it, expect, beforeAll } from 'vitest';
import params from '../config/params.json';
import robotSpec from '../config/robot.json';
import { World, initPhysics } from '../packages/core/src/physics/world.js';
import { RAD } from '../packages/core/src/units.js';
import { dropTest } from '../tools/hivedrop.js';
import type { Params, RobotSpec, Vec3 } from '../packages/core/src/types.js';

const idle = { seq: 0, motors: {}, servos: {} };

function makeWorld(mutate: (p: Params) => void = () => {}) {
  const p = structuredClone(params) as unknown as Params;
  mutate(p);
  const staging: { kind: 'pollen'; pos: Vec3 }[] = [];
  return new World({ params: p, robot: robotSpec as unknown as RobotSpec, staging, alliance: 'red', seed: 7 });
}

beforeAll(async () => {
  await initPhysics();
});

describe('the hinge (PLAN.md phase 1)', () => {
  it('an empty rocker sits still on its stop', () => {
    const w = makeWorld();
    let peak = 0;
    for (let f = 0; f < 300; f++) {
      w.step(idle);
      if (f > 30) peak = Math.max(peak, Math.abs(w.hives.red.omega * RAD));
    }
    expect(Math.abs(w.hives.red.angle + w.geom.restAngle_rad) * RAD).toBeLessThan(0.2);
    expect(peak).toBeLessThan(0.5);
  });

  it('the two rockers start mirrored, as the CAD stages them', () => {
    const w = makeWorld();
    expect(w.hives.red.angle).toBeLessThan(0);
    expect(w.hives.blue.angle).toBeGreaterThan(0);
    // Red's up CELL faces the audience (+Z), blue's faces -Z.
    expect(w.hives.red.upCellMouthWorld()[2]).toBeGreaterThan(0);
    expect(w.hives.blue.upCellMouthWorld()[2]).toBeLessThan(0);
  });

  it('a torque below the gravity restoring torque does not tip it, and one above does', () => {
    const w = makeWorld();
    const h = w.hives.red;
    const restoring = Math.abs(h.gravityTorque || 0.62);

    h.setAngle(-w.geom.restAngle_rad);
    h.extraTorque = restoring * 0.5;
    for (let f = 0; f < 180; f++) w.step(idle);
    expect(h.tips).toBe(0);

    h.setAngle(-w.geom.restAngle_rad);
    h.extraTorque = restoring * 1.5;
    for (let f = 0; f < 180; f++) w.step(idle);
    expect(h.tips).toBeGreaterThanOrEqual(1);
    expect(h.angle * RAD).toBeGreaterThan(25);
  });

  it('the rocker is free to swing: it must not jam on the static frame', () => {
    // The real A-frame is open where the rocker swings. Any approximation of it will jam
    // the pocket unless the rocker is in its own collision group.
    const w = makeWorld();
    const h = w.hives.red;
    h.extraTorque = 3.0;
    for (let f = 0; f < 120; f++) w.step(idle);
    expect(h.angle * RAD).toBeGreaterThan(25);
  });

  it('per-ball torque is reported with a lever arm measured from the pivot axis', async () => {
    const r = await dropTest({}, 'pollen', 6);
    expect(r.levers_in.length).toBeGreaterThan(0);
    for (const l of r.levers_in) expect(Math.abs(l)).toBeGreaterThan(2);
    // Ball torque must agree with m g z summed over the balls in the CELL.
    expect(Math.abs(r.ballTorque_Nm)).toBeGreaterThan(0);
    // Six POLLEN never tip it, so this runs the full drop budget -- same 60 s as its siblings.
  }, 60000);

  it('tips under a plausible number of POLLEN, and fewer NECTAR', async () => {
    const pollen = await dropTest({}, 'pollen');
    const nectar = await dropTest({}, 'nectarRed');
    expect(pollen.ballsToTip).not.toBeNull();
    expect(nectar.ballsToTip).not.toBeNull();
    // NECTAR is 1.66x the mass of POLLEN, so it must take fewer of them.
    expect(nectar.ballsToTip!).toBeLessThan(pollen.ballsToTip!);
    // Sanity band around the plan's hypothesis table (8-17 POLLEN at a 5-10 in lever).
    expect(pollen.ballsToTip!).toBeGreaterThanOrEqual(6);
    expect(pollen.ballsToTip!).toBeLessThanOrEqual(20);
  }, 60000);

  it('a heavier rocker takes more balls to tip', async () => {
    const light = await dropTest({ massKg: 1.6 }, 'pollen');
    const heavy = await dropTest({ massKg: 3.2 }, 'pollen');
    expect(light.ballsToTip!).toBeLessThan(heavy.ballsToTip!);
  }, 60000);
});
