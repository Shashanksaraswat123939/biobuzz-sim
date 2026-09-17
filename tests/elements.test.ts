/**
 * The SCORING ELEMENTS, as rules rather than as decoration.
 *
 * Two behaviours here used to be absent, and both were invisible because nothing could see
 * the difference between "the rule is enforced" and "the situation never arises":
 *
 *   G408, opponent NECTAR. There was no colour check anywhere, so a red robot could hoover up
 *   blue NECTAR all match and score it. The fix is not a penalty, it is the colour sensor
 *   reversing the roller -- so the test is that the ball ends up OUTSIDE.
 *
 *   The FLOWER retrieval opening. The tube was closed, so a ball that went into a flower was
 *   gone. That deletes the counter-play the endgame is built on AND the reason a NECTAR plug
 *   is worth 5 points. Both come from one number -- a 3.55 in doorway takes a 2.80 in POLLEN
 *   and not a 3.62 in NECTAR -- so the test is that the geometry says so with no rule code.
 */
import { describe, it, expect, beforeAll } from 'vitest';
import params from '../config/params.json';
import robotSpec from '../config/robot.json';
import { World, initPhysics, emptyGamepad } from '../packages/core/src/physics/world.js';
import { BuiltinTeleOp, ShotTable } from '../packages/core/src/robot/builtinTeleOp.js';
import { inches, M_TO_IN } from '../packages/core/src/units.js';
import type { BallKind, Params, RobotSpec, Vec3 } from '../packages/core/src/types.js';

beforeAll(async () => { await initPhysics(); });

const spec = () => robotSpec as unknown as RobotSpec;

function rig(staging: { kind: BallKind; pos: Vec3 }[], alliance: 'red' | 'blue' = 'red') {
  const p = structuredClone(params) as unknown as Params;
  return new World({ params: p, robot: spec(), staging, alliance, seed: 5 });
}

describe('G408: the intake will not take the opponent\'s NECTAR', () => {
  /** Drive forward over a ball sitting just in front of the mouth; report whether it is aboard. */
  const tryToEat = (kind: BallKind) => {
    const ahead: Vec3 = [0, 0.05, 0.42];
    const w = rig([{ kind, pos: ahead }], 'red');
    w.robot.place([0, 0.17, 0], 0);
    // NECTAR gets swept into a CELL by the world's own staging, so put it back in front of
    // the mouth explicitly rather than trusting where it was asked to start.
    const b = w.balls.balls.find((x) => x.kind === kind)!;
    w.balls.release(b, ahead, [0, 0, 0], [0, 0, 0], 'free');
    const brain = new BuiltinTeleOp(spec(), new ShotTable([]));
    const g = emptyGamepad();
    g.left_stick_y = -0.35;                       // creep onto it, intake always running
    for (let i = 0; i < 150; i++) w.step(brain.update(w.sensors(), g, i, 1 / 60));
    return w.snapshot().robot.hopper.count;
  };

  it('takes POLLEN and its own NECTAR, and spits the opponent\'s back out', () => {
    expect(tryToEat('pollen')).toBeGreaterThan(0);
    expect(tryToEat('nectarRed')).toBeGreaterThan(0);
    expect(tryToEat('nectarBlue')).toBe(0);
  });
});

describe('the FLOWER retrieval opening', () => {
  /**
   * Shove a ball resting in the bottom slot straight at the doorway and see whether it gets
   * out. This is deliberately a push, not an intake run: the claim under test is about the
   * hole in the tube, and involving the robot would let an intake bug masquerade as a
   * geometry result.
   */
  const shoveOut = (kind: BallKind) => {
    const w = rig([{ kind, pos: [0, 0.05, 0] }]);
    const f = w.geom.flowers[0];
    const b = w.balls.balls.find((x) => x.kind === kind);
    expect(b, `no ${kind} staged`).toBeTruthy();
    const r0: Vec3 = [f.x_m, inches(0.2) + b!.radius + 0.002, f.z_m];
    w.balls.release(b!, r0, [0, 0, 0], [0, 0, 0], 'free');
    for (let i = 0; i < 60; i++) w.step({ seq: i, motors: {}, servos: {} });   // let it settle
    // Push it at the doorway, hard enough to clear the tube if it fits at all.
    for (let i = 0; i < 90; i++) {
      b!.body.wakeUp();
      // SET the velocity, do not add a force: BallSet.applyAero calls resetForces() every
      // substep, so an external force never survives to the solver. This is what an intake
      // dragging the ball out looks like anyway.
      const v = b!.body.linvel();
      b!.body.setLinvel({ x: f.openingDir[0] * 0.8, y: v.y, z: f.openingDir[1] * 0.8 }, true);
      w.step({ seq: i, motors: {}, servos: {} });
    }
    const p = w.balls.pos(b!);
    return Math.hypot(p[0] - f.x_m, p[2] - f.z_m) * M_TO_IN;
  };

  it('passes a POLLEN and keeps a NECTAR, from the doorway height alone', () => {
    const pollenOut = shoveOut('pollen');
    const nectarOut = shoveOut('nectarRed');
    // A POLLEN leaves the tube entirely; a NECTAR cannot get past the wall.
    expect(pollenOut).toBeGreaterThan(4);
    expect(nectarOut).toBeLessThan(3);
  });

  it('the doorway is shorter than a NECTAR and taller than a POLLEN, which is the whole rule', () => {
    const f = rig([{ kind: 'pollen', pos: [0, 0.05, 0] }]).geom.flowers[0];
    const dia_in = (k: 'pollen' | 'nectar') => params.ball[k].d_m * M_TO_IN;
    expect(f.retrievalTopY_m * M_TO_IN).toBeGreaterThan(dia_in('pollen'));
    expect(f.retrievalTopY_m * M_TO_IN).toBeLessThan(dia_in('nectar'));
  });
});
