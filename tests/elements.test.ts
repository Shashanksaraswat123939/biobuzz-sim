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
import staging from '../assets/staging.json';
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

describe('the staging is the staging the manual describes (10.3.1)', () => {
  /**
   * THREE SEPARATE BUGS PUT 19 OF 56 BALLS OUT OF PLAY BEFORE THE MATCH STARTED, and none of
   * them reported anything -- the balls were simply never there.
   *
   *  - The GARDEN POLLEN come from the STEP at |x| 73.1 in, two and a half inches the far side
   *    of a 70.68 in wall, so all sixteen were benched as out of bounds.
   *  - `parkOffField` called a ball out of bounds when its CENTRE was within a RADIUS of the
   *    wall, which is every POLLEN in the two FLOWERs that stand on the +-X walls.
   *  - The preload took the four nearest `free` POLLEN, and at construction time nothing has
   *    been classified yet, so the nearest flower emptied itself into the hopper.
   */
  const built = () => {
    const w = rig(staging.balls.map((b) => ({ kind: b.kind as BallKind, pos: b.pos as Vec3 })));
    for (let i = 0; i < 90; i++) w.step({ seq: i, motors: {}, servos: {} });
    return w;
  };

  it('has all 40 POLLEN in play, 4 up every FLOWER and 8 in each GARDEN', () => {
    const w = built();
    const live = w.balls.balls.filter((b) => b.body.isEnabled());
    expect(live.filter((b) => b.kind === 'pollen')).toHaveLength(40);

    for (const [i, f] of w.geom.flowers.entries()) {
      const n = live.filter((b) => {
        const p = w.balls.pos(b);
        return Math.hypot(p[0] - f.x_m, p[2] - f.z_m) < f.openingR_m + b.radius;
      }).length;
      expect(n, `flower ${i} at ${(f.x_m * M_TO_IN).toFixed(0)}, ${(f.z_m * M_TO_IN).toFixed(0)}`).toBe(4);
    }

    for (const a of ['red', 'blue'] as const) {
      const z = w.geom.zones.find((q) => q.name === 'GARDEN' && q.alliance === a)!;
      const n = live.filter((b) => {
        const p = w.balls.pos(b);
        return p[0] > z.min[0] && p[0] < z.max[0] && p[2] > z.min[2] && p[2] < z.max[2] && p[1] < z.max[1];
      }).length;
      expect(n, `${a} GARDEN`).toBe(8);
    }
  });

  it('starts 3 NECTAR in each up CELL and leaves 5 per alliance with the human player', () => {
    const w = built();
    const parked = w.balls.balls.filter((b) => !b.body.isEnabled());
    expect(parked).toHaveLength(10);
    expect(parked.every((b) => b.kind !== 'pollen'), 'a POLLEN is never out of play at the start').toBe(true);
    expect(w.balls.balls.filter((b) => b.state === 'cell')).toHaveLength(6);
  });

  it('a preload is 4 POLLEN per robot and never comes out of a FLOWER or a GARDEN', () => {
    const p = structuredClone(params) as unknown as Params;
    const st = staging.balls.map((b) => ({ kind: b.kind as BallKind, pos: b.pos as Vec3 }));
    const w = new World({ params: p, robot: spec(), staging: st, alliance: 'red', seed: 5, preload: 4, opponent: true });
    for (let i = 0; i < 90; i++) w.step({ seq: i, motors: {}, servos: {} });
    const snap = w.snapshot();
    expect(snap.robot.hopper.count).toBe(4);
    expect(snap.opponent?.hopper.count).toBe(4);
    for (const [i, f] of w.geom.flowers.entries()) {
      const n = w.balls.balls.filter((b) => {
        if (!b.body.isEnabled()) return false;
        const q = w.balls.pos(b);
        return Math.hypot(q[0] - f.x_m, q[2] - f.z_m) < f.openingR_m + b.radius;
      }).length;
      expect(n, `flower ${i} after both preloads`).toBe(4);
    }
  });
});

describe('a FLOWER is worth what STRATEGY.md 7.1 says it is', () => {
  /**
   * The whole endgame is "plug early, cap late": the first NECTAR into a tube is 5 for the
   * bottom bonus plus 2 for itself, and it makes every ball above it yours. The scorer had the
   * rule; nothing had ever checked that a ball dropped down a real tube lands in the volume
   * the rule reads, which is the half that can silently stop working.
   */
  const drop = (kind: BallKind, flower: number) => {
    const w = rig(staging.balls.map((b) => ({ kind: b.kind as BallKind, pos: b.pos as Vec3 })));
    for (let i = 0; i < 60; i++) w.step({ seq: i, motors: {}, servos: {} });
    const f = w.geom.flowers[flower];
    // A NECTAR is with the human player at the start, so take a parked one and post it in.
    const b = w.balls.balls.find((x) => x.kind === kind && !x.body.isEnabled())!;
    w.balls.release(b, [f.x_m, f.topY_m + 0.05, f.z_m], [0, 0, -0.05], [0, 0, 0], 'free');
    for (let i = 0; i < 240; i++) w.step({ seq: i, motors: {}, servos: {} });
    return w;
  };

  it('three staged POLLEN sit in the scoring volume and belong to nobody', () => {
    const w = rig(staging.balls.map((b) => ({ kind: b.kind as BallKind, pos: b.pos as Vec3 })));
    for (let i = 0; i < 90; i++) w.step({ seq: i, motors: {}, servos: {} });
    const c = w.endOfMatchCounts('red');
    // 4 staged per tube: one in the bottom slot under the middle ring, three in the volume.
    expect(c.flowers.map((f) => f.elements)).toEqual([3, 3, 3, 3]);
    expect(c.flowers.every((f) => f.topNectar === null)).toBe(true);
    expect(c.bottomNectar.every((b) => b.alliance === null)).toBe(true);
  });

  it('one NECTAR down a tube takes the bottom bonus and everything above it', () => {
    const w = drop('nectarRed', 0);
    const red = w.endOfMatchCounts('red');
    expect(red.flowers[0].topNectar, 'the plug owns the flower').toBe('red');
    expect(red.bottomNectar[0].alliance, 'and it is the bottom-most NECTAR').toBe('red');
    // 5 for the bottom bonus, 2 for each element in the volume. The NECTAR is one of them.
    const before = w.scorer.project('red', w.endOfMatchCounts('red'));
    w.scorer.finalise('red', w.endOfMatchCounts('red'));
    expect(w.scorer.state.red.total, 'the projection is the same arithmetic').toBe(before);
    expect(w.scorer.state.red.bottomNectar).toBe(1);
    expect(w.scorer.state.red.flower).toBeGreaterThanOrEqual(4);
  });

  it('the opponent NECTAR takes the same flower away from us', () => {
    const w = drop('nectarBlue', 0);
    expect(w.endOfMatchCounts('red').flowers[0].topNectar).toBe('blue');
    expect(w.endOfMatchCounts('blue').flowers[0].topNectar).toBe('blue');
    w.scorer.finalise('red', w.endOfMatchCounts('red'));
    w.scorer.finalise('blue', w.endOfMatchCounts('blue'));
    expect(w.scorer.state.red.flower, 'a flower we do not own scores us nothing').toBe(0);
    expect(w.scorer.state.blue.flower).toBeGreaterThanOrEqual(4);
  });
});
