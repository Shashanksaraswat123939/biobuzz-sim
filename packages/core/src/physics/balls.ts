/**
 * The 56 game elements: 40 POLLEN, 8 red NECTAR, 8 blue NECTAR.
 *
 * Every ball is a Rapier sphere with CCD on, because a ball leaving a flywheel at 10 m/s
 * moves 4 cm per 1/240 s step and will tunnel through a 6 mm pocket skin otherwise.
 * A ball being carried by the robot is disabled in the solver and teleported instead.
 */
import type RAPIER_NS from '@dimforge/rapier3d-compat';
import { aeroForce } from './aero.js';
import type { BallKind, BallSnapshot, Params, Vec3 } from '../types.js';
import type { Rng } from '../io/rng.js';
import { GROUPS } from './groups.js';

type RAPIER = typeof RAPIER_NS;

export type BallState = BallSnapshot['state'];

export interface Ball {
  id: number;
  kind: BallKind;
  mass: number;
  radius: number;
  body: RAPIER_NS.RigidBody;
  collider: RAPIER_NS.Collider;
  state: BallState;
  /** Set while the ball is being carried, so the carrier can place it. */
  home: Vec3;
}

export class BallSet {
  readonly balls: Ball[] = [];

  constructor(
    private readonly R: RAPIER,
    private readonly world: RAPIER_NS.World,
    private readonly params: Params,
    rng: Rng,
    staging: { kind: BallKind; pos: Vec3 }[],
  ) {
    staging.forEach((s, id) => {
      const spec = s.kind === 'pollen' ? params.ball.pollen : params.ball.nectar;
      const d = spec.d_m + rng.gauss(0, spec.dVar / 3);
      const m = spec.m_kg + rng.gauss(0, spec.mVar / 3);
      const body = world.createRigidBody(
        R.RigidBodyDesc.dynamic()
          .setTranslation(s.pos[0], s.pos[1], s.pos[2])
          .setCcdEnabled(params.sim.ccdOnBalls)
          .setLinearDamping(0)
          .setAngularDamping(params.ball.spinDecay),
      );
      const collider = world.createCollider(
        R.ColliderDesc.ball(d / 2)
          .setMass(Math.max(0.005, m))
          .setRestitution(params.ball.e_ball)
          // Min, not Rapier's default Average: a pair's restitution is set by the softer
          // surface, and Average made a ball bounce back out of the CELL mouth every time.
          .setRestitutionCombineRule(R.CoefficientCombineRule.Min)
          .setFriction(params.ball.mu)
          .setCollisionGroups(GROUPS.ball),
        body,
      );
      this.balls.push({ id, kind: s.kind, mass: Math.max(0.005, m), radius: d / 2, body, collider, state: 'free', home: [...s.pos] as Vec3 });
    });
  }

  /** Drag and Magnus on every ball that is off the floor. Call once per physics step. */
  applyAero(dt: number): void {
    const rho = this.params.env.rho;
    for (const b of this.balls) {
      // Every ball is integrated, including the ones inside the robot: they are real rigid
      // bodies sitting in a real bin now, not entries in a list. Aero on a ball moving at
      // 0.2 m/s inside a hopper is negligible, so there is nothing to special-case.
      b.body.resetForces(false); // addForce persists across steps otherwise
      const v = b.body.linvel();
      const w = b.body.angvel();
      const { force } = aeroForce(this.params.ball, rho, b.radius, [v.x, v.y, v.z], [w.x, w.y, w.z], dt);
      if (force[0] || force[1] || force[2]) b.body.addForce({ x: force[0], y: force[1], z: force[2] }, true);
    }
  }

  /**
   * Where the ball is. Always the body's own translation: `state` says what the ball is
   * DOING, never where it is, so the two can no longer disagree.
   */
  pos(b: Ball): Vec3 {
    const t = b.body.translation();
    return [t.x, t.y, t.z];
  }

  /**
   * Take a ball out of play entirely (scenarios, tools, elements not yet handed in).
   * Coincident enabled spheres generate huge separation impulses and rocket across the
   * field, so anything not in play must be disabled, not just moved somewhere unused.
   */
  park(b: Ball): void {
    b.state = 'free';
    b.body.setLinvel({ x: 0, y: 0, z: 0 }, true);
    b.body.setAngvel({ x: 0, y: 0, z: 0 }, true);
    b.body.setEnabled(false);
  }

  /** Put a ball back into the solver at a pose and velocity. */
  release(b: Ball, pos: Vec3, vel: Vec3, spin: Vec3, state: BallState): void {
    b.state = state;
    b.body.setEnabled(true);
    b.body.setTranslation({ x: pos[0], y: pos[1], z: pos[2] }, true);
    b.body.setLinvel({ x: vel[0], y: vel[1], z: vel[2] }, true);
    b.body.setAngvel({ x: spin[0], y: spin[1], z: spin[2] }, true);
  }

  reset(): void {
    for (const b of this.balls) {
      b.state = 'free';
      b.body.setEnabled(true);
      b.body.setTranslation({ x: b.home[0], y: b.home[1], z: b.home[2] }, true);
      b.body.setLinvel({ x: 0, y: 0, z: 0 }, true);
      b.body.setAngvel({ x: 0, y: 0, z: 0 }, true);
    }
  }

  snapshot(): BallSnapshot[] {
    return this.balls.map((b) => ({ id: b.id, kind: b.kind, p: this.pos(b), r: b.radius, state: b.state }));
  }
}
