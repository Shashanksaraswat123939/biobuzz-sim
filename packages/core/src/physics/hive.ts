/**
 * The HIVE rocker: the thing this project exists to answer questions about.
 *
 * It is an over-centre see-saw. At theta = 0 the CG sits directly above the pivot, so
 * gravity torque is m g r sin(theta): zero at the middle, pushing outward to either end
 * stop. At rest (theta = +-30.04 deg) that is about 0.63 N.m holding the rocker down.
 * Balls landing in the up-facing CELL push back with m g z (z = horizontal distance from
 * the pivot axis). When they win, it goes over and slams to the other stop: a TIP.
 *
 * Nothing here scripts a tip. The balls are ordinary rigid bodies resting in a pocket
 * built from convex boxes, and Rapier's contacts do the work. The per-ball torque numbers
 * below are diagnostics computed from the same quasi-static relation, so they can be
 * compared against what the solver actually did.
 */
import type RAPIER_NS from '@dimforge/rapier3d-compat';
import { DEG, RAD, M_TO_IN } from '../units.js';
import type { Alliance, BallKind, HiveSnapshot, Params, Vec3 } from '../types.js';
import { type FieldGeometry, type CellGeometry, pointInCell } from '../field/geometry.js';
import { GROUPS } from './groups.js';

type RAPIER = typeof RAPIER_NS;

export interface BallRef {
  id: number;
  kind: BallKind;
  mass: number;
  radius: number;
  pos: Vec3;
}

export class Hive {
  readonly body: RAPIER_NS.RigidBody;
  readonly joint: RAPIER_NS.RevoluteImpulseJoint;
  readonly pivot: Vec3;
  angle = 0;
  omega = 0;
  tips = 0;
  tipping = false;
  /** Which side of centre the rocker last settled on: -1 => CELL A up, +1 => CELL B up. */
  private side: -1 | 1;
  private armed = false;
  private lastTipT = -1;
  perBallTorque: HiveSnapshot['perBallTorque'] = [];
  ballTorque = 0;
  gravityTorque = 0;
  ballsInUpCell = 0;
  /** Test/tuning hook: a constant external torque about the pivot, N.m. */
  extraTorque = 0;
  /** Ball ids currently inside the up CELL, so scoring and NECTAR release can read them. */
  ballsInCells: { A: number[]; B: number[] } = { A: [], B: [] };

  constructor(
    private readonly R: RAPIER,
    world: RAPIER_NS.World,
    private readonly params: Params,
    private readonly geom: FieldGeometry,
    readonly alliance: Alliance,
    frameBody: RAPIER_NS.RigidBody,
  ) {
    const hp = params.hive;
    const x = alliance === 'red' ? geom.hiveX_m.red : geom.hiveX_m.blue;
    this.pivot = [x, geom.pivotY_m, 0];
    // The two rockers start mirrored. In the STEP the red AUDIENCE cell is up (bottom skin
    // at Y 51.95, Z +13.30) while the blue SCORING cell is up (Y 51.95, Z -13.30), so red
    // starts at -restAngle and blue at +restAngle. The two up-CELLs face opposite Z.
    this.side = alliance === 'red' ? -1 : 1;
    this.angle = this.side * geom.restAngle_rad;

    const q = quatX(this.angle);
    const desc = R.RigidBodyDesc.dynamic()
      .setTranslation(x, geom.pivotY_m, 0)
      .setRotation(q)
      .setLinearDamping(0)
      .setAngularDamping(0);
    this.body = world.createRigidBody(desc);

    for (const cell of geom.cells) {
      for (const piece of cell.pieces) {
        const pq = quatX(piece.rotX);
        const cd = R.ColliderDesc.cuboid(piece.half[0], piece.half[1], piece.half[2])
          .setTranslation(piece.pos[0], piece.pos[1], piece.pos[2])
          .setRotation(pq)
          .setDensity(0)
          .setRestitution(params.ball.e_poly)
          .setRestitutionCombineRule(R.CoefficientCombineRule.Min)
          .setFriction(0.35)
          .setCollisionGroups(GROUPS.rocker);
        world.createCollider(cd, this.body);
      }
    }

    // The rocker's mass comes from the CAD estimate, not from the collider boxes, whose
    // volume means nothing. It has to ride on a collider: RigidBodyDesc's additional-mass
    // setters are silently ignored in this Rapier build (mass 0, invMass 0 -- immovable).
    // I_cm = I_pivot - m d^2 by the parallel axis theorem.
    const massCarrier = R.ColliderDesc.cuboid(0.01, 0.01, 0.01)
      .setTranslation(0, 0, 0)
      .setMassProperties(
        hp.massKg,
        { x: hp.cgOffset_m[0], y: hp.cgOffset_m[1], z: hp.cgOffset_m[2] },
        {
          x: Math.max(1e-4, hp.Ipivot_kgm2 - hp.massKg * geom.cgRadius_m * geom.cgRadius_m),
          y: 0.15,
          z: 0.15,
        },
        { x: 0, y: 0, z: 0, w: 1 },
      )
      .setSensor(true)
      .setCollisionGroups(GROUPS.rocker);
    world.createCollider(massCarrier, this.body);

    const jd = R.JointData.revolute(
      { x, y: geom.pivotY_m, z: 0 },
      { x: 0, y: 0, z: 0 },
      { x: 1, y: 0, z: 0 },
    );
    // The end stops must be set on the created joint. Setting JointData.limits/limitsEnabled
    // before creation is silently ignored, which leaves the rocker free to spin right round.
    this.joint = world.createImpulseJoint(jd, frameBody, this.body, true) as RAPIER_NS.RevoluteImpulseJoint;
    this.joint.setLimits(-geom.restAngle_rad, geom.restAngle_rad);
  }

  /** Place the rocker at an angle, for tests and the scenario editor. */
  setAngle(a: number): void {
    this.angle = a;
    this.body.setRotation(quatX(a), true);
    this.body.setAngvel({ x: 0, y: 0, z: 0 }, true);
    this.side = a >= 0 ? 1 : -1; // teleporting is not a TIP
    this.armed = false;
  }

  /** Read the joint angle back out of the body rotation (the joint allows X only). */
  private readAngle(): number {
    const r = this.body.rotation();
    return 2 * Math.atan2(r.x, r.w);
  }

  /** World-frame point -> rocker body frame. */
  toBody(p: Vec3): Vec3 {
    const c = Math.cos(-this.angle);
    const s = Math.sin(-this.angle);
    const dy = p[1] - this.pivot[1];
    const dz = p[2] - this.pivot[2];
    return [p[0] - this.pivot[0], dy * c - dz * s, dy * s + dz * c];
  }

  /** Rocker body frame point -> world. */
  toWorld(p: Vec3): Vec3 {
    const c = Math.cos(this.angle);
    const s = Math.sin(this.angle);
    return [p[0] + this.pivot[0], p[1] * c - p[2] * s + this.pivot[1], p[1] * s + p[2] * c + this.pivot[2]];
  }

  get upCell(): CellGeometry {
    // The up cell is the one whose radial direction points upward at the current angle.
    return Math.cos(this.geom.cells[0].bodyAngle_rad + this.angle) > 0 ? this.geom.cells[0] : this.geom.cells[1];
  }

  get downCell(): CellGeometry {
    return this.upCell.id === 'A' ? this.geom.cells[1] : this.geom.cells[0];
  }

  /** Mouth centre of the up-facing CELL, in world coordinates. Aim here. */
  upCellMouthWorld(): Vec3 {
    const cell = this.upCell;
    const r = cell.radius_m + cell.halfInterior[1];
    const phi = cell.bodyAngle_rad;
    return this.toWorld([0, r * Math.cos(phi), r * Math.sin(phi)]);
  }

  /**
   * Once per physics step, before world.step():
   *  - apply dry pivot friction and the end-stop dampers as external torques
   *  - work out which balls are in which CELL and what each one contributes
   *  - detect a TIP
   */
  preStep(balls: BallRef[], dt: number, g: number): void {
    // Rapier accumulates addTorque across steps until it is reset.
    this.body.resetTorques(false);
    this.angle = this.readAngle();
    this.omega = this.body.angvel().x;

    const hp = this.params.hive;
    let tau = 0;

    // Dry friction at the pivot. Rapier joints have no frictionloss, so it is an external
    // torque opposing motion, capped so it can never reverse the rotation within one step.
    if (Math.abs(this.omega) > 1e-4) {
      const cap = Math.abs(this.omega) * hp.Ipivot_kgm2 / dt;
      tau -= Math.sign(this.omega) * Math.min(hp.frictionTorque_Nm, cap);
    }

    // End-stop dampers (Blum 970A). Only when moving toward the stop we are near.
    const engage = hp.damperEngage_deg * DEG;
    const lim = this.geom.restAngle_rad;
    const nearHigh = this.angle > lim - engage && this.omega > 0;
    const nearLow = this.angle < -lim + engage && this.omega < 0;
    if (nearHigh || nearLow) tau -= hp.damperC_Nms * this.omega;

    tau += this.extraTorque;
    if (tau !== 0) this.body.addTorque({ x: tau, y: 0, z: 0 }, true);

    this.gravityTorque = hp.massKg * g * this.geom.cgRadius_m * Math.sin(this.angle + cgBodyAngle(hp.cgOffset_m));

    // Which balls are where, and what each contributes about the pivot axis.
    this.ballsInCells = { A: [], B: [] };
    this.perBallTorque = [];
    this.ballTorque = 0;
    for (const b of balls) {
      const local = this.toBody(b.pos);
      for (const cell of this.geom.cells) {
        if (!pointInCell(cell, local, b.radius)) continue;
        this.ballsInCells[cell.id].push(b.id);
        // tau = (r x F).x with F = (0, -m g, 0) is m g z, z measured from the pivot axis.
        const lever = b.pos[2] - this.pivot[2];
        const t = b.mass * g * lever;
        this.perBallTorque.push({ id: b.id, kind: b.kind, torque_Nm: t, lever_in: lever * M_TO_IN });
        this.ballTorque += t;
        break;
      }
    }
    const up = this.upCell.id;
    this.ballsInUpCell = this.ballsInCells[up].length;

    // TIP: the rocker changes state and then reaches the far stop (manual 10.5.1).
    const beyond = Math.abs(this.angle) > lim * 0.9;
    const s: -1 | 1 = this.angle >= 0 ? 1 : -1;
    if (s !== this.side && beyond) {
      this.side = s;
      this.tips++;
      this.armed = true;
    }
    this.tipping = Math.abs(this.omega) > 0.2;
  }

  /** A tip that happened since the last call, for the scorer to consume. */
  takeTip(t: number): boolean {
    if (!this.armed) return false;
    this.armed = false;
    this.lastTipT = t;
    return true;
  }

  snapshot(): HiveSnapshot {
    return {
      alliance: this.alliance,
      angleDeg: this.angle * RAD,
      omegaDps: this.omega * RAD,
      upCell: this.upCell.id,
      ballsInUpCell: this.ballsInUpCell,
      ballTorque_Nm: this.ballTorque,
      gravityTorque_Nm: this.gravityTorque,
      netTorque_Nm: this.ballTorque + this.gravityTorque,
      perBallTorque: this.perBallTorque,
      tips: this.tips,
      tipping: this.tipping,
    };
  }

  reset(): void {
    this.side = this.alliance === 'red' ? -1 : 1;
    this.angle = this.side * this.geom.restAngle_rad;
    this.body.setRotation(quatX(this.angle), true);
    this.body.setAngvel({ x: 0, y: 0, z: 0 }, true);
    this.armed = false;
    this.tips = 0;
    this.tipping = false;
    this.lastTipT = -1;
  }
}

/** Angle of the CG from +Y in the rocker body frame (normally ~0: the over-centre point). */
export function cgBodyAngle(cg: Vec3): number {
  return Math.atan2(cg[2], cg[1]);
}

export function quatX(a: number): { x: number; y: number; z: number; w: number } {
  return { x: Math.sin(a / 2), y: 0, z: 0, w: Math.cos(a / 2) };
}
