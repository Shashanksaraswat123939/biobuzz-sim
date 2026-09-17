/**
 * The world: it decides what happens to matter. The brain decides what the motors are told.
 * Nothing in here knows about the DOM, Three.js or the bridge.
 */
import RAPIER from '@dimforge/rapier3d-compat';
import { buildFieldGeometry, pointInCell, type FieldGeometry } from '../field/geometry.js';
import { worldToFtc, worldYawToFtcHeadingDeg, worldDirToFtcBearingDeg } from '../field/ftcFrame.js';
import { BallSet, type Ball } from './balls.js';
import { Hive, type BallRef, quatX } from './hive.js';
import { Robot, quatY } from './robot.js';
import { Battery } from './battery.js';
import { MatchClock } from '../rules/match.js';
import { Scorer, inFlowerScoringVolume, allianceOfNectar, type EndOfMatchCounts } from '../rules/scoring.js';
import { Rng } from '../io/rng.js';
import { GROUPS } from './groups.js';
import { inches, M_TO_IN, RAD, DEG, clamp, wrapPi } from '../units.js';
import type { ActuatorFrame, Alliance, BallKind, GamepadState, Params, RobotSpec, SensorFrame, ShotRecord, Snapshot, Vec3 } from '../types.js';

export interface StagedBall { kind: BallKind; pos: Vec3 }

export interface WorldOptions {
  params: Params;
  robot: RobotSpec;
  staging: StagedBall[];
  alliance: Alliance;
  /** Robot start pose: world metres and yaw in degrees. */
  start?: { p: Vec3; yawDeg: number };
  seed?: number;
  /** POLLEN the robot starts the match holding. Real robots do; autos rely on it. */
  preload?: number;
}

export const emptyGamepad = (): GamepadState => ({
  left_stick_x: 0, left_stick_y: 0, right_stick_x: 0, right_stick_y: 0,
  left_trigger: 0, right_trigger: 0,
  a: false, b: false, x: false, y: false,
  dpad_up: false, dpad_down: false, dpad_left: false, dpad_right: false,
  left_bumper: false, right_bumper: false, start: false, back: false,
  left_stick_button: false, right_stick_button: false,
});

export async function initPhysics(): Promise<void> {
  await RAPIER.init();
}

export class World {
  readonly geom: FieldGeometry;
  readonly physics: RAPIER.World;
  readonly balls: BallSet;
  readonly hives: Record<Alliance, Hive>;
  readonly robot: Robot;
  readonly clock: MatchClock;
  readonly scorer = new Scorer();
  readonly battery: Battery;
  readonly alliance: Alliance;

  t = 0;
  seq = 0;
  telemetry: [string, string][] = [];

  private readonly rng: Rng;
  private readonly params: Params;
  private lastActuators: ActuatorFrame = { seq: 0, motors: {}, servos: {} };
  private gamepads: [GamepadState, GamepadState] = [emptyGamepad(), emptyGamepad()];
  private imuBuffer: { t: number; yaw: number; rate: number }[] = [];
  private leftStart: Record<Alliance, boolean> = { red: false, blue: false };
  private nectarEntitlement: Record<Alliance, number> = { red: 0, blue: 0 };
  private finalised = false;

  constructor(opts: WorldOptions) {
    this.params = opts.params;
    this.alliance = opts.alliance;
    this.rng = new Rng(opts.seed ?? opts.params.sim.seed);
    this.geom = buildFieldGeometry(opts.params);
    this.clock = new MatchClock(opts.params.match);
    this.battery = new Battery(opts.params.battery);

    this.physics = new RAPIER.World({ x: 0, y: -opts.params.env.g, z: 0 });
    this.physics.timestep = opts.params.sim.dt;
    this.physics.numSolverIterations = opts.params.sim.solverIterations;

    const statics = this.physics.createRigidBody(RAPIER.RigidBodyDesc.fixed());
    this.buildStatics(statics);

    this.balls = new BallSet(RAPIER, this.physics, opts.params, this.rng.fork(11), opts.staging);
    this.parkOffField();
    this.hives = {
      red: new Hive(RAPIER, this.physics, opts.params, this.geom, 'red', statics),
      blue: new Hive(RAPIER, this.physics, opts.params, this.geom, 'blue', statics),
    };
    // After the hives exist: staging asks each one where its up CELL is.
    this.stageCells();

    const start = opts.start ?? this.defaultStart(opts.alliance, opts.robot);
    this.robot = new Robot(RAPIER, this.physics, opts.params, opts.robot, this.rng.fork(23), {
      p: start.p,
      yaw: start.yawDeg * DEG,
    });
    this.preload = Math.max(0, Math.min(opts.preload ?? 0, opts.robot.hopper.capacity));
    this.loadPreload();
  }

  private readonly preload: number;
  /** Balls credited by a TIP emptying our own up CELL, per alliance. See landedInUpCell(). */
  private dumpedAtTips: Record<Alliance, number> = { red: 0, blue: 0 };
  private tipsSeen: Record<Alliance, number> = { red: 0, blue: 0 };
  private inUpCellPrev: Record<Alliance, number> = { red: 0, blue: 0 };

  /**
   * The STEP stages 16 POLLEN and 10 NECTAR OUTSIDE the glass -- those are the human
   * player's hands, not the field. Leaving them there makes balls float in mid-air beyond
   * the wall with nothing under them. They are out of play until a human hands them in
   * (G426/G427), so they start disabled.
   */
  /**
   * Take out of play everything the CAD stages that is not actually on the field at the
   * start of a match.
   *
   * Two groups. Balls beyond the glass are the alliance's supply in the human-player area,
   * and they sit on the floor of the venue rather than on the field.
   *
   * The other group is the six NECTAR the STEP stages INSIDE the two CELLs. Those are cleared
   * here and then RE-STAGED properly by stageCells(), because the CAD's display state is not
   * a match start -- it fills both CELLs of both hives, and the manual fills only the
   * upward-facing one of each.
   *
   * This used to clear them and stop there, on the grounds that "a match starts with empty
   * CELLs". It does not: manual 10.3.1 B.i stages "3 NECTAR in each upward-facing CELL of
   * corresponding color", and Fig 10-2 shows them. The second reason given -- that staged
   * balls rendered floating under the CAD skin -- was a symptom of the pocket being 11 deg
   * out (PHYSICS 9.3); the reconstructed pocket now matches the manual's lip heights to
   * better than half an inch, so they sit where they should.
   *
   * It matters more than any other game-model error here, because the FIRST TIP is the one
   * both alliances race for and this changes its price: three NECTAR at a 10 in lever arm are
   * roughly half way over, so a real first tip costs 5-7 POLLEN and an empty-CELL simulator
   * charges 12. Every autonomous plan timed in this simulator was wrong in the same
   * direction.
   */
  private parkOffField(): void {
    const hw = this.geom.halfWidth_m;
    for (const b of this.balls.balls) {
      const q = this.balls.pos(b);
      if (Math.abs(q[0]) > hw - b.radius || Math.abs(q[2]) > hw - b.radius) {
        this.balls.park(b);
        continue;
      }
      // Above the pivot means inside the rocker: nothing else up there is reachable.
      if (q[1] > this.geom.pivotY_m - b.radius) this.balls.park(b);
    }
  }

  /**
   * Manual 10.3.1 B.i: "3 NECTAR in each upward-facing CELL of corresponding color", placed
   * against the back wall. Fig 10-2 shows them.
   *
   * Called after parkOffField(), which has just cleared the CAD's own display staging (it
   * fills all four CELLs; the manual fills two). Doing it explicitly rather than trusting the
   * CAD keeps the start deterministic and the count right.
   */
  private stageCells(): void {
    for (const alliance of ['red', 'blue'] as const) {
      const hive = this.hives[alliance];
      const kind: BallKind = alliance === 'red' ? 'nectarRed' : 'nectarBlue';
      this.balls.balls
        .filter((b) => b.kind === kind)
        .slice(0, 3)
        .forEach((b, i) => this.balls.release(b, hive.upCellStagePos(i, b.radius), [0, 0, 0], [0, 0, 0], 'cell'));
    }
  }

  /**
   * Sampled every step so a TIP is caught at the instant it happens: once the rocker goes
   * over, its CELL is empty and the balls it dumped are gone from any later count.
   */
  private censusUpCells(): void {
    for (const a of ['red', 'blue'] as const) {
      if (this.hives[a].tips > this.tipsSeen[a]) {
        this.dumpedAtTips[a] += this.inUpCellPrev[a];
        this.tipsSeen[a] = this.hives[a].tips;
      }
      this.inUpCellPrev[a] = this.balls.balls.filter((b) => this.inOwnUpCell(a, b)).length;
    }
  }

  /** Is this ball inside the alliance's OWN upward-facing CELL right now? */
  private inOwnUpCell(alliance: Alliance, b: { radius: number; id: number }): boolean {
    const h = this.hives[alliance];
    const local = h.toBody(this.balls.pos(this.balls.balls[b.id]));
    return pointInCell(h.upCell, local, (b as { radius: number }).radius);
  }

  /**
   * ONE LAND-RATE CENSUS, which every tool reads instead of rolling its own (PHYSICS 9.15).
   *
   * What scores is what is sitting in our own up CELL when it matters: at the buzzer, plus
   * whatever was in there at the instant of each TIP, because a tip is the CELL doing its job
   * and emptying itself. A ball that only visits is counted by neither.
   *
   * The two homegrown versions this replaces were wrong in opposite directions and their
   * disagreement is how the problem was found -- tools/gatecal.ts read 1.0% where
   * tools/landcal.ts had just measured 66% on the same robot and table:
   *
   *   `hives.red.ballsInUpCell` counts one CELL of one rocker, and the rocker ROCKS. A
   *   rotation short of a scored tip carries balls into the down CELL, where it reads zero,
   *   so a run could score all afternoon and report nothing.
   *
   *   The shot log's `result === 'cell'` tested every cell of both hives, so it counted the
   *   down CELL and the opponent's hive.
   */
  landedInUpCell(alliance: Alliance): number {
    return this.dumpedAtTips[alliance] + this.balls.balls.filter((b) => this.inOwnUpCell(alliance, b)).length;
  }

  /** Take the POLLEN nearest the robot off the field and into its hopper. */
  private loadPreload(): void {
    if (this.preload <= 0) return;
    const p = this.robot.pos;
    const free = this.balls.balls
      .filter((b) => b.kind === 'pollen' && b.state === 'free' && b.body.isEnabled())
      .map((b) => ({ b, d: Math.hypot(...(this.balls.pos(b).map((v, i) => v - p[i]) as Vec3)) }))
      .sort((a, c) => a.d - c.d);
    for (let i = 0; i < this.preload && i < free.length; i++) this.robot.preload(this.balls, free[i].b);
  }

  private defaultStart(alliance: Alliance, spec: RobotSpec): { p: Vec3; yawDeg: number } {
    // On the alliance wall (the stations are at +-X), inside its own LOADING ZONE so that
    // "drive out and come back" actually parks, facing into the field.
    const sign = alliance === 'red' ? -1 : 1;
    const x = sign * (this.geom.halfWidth_m - spec.chassis.length_m * 0.75);
    return { p: [x, spec.chassis.height_m / 2 + spec.chassis.clearance_m, 0], yawDeg: sign > 0 ? -90 : 90 };
  }

  private buildStatics(body: RAPIER.RigidBody): void {
    const R = RAPIER;
    const p = this.params;
    const hw = this.geom.halfWidth_m;
    const add = (half: Vec3, pos: Vec3, rotX = 0, friction = 0.6, restitution = 0.3) => {
      this.physics.createCollider(
        R.ColliderDesc.cuboid(half[0], half[1], half[2])
          .setTranslation(pos[0], pos[1], pos[2])
          .setRotation(quatX(rotX))
          .setFriction(friction)
          .setRestitution(restitution)
          .setRestitutionCombineRule(R.CoefficientCombineRule.Min)
          .setCollisionGroups(GROUPS.field),
        body,
      );
    };

    // tiles
    add([hw + 0.2, p.env.tileThick_m, hw + 0.2], [0, -p.env.tileThick_m, 0], 0, p.env.tileMu, p.ball.e_foam);
    // perimeter wall: glass up to railTop
    const wallH = p.env.railTop_m;
    const t = 0.03;
    add([t, wallH / 2, hw + t], [hw + t, wallH / 2, 0], 0, 0.2, p.ball.e_poly);
    add([t, wallH / 2, hw + t], [-hw - t, wallH / 2, 0], 0, 0.2, p.ball.e_poly);
    add([hw + t, wallH / 2, t], [0, wallH / 2, hw + t], 0, 0.2, p.ball.e_poly);
    add([hw + t, wallH / 2, t], [0, wallH / 2, -hw - t], 0, 0.2, p.ball.e_poly);

    for (const f of this.geom.frame) add(f.half, f.pos, f.rotX, 0.4, 0.4);

    // FLOWERs: real tubes, made of a ring of thin boxes, so balls drop in and stack.
    for (const f of this.geom.flowers) {
      const segs = 12;
      const r = f.openingR_m;
      const h = (f.topY_m - inches(0.2)) / 2;
      const w = (Math.PI * r) / segs;
      for (let i = 0; i < segs; i++) {
        const a = (i / segs) * Math.PI * 2;
        this.physics.createCollider(
          R.ColliderDesc.cuboid(w, h, 0.004)
            .setTranslation(f.x_m + Math.cos(a) * (r + 0.004), inches(0.2) + h, f.z_m + Math.sin(a) * (r + 0.004))
            .setRotation(quatY(-a))
            .setFriction(0.3)
            .setRestitution(0.4)
            .setRestitutionCombineRule(R.CoefficientCombineRule.Min)
            .setCollisionGroups(GROUPS.field),
          body,
        );
      }
      add([r, 0.005, r], [f.x_m, inches(0.2), f.z_m], 0, 0.5, 0.2);
    }
  }

  // ------------------------------------------------------------------ step

  setGamepads(g1: GamepadState, g2: GamepadState): void {
    this.gamepads = [g1, g2];
  }

  /** One rendered frame: substepsPerFrame physics steps. */
  step(act: ActuatorFrame): void {
    this.lastActuators = act;
    const dt = this.params.sim.dt;
    for (let i = 0; i < this.params.sim.substepsPerFrame; i++) this.substep(act, dt);
    this.seq++;
  }

  private substep(act: ActuatorFrame, dt: number): void {
    const g = this.params.env.g;

    // Aero FIRST, because applyAero clears each ball's accumulated force before adding
    // drag -- running it after the robot wiped every force the intake and the feed had
    // just applied, and the mechanisms silently did nothing at all.
    this.balls.applyAero(dt);

    const amps = this.robot.preStep(act, dt, this.t, this.battery.volts, this.balls, this.aimPoint());
    this.battery.update(amps, dt);

    // Balls the robot is carrying are inside its shell, nowhere near a CELL, so they
    // cannot contribute tipping torque -- but they are real bodies now, so they have to be
    // filtered out explicitly rather than by being disabled.
    const refs: BallRef[] = this.balls.balls
      // A parked ball is out of play. park() disables the body but leaves it where it was,
      // so without the isEnabled check the six NECTAR the CAD stages inside the CELLs went
      // on contributing tipping torque and showing in the count after being taken out.
      .filter((b) => b.body.isEnabled() && b.state !== 'hopper' && b.state !== 'intake')
      .map((b) => ({ id: b.id, kind: b.kind, mass: b.mass, radius: b.radius, pos: this.balls.pos(b) }));
    this.hives.red.preStep(refs, dt, g);
    this.hives.blue.preStep(refs, dt, g);

    this.physics.step();
    this.t += dt;

    for (const a of ['red', 'blue'] as const) {
      if (this.hives[a].takeTip()) {
        this.scorer.tip(a, this.clock.period, this.t);
        this.nectarEntitlement[a]++;
      }
    }
    this.recordShot();
    this.trackBallStates();
    this.censusUpCells();
    this.retireOutOfBounds();
    this.settleShots();
    this.trackLeave();

    const ended = this.clock.tick(dt);
    if (ended === 'AUTO') this.assessAuto();
    if (ended === 'TELEOP') this.finaliseMatch();
  }

  private trackBallStates(): void {
    for (const b of this.balls.balls) {
      if (b.state === 'hopper' || b.state === 'intake') continue;
      if (!b.body.isEnabled()) continue;   // parked: out of play, and not in any volume
      const p = this.balls.pos(b);
      let state: Ball['state'] = 'free';
      for (const a of ['red', 'blue'] as const) {
        const h = this.hives[a];
        const local = h.toBody(p);
        if (this.geom.cells.some((c) => pointInCell(c, local, b.radius))) {
          state = 'cell';
          break;
        }
      }
      if (state === 'free') {
        for (const f of this.geom.flowers) {
          if (inFlowerScoringVolume(p, f, b.radius) || (p[1] > inches(0.2) && p[1] < f.topY_m && Math.hypot(p[0] - f.x_m, p[2] - f.z_m) < f.openingR_m)) {
            state = 'flower';
            break;
          }
        }
      }
      if (state === 'free' && p[1] > inches(14)) state = 'flight';
      b.state = state;
    }
  }

  /**
   * A ball that clears the wall or drops below the tiles has left the game. Without this it
   * falls forever -- there is no floor outside the perimeter -- which shows up as balls
   * streaking away to infinity and as a snapshot hash that never settles.
   */
  private retireOutOfBounds(): void {
    const lim = this.geom.halfWidth_m + 0.15;
    for (const b of this.balls.balls) {
      if (!b.body.isEnabled() || b.state === 'hopper' || b.state === 'intake') continue;
      const q = this.balls.pos(b);
      if (Math.abs(q[0]) > lim || Math.abs(q[2]) > lim || q[1] < -0.3 || q[1] > 6) {
        this.balls.park(b);
        this.outOfPlay++;
      }
    }
  }

  /** Balls that have left the field this match. Shown in the UI; they do not come back. */
  outOfPlay = 0;

  /** Every shot taken this match, with how it ended. The autonomous training record. */
  readonly shotLog: ShotRecord[] = [];
  /** Each in-flight shot, with the horizontal shot line it was fired along. */
  private pendingShots: {
    rec: ShotRecord; ballId: number; mouth: Vec3; ux: number; uz: number;
    /** Ball height last frame, for spotting the descent through the mouth plane. */
    prevY: number;
    arrived: boolean;
  }[] = [];

  /** Called the frame a ball leaves the muzzle. */
  private recordShot(): void {
    const r = this.robot;
    if (!r.lastShot || this.shotLog.length >= r.shots) return;
    const s = this.sensors();
    const rec: ShotRecord = {
      n: this.shotLog.length + 1,
      t: this.t,
      rangeIn: s.game.upCellRangeIn,
      bearingDeg: s.game.upCellAzimuthDeg,
      hoodDeg: r.hoodAngle,
      rpm: r.lastShotRpm,
      targetRpm: r.lastTargetRpm,
      exitSpeed: r.lastShot.v_exit,
      result: 'flight',
      missBy: NaN,
      long_in: NaN,
      lat_in: NaN,
      aimErrDeg: NaN,
    };
    this.shotLog.push(rec);
    const id = r.lastShotBallId;
    if (id < 0) return;
    // The shot line, frozen now: the robot will have moved by the time the ball lands, and
    // "long" has to mean long along the line it was actually fired down.
    const mouth = this.hives[this.alliance].upCellMouthWorld();
    const dx = mouth[0] - r.pos[0];
    const dz = mouth[2] - r.pos[2];
    const len = Math.hypot(dx, dz) || 1;
    // What the shot was actually pointing at, against what it should have been. Both are
    // world azimuths measured the same way (atan2(x, z)), so the difference is the bearing
    // error the ball left with, scatter included.
    rec.aimErrDeg = (wrapPi((r.lastShot.azDeg * DEG) - Math.atan2(dx, dz)) * 180) / Math.PI;
    this.pendingShots.push({ rec, ballId: id, mouth, ux: dx / len, uz: dz / len, prevY: this.balls.pos(this.balls.balls[id])[1], arrived: false });
  }

  /**
   * Watch each shot down, and write the AIMING error separately from the outcome.
   *
   * The error that matters is where the ball arrived -- where it crossed the mouth's height
   * on the way down -- not where it eventually stopped rolling. A ball that drops an inch
   * wide of the lip and then bounces forty inches across the field was an inch out, and
   * scoring it as forty makes the statistics measure the floor rather than the shooter.
   * `result` still comes from where it settles, because that is the outcome.
   */
  private settleShots(): void {
    for (let i = this.pendingShots.length - 1; i >= 0; i--) {
      const ps = this.pendingShots[i];
      const { rec, ballId, mouth, ux, uz } = ps;
      const b = this.balls.balls[ballId];
      if (!b) { this.pendingShots.splice(i, 1); continue; }
      const q = this.balls.pos(b);

      // 1. arrival: the descending crossing of the mouth plane.
      if (!ps.arrived && ps.prevY >= mouth[1] && q[1] < mouth[1]) {
        const f = (ps.prevY - mouth[1]) / (ps.prevY - q[1] || 1);
        const px = this.lastBallPos[ballId] ?? q;
        const ax = px[0] + (q[0] - px[0]) * f - mouth[0];
        const az = px[2] + (q[2] - px[2]) * f - mouth[2];
        rec.long_in = (ax * ux + az * uz) * M_TO_IN;
        // Left of the shot line: in a Y-up right-handed frame the left normal of (ux, uz)
        // is (uz, -ux).
        rec.lat_in = (ax * uz - az * ux) * M_TO_IN;
        rec.missBy = Math.hypot(ax, az) * M_TO_IN;
        ps.arrived = true;
      }
      ps.prevY = q[1];
      this.lastBallPos[ballId] = [q[0], q[1], q[2]];

      // 2. outcome: once it has stopped moving, or given up on.
      const v = b.body.isEnabled() ? b.body.linvel() : { x: 0, y: 0, z: 0 };
      const slow = Math.hypot(v.x, v.y, v.z) < 0.15;
      if (!slow && this.t - rec.t < 5) continue;
      if (!ps.arrived) {
        // It never reached the mouth's height at all -- it fell short of the goal entirely.
        // Score it where it stopped, which is the honest answer for a shot that short.
        const ex = q[0] - mouth[0];
        const ez = q[2] - mouth[2];
        rec.missBy = Math.hypot(ex, ez) * M_TO_IN;
        rec.long_in = (ex * ux + ez * uz) * M_TO_IN;
        rec.lat_in = (ex * uz - ez * ux) * M_TO_IN;
      }
      // OUR OWN UP CELL, not `b.state === 'cell'`. That flag is set by trackBallStates over
      // EVERY cell of BOTH hives, so it credited a ball that fell into the DOWN cell and a
      // ball that landed in the OPPONENT's hive. It is the right answer to "is this ball in a
      // pocket" and the wrong one to "did this shot score" (PHYSICS 9.15).
      rec.result = this.inOwnUpCell(this.alliance, b) ? 'cell' : 'miss';
      this.pendingShots.splice(i, 1);
    }
  }

  /** Previous-frame position per ball, kept only for shots being watched down. */
  private lastBallPos: Record<number, Vec3> = {};

  private trackLeave(): void {
    if (this.clock.period !== 'AUTO') return;
    const sign = this.alliance === 'red' ? -1 : 1;
    const x = this.robot.pos[0] * sign;
    if (x < this.geom.halfWidth_m - inches(24)) this.leftStart[this.alliance] = true;
  }

  private assessAuto(): void {
    const parked = this.inLoadingZone();
    this.scorer.setAutoResult(this.alliance, this.leftStart[this.alliance], parked);
  }

  /** PARK: the robot's footprint overlaps its own LOADING ZONE. */
  private inLoadingZone(): boolean {
    const p = this.robot.pos;
    const z = this.geom.zones.find((q) => q.name === 'LOADING' && q.alliance === this.alliance);
    if (!z) return false;
    const halfX = this.robot.spec.chassis.width_m / 2;
    const halfZ = this.robot.spec.chassis.length_m / 2;
    return p[0] + halfX > z.min[0] && p[0] - halfX < z.max[0] && p[2] + halfZ > z.min[2] && p[2] - halfZ < z.max[2];
  }

  private finaliseMatch(): void {
    if (this.finalised) return;
    this.finalised = true;
    for (const a of ['red', 'blue'] as const) this.scorer.finalise(a, this.endOfMatchCounts(a));
  }

  endOfMatchCounts(alliance: Alliance): EndOfMatchCounts {
    const h = this.hives[alliance];
    const up = h.upCell.id;
    const upCell = h.ballsInCells[up].length;

    const flowers = this.geom.flowers.map(() => ({ elements: 0, redNectar: 0, blueNectar: 0, topNectar: null as Alliance | null }));
    const bottoms: { flower: number; alliance: Alliance | null; y: number }[] = this.geom.flowers.map((_, i) => ({ flower: i, alliance: null, y: Infinity }));
    // OWNERSHIP IS THE TOP-MOST NECTAR, NOT THE COUNT (manual 10.5.2). Same sweep as the
    // bottom-most one below it, opposite comparison.
    const tops: { alliance: Alliance | null; y: number }[] = this.geom.flowers.map(() => ({ alliance: null, y: -Infinity }));
    let garden = 0;

    for (const b of this.balls.balls) {
      const p = this.balls.pos(b);
      this.geom.flowers.forEach((f, i) => {
        if (!inFlowerScoringVolume(p, f, 0)) return;
        flowers[i].elements++;
        const na = allianceOfNectar(b.kind);
        if (na === 'red') flowers[i].redNectar++;
        if (na === 'blue') flowers[i].blueNectar++;
        if (na && p[1] < bottoms[i].y) {
          bottoms[i].y = p[1];
          bottoms[i].alliance = na;
        }
        if (na && p[1] > tops[i].y) {
          tops[i].y = p[1];
          tops[i].alliance = na;
        }
      });
      for (const z of this.geom.zones) {
        if (z.name !== 'GARDEN' || z.alliance !== alliance) continue;
        if (p[0] > z.min[0] && p[0] < z.max[0] && p[2] > z.min[2] && p[2] < z.max[2] && p[1] < z.max[1]) garden++;
      }
    }

    flowers.forEach((f, i) => { f.topNectar = tops[i].alliance; });

    return {
      upCell,
      garden,
      flowers,
      bottomNectar: bottoms.map((b) => ({ flower: b.flower, alliance: b.alliance })),
      parked: this.inLoadingZone(),
      left: this.leftStart[alliance],
    };
  }

  // ------------------------------------------------------------------ io

  /** World point the robot should aim at: the mouth of its own hive's up CELL. */
  aimPoint(): Vec3 {
    return this.hives[this.alliance].upCellMouthWorld();
  }

  private distanceReadings(): Record<string, number> {
    const out: Record<string, number> = {};
    const p = this.robot.pos;
    for (const s of this.robot.spec.sensors.distance) {
      const mount = Robot.mountToLocal(s.mount_m as Vec3);
      const dirLocal = Robot.mountToLocal(s.dir as Vec3);
      const o = this.robot.toWorld(mount);
      const d = this.robot.toWorld(dirLocal);
      const ray = new RAPIER.Ray({ x: p[0] + o[0], y: p[1] + o[1], z: p[2] + o[2] }, { x: d[0], y: d[1], z: d[2] });
      const hit = this.physics.castRay(ray, s.max_m, true, undefined, undefined, undefined, this.robot.body);
      const m = hit ? hit.timeOfImpact : s.max_m;
      out[s.name] = s.unit === 'in' ? m * M_TO_IN : m * 1000;
    }
    return out;
  }

  /**
   * What the odometry says, which is not what is true.
   *
   * This used to hand the brain ground truth and `sensors.localizer.noise` sat in robot.json
   * at zero, read by nothing -- a perfect oracle for position, heading AND velocity. The
   * velocity is the one that matters: the whole motion lead is built on it, so a lead tested
   * against a perfect estimate has never been tested at all. A real two-pod odometry puck
   * reports a DIFFERENTIATED velocity, and differentiating a quantised encoder is the noisiest
   * thing on the robot.
   *
   * Drawn from the world's own seeded RNG so a seed still reproduces a run exactly.
   */
  /**
   * Angle between the up CELL's outward mouth normal and the direction from the mouth to the
   * robot, in the horizontal plane. 0 is square onto the opening; past 90 the robot is behind
   * the mouth plane and no launch can enter.
   */
  private upCellOpenAngleDeg(): number {
    const hive = this.hives[this.alliance];
    const m = hive.upCellMouthWorld();
    const n = hive.upCellMouthNormalWorld();
    const p = this.robot.pos;
    const dx = p[0] - m[0];
    const dz = p[2] - m[2];
    const dn = Math.hypot(dx, dz) || 1;
    const nn = Math.hypot(n[0], n[2]) || 1;
    const cos = (dx * n[0] + dz * n[2]) / (dn * nn);
    return Math.acos(Math.max(-1, Math.min(1, cos))) * RAD;
  }

  private localizerReading(ftcP: Vec3, ftcV: Vec3, yawDeg: number, yawRate: number): SensorFrame['localizer'] {
    const n = this.robot.spec.sensors.localizer.noise;
    const g = (sigma: number) => (sigma > 0 ? this.rng.gauss(0, sigma) : 0);
    const velSigma = n.vel_mps ?? 0;
    return {
      x: ftcP[0] + g(n.xy_in),
      y: ftcP[1] + g(n.xy_in),
      heading: yawDeg + g(n.heading_deg),
      // Metres per second in, inches per second out: the frame is FTC's.
      vx: ftcV[0] + g(velSigma) * M_TO_IN,
      vy: ftcV[1] + g(velSigma) * M_TO_IN,
      omega: yawRate + g(n.omegaDps ?? 0),
    };
  }

  sensors(): SensorFrame {
    const r = this.robot;
    const p = r.pos;
    const ftcP = worldToFtc(p);
    const v = r.body.linvel();
    const ftcV = worldToFtc([v.x, v.y, v.z]);
    const yawDeg = worldYawToFtcHeadingDeg(r.yaw);
    const yawRate = r.body.angvel().y * RAD;

    // IMU latency: the brain gets a reading from imuLatencyMs ago, like the hub's.
    this.imuBuffer.push({ t: this.t, yaw: yawDeg, rate: yawRate });
    const lag = r.spec.hub.imuLatencyMs / 1000;
    while (this.imuBuffer.length > 1 && this.t - this.imuBuffer[0].t > lag) this.imuBuffer.shift();
    const imu = this.imuBuffer[0];

    const motors: SensorFrame['motors'] = {};
    for (const name of r.motors.keys()) motors[name] = r.motorState(name);
    const servos: SensorFrame['servos'] = {};
    for (const [name, s] of r.servos) servos[name] = { pos: s.pos };

    const aim = this.aimPoint();
    const dx = aim[0] - p[0];
    const dz = aim[2] - p[2];
    const bearing = worldDirToFtcBearingDeg(dx, dz);

    return {
      type: 'sensor',
      seq: this.seq,
      t: this.t,
      match: { period: this.clock.period, remaining: this.clock.remaining, started: this.clock.started, stopped: this.clock.stopped },
      motors,
      servos,
      imu: { yaw: imu.yaw, pitch: 0, roll: 0, yawRate: imu.rate },
      battery: { volts: this.battery.volts + this.rng.gauss(0, r.spec.hub.voltageNoise_V) },
      distance: this.distanceReadings(),
      localizer: this.localizerReading(ftcP, ftcV, yawDeg, yawRate),
      gamepad1: this.gamepads[0],
      gamepad2: this.gamepads[1],
      game: {
        upCellAzimuthDeg: wrapPi((bearing - yawDeg) * DEG) * RAD,
        upCellRangeIn: Math.hypot(dx, dz) * M_TO_IN,
        hiveTipping: this.hives[this.alliance].tipping,
        // IS THE MOUTH OPEN TOWARDS ME? 0 deg is square onto the opening, 90 is edge-on to
        // the mouth plane, 180 is behind the goal looking at the back of the pocket.
        //
        // A TIP SWAPS WHICH FACE IS UP, and the new up CELL opens the other way: measured
        // here, the mouth jumps from Z +15.7 in to -16.1 in the instant the rocker goes over.
        // Nothing in the robot knew that. It stood where it was, kept reporting "clear to
        // fire", and put 40 more balls into the back of the goal -- which is most of why the
        // stopped control case read 0.12 landed per second while tools/landcal.ts, which
        // stops before a tip, measured 85%. On a real robot this is the AprilTag going out
        // of view; here it is ground truth, like the bearing and the range (PHYSICS 9.10).
        upCellOpenDeg: this.upCellOpenAngleDeg(),
        // Bin plus magazine: a ball waiting at the nip is still a ball you can fire.
        hopper: r.heldBalls().length,
        flywheelRpm: r.reportedFlywheelRpm,
      },
    };
  }

  snapshot(): Snapshot {
    const p = this.robot.pos;
    const ftc = worldToFtc(p);
    return {
      t: this.t,
      seq: this.seq,
      period: this.clock.period,
      remaining: this.clock.remaining,
      balls: this.balls.snapshot(),
      hives: [this.hives.red.snapshot(), this.hives.blue.snapshot()],
      robot: this.robot.snapshot(this.battery.volts, this.battery.soc, this.battery.amps, {
        x: ftc[0],
        y: ftc[1],
        heading: worldYawToFtcHeadingDeg(this.robot.yaw),
      }),
      score: this.scorer.state,
      telemetry: this.telemetry,
      shots: this.shotLog,
      outOfPlay: this.outOfPlay,
    };
  }

  /** Cheap but sensitive: enough to catch a determinism break. */
  createSnapshotHash(): string {
    let h = 0x811c9dc5;
    const mix = (x: number) => {
      const q = Math.round(x * 1e5) | 0;
      h ^= q;
      h = Math.imul(h, 0x01000193) >>> 0;
    };
    for (const b of this.balls.balls) {
      const q = this.balls.pos(b);
      mix(q[0]); mix(q[1]); mix(q[2]);
    }
    for (const a of ['red', 'blue'] as const) {
      mix(this.hives[a].angle);
      mix(this.hives[a].omega);
      mix(this.hives[a].tips);
    }
    const rp = this.robot.pos;
    mix(rp[0]); mix(rp[1]); mix(rp[2]); mix(this.robot.yaw);
    mix(this.robot.flywheelOmega); mix(this.robot.turretAngle);
    return (h >>> 0).toString(16).padStart(8, '0');
  }

  reset(): void {
    this.t = 0;
    this.seq = 0;
    this.finalised = false;
    this.imuBuffer.length = 0;
    this.telemetry = [];
    this.leftStart = { red: false, blue: false };
    this.outOfPlay = 0;
    this.shotLog.length = 0;
    this.pendingShots.length = 0;
    this.lastBallPos = {};
    this.nectarEntitlement = { red: 0, blue: 0 };
    this.dumpedAtTips = { red: 0, blue: 0 };
    this.tipsSeen = { red: 0, blue: 0 };
    this.inUpCellPrev = { red: 0, blue: 0 };
    this.balls.reset();
    this.parkOffField();
    this.hives.red.reset();
    this.hives.blue.reset();
    // AFTER the rockers are back on their starting stops: which CELL is up is what
    // decides where the staged NECTAR go, and a reset from a tipped hive staged them
    // into the CELL that was about to swing underneath.
    this.stageCells();
    this.robot.reset();
    this.loadPreload();
    this.battery.reset();
    this.clock.reset();
    this.scorer.reset();
  }

  /** How many NECTAR the human player may currently hand in (G426). */
  nectarAllowed(a: Alliance): number {
    return this.clock.endgame ? 8 : this.nectarEntitlement[a];
  }

  get lastAct(): ActuatorFrame {
    return this.lastActuators;
  }
}

export { clamp, inches };
