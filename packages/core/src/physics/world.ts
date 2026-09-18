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
  /** Put a second, real robot on the other alliance. See `World.opponent`. */
  opponent?: boolean;
}

interface ImuSample { t: number; yaw: number; rate: number }

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

/**
 * Bring any staged POLLEN inside the wall. See `World.parkOffField` for why.
 */
function stageOnField(staging: StagedBall[], halfWidth_m: number, params: Params): StagedBall[] {
  const r = params.ball.pollen.d_m / 2;
  // Only a ball that is really OUTSIDE the wall moves, and it moves just inside it. A blanket
  // clamp at halfWidth - r would also pull the FLOWER staging in: those tubes stand at 69.77
  // in, which is inside the 70.68 in wall but outside that margin, and half an inch of drift
  // is enough to leave a ball sitting on the tube's rim instead of down it.
  const inside = (v: number) => (Math.abs(v) <= halfWidth_m ? v : Math.sign(v) * (halfWidth_m - r * 1.05));
  return staging.map((b) => {
    if (b.kind !== 'pollen') return b;
    const [x, y, z] = b.pos;
    const cx = inside(x);
    const cz = inside(z);
    return cx === x && cz === z ? b : { ...b, pos: [cx, y, cz] as Vec3 };
  });
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
  /**
   * The OPPONENT, when one was asked for: a second real robot on the other alliance, with its
   * own body, its own battery and its own brain driving it through the same gamepad a human
   * would. It is not a ghost and not a scripted animation -- it collides, it takes balls out
   * of play, it tips its own HIVE and it scores against you.
   */
  readonly opponent: Robot | null = null;
  readonly opponentBattery: Battery | null = null;

  t = 0;
  seq = 0;
  telemetry: [string, string][] = [];

  private readonly rng: Rng;
  private readonly params: Params;
  private lastActuators: ActuatorFrame = { seq: 0, motors: {}, servos: {} };
  private gamepads: [GamepadState, GamepadState] = [emptyGamepad(), emptyGamepad()];
  private imuBuffer: ImuSample[] = [];
  private oppImuBuffer: ImuSample[] = [];
  private oppActuators: ActuatorFrame = { seq: 0, motors: {}, servos: {} };
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

    this.balls = new BallSet(RAPIER, this.physics, opts.params, this.rng.fork(11), stageOnField(opts.staging, this.geom.halfWidth_m, opts.params));
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
    }, opts.alliance);
    if (opts.opponent) {
      const other: Alliance = opts.alliance === 'red' ? 'blue' : 'red';
      const os = this.defaultStart(other, opts.robot);
      this.opponent = new Robot(RAPIER, this.physics, opts.params, opts.robot, this.rng.fork(29), {
        p: os.p,
        yaw: os.yawDeg * DEG,
      }, other);
      this.opponentBattery = new Battery(opts.params.battery);
    }
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
  /**
   * ALL FORTY POLLEN ARE IN PLAY (manual 10.3.1), and sixteen of them were not.
   *
   * The GARDEN POLLEN come from the STEP's own centroids, which put them at |x| 73.1 in. The
   * playing surface is 141.35 in across, so its inside face is at 70.68 -- the CAD has them
   * two and a half inches the far side of the wall, resting in the garden trough as the
   * physical part models it. `parkOffField` then did exactly what it is for and benched all
   * sixteen before the match started, so the field ran the whole game on 24 POLLEN instead of
   * 40 and each GARDEN was empty. Nothing reported it; they were simply never there.
   *
   * The fix is at the LOAD, not in the retirement rule: a POLLEN is always in play, so one
   * staged outside is a placement to correct, and the correction is to bring it inside the
   * wall by its own radius -- which lands it in the GARDEN zone the scorer actually uses.
   *
   * NECTAR is left exactly where the CAD puts it. Five per alliance genuinely do start OFF the
   * field, with the human player (10.3.1, G426), and parking them is the right answer.
   */
  private parkOffField(): void {
    // OUT OF BOUNDS IS THE CENTRE PAST THE WALL, not the centre within a radius of it. A ball
    // touching the glass is in play, and the margin cost a whole FLOWER: the two tubes on the
    // +-X walls stand at |x| 69.77 in, so their four staged POLLEN each sat 0.91 in inside the
    // 70.68 in wall -- inside the field by any reading -- and were benched before the match.
    const hw = this.geom.halfWidth_m;
    for (const b of this.balls.balls) {
      const q = this.balls.pos(b);
      if (Math.abs(q[0]) > hw || Math.abs(q[2]) > hw) {
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
  /**
   * G304: each robot starts in contact with exactly 4 preload POLLEN. The staging has them --
   * 40 POLLEN is 16 in the FLOWERs, 16 in the GARDENs and 8 left over, which is 4 for each of
   * the two robots on the field.
   *
   * IT HAS TO SKIP THE STAGED ONES BY POSITION, not by `state`. Nothing has stepped yet when
   * this runs, so `trackBallStates` has never classified anything and every ball still reads
   * `free` -- including the ones standing in a flower tube. The nearest-four rule then emptied
   * the flower closest to the start tile straight into the hopper, and that flower was simply
   * missing for the rest of the match.
   */
  private loadPreload(): void {
    const takers: { r: Robot; n: number }[] = [{ r: this.robot, n: this.preload }];
    if (this.opponent) takers.push({ r: this.opponent, n: this.preload });
    const staged = (b: Ball): boolean => {
      const q = this.balls.pos(b);
      if (this.geom.flowers.some((f) => Math.hypot(q[0] - f.x_m, q[2] - f.z_m) < f.openingR_m + b.radius)) return true;
      return this.geom.zones.some((z) => z.name === 'GARDEN'
        && q[0] > z.min[0] && q[0] < z.max[0] && q[2] > z.min[2] && q[2] < z.max[2] && q[1] < z.max[1]);
    };
    const pool = this.balls.balls.filter((b) => b.kind === 'pollen' && b.state === 'free' && b.body.isEnabled() && !staged(b));
    for (const { r, n } of takers) {
      if (n <= 0) continue;
      const p = r.pos;
      pool.sort((a, c) => Math.hypot(this.balls.pos(a)[0] - p[0], this.balls.pos(a)[2] - p[2])
        - Math.hypot(this.balls.pos(c)[0] - p[0], this.balls.pos(c)[2] - p[2]));
      for (let i = 0; i < n && pool.length; i++) r.preload(this.balls, pool.shift()!);
    }
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

    // FLOWERs: real tubes, made of a ring of thin boxes, so balls drop in and stack --
    // with the RETRIEVAL OPENING actually cut in them.
    //
    // The tube used to be a closed cylinder on a solid disc, which meant a ball that went
    // into a flower stayed there for the rest of the match. That removed the counter-play the
    // whole flower endgame is built on (pull the bottom ball, the stack drops, the top ring
    // reopens) and it silently removed the rule that makes a NECTAR plug worth 5 points: the
    // plug is permanent because a 3.62 in NECTAR does not fit through a 3.55 in opening and a
    // 2.80 in POLLEN does. Cut the opening and both behaviours are just geometry.
    //
    // The tube is therefore built in two height bands: closed all the way round above the
    // opening, and open over the field-facing arc below it.
    for (const f of this.geom.flowers) {
      const segs = 16;
      const r = f.openingR_m;
      const t = 0.004;
      const yLo = inches(0.2);
      const yOpen = f.retrievalTopY_m;
      // Half-angle of the opening, from the chord it has to be wide enough to pass.
      const halfArc = Math.asin(Math.min(1, f.retrievalHalfW_m / (r + t)));
      const faceA = Math.atan2(f.openingDir[1], f.openingDir[0]);
      for (let i = 0; i < segs; i++) {
        const a = (i / segs) * Math.PI * 2;
        // Signed angular distance from the opening's centre, wrapped to +-pi.
        const d = Math.abs(Math.atan2(Math.sin(a - faceA), Math.cos(a - faceA)));
        const bands: [number, number][] = d < halfArc
          ? [[yOpen, f.topY_m]]           // this segment is the doorway: start above it
          : [[yLo, f.topY_m]];
        const w = (Math.PI * r) / segs;
        for (const [y0, y1] of bands) {
          const h = (y1 - y0) / 2;
          if (h <= 0) continue;
          this.physics.createCollider(
            // quatY(th) sends local +Z to (sin th, 0, cos th). The plate's THIN axis is local
            // +Z and has to point radially outward, at (cos a, 0, sin a) -- so th = pi/2 - a.
            // This was quatY(-a), which is that rotation turned through a right angle: every
            // plate stood edge-on to the tube it was supposed to be a wall of, so the "tube"
            // was a pinwheel with 0.8 in gaps between the blades and a ball inside it rattled
            // out through the side. It is why balls in flowers never behaved.
            R.ColliderDesc.cuboid(w, h, t)
              .setTranslation(f.x_m + Math.cos(a) * (r + t), y0 + h, f.z_m + Math.sin(a) * (r + t))
              .setRotation(quatY(Math.PI / 2 - a))
              .setFriction(0.3)
              .setRestitution(0.4)
              .setRestitutionCombineRule(R.CoefficientCombineRule.Min)
              .setCollisionGroups(GROUPS.field),
            body,
          );
        }
      }
      // The bottom ring the stack rests on. Solid: a ball leaves through the doorway, not
      // through the floor.
      add([r, 0.005, r], [f.x_m, yLo, f.z_m], 0, 0.5, 0.2);
    }
  }

  // ------------------------------------------------------------------ step

  setGamepads(g1: GamepadState, g2: GamepadState): void {
    this.gamepads = [g1, g2];
  }

  /** What the opponent's brain decided this frame. Applied in the next step, like ours. */
  setOpponentActuators(a: ActuatorFrame): void {
    this.oppActuators = a;
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

    if (this.opponent && this.opponentBattery) {
      const oa = this.opponent.preStep(
        this.oppActuators, dt, this.t, this.opponentBattery.volts, this.balls, this.aimPointFor(this.opponent.alliance),
      );
      this.opponentBattery.update(oa, dt);
    }

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
      if (!b.body.isEnabled()) continue;    // out of play: with the human player, or retired
      const p = this.balls.pos(b);
      this.geom.flowers.forEach((f, i) => {
        // THE BALL'S OWN RADIUS, not zero. The census treated every ball as a point, so a
        // ball counted only once its CENTRE cleared the middle ring -- and the manual counts
        // it when any part of it is inside. With four POLLEN staged on the bottom ring the
        // second one's centre lands within a third of an inch of that ring, so three of the
        // four flowers scored 2 and one scored 3, from settling noise alone.
        if (!inFlowerScoringVolume(p, f, b.radius)) return;
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

  /**
   * WHAT THE CAMERA SEES OF OUR OWN CELL'S APRILTAG, or null when it does not see it.
   *
   * Every fiducial on this field is bolted to a rocker (am-5888, one on the underside of each
   * CELL, facing out of the mouth) and the rocker moves -- there is no static tag anywhere.
   * What saves it is that the rocker is bistable, so a tag has exactly two poses, and each
   * CELL carries its own ID, so the ID you are reading tells you which. That is the premise
   * `tools/tagstudy.ts` works through and this implements.
   *
   * THE BEARING IT RETURNS HAS NO HEADING TERM IN IT. That is the whole point, and the
   * strongest argument in the study: aiming off a global pose inherits the IMU's yaw drift
   * directly, and 5 deg of drift over a match is half the mouth. A bearing measured to the
   * tag is relative, so there is nothing to drift.
   *
   * The camera is turret-mounted, so its boresight IS the turret's bearing and it is pointing
   * at the tag whenever the shooter is aimed. Visibility is therefore three real conditions
   * and no fudge: inside the horizontal field of view, not so oblique that the panel stops
   * fitting a homography, and big enough in pixels to decode.
   */
  tagSighting(): { bearingDeg: number; rangeIn: number; obliquityDeg: number; px: number } | null {
    const cam = this.robot.spec.sensors.camera;
    if (!cam) return null;
    const hive = this.hives[this.robot.alliance];
    const m = hive.upCellMouthWorld();
    const n = hive.upCellMouthNormalWorld();
    const p = this.robot.pos;

    const dx = m[0] - p[0];
    const dz = m[2] - p[2];
    const range_m = Math.hypot(dx, dz);
    if (range_m < 1e-3) return null;

    // Obliquity: the angle between the tag's own outward normal and the line back to the
    // camera. A planar tag seen at t presents cos(t) of its width.
    const nn = Math.hypot(n[0], n[2]) || 1;
    const cosOb = -((dx * n[0] + dz * n[2]) / (range_m * nn));
    const obliquity = Math.acos(clamp(-cosOb, -1, 1)) * RAD;
    if (obliquity > cam.maxObliquity_deg) return null;

    // Apparent width in pixels, foreshortened by the obliquity.
    const fpx = cam.widthPx / (2 * Math.tan((cam.hfov_deg / 2) * DEG));
    const tag_m = cam.tagSize_in * 0.0254;
    const px = (tag_m * fpx * Math.cos(obliquity * DEG)) / range_m;
    if (px < cam.minTagPx) return null;

    // In the frame at all? The boresight is the turret, so this is the turret's own error.
    const bearingWorld = Math.atan2(dx, dz) * RAD;
    // `yaw` is radians and `turretAngle` is already degrees, which is exactly the kind of
    // mixed-unit pair that reads fine and is wrong by a factor of 57.
    const boresight = this.robot.yaw * RAD + this.robot.turretAngle;
    const off = wrapPi((bearingWorld - boresight) * DEG) * RAD;
    if (Math.abs(off) > cam.hfov_deg / 2) return null;

    // Bearing noise from the pixel model: a corner located to `cornerNoise_px` over a focal
    // length of `fpx` pixels is that many radians of angle, and the tag's own width averages
    // the two side corners down by root two.
    const sigmaBearing = ((cam.cornerNoise_px / Math.SQRT2) / fpx) * RAD;
    // Range comes from apparent SIZE, so its error grows with the square of range.
    const sigmaRange = (range_m * cam.cornerNoise_px) / Math.max(px, 1);
    return {
      bearingDeg: off + this.rng.gauss(0, sigmaBearing),
      rangeIn: (range_m + this.rng.gauss(0, sigmaRange)) * M_TO_IN,
      obliquityDeg: obliquity,
      px,
    };
  }

  /** World point the robot should aim at: the mouth of its own hive's up CELL. */
  aimPoint(): Vec3 {
    return this.aimPointFor(this.alliance);
  }

  /** The same for either alliance, so the opponent aims at ITS hive and not at ours. */
  aimPointFor(a: Alliance): Vec3 {
    return this.hives[a].upCellMouthWorld();
  }

  private distanceReadings(r: Robot): Record<string, number> {
    const out: Record<string, number> = {};
    const p = r.pos;
    for (const s of r.spec.sensors.distance) {
      const mount = Robot.mountToLocal(s.mount_m as Vec3);
      const dirLocal = Robot.mountToLocal(s.dir as Vec3);
      const o = r.toWorld(mount);
      const d = r.toWorld(dirLocal);
      const ray = new RAPIER.Ray({ x: p[0] + o[0], y: p[1] + o[1], z: p[2] + o[2] }, { x: d[0], y: d[1], z: d[2] });
      const hit = this.physics.castRay(ray, s.max_m, true, undefined, undefined, undefined, r.body);
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
  private upCellOpenAngleDeg(r: Robot): number {
    const hive = this.hives[r.alliance];
    const m = hive.upCellMouthWorld();
    const n = hive.upCellMouthNormalWorld();
    const p = r.pos;
    const dx = p[0] - m[0];
    const dz = p[2] - m[2];
    const dn = Math.hypot(dx, dz) || 1;
    const nn = Math.hypot(n[0], n[2]) || 1;
    const cos = (dx * n[0] + dz * n[2]) / (dn * nn);
    return Math.acos(Math.max(-1, Math.min(1, cos))) * RAD;
  }

  private localizerReading(r: Robot, ftcP: Vec3, ftcV: Vec3, yawDeg: number, yawRate: number): SensorFrame['localizer'] {
    const n = r.spec.sensors.localizer.noise;
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
    return this.sensorsFor(this.robot, this.battery, this.imuBuffer, this.gamepads);
  }

  /**
   * What the OPPONENT's brain sees. Same construction, its own robot, its own battery, its own
   * IMU history and its own hive -- so the bot is solving the same problem you are, from the
   * same quality of information, rather than from ground truth.
   */
  opponentSensors(): SensorFrame {
    if (!this.opponent || !this.opponentBattery) throw new Error('no opponent in this world');
    const idle = emptyGamepad();
    return this.sensorsFor(this.opponent, this.opponentBattery, this.oppImuBuffer, [idle, idle]);
  }

  private sensorsFor(
    r: Robot,
    battery: Battery,
    imuBuffer: ImuSample[],
    gamepads: [GamepadState, GamepadState],
  ): SensorFrame {
    const p = r.pos;
    const ftcP = worldToFtc(p);
    const v = r.body.linvel();
    const ftcV = worldToFtc([v.x, v.y, v.z]);
    const yawDeg = worldYawToFtcHeadingDeg(r.yaw);
    const yawRate = r.body.angvel().y * RAD;

    // IMU latency: the brain gets a reading from imuLatencyMs ago, like the hub's.
    imuBuffer.push({ t: this.t, yaw: yawDeg, rate: yawRate });
    const lag = r.spec.hub.imuLatencyMs / 1000;
    while (imuBuffer.length > 1 && this.t - imuBuffer[0].t > lag) imuBuffer.shift();
    const imu = imuBuffer[0];

    const motors: SensorFrame['motors'] = {};
    for (const name of r.motors.keys()) motors[name] = r.motorState(name);
    const servos: SensorFrame['servos'] = {};
    for (const [name, s] of r.servos) servos[name] = { pos: s.pos };

    const aim = this.aimPointFor(r.alliance);
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
      battery: { volts: battery.volts + this.rng.gauss(0, r.spec.hub.voltageNoise_V) },
      distance: this.distanceReadings(r),
      localizer: this.localizerReading(r, ftcP, ftcV, yawDeg, yawRate),
      gamepad1: gamepads[0],
      gamepad2: gamepads[1],
      game: {
        upCellAzimuthDeg: wrapPi((bearing - yawDeg) * DEG) * RAD,
        upCellRangeIn: Math.hypot(dx, dz) * M_TO_IN,
        hiveTipping: this.hives[r.alliance].tipping,
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
        upCellOpenDeg: this.upCellOpenAngleDeg(r),
        // Bin plus magazine: a ball waiting at the nip is still a ball you can fire.
        hopper: r.heldBalls().length,
        flywheelRpm: r.reportedFlywheelRpm,
        // Only the robot whose sensors these are gets a sighting; the opponent's camera is
        // its own problem and it has its own frame.
        tag: r === this.robot ? (() => {
          const t = this.tagSighting();
          return t ? { azimuthDeg: r.turretAngle + t.bearingDeg, rangeIn: t.rangeIn, px: t.px, obliquityDeg: t.obliquityDeg } : null;
        })() : null,
      },
    };
  }

  /**
   * What each alliance would score if the buzzer went now. Cheap enough to call once a frame
   * -- it is the same census `finaliseMatch` uses -- and it is what the scoreboard shows
   * while the clock is running, labelled as a projection.
   */
  projectedScore(): Record<Alliance, number> {
    return {
      red: this.scorer.project('red', this.endOfMatchCounts('red')),
      blue: this.scorer.project('blue', this.endOfMatchCounts('blue')),
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
      opponent: this.opponent
        ? this.opponent.snapshot(
            this.opponentBattery!.volts, this.opponentBattery!.soc, this.opponentBattery!.amps,
            (() => {
              const of = worldToFtc(this.opponent!.pos);
              return { x: of[0], y: of[1], heading: worldYawToFtcHeadingDeg(this.opponent!.yaw) };
            })(),
          )
        : undefined,
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
    this.oppImuBuffer.length = 0;
    this.opponent?.reset();
    this.opponentBattery?.reset();
    this.oppActuators = { seq: 0, motors: {}, servos: {} };
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
