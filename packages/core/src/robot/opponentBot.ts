/**
 * THE OPPONENT: a robot on the other alliance that plays the whole match against you.
 *
 * It is not a difficulty setting and there is no cheating dial. It is one competent driver:
 * collect POLLEN, get in front of its own CELL's opening, empty the magazine, go again, and
 * PARK before the buzzer. It drives a real `Robot` body through a real `BuiltinTeleOp` with a
 * `GamepadState`, exactly as the human and the autonomous routine do, so everything it does is
 * something you could do -- and everything that limits you limits it. It spins its flywheel up
 * while driving, it waits for readiness, its shots go through the same gate, and it runs its
 * own battery down.
 *
 * WHAT IT KNOWS THAT A REAL ROBOT WOULD NOT: where the loose balls are. A real one needs
 * vision for that. The alternative is a bot that drives a fixed lap, which is a demo rather
 * than an opponent -- and the balls move, because you are taking them. Everything else (where
 * it is, which way its own mouth opens, whether the wheel is up to speed) comes from the same
 * noisy `SensorFrame` your own brain gets.
 *
 * NOTHING HERE IS A COORDINATE. The shooting spot is derived from the live mouth and its
 * normal every frame, so a TIP -- which turns the opening round -- moves the bot rather than
 * leaving it firing into the back of its own goal.
 */
import { emptyGamepad } from '../physics/world.js';
import { worldToFtc } from '../field/ftcFrame.js';
import { clamp, M_TO_IN } from '../units.js';
import { driveTo, type AutoRoutineField } from './autoRoutine.js';
import type { GamepadState, Period, SensorFrame } from '../types.js';

export type OpponentPhase = 'collect' | 'position' | 'shoot' | 'park' | 'idle';

export interface OpponentSight {
  /** Loose balls it may pick up, FTC inches. G408 filtering belongs to the caller. */
  loose: [number, number][];
  /** Seconds left in the current period, and which period it is. */
  remaining: number;
  period: Period;
  /** Shots the world has seen leave this robot's muzzle. */
  shotsTaken: number;
}

/**
 * NOTE ON `game.truth`. Everything below reads the oracle -- exact bearing, exact range --
 * and that is deliberate. This is a scripted practice opponent, not the deliverable: its job
 * is to be a consistent thing to play against, and giving it a camera of its own would make
 * the player's practice depend on the opponent's luck with a tag. `BuiltinTeleOp`, which IS
 * the deliverable, reads the tag pipeline and never touches this block.
 */
export class OpponentBot {
  phase: OpponentPhase = 'idle';
  note = 'waiting for the match';
  fired = 0;
  /** Where it is heading, FTC inches, for the UI to draw. */
  target: [number, number] = [0, 0];

  private since = 0;
  private armed = false;
  private startedShots = 0;
  private clock = 0;
  /** Closest it has been to the current target, and how long it has failed to beat that. */
  private bestD = Infinity;
  private stuckFor = 0;
  private unstickTill = -1;
  private unstickDir = 1;

  /**
   * driveTo, plus the one thing a straight-line controller cannot do: get off something.
   *
   * `driveTo` steers at a point and has no idea the world contains obstacles, so the first
   * time the chassis caught the red A-frame's corner post it pushed into it for the remaining
   * 138 seconds of the match -- measured, and the reason the bot managed 3.5 shots. Nothing
   * is wrong with the steering; the missing behaviour is the one a human does without
   * thinking, which is to back off and come round the other side.
   *
   * Stuck is defined by PROGRESS, not by contact: if it has not beaten its own best distance
   * to the target for a second and a half, it is stuck, whatever it is touching. The recovery
   * reverses and strafes for a second, alternating sides so a first attempt that picks the
   * wrong way round is not repeated forever.
   */
  private go(g: GamepadState, s: SensorFrame, x: number, y: number, fx: number, fy: number, dt: number): number {
    this.clock += dt;
    const d = driveTo(g, s, x, y, fx, fy);
    if (this.clock < this.unstickTill) {
      g.left_stick_y = 0.7;                     // stick down: reverse
      g.left_stick_x = 0.7 * this.unstickDir;   // and slide off to one side
      g.right_stick_x = 0;
      return d;
    }
    if (d < this.bestD - 2) { this.bestD = d; this.stuckFor = 0; } else { this.stuckFor += dt; }
    if (this.stuckFor > 1.5) {
      this.unstickTill = this.clock + 1.0;
      this.unstickDir = -this.unstickDir;
      this.stuckFor = 0;
      this.bestD = Infinity;
    }
    return d;
  }

  /** A new target means the old best distance means nothing. */
  private retarget(phase: OpponentPhase): void {
    if (this.phase === phase) return;
    this.phase = phase;
    this.since = 0;
    this.bestD = Infinity;
    this.stuckFor = 0;
  }

  /**
   * Stand-off from the mouth: a quarter into the shot table's band, for the same reason the
   * autonomous routine uses it -- the table solves further than the field is wide, so the
   * mid-band spot is not a place that exists.
   */
  private shootSpot(f: AutoRoutineField): [number, number] {
    const m = worldToFtc(f.mouth);
    const nf = worldToFtc(f.mouthNormal);
    const n = Math.hypot(nf[0], nf[1]) || 1;
    const stand = f.band_in[0] + (f.band_in[1] - f.band_in[0]) * 0.25;
    const lim = f.halfWidth_m * M_TO_IN - 16;   // keep the whole chassis off the glass
    return [
      clamp(m[0] + (nf[0] / n) * stand, -lim, lim),
      clamp(m[1] + (nf[1] / n) * stand, -lim, lim),
    ];
  }

  private parkSpot(f: AutoRoutineField): [number, number] {
    const [wx0, wz0, wx1, wz1] = f.loading;
    const a = worldToFtc([wx0, 0, wz0]);
    const b = worldToFtc([wx1, 0, wz1]);
    return [(a[0] + b[0]) / 2, (a[1] + b[1]) / 2];
  }

  update(s: SensorFrame, dt: number, f: AutoRoutineField, see: OpponentSight): GamepadState {
    const g = emptyGamepad();
    this.since += dt;
    this.fired = see.shotsTaken - this.startedShots;

    // Nothing happens before the match does, or after it ends.
    if (see.period === 'STAGING' || see.period === 'FINISHED') {
      this.phase = 'idle';
      this.note = see.period === 'FINISHED' ? `done, ${see.shotsTaken} away` : 'waiting for the match';
      return g;
    }
    if (this.phase === 'idle') { this.retarget('collect'); }

    // Spin up once and leave it up. The wheel costs a couple of amps idling and about three
    // seconds to bring back, and three seconds is a whole cycle.
    if (!this.armed) { g.dpad_up = true; this.armed = true; }

    const mouth = worldToFtc(f.mouth);
    const face: [number, number] = [mouth[0], mouth[1]];
    const shootAt = this.shootSpot(f);
    const parkAt = this.parkSpot(f);

    // PARK IS 5 POINTS AND THE SHOT IT INTERRUPTS IS 2. Budget the crossing from the field's
    // own size at the pace a loaded robot really makes across traffic, rather than a constant.
    const crossing_in = f.halfWidth_m * M_TO_IN * 2;
    const parkBudget = crossing_in / 40;
    if (see.period === 'TELEOP' && see.remaining <= parkBudget && this.phase !== 'park') {
      if (this.phase === 'shoot') g.right_bumper = true;   // edge: latch the fire off
      this.retarget('park');
    }

    switch (this.phase) {
      case 'collect': {
        const full = s.game.hopper >= 4;
        const nearest = see.loose.length ? nearestTo(see.loose, s.localizer.x, s.localizer.y) : null;
        // Full, nothing left to chase, or long enough spent chasing with something aboard:
        // all three mean the next points come from shooting, not from collecting.
        if (full || !nearest || (s.game.hopper > 0 && this.since > 8)) {
          this.retarget('position');
          break;
        }
        this.target = nearest;
        // FACE THE BALL. The intake is on the nose, so this is the one job where the chassis
        // heading is the point.
        const d = this.go(g, s, nearest[0], nearest[1], nearest[0], nearest[1], dt);
        this.note = `collecting (${s.game.hopper} aboard, ${d.toFixed(0)} in to the next)`;
        break;
      }

      case 'position': {
        this.target = shootAt;
        const d = this.go(g, s, shootAt[0], shootAt[1], face[0], face[1], dt);
        const facing = s.game.truth.upCellOpenDeg;
        this.note = `lining up (${d.toFixed(0)} in out, ${facing.toFixed(0)} deg off the opening)`;
        if (s.game.hopper === 0) { this.retarget('collect'); break; }
        // BOTH, because either alone is a shot that cannot score: inside the table's band,
        // and on the side the mouth actually opens.
        const inBand = s.game.truth.upCellRangeIn > f.band_in[0] && s.game.truth.upCellRangeIn < f.band_in[1];
        if ((d < 8 && inBand && facing < 60) || this.since > 12) {
          this.retarget('shoot');
          this.startedShots = see.shotsTaken;
        }
        break;
      }

      case 'shoot': {
        this.target = shootAt;
        this.go(g, s, shootAt[0], shootAt[1], face[0], face[1], dt);
        if (this.since < dt * 1.5) g.right_bumper = true;    // edge: latch the fire on
        this.note = `firing (${this.fired} away, ${s.game.hopper} left)`;
        // Empty, its own hive went over and turned the opening away, or the gate is simply
        // refusing from here: all three mean stop and go round again.
        if (s.game.hopper === 0 || s.game.truth.upCellOpenDeg > 75 || this.since > 15) {
          g.right_bumper = true;      // edge again: latch back off
          this.retarget('collect');
        }
        break;
      }

      case 'park': {
        this.target = parkAt;
        const d = this.go(g, s, parkAt[0], parkAt[1], face[0], face[1], dt);
        this.note = d < 6 ? `parked, ${see.shotsTaken} away` : `parking (${d.toFixed(0)} in)`;
        break;
      }
    }
    return g;
  }
}

function nearestTo(pts: [number, number][], x: number, y: number): [number, number] {
  let best = pts[0];
  let bestD = Infinity;
  for (const p of pts) {
    const d = (p[0] - x) ** 2 + (p[1] - y) ** 2;
    if (d < bestD) { bestD = d; best = p; }
  }
  return best;
}
