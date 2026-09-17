/**
 * The autonomous ROUTINE: what the robot does for itself in the 30 s AUTO period.
 *
 * Not to be confused with `AutoDriver`, which sweeps the field taking measurements. This one
 * plays the game: LEAVE, get into the shooting sector, empty the preload into the up CELL,
 * and be PARKED before the buzzer. Like AutoDriver it produces a `GamepadState` and hands it
 * to the same `BuiltinTeleOp` a human drives through, so every shot goes through the real aim
 * solver, the real readiness gate and the real feed. There is no privileged path: if the auto
 * scores here, the same sequence of stick inputs would score on the robot.
 *
 * NOTHING HERE IS A MAGIC COORDINATE. The shooting spot is derived from where the CELL mouth
 * actually is and which way it opens; the park spot is the LOADING zone out of
 * `buildFieldGeometry`; the range it stands off at is the middle of the shot table's own
 * usable band. Move the hive, change the table or resize the field and the routine follows,
 * which is the whole reason it takes them as arguments instead of knowing them.
 *
 * WHY IT HAS TO DRIVE ROUND FIRST. A robot starts against its own alliance wall, and the up
 * CELL opens toward the audience: measured from the default start pose, the mouth is about
 * 109 deg off its opening, and no launch from there can enter (see `game.upCellOpenDeg`). An
 * auto that stands still and fires scores nothing, however well it aims.
 */
import { emptyGamepad } from '../physics/world.js';
import { worldToFtc } from '../field/ftcFrame.js';
import { clamp, DEG, RAD, M_TO_IN, wrapPi } from '../units.js';
import type { GamepadState, SensorFrame, Vec3 } from '../types.js';

export type AutoPhase = 'leave' | 'position' | 'shoot' | 'park' | 'done';

export interface AutoRoutineField {
  /** Up CELL mouth centre, world metres. */
  mouth: Vec3;
  /** Unit vector out of that mouth, world. The robot has to stand on this side of it. */
  mouthNormal: Vec3;
  /** The robot's own LOADING zone, world metres, as [minX, minZ, maxX, maxZ]. */
  loading: [number, number, number, number];
  /** Half the field, metres. */
  halfWidth_m: number;
  /** The shot table's usable range band, inches. The stand-off is the middle of it. */
  band_in: [number, number];
}

/**
 * Drive the holonomic chassis toward a field point while facing another one, straight onto
 * the sticks. Shared with `AutoDriver` so there is one piece of driving code to be wrong.
 *
 * `x, y` are FTC field inches, which is what the localizer reports.
 */
export function driveTo(
  g: GamepadState,
  s: SensorFrame,
  x: number,
  y: number,
  faceX: number,
  faceY: number,
): number {
  // This function does the field-to-robot rotation itself, so it needs the ROBOT-CENTRIC
  // interpretation of the sticks -- which is now the brain's default, so there is nothing to
  // press. (It used to press Y for it. Y is the speed gear now: holding it here would have
  // wound the gear up every frame and never selected the frame it wanted.)
  const ex = x - s.localizer.x;
  const ey = y - s.localizer.y;
  const dist = Math.hypot(ex, ey);
  // Field-relative error rotated into the robot's own frame. The drivetrain is holonomic,
  // so there is nothing to turn toward -- heading is a separate, independent axis.
  const c = Math.cos(s.imu.yaw * DEG);
  const sn = Math.sin(s.imu.yaw * DEG);
  const fwd = ex * c + ey * sn;
  const left = -ex * sn + ey * c;
  // Taper inside 24 in so it arrives stopped rather than overshooting and hunting.
  const gain = clamp(dist / 24, 0.18, 1) / Math.max(1, dist);
  g.left_stick_y = clamp(-fwd * gain, -1, 1);
  g.left_stick_x = clamp(-left * gain, -1, 1);
  const want = Math.atan2(faceY - s.localizer.y, faceX - s.localizer.x) * RAD;
  const yawErr = wrapPi((want - s.imu.yaw) * DEG) * RAD;
  g.right_stick_x = clamp(-yawErr * 0.02, -0.6, 0.6);
  return dist;
}

export class AutoRoutine {
  phase: AutoPhase = 'leave';
  note = 'starting';
  /** Shots this routine has seen leave the muzzle. */
  fired = 0;

  private since = 0;
  private armed = false;
  private startedShots = 0;
  private readonly shootAt: [number, number];
  private readonly parkAt: [number, number];
  private readonly faceAt: [number, number];

  constructor(private readonly f: AutoRoutineField) {
    // EVERYTHING IN FTC INCHES, because that is the frame the localizer reports and therefore
    // the frame driveTo() steers in. The world frame is Y-up with X along the pivot axis; the
    // FTC frame is Z-up with X toward the audience, so the two are a permutation apart and
    // mixing them silently sends the robot to the mirror image of where it meant to go. It
    // did exactly that: built in world (x, z) and compared against FTC (x, y), the routine
    // drove confidently to a spot 113 deg off the opening and sat there. worldToFtc is the
    // one place that conversion is allowed to happen.
    const m = worldToFtc(f.mouth);
    this.faceAt = [m[0], m[1]];

    // STAND IN FRONT OF THE OPENING. The normal is where the mouth actually points, so a TIP
    // -- which swaps the up CELL and turns the opening round -- moves this spot with it
    // rather than leaving the routine parked behind the goal. A direction takes the same
    // permutation as a point, without the offset.
    const nf = worldToFtc(f.mouthNormal);
    const n = Math.hypot(nf[0], nf[1]) || 1;
    // A quarter into the band, not the middle of it: the table solves 30-150 in but the field
    // is only 141 in across, so the mid-band stand-off is a spot that does not exist and gets
    // clamped into the wall. A quarter in is a real range that is comfortably on the tiles.
    const stand = f.band_in[0] + (f.band_in[1] - f.band_in[0]) * 0.25;
    const lim = f.halfWidth_m * M_TO_IN - 16;   // keep the whole chassis off the glass
    this.shootAt = [
      clamp(m[0] + (nf[0] / n) * stand, -lim, lim),
      clamp(m[1] + (nf[1] / n) * stand, -lim, lim),
    ];

    // Park in the middle of our own LOADING zone, from the geometry rather than a constant.
    const [wx0, wz0, wx1, wz1] = f.loading;
    const a = worldToFtc([wx0, 0, wz0]);
    const b = worldToFtc([wx1, 0, wz1]);
    this.parkAt = [(a[0] + b[0]) / 2, (a[1] + b[1]) / 2];
  }

  /** Where the routine is trying to get to right now, FTC inches. For the UI to draw. */
  get target(): [number, number] {
    return this.phase === 'park' ? this.parkAt : this.shootAt;
  }

  /**
   * One frame. `shotsTaken` is the world's own counter, so the routine knows a ball really
   * left rather than assuming the command worked; `remaining` is the seconds left in AUTO,
   * which is what decides when to stop shooting and go and park.
   */
  update(s: SensorFrame, dt: number, shotsTaken: number, remaining: number): GamepadState {
    const g = emptyGamepad();
    this.since += dt;
    this.fired = shotsTaken - this.startedShots;

    // SPIN THE WHEEL UP WHILE DRIVING. Two motors reach the table's 2300 rpm in about 2.9 s
    // (tools/slew.ts: 794 rpm/s), and AUTO is thirty seconds long -- arriving at the shooting
    // spot and only then starting the wheel spends a tenth of the period standing still.
    // D-pad up is the edge-triggered pre-spin latch in BuiltinTeleOp, pressed once, on the
    // first frame.
    if (!this.armed) { g.dpad_up = true; this.armed = true; }

    // PARKING IS WORTH MORE THAN THE SHOT IT INTERRUPTS. Leave enough to cross the field:
    // the drive does about 1.5 m/s and the far corner is under 4 m away, so five seconds is
    // the honest budget and it is derived, not tuned.
    const parkBudget = 5;
    if (this.phase !== 'done' && this.phase !== 'park' && remaining <= parkBudget) {
      this.phase = 'park';
      this.since = 0;
    }

    switch (this.phase) {
      case 'leave': {
        // LEAVE is scored on being 24 in clear of the alliance wall, so drive at the
        // shooting spot and let the phase end the moment the condition is met -- there is no
        // separate "leave point" to invent.
        const d = driveTo(g, s, this.shootAt[0], this.shootAt[1], this.faceAt[0], this.faceAt[1]);
        this.note = `leaving the wall (${d.toFixed(0)} in to the shooting spot)`;
        // The world scores LEAVE off the robot's own position; all this phase does is get
        // moving. Hand over once it is inside the taper distance of the shooting spot.
        if (d < 24 || this.since > 6) { this.phase = 'position'; this.since = 0; }
        break;
      }

      case 'position': {
        const d = driveTo(g, s, this.shootAt[0], this.shootAt[1], this.faceAt[0], this.faceAt[1]);
        const facing = s.game.upCellOpenDeg;
        this.note = `lining up: ${d.toFixed(0)} in out, ${facing.toFixed(0)} deg off the opening`;
        // BOTH conditions, because either alone is a shot that cannot score: close enough
        // for the table to have an answer, and on the side the mouth actually opens.
        const inBand = s.game.upCellRangeIn > this.f.band_in[0] && s.game.upCellRangeIn < this.f.band_in[1];
        if ((d < 6 && inBand && facing < 60) || this.since > 10) {
          this.phase = 'shoot';
          this.since = 0;
          this.startedShots = shotsTaken;
        }
        break;
      }

      case 'shoot': {
        // Hold station and let the aim work. The fire latch is an edge, so it is pressed on
        // the first frame of the phase and not held down.
        driveTo(g, s, this.shootAt[0], this.shootAt[1], this.faceAt[0], this.faceAt[1]);
        if (this.since < dt * 1.5) g.right_bumper = true;
        this.note = s.game.hopper > 0
          ? `firing (${this.fired} away, ${s.game.hopper} left)`
          : `hopper empty after ${this.fired}`;
        // Out of balls, or the hive went over and the mouth is now facing away: either way
        // there is nothing more to shoot at, so go and bank the PARK.
        if (s.game.hopper === 0 || s.game.upCellOpenDeg > 75) {
          g.right_bumper = true;      // edge again: latch back off
          this.phase = 'park';
          this.since = 0;
        }
        break;
      }

      case 'park': {
        const d = driveTo(g, s, this.parkAt[0], this.parkAt[1], this.faceAt[0], this.faceAt[1]);
        this.note = d < 6 ? `parked, ${this.fired} away` : `parking (${d.toFixed(0)} in)`;
        if (d < 6 && this.since > 1) { this.phase = 'done'; this.note = `done: ${this.fired} away, parked`; }
        break;
      }

      case 'done':
        this.note = `done: ${this.fired} away, parked`;
        break;
    }
    return g;
  }
}
