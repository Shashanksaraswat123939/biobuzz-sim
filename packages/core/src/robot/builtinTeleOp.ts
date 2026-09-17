/**
 * A driver for the sim's own UI, so the app is usable with a gamepad before the Java
 * runner is attached. It is deliberately a mirror of what `TeleOpMain` does on the hub
 * and it is NOT the deliverable -- `java/teamcode` is. If the two ever disagree, the Java
 * is right. Keeping it here (rather than in the UI) means it is testable headlessly.
 */
import { clamp, DEG, RAD, inches, wrapPi, rpmToRadS } from '../units.js';
import { pThread } from '../physics/ballistics.js';
import type { LandCalibration } from './entryModel.js';
import type { HoodCell, HoodTable } from './hoodTable.js';
import type { ActuatorFrame, GamepadState, RobotSpec, SensorFrame } from '../types.js';

export interface ShotRow {
  range_in: number; hoodPos: number; rpm: number; margin: number;
  /** Absolute elevation this row was solved at, degrees. Absent in tables written before it. */
  hoodDeg?: number;
  /** The exit-speed band that threads the mouth, m/s. */
  speedLo?: number;
  speedHi?: number;
  /** One sigma of exit-speed error from all launch scatter, m/s. */
  sigmaSpeed?: number;
  /** Measured fraction of balls arriving like this that stay in (tools/entrycheck.ts). */
  pStay?: number;
  /**
   * Half-width of the CELL mouth ACROSS the shot line, metres, less the ball's radius.
   * Constant for every row -- it is field geometry -- but it rides in the table because the
   * table is the only field artefact the hub reads.
   */
  halfLat_m?: number;
}

export class ShotTable {
  constructor(readonly rows: ShotRow[]) {
    this.rows = [...rows].sort((a, b) => a.range_in - b.range_in);
  }

  static fromCsv(csv: string): ShotTable {
    const rows: ShotRow[] = [];
    for (const line of csv.trim().split(/\r?\n/).slice(1)) {
      const [r, h, rpm, m, deg, sLo, sHi, sig, ps, lat] = line.split(',').map(Number);
      if (Number.isFinite(r)) {
        rows.push({
          range_in: r, hoodPos: h, rpm, margin: m,
          hoodDeg: Number.isFinite(deg) ? deg : undefined,
          speedLo: Number.isFinite(sLo) ? sLo : undefined,
          speedHi: Number.isFinite(sHi) ? sHi : undefined,
          sigmaSpeed: Number.isFinite(sig) ? sig : undefined,
          pStay: Number.isFinite(ps) ? ps : undefined,
          halfLat_m: Number.isFinite(lat) ? lat : undefined,
        });
      }
    }
    return new ShotTable(rows);
  }

  lookup(range_in: number): ShotRow {
    const r = this.rows;
    if (!r.length) return { range_in, hoodPos: 0.5, rpm: 0, margin: 0 };
    if (range_in <= r[0].range_in) return r[0];
    if (range_in >= r[r.length - 1].range_in) return r[r.length - 1];
    for (let i = 1; i < r.length; i++) {
      if (range_in <= r[i].range_in) {
        const t = (range_in - r[i - 1].range_in) / (r[i].range_in - r[i - 1].range_in);
        const mix = (k: keyof ShotRow): number | undefined => {
          const a = r[i - 1][k] as number | undefined;
          const b = r[i][k] as number | undefined;
          return a === undefined || b === undefined ? undefined : a + t * (b - a);
        };
        return {
          range_in,
          hoodPos: r[i - 1].hoodPos + t * (r[i].hoodPos - r[i - 1].hoodPos),
          rpm: r[i - 1].rpm + t * (r[i].rpm - r[i - 1].rpm),
          margin: Math.min(r[i - 1].margin, r[i].margin),
          hoodDeg: mix('hoodDeg'),
          speedLo: mix('speedLo'),
          speedHi: mix('speedHi'),
          sigmaSpeed: mix('sigmaSpeed'),
          // Entry probability is NOT interpolated optimistically: between two rows, take the
          // worse one. A shot halfway between a 90% row and a 40% row is not 65% reliable in
          // any sense the robot can bank on.
          pStay: r[i - 1].pStay === undefined || r[i].pStay === undefined
            ? undefined
            : Math.min(r[i - 1].pStay as number, r[i].pStay as number),
          halfLat_m: mix('halfLat_m'),
        };
      }
    }
    return r[r.length - 1];
  }

  /** The range band with the widest margin: where DriveToRange wants the robot to be. */
  bestBand(): [number, number] {
    if (!this.rows.length) return [60, 90];
    const best = Math.max(...this.rows.map((r) => r.margin));
    const good = this.rows.filter((r) => r.margin >= best * 0.95);
    return [good[0].range_in, good[good.length - 1].range_in];
  }
}

/**
 * Aim that accounts for the robot's own motion.
 *
 * The ball leaves with v_exit*dir + v_robot, so the shot has to be solved for the velocity
 * the BALL must have in the ground frame -- which is the table's answer, as a vector -- and
 * the robot's own velocity subtracted from it.
 *
 *     ground horizontal   S*cos(el) along the bearing        vertical   S*sin(el)
 *
 * THE VERTICAL IS PART OF THE ANSWER, and leaving it out is what broke this. The first
 * version solved the horizontal triangle only and kept the hood where the table put it, so
 * the ball left with a vertical of mag*tan(el) instead of S*sin(el) -- the horizontal ground
 * track was perfect and the hang time was wrong. Closing at 0.4 m/s from 40 in, the exit
 * speed came down from 5.28 to 4.09 m/s at a fixed 70 deg, which drops the vertical from
 * 4.97 to 3.85 and the ball NEVER REACHES the mouth's 1.46 m: not a miss, a shot that cannot
 * arrive. Receding sailed over it the same way. tools/movingfire.ts read that as "shooting
 * while accelerating is limited by the flywheel tachometer".
 *
 * Three components, three unknowns -- azimuth, ELEVATION and speed -- so solve all three:
 *
 *     el = atan2(vert, mag)        speed = hypot(mag, vert)
 *
 * Measured with no scatter and no gate (tools/_lead.ts): exact at every velocity and range,
 * against 14-49 cm long for lateral motion and no arrival at all for radial.
 *
 * It also all but removes the flywheel from the problem, which is the second prize. Over
 * +-0.8 m/s of closing speed at 40 in the old lead swung the target 1283-3388 rpm and the
 * wheel slews 1102 rpm/s; this one asks for 2242-2477, nine times less, and gives the rest
 * to a hood servo that is commanded rather than measured.
 *
 * Only velocity is compensated, not acceleration: over a one-second flight the a*t^2 term
 * is small next to the 1-2 deg of launch scatter, and a lead that differentiates a noisy
 * velocity is worse than no lead at all. (`transfer.leadLatency_s` is a different thing --
 * it predicts the velocity at RELEASE, not during the flight.)
 */
export function leadShot(
  bearingDeg: number,
  tableSpeed: number,
  elevationDeg: number,
  vxField: number,
  vyField: number,
  headingDeg: number,
  /** The hood's travel. The solved elevation is clamped into it. */
  hoodRange: readonly [number, number] = [-90, 90],
): { azimuthDeg: number; speed: number; elevationDeg: number } {
  const horiz = tableSpeed * Math.cos(elevationDeg * DEG);
  const vert = tableSpeed * Math.sin(elevationDeg * DEG);
  const still = { azimuthDeg: bearingDeg, speed: tableSpeed, elevationDeg };
  if (horiz < 1e-3) return still;

  const bearingField = (headingDeg + bearingDeg) * DEG;
  const wantX = horiz * Math.cos(bearingField) - vxField;
  const wantY = horiz * Math.sin(bearingField) - vyField;
  const mag = Math.hypot(wantX, wantY);
  if (mag < 1e-3) return still;

  return {
    // WRAP. atan2 returns (-180, 180] and the heading is subtracted from it, so the result
    // can land anywhere in (-540, 540). Unwrapped, a bearing of +90 came out as -270 and
    // clamped to the turret's -120 limit -- the robot aimed at its end stop and fired over
    // the wall. Every azimuth crossing the +-180 seam did this.
    azimuthDeg: wrapPi((Math.atan2(wantY, wantX) * RAD - headingDeg) * DEG) * RAD,
    // Clamped, and the speed kept as the magnitude of the vector we wanted rather than
    // re-solved for the clamped angle: past the hood's travel no launch matches the table's
    // vector at all, and this degrades smoothly instead of dividing by cos(85 deg). It bites
    // only when charging the goal at most of top speed from close in.
    elevationDeg: clamp(Math.atan2(vert, mag) * RAD, hoodRange[0], hoodRange[1]),
    speed: Math.hypot(mag, vert),
  };
}

export interface TeleOpState {
  autoAim: boolean;
  /** Fire is a latch, not a trigger you hold: the cycle time paces it, not your thumb. */
  firing: boolean;
  /** Manual turret command when auto-aim is off. */
  turretManualDeg: number;
  /**
   * Pre-spin latch. Firing implies it, so a driver never has to arm two things to shoot;
   * it exists on its own only so the wheel can be brought up before committing.
   */
  flywheelOn: boolean;
  ready: boolean;
  readyCount: number;
  /**
   * Probability THIS shot lands, right now: the chance the exit speed falls in the band that
   * threads the mouth, times the measured chance a ball arriving like that stays in.
   * -1 when the table predates the columns needed to compute it.
   */
  pLand: number;
  /** The model's raw score before calibration. Kept so the two can be compared. */
  pLandRaw: number;
  /** True when pLand has been through a measured calibration and is a real probability. */
  calibrated: boolean;
  /** Why the shot is being held, or '' if it is not. */
  hold: string;
  targetRpm: number;
  turretErrDeg: number;
  /** Hood angle minus the angle this shot needs, degrees. The lead moves it every loop. */
  hoodErrDeg: number;
  /** How far the motion lead moved the aim, degrees. */
  leadDeg: number;
  /** The bearing the lead ASKED for, before the turret's travel clamped it. */
  leadAzDeg: number;
  /** How far outside its travel that bearing was. Non-zero means the turret cannot take it. */
  turretPastStopDeg: number;
  note: string;
  /** Match time of the last feed pulse, and whether one is running. Transfer.java's timers. */
  lastFeedT: number;
  pulsing: boolean;
  /** Closing speed on the mouth, m/s. The fixed-speed table's second axis. */
  vRadial: number;
}

export const newTeleOpState = (): TeleOpState => ({
  autoAim: true, firing: false,
  turretManualDeg: 0, flywheelOn: false,
  ready: false, readyCount: 0, targetRpm: 0, turretErrDeg: 0, hoodErrDeg: 0, leadDeg: 0, leadAzDeg: 0, turretPastStopDeg: 0,
  pLand: -1, pLandRaw: -1, calibrated: false, hold: '', note: '',
  lastFeedT: -999, pulsing: false, vRadial: 0,
});

const edge = (now: boolean, was: boolean) => now && !was;

export class BuiltinTeleOp {
  readonly state = newTeleOpState();
  private prev: GamepadState | null = null;

  constructor(
    private readonly spec: RobotSpec,
    private readonly table: ShotTable,
    /**
     * Measured score -> real-frequency mapping. Without it `pLand` is the raw model score,
     * which tools/gatecal.ts showed to be about 20 points optimistic, so the threshold does
     * not mean what it says.
     */
    private readonly landCal: LandCalibration | null = null,
    /**
     * The FIXED-SPEED table. When present the wheel is held at one speed all match and the
     * hood does the aiming, which takes the flywheel out of the control loop entirely -- it
     * never chases a target, so it never lags one, and its tachometer stops gating shots.
     */
    private readonly hoodTable: HoodTable | null = null,
  ) {}

  /** Recent flywheel readings, for the moving average the gate compares against tolRpm. */
  private readonly rpmHistory: number[] = [];
  /** Last velocity sample and the filtered acceleration built from it, for the lead. */
  private lastVel = { x: 0, y: 0, t: 0 };
  private accel = { x: 0, y: 0 };
  /** One-pole filtered localizer velocity. The lead is only ever as good as this. */
  private velFilt = { x: 0, y: 0 };
  /** The fixed-speed solution for this loop, or null when there is no shot from here. */
  private hoodCell: HoodCell | null = null;

  /** Servo position for a hood ANGLE, which is what both tables now deal in. */
  private hoodCommand(leadElevDeg: number): number {
    const [lo, hi] = this.spec.hood.angleRange_deg;
    // The fixed-speed table owns the hood outright when it is loaded; otherwise it is the
    // lead's solved elevation, which equals the shot table's own angle when standing still.
    const deg = this.hoodCell ? this.hoodCell.mid : leadElevDeg;
    return clamp((deg - lo) / Math.max(1e-6, hi - lo), 0, 1);
  }

  /** Hood angle the servo is actually at, from its reported position. */
  private hoodActualDeg(pos: number): number {
    const [lo, hi] = this.spec.hood.angleRange_deg;
    return lo + pos * (hi - lo);
  }

  /**
   * Robot-centric mecanum drive plus the mechanisms, from one gamepad.
   * `dt` is the frame period; the manual turret slews at a rate, not per-call, so the aim
   * does not move faster on a faster machine.
   */
  update(s: SensorFrame, g: GamepadState, seq: number, dt = 1 / 60): ActuatorFrame {
    const st = this.state;
    const p = this.prev;
    if (p) {
      if (edge(g.a, p.a)) st.flywheelOn = !st.flywheelOn;
      if (edge(g.x, p.x)) st.autoAim = !st.autoAim;
      if (edge(g.right_bumper, p.right_bumper)) st.firing = !st.firing;
    }
    this.prev = { ...g };

    // ---- drive
    // Robot-centric, full stop. Pushing the stick forward drives the robot forward, which
    // is what everyone expects the first time they pick it up, and it is what the turret
    // makes possible: the chassis never has to face the goal, so there is nothing a
    // field-centric mode would buy.
    const slow = g.left_bumper ? 0.35 : 1;
    const fwdF = -g.left_stick_y * slow;
    const leftF = -g.left_stick_x * slow;
    const om = -g.right_stick_x * slow;

    const vx = fwdF; // robot forward
    const vy = leftF; // robot left

    // The wire carries what the motor is actually told, so a reversed motor is negated
    // here exactly as the Java does with setDirection(REVERSE). The world then applies the
    // physical reversal; forgetting this half makes the robot sit still with all four
    // wheels fighting each other.
    const denom = Math.max(1, Math.abs(vx) + Math.abs(vy) + Math.abs(om));
    const dm = this.spec.drivetrain.motors;
    const dir = (w: 'fl' | 'fr' | 'bl' | 'br') => (dm[w].reversed ? -1 : 1);
    const motors: ActuatorFrame['motors'] = {
      fl: { mode: 'RUN_WITHOUT_ENCODER', power: (dir('fl') * (vx - vy - om)) / denom },
      fr: { mode: 'RUN_WITHOUT_ENCODER', power: (dir('fr') * (vx + vy + om)) / denom },
      bl: { mode: 'RUN_WITHOUT_ENCODER', power: (dir('bl') * (vx + vy - om)) / denom },
      br: { mode: 'RUN_WITHOUT_ENCODER', power: (dir('br') * (vx - vy + om)) / denom },
    };

    // ---- intake: always running, because a real one is. The left trigger reverses it.
    const intake = g.left_trigger > 0.1 ? -1 : 1;
    motors.intake = { mode: 'RUN_WITHOUT_ENCODER', power: intake };

    // ---- aim
    // The table is looked up at the TRIMMED range. A group that lands 8 in long means the
    // table's answer for R actually reaches R+8, so asking it for R-8 lands on the mouth --
    // one number, measured from a collected run, that corrects every range at once. This is
    // what a team adjusts between matches instead of regenerating the table.
    const cal = this.spec.calibration ?? { rangeTrim_in: 0, turretTrim_deg: 0 };
    const row = this.table.lookup(s.game.upCellRangeIn - cal.rangeTrim_in);
    const ticksPerDeg = this.spec.turret.motor.ticksPerDeg ?? 8;
    const hoodDeg = this.spec.hood.enabled
      ? this.spec.hood.angleRange_deg[0] + row.hoodPos * (this.spec.hood.angleRange_deg[1] - this.spec.hood.angleRange_deg[0])
      : this.spec.hood.fixedAngle_deg;
    const tableSpeed = this.spec.flywheel.k * this.spec.flywheel.r_fly_m * rpmToRadS(row.rpm);
    // LEAD ON THE VELOCITY THE ROBOT WILL HAVE WHEN THE BALL LEAVES, not the one it has now.
    //
    // leadShot's own note says acceleration is not worth compensating because the a*t^2 term
    // over a one-second flight is small next to launch scatter. That is true and it is about
    // the wrong interval: the ball does not care what the robot does after release. What
    // matters is the gap between COMMANDING the shot and the ball LEAVING -- the feed pulse
    // plus the wheel's own lag -- because the exit speed was chosen for the velocity at the
    // start of it. First-order kinematics covers it: v_release = v + a*tau.
    //
    // The measurement that forced this: driving with a wobbling stick, the robot fired MORE
    // than in any other case and landed NOTHING, with the lowest rpm error at fire of the
    // lot. The wheel was exactly on its target; the target was stale.
    // FILTER THE REPORTED VELOCITY BEFORE AIMING ON IT. The whole lead hangs off this number,
    // and on a real robot it is a differentiated encoder rather than the ground truth the
    // simulator used to hand over. With the odometry noise now modelled at all -- 0.04 m/s,
    // which is a degree of bearing against a 2.3 m/s ball -- the raw reading jitters the lead
    // azimuth, the turret chases the jitter, its tracking error never settles under the gate's
    // 3 deg, and the robot stops shooting: three of the shoot tests fired nothing.
    //
    // One pole, because the thing being estimated moves on the timescale of the robot's own
    // acceleration (tenths of a second) and the noise is per loop. alpha = 0.25 at 60 Hz is
    // about 50 ms of lag for roughly a third of the noise, which is the trade a team makes on
    // a real puck.
    const rawVx = s.localizer.vx * 0.0254;
    const rawVy = s.localizer.vy * 0.0254;
    const a = clamp(this.spec.sensors.localizer.velFilterAlpha ?? 1, 0.01, 1);
    this.velFilt.x += a * (rawVx - this.velFilt.x);
    this.velFilt.y += a * (rawVy - this.velFilt.y);
    const velX = this.velFilt.x;
    const velY = this.velFilt.y;
    const tau = this.spec.transfer.leadLatency_s ?? 0;
    let leadVx = velX;
    let leadVy = velY;
    if (tau > 0) {
      const dt = s.t - this.lastVel.t;
      if (dt > 1e-4 && this.lastVel.t > 0) {
        // Low-passed, because this differentiates a velocity estimate. In the sim that
        // estimate is exact; on a robot it is odometry and the filter is what stops a single
        // noisy sample from throwing the aim. Prefer an IMU's own accelerometer if there is
        // one -- it measures acceleration instead of inferring it.
        const k = 0.25;
        this.accel.x += k * ((velX - this.lastVel.x) / dt - this.accel.x);
        this.accel.y += k * ((velY - this.lastVel.y) / dt - this.accel.y);
        // DEADBAND, because this differentiates a velocity and a standing robot still jitters.
        //
        // The correction is only worth making when it exceeds what the wheel can resolve: one
        // encoder count over a 20 ms window is about 107 rpm, and the lead moves the target by
        // roughly 912 rpm per m/s, so a*tau has to be worth more than 0.12 m/s to mean
        // anything. Below that it is noise being fed into the aim -- and it cost shots: a
        // stationary rig that had been firing three times in ten seconds fired twice.
        const dv = Math.hypot(this.accel.x, this.accel.y) * tau;
        if (dv > 0.12) {
          leadVx = velX + this.accel.x * tau;
          leadVy = velY + this.accel.y * tau;
        }
      }
      this.lastVel = { x: velX, y: velY, t: s.t };
    }
    const lead = leadShot(s.game.upCellAzimuthDeg, tableSpeed, hoodDeg, leadVx, leadVy, s.imu.yaw, this.spec.hood.angleRange_deg);

    // ---- FIXED-SPEED PATH: the wheel holds one speed and the hood aims.
    //
    // The robot's radial velocity -- how fast it is closing on the mouth -- is the table's
    // second axis rather than something to cancel. Positive is closing. Lateral motion barely
    // moves the answer, which is why this is the only component that has to be known.
    const bearingField = (s.imu.yaw + s.game.upCellAzimuthDeg) * DEG;
    const vRadial = velX * Math.cos(bearingField) + velY * Math.sin(bearingField);
    st.vRadial = vRadial;
    this.hoodCell = this.hoodTable && !this.hoodTable.isEmpty
      ? this.hoodTable.lookup(s.game.upCellRangeIn - cal.rangeTrim_in, vRadial)
      : null;
    st.leadDeg = wrapPi((lead.azimuthDeg - s.game.upCellAzimuthDeg) * DEG) * RAD;
    st.leadAzDeg = lead.azimuthDeg;
    // Where the turret ACTUALLY is, from its encoder -- not where it was told to go. The
    // axis is acceleration limited, so a 137 deg swing takes most of a second, and firing
    // on the commanded angle means firing at nothing.
    const turretActualDeg = s.motors.turret ? s.motors.turret.pos / ticksPerDeg : 0;
    let turretDeg: number;
    if (st.autoAim) {
      // Lead-corrected bearing to the up CELL, relative to the robot's heading.
      // The lateral trim also absorbs a turret encoder zero that is a degree out, which
      // looks identical in the data and has the same fix.
      const want = lead.azimuthDeg - cal.turretTrim_deg;
      turretDeg = clamp(want, this.spec.turret.range_deg[0], this.spec.turret.range_deg[1]);
      // HOW FAR PAST THE END STOP THE SHOT WANTED TO BE, which is not the same question as
      // how far the axis is from its command and is the one nothing was asking.
      //
      // `turretErrDeg` below is measured against the CLAMPED command, so an axis pinned on its
      // stop reports a fraction of a degree of error and the readiness gate calls it aimed. A
      // robot turning under the turret runs out of travel constantly -- measured over a
      // spinning run, the lead asked for a bearing 17.7 +- 18.5 deg OUTSIDE +-120, the gate
      // said on-target, and the shots went out up to 50 deg wide: lateral miss -51 +- 65 cm
      // against +-11 standing still, and the single largest error term in the whole harness.
      // `TurretTracker.canReach` on the hub has always tested this; the mirror never did.
      st.turretPastStopDeg = Math.abs(want - turretDeg);
    } else {
      // Rate control, in degrees per second: the axis has its own acceleration limit, so
      // this is a request, not a teleport. 100 deg/s covers the full arc in about 2.4 s.
      const nudge = (g.dpad_right ? 1 : 0) - (g.dpad_left ? 1 : 0);
      st.turretManualDeg = clamp(st.turretManualDeg + nudge * 100 * dt, this.spec.turret.range_deg[0], this.spec.turret.range_deg[1]);
      turretDeg = st.turretManualDeg;
      st.turretPastStopDeg = 0;
    }
    st.turretErrDeg = wrapPi((turretDeg - turretActualDeg) * DEG) * RAD;
    if (this.spec.turret.enabled) {
      motors.turret = { mode: 'RUN_TO_POSITION', target: Math.round(turretDeg * ticksPerDeg), power: 1 };
    }

    // ---- flywheel with a readiness gate (this is what FlywheelGate does on the hub)
    // The lead also changes how fast the ball has to leave: shooting while closing needs
    // less, shooting while retreating needs more.
    // ONE SPEED, ALL MATCH, when the fixed-speed table is loaded: the lead's rpm correction is
    // exactly the thing that made the wheel chase a moving target, so there is none.
    const leadRpm = this.hoodTable && !this.hoodTable.isEmpty
      ? this.hoodTable.fixedRpm
      : st.autoAim && tableSpeed > 0 ? (row.rpm * lead.speed) / tableSpeed : row.rpm;
    // Firing implies spinning. Asking a driver to arm the wheel and then arm the feed is
    // two controls where the game only has one decision: shoot or do not.
    const wheelOn = st.flywheelOn || st.firing;
    st.targetRpm = wheelOn ? clamp(leadRpm, 0, this.spec.flywheel.maxRpm) : 0;
    const f = this.spec.flywheel;
    // FILTER THE TACHOMETER. The hub reports whole encoder counts over a 20 ms window and the
    // flywheel is direct-driven on 28 ticks a rev, so a single reading is quantised to about
    // 107 rpm -- worth 15 in of range. Comparing that against a 30 rpm tolerance is comparing
    // against noise. A moving average over a few loops costs lag and buys resolution, and it
    // is the same handful of lines on the hub as it is here.
    const n = Math.max(1, Math.round(this.spec.flywheel.rpmFilterFrames ?? 1));
    this.rpmHistory.push(s.game.flywheelRpm);
    while (this.rpmHistory.length > n) this.rpmHistory.shift();
    const rpm = this.rpmHistory.reduce((a, b) => a + b, 0) / this.rpmHistory.length;

    // FEEDFORWARD PLUS P ON THE FILTERED SPEED, which is what FlywheelGate.java does.
    //
    // This used to hand the target to the hub's own velocity PIDF (RUN_USING_ENCODER). That
    // is a different robot from the deliverable: the hub's loop is fed by the hub's own
    // velocity estimate, quantised to about 107 rpm on this encoder, and no filtering a team
    // writes can get between the two. The Java has always driven the wheel open-loop from a
    // feedforward with a P trim, which lets the SAME filtered reading serve the controller
    // and the readiness gate. Measured 31.5% against 48% for a smoothed loop, and the mirror
    // disagreeing with the deliverable about something this basic is its own bug.
    //
    // Gains are MEASURED (tools/flywheeltune.ts --ff), not derived from the free speed. The
    // generic kS = 0.03, kV = 1/freeRpm asks for 0.497 duty at 2800 rpm where the wheel needs
    // 0.466, which parks it about 190 rpm high and never opens a 60 rpm window.
    const kV = f.kV ?? 1 / 6000;
    const kS = f.kS ?? 0.03;
    const kP = f.kP ?? 0.00025;
    const volts = s.battery.volts < 6 ? 12 : s.battery.volts;
    const ff = (kS + kV * st.targetRpm) * (12 / volts);
    motors.flywheel = wheelOn && st.targetRpm > 0
      ? { mode: 'RUN_WITHOUT_ENCODER', power: clamp(ff + kP * (st.targetRpm - rpm), -1, 1), brake: false }
      : { mode: 'RUN_WITHOUT_ENCODER', power: 0, brake: false };

    // Same guard as the Java FlywheelGate: a live target moves every loop, so readiness
    // must not be keyed to it changing at all.
    // ---- WILL THIS SHOT LAND? A probability, not a tolerance.
    //
    // The old gate asked "is the wheel within tolRpm of target", which is a proxy with two
    // problems. It is the same window at every range, though a 30 in shot tolerates six
    // times the speed error a 130 in one does; and it says nothing at all about whether a
    // ball that threads the mouth then stays in it, which the measurement says varies from
    // 25% to 96% depending only on how the ball arrives.
    //
    // So compute the thing itself:
    //
    //     P(land) = P(exit speed lands in the threading band) x P(stays in | arrival)
    //
    // The first factor uses the speed the wheel is ACTUALLY doing as the mean, so a wheel
    // that is off-target does not fail a threshold test -- it lowers a probability, by an
    // amount that depends on how forgiving this particular shot is. The second is measured
    // by tools/entrycheck.ts and carried in the table.
    const exitNow = f.k * f.r_fly_m * rpmToRadS(rpm);
    // MEASURED IN THE FRAME THE BAND WAS SOLVED IN. speedLo/speedHi thread the mouth from a
    // STANDING robot at the table's own hood angle; the lead moves both the angle and the
    // speed, so the band moves with it. Comparing the raw reading against a fixed band marked
    // every moving shot as unlikely no matter how well aimed it was. Zero correction at rest.
    const exitRel = exitNow - (lead.speed - tableSpeed);
    const haveModel = row.speedLo !== undefined && row.speedHi !== undefined
      && row.sigmaSpeed !== undefined && row.pStay !== undefined;
    // ---- AND WILL IT BE POINTING THE RIGHT WAY?
    //
    // The two factors above are both about the shot's LENGTH. Neither can see left and
    // right, and left and right is where the shots were going: tools/missmix.ts found 33%
    // of shots at 70 in missing WIDE, a failure mode the probability could not represent at
    // all -- so it happily reported 85% for a shot pointing a degree and a half off.
    //
    // The bearing error at the target is range x tan(aim error), and the launch adds a yaw
    // scatter of its own, so the arrival across the shot line is normal about the current
    // pointing error. That makes it the same integral as the speed band, over the mouth's
    // width instead of its depth.
    //
    // It is range-dependent in a way a fixed "within 3 degrees" gate never was: 3 deg is 2
    // in at 40 in and 4 in at 70 in, against a half-width of 8 in.
    const rangeM = inches(s.game.upCellRangeIn);
    const sigmaLat = rangeM * Math.tan(f.scatter.yaw_deg * DEG);
    const meanLat = rangeM * Math.tan(st.turretErrDeg * DEG);
    const pAim = row.halfLat_m === undefined
      ? 1
      : pThread(-row.halfLat_m, row.halfLat_m, meanLat, sigmaLat);
    // FIXED-SPEED MODEL. The uncertainty has moved from the wheel to the hood, so the first
    // factor is a normal integral over HOOD ANGLE instead of exit speed: the band the table
    // measured, against the sigma it measured, centred on where the hood actually IS rather
    // than where it was told to go. The wheel contributes nothing to this term, because it is
    // not moving -- which was the entire point.
    const cell = this.hoodCell;
    const hoodNow = this.hoodActualDeg(s.servos.hood?.pos ?? 0.5);
    const fixedP = cell && wheelOn
      ? pThread(cell.lo, cell.hi, hoodNow, cell.sigmaHood) * cell.pStay * pAim
      : -1;
    const rawP = this.hoodTable && !this.hoodTable.isEmpty
      ? fixedP
      : haveModel && wheelOn
        ? pThread(row.speedLo as number, row.speedHi as number, exitRel, row.sigmaSpeed as number) * (row.pStay as number) * pAim
        : -1;
    st.pLandRaw = rawP;
    // Calibrated if a measurement is available, raw otherwise -- and `calibrated` says which,
    // so a threshold is never quietly compared against the wrong kind of number.
    st.pLand = rawP < 0 ? -1 : this.landCal ? this.landCal.apply(rawP) : rawP;
    st.calibrated = this.landCal !== null;

    const minP = f.minLandProb ?? 0.9;
    // The probability gate is STRICTLY ON TOP of the old RPM window, never instead of it.
    //
    // Replacing the window outright looked cleaner and had a nasty edge: with the threshold
    // set to 0 -- which is what every mechanism test and every calibration run does, because
    // they need an unfiltered sample -- `pLand >= 0` is always true, so the speed check
    // disappeared completely and the robot would fire part-way through spin-up at whatever
    // RPM it happened to be at. Keeping the window as a floor means a threshold of 0 is
    // exactly the old behaviour, and every value above it is a real added restriction.
    const inWindow = Math.abs(rpm - st.targetRpm) < f.tolRpm && rpm > st.targetRpm * f.minRpmFrac;
    // With the fixed-speed table the readiness question changes: the wheel is always at its
    // one speed, so what has to arrive is the HOOD. A servo settles in tens of milliseconds
    // against the flywheel's tenths of a second, and it is a commanded position rather than a
    // measured speed -- there is nothing in the loop to be wrong about.
    //
    // AND IT IS NOW THE QUESTION FOR THE SPEED-SOLVING TABLE TOO. `hoodThere` used to be
    // hard-wired true whenever the fixed-speed table was absent, from when the hood only ever
    // held the table's stationary angle and arrived long before the wheel did. The motion lead
    // solves the hood now -- that is what makes shooting on the move work at all -- so the
    // hood is the axis carrying the correction, it moves every loop, and nothing was waiting
    // for it. Shots went out mid-slew, at an elevation that belonged to a velocity the robot
    // had already left, and the probability model could not see it happen: there is no hood
    // term in the speed-solving product, so a shot taken 10 deg off still scored 85%.
    //
    // A servo has no measurement uncertainty -- it is commanded, not read -- so this is a
    // readiness question rather than another factor in the probability. Once the hood IS at
    // the solved elevation the ball leaves with the table's launch vector exactly, and the
    // speed band the table measured standing still is valid again.
    const usingHood = !!(this.hoodTable && !this.hoodTable.isEmpty);
    st.hoodErrDeg = usingHood ? (cell ? hoodNow - cell.mid : NaN) : hoodNow - lead.elevationDeg;
    const hoodThere = usingHood
      ? (!!cell && hoodNow >= cell.lo && hoodNow <= cell.hi)
      : Math.abs(st.hoodErrDeg) <= (this.spec.hood.tolDeg ?? 2);
    const haveShot = !usingHood || !!cell;
    const probOk = usingHood ? st.pLand >= minP : !haveModel || st.pLand >= minP;
    const atSpeed = wheelOn && st.targetRpm > 0 && inWindow && probOk && hoodThere && haveShot;
    st.readyCount = atSpeed ? st.readyCount + 1 : 0;
    st.ready = st.readyCount >= f.readySteps && Math.abs(st.turretErrDeg) < 3
      && st.turretPastStopDeg < 0.5 && !s.game.hiveTipping;

    st.hold = !wheelOn ? ''
      : s.game.hiveTipping ? 'hive is tipping'
      : st.turretPastStopDeg >= 0.5 ? `turret cannot reach, ${st.turretPastStopDeg.toFixed(0)} deg past its stop`
      : Math.abs(st.turretErrDeg) >= 3 ? `turret ${st.turretErrDeg.toFixed(0)} deg off`
      // No cell is a real answer, not a failure: there is no hood angle that scores from here
      // at this closing speed, and saying so beats holding with an unexplained low number.
      : usingHood && !cell ? 'no shot from here at this speed'
      : !hoodThere ? `hood ${hoodNow.toFixed(0)} deg, want ${(usingHood ? cell?.mid : lead.elevationDeg)?.toFixed(0)}`
      : !haveModel && !usingHood ? ''
      // A threshold ABOVE THE MEASURED CEILING cannot be met by any shot this shooter can
      // take, so the robot sits there forever printing a number that reads like bad luck.
      // Name it, or the honest answer (the shooter cannot do it) looks like a jam.
      : st.pLand < minP ? `P(land) ${(st.pLand * 100).toFixed(0)}% < ${(minP * 100).toFixed(0)}%`
        + (this.landCal && minP > this.landCal.ceiling
          ? ` - UNREACHABLE, best measured ${(this.landCal.ceiling * 100).toFixed(0)}%`
          : '')
      : '';

    // ---- feed
    // The belt runs the whole time the shooter is armed, so the feed tube stays loaded
    // against the gate; the GATE is the release. Gating the belt instead emptied the tube
    // back into the hopper after every shot and cost a second per cycle re-lifting the
    // same ball.
    const wantFire = st.firing || g.b;
    // METER THE BALLS, the way Transfer.java does: one pulse per shot, and no pulse until
    // cycleTime has passed since the last one.
    //
    // This used to hold the gate open for as long as the robot was ready, and relied -- by
    // accident -- on readiness FLICKERING to break the stream into single balls. Once the
    // tachometer was filtered, readiness stopped flickering and the magazine emptied itself
    // through the open gate: four balls in, one shot, three gone. The deliverable never had
    // this bug, which is the point of the mirror agreeing with it.
    const tp = this.spec.transfer;
    const canFeed = s.t - st.lastFeedT >= tp.cycleTime_s && !st.pulsing;
    if (wantFire && st.ready && canFeed) {
      st.pulsing = true;
      st.lastFeedT = s.t;
    }
    if (st.pulsing && s.t - st.lastFeedT >= tp.feedPulse_s) st.pulsing = false;
    // THE GATE STAYS OPEN ONLY WHILE THE SHOT IS STILL GOOD.
    //
    // The decision to feed is taken about four tenths of a second before the ball actually
    // leaves -- the feed pulse plus the climb up the tube -- and it used to be final: the
    // pulse opened the gate for its 0.25 s whatever happened next. Every axis keeps tracking
    // in the meantime, so the AIM at release is current; what is stale is the PERMISSION.
    //
    // It shows up as a small tail of badly wrong shots rather than as a loss of precision.
    // Shuttling fore and aft at 0.5 Hz the typical shot is fine -- median 1 cm off line, IQR
    // [-10, +10] cm downrange, better than standing still -- while 20 of 108 landed more than
    // 60 cm out. So the release re-checks the two things that can go bad inside those four
    // tenths and that are what make a shot WILD rather than merely imprecise: the wheel
    // sagging under its floor, and the turret running out of travel under a turning chassis.
    //
    // NOT the full readiness latch. That was the first attempt and it is far too strict: it
    // needs three consecutive good loops, the tachometer flickers in and out of a 60 rpm
    // window, and requiring an unbroken 0.19 s while the gate servo travels took the stopped
    // case from 78 shots to 3. These three are smooth over the pulse and do not flicker.
    //
    // The turret's own tracking error belongs here for the same reason as the end stop: a
    // chassis spinning under the turret drags it off target during the four tenths, and every
    // wild shot left in the turning case was the same picture -- 134 deg/s of yaw with the
    // axis 10.8 deg behind, approved when it was still on target and fired when it was not.
    //
    // THE WHEEL IS NOT RE-CHECKED HERE, and two attempts to do it both made things worse. A
    // floor -- `rpm > target * minRpmFrac` -- is the wrong shape, because a robot whose range
    // is growing has a target rising ahead of the wheel the whole way, so the floor is
    // permanently unmet: it took the strafing case from 0.28 landed per second to zero, on a
    // run where every shot it used to take went in. Re-checking P(land) instead is the right
    // shape and the wrong input, since it rides the tachometer's 107 rpm quantisation and
    // flickers: stopped fell from 0.36 to 0.12 and the shuttle to zero.
    //
    // It does not need re-checking anyway. The shots that used to go out on a sagging wheel
    // were the bare grip wheel's doing -- one ball took 210 rpm out of it -- and putting a
    // real flywheel behind the wheel cut that to 70 and took the wild shots with it. The two
    // that remain here are geometric, smooth over the pulse, and cannot be fixed by a part.
    const stillGood = st.turretPastStopDeg < 0.5 && Math.abs(st.turretErrDeg) < 3;
    const mayFire = st.pulsing && stillGood;
    motors.transfer = { mode: 'RUN_WITHOUT_ENCODER', power: wheelOn ? 1 : 0 };

    st.note = !wheelOn
      ? 'shooter idle'
      : !atSpeed
        ? `spinning up ${rpm.toFixed(0)}/${st.targetRpm.toFixed(0)}`
        : s.game.hiveTipping
          ? 'hive is tipping - hold'
          : Math.abs(st.turretErrDeg) >= 3
            ? `turret slewing ${st.turretErrDeg.toFixed(0)} deg`
            : s.game.hopper === 0
              ? 'hopper empty'
              : st.firing
                ? 'FIRING'
                : 'ready to fire';

    return {
      seq,
      motors,
      servos: {
        hood: st.autoAim ? this.hoodCommand(lead.elevationDeg) : 0.5,
        gate: mayFire ? this.spec.transfer.gate.open : this.spec.transfer.gate.closed,
      },
      telemetry: [
        // Metres for the reader; the hub's own sensor stays in the FTC frame's inches.
        ['range', `${(s.game.upCellRangeIn * 0.0254).toFixed(2)} m`],
        ['target rpm', st.targetRpm.toFixed(0)],
        ['rpm', rpm.toFixed(0)],
        ['ready', String(st.ready)],
        ['P(land)', st.pLand < 0 ? 'no model' : `${(st.pLand * 100).toFixed(0)}%${st.calibrated ? '' : ' (UNCALIBRATED)'}`],
        ['holding', st.hold || '-'],
        ['hopper', String(s.game.hopper)],
        ['margin', `${(row.margin * 100).toFixed(1)}%`],
        ['lead deg', st.leadDeg.toFixed(1)],
        ['turret err', st.turretErrDeg.toFixed(1)],
        ['past stop', st.turretPastStopDeg.toFixed(1)],
        ['hood err', Number.isFinite(st.hoodErrDeg) ? st.hoodErrDeg.toFixed(1) : '-'],
      ],
    };
  }
}

