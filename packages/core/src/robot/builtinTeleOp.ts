/**
 * A driver for the sim's own UI, so the app is usable with a gamepad before the Java
 * runner is attached. It is deliberately a mirror of what `TeleOpMain` does on the hub
 * and it is NOT the deliverable -- `java/teamcode` is. If the two ever disagree, the Java
 * is right. Keeping it here (rather than in the UI) means it is testable headlessly.
 */
import { clamp, DEG, RAD, inches, wrapPi, rpmToRadS } from '../units.js';
import { pThread } from '../physics/ballistics.js';
import type { LandCalibration } from './entryModel.js';
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
 * The ball leaves with v_exit*dir + v_robot, so a robot moving across the shot throws the
 * ball off line by exactly its own cross-track speed. Solving the triangle is both exact
 * and shorter than approximating it: the horizontal velocity the ball must leave with is
 * (wanted speed along the bearing) minus (the robot's own velocity), and the turret points
 * wherever that vector points.
 *
 * Only velocity is compensated, not acceleration: over a one-second flight the a*t^2 term
 * is small next to the 1-2 deg of launch scatter, and a lead that differentiates a noisy
 * velocity is worse than no lead at all.
 */
export function leadShot(
  bearingDeg: number,
  tableSpeed: number,
  elevationDeg: number,
  vxField: number,
  vyField: number,
  headingDeg: number,
): { azimuthDeg: number; speed: number } {
  const horiz = tableSpeed * Math.cos(elevationDeg * DEG);
  if (horiz < 1e-3) return { azimuthDeg: bearingDeg, speed: tableSpeed };

  const bearingField = (headingDeg + bearingDeg) * DEG;
  const wantX = horiz * Math.cos(bearingField) - vxField;
  const wantY = horiz * Math.sin(bearingField) - vyField;
  const mag = Math.hypot(wantX, wantY);
  if (mag < 1e-3) return { azimuthDeg: bearingDeg, speed: tableSpeed };

  return {
    // WRAP. atan2 returns (-180, 180] and the heading is subtracted from it, so the result
    // can land anywhere in (-540, 540). Unwrapped, a bearing of +90 came out as -270 and
    // clamped to the turret's -120 limit -- the robot aimed at its end stop and fired over
    // the wall. Every azimuth crossing the +-180 seam did this.
    azimuthDeg: wrapPi((Math.atan2(wantY, wantX) * RAD - headingDeg) * DEG) * RAD,
    speed: mag / Math.cos(elevationDeg * DEG),
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
  /** How far the motion lead moved the aim, degrees. */
  leadDeg: number;
  note: string;
  /** Match time of the last feed pulse, and whether one is running. Transfer.java's timers. */
  lastFeedT: number;
  pulsing: boolean;
}

export const newTeleOpState = (): TeleOpState => ({
  autoAim: true, firing: false,
  turretManualDeg: 0, flywheelOn: false,
  ready: false, readyCount: 0, targetRpm: 0, turretErrDeg: 0, leadDeg: 0,
  pLand: -1, pLandRaw: -1, calibrated: false, hold: '', note: '',
  lastFeedT: -999, pulsing: false,
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
  ) {}

  /** Recent flywheel readings, for the moving average the gate compares against tolRpm. */
  private readonly rpmHistory: number[] = [];

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
    const lead = leadShot(s.game.upCellAzimuthDeg, tableSpeed, hoodDeg, s.localizer.vx * 0.0254, s.localizer.vy * 0.0254, s.imu.yaw);
    st.leadDeg = wrapPi((lead.azimuthDeg - s.game.upCellAzimuthDeg) * DEG) * RAD;
    // Where the turret ACTUALLY is, from its encoder -- not where it was told to go. The
    // axis is acceleration limited, so a 137 deg swing takes most of a second, and firing
    // on the commanded angle means firing at nothing.
    const turretActualDeg = s.motors.turret ? s.motors.turret.pos / ticksPerDeg : 0;
    let turretDeg: number;
    if (st.autoAim) {
      // Lead-corrected bearing to the up CELL, relative to the robot's heading.
      // The lateral trim also absorbs a turret encoder zero that is a degree out, which
      // looks identical in the data and has the same fix.
      turretDeg = clamp(lead.azimuthDeg - cal.turretTrim_deg, this.spec.turret.range_deg[0], this.spec.turret.range_deg[1]);
    } else {
      // Rate control, in degrees per second: the axis has its own acceleration limit, so
      // this is a request, not a teleport. 100 deg/s covers the full arc in about 2.4 s.
      const nudge = (g.dpad_right ? 1 : 0) - (g.dpad_left ? 1 : 0);
      st.turretManualDeg = clamp(st.turretManualDeg + nudge * 100 * dt, this.spec.turret.range_deg[0], this.spec.turret.range_deg[1]);
      turretDeg = st.turretManualDeg;
    }
    st.turretErrDeg = wrapPi((turretDeg - turretActualDeg) * DEG) * RAD;
    if (this.spec.turret.enabled) {
      motors.turret = { mode: 'RUN_TO_POSITION', target: Math.round(turretDeg * ticksPerDeg), power: 1 };
    }

    // ---- flywheel with a readiness gate (this is what FlywheelGate does on the hub)
    // The lead also changes how fast the ball has to leave: shooting while closing needs
    // less, shooting while retreating needs more.
    const leadRpm = st.autoAim && tableSpeed > 0 ? (row.rpm * lead.speed) / tableSpeed : row.rpm;
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
    const rawP = haveModel && wheelOn
      ? pThread(row.speedLo as number, row.speedHi as number, exitNow, row.sigmaSpeed as number) * (row.pStay as number) * pAim
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
    const probOk = !haveModel || st.pLand >= minP;
    const atSpeed = wheelOn && st.targetRpm > 0 && inWindow && probOk;
    st.readyCount = atSpeed ? st.readyCount + 1 : 0;
    st.ready = st.readyCount >= f.readySteps && Math.abs(st.turretErrDeg) < 3 && !s.game.hiveTipping;

    st.hold = !wheelOn ? ''
      : s.game.hiveTipping ? 'hive is tipping'
      : Math.abs(st.turretErrDeg) >= 3 ? `turret ${st.turretErrDeg.toFixed(0)} deg off`
      : !haveModel ? ''
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
    const mayFire = st.pulsing;
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
        hood: st.autoAim ? row.hoodPos : 0.5,
        gate: mayFire ? this.spec.transfer.gate.open : this.spec.transfer.gate.closed,
      },
      telemetry: [
        ['range in', s.game.upCellRangeIn.toFixed(1)],
        ['target rpm', st.targetRpm.toFixed(0)],
        ['rpm', rpm.toFixed(0)],
        ['ready', String(st.ready)],
        ['P(land)', st.pLand < 0 ? 'no model' : `${(st.pLand * 100).toFixed(0)}%${st.calibrated ? '' : ' (UNCALIBRATED)'}`],
        ['holding', st.hold || '-'],
        ['hopper', String(s.game.hopper)],
        ['margin', `${(row.margin * 100).toFixed(1)}%`],
        ['lead deg', st.leadDeg.toFixed(1)],
        ['turret err', st.turretErrDeg.toFixed(1)],
      ],
    };
  }
}

