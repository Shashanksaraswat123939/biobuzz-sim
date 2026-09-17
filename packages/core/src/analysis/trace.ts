/**
 * A flight recorder for the whole machine.
 *
 * The shot log answers "did it go in". This answers "what was every actuator doing at the
 * moment it left" -- which is the question you are really asking when a shot is wrong and
 * the ballistics say it should not have been. It samples the same Snapshot the UI paints
 * from, at a fixed rate, into a ring buffer, and writes out as CSV.
 *
 * Fixed rate, not every frame: at 120 Hz a two-minute match is 14400 rows of almost
 * identical numbers. 20 Hz resolves everything on this robot (the fastest real transient is
 * the flywheel dip after a shot, which takes ~300 ms) and a full match fits in 2400 rows.
 *
 * Ring buffer, not an array: a practice session left running for an hour must not turn into
 * an out-of-memory crash, and the last few minutes are the interesting part anyway.
 */
import { M_TO_IN } from '../units.js';
import type { Snapshot } from '../types.js';

/** One sample. Flat numbers only, so it maps onto a CSV row with no thought. */
export interface TraceRow {
  t: number;
  x_in: number;
  y_in: number;
  heading_deg: number;
  speed_ips: number;
  turret_deg: number;
  turretTarget_deg: number;
  turretRate_dps: number;
  hood_deg: number;
  rpm: number;
  rpmTarget: number;
  flywheelAmps: number;
  hopper: number;
  shots: number;
  volts: number;
  amps: number;
  hiveAngle_deg: number;
  hiveBallTorque_Nm: number;
  hiveGravity_Nm: number;
  ballsInUpCell: number;
  /** Worst wheel slip this frame: the one number that says "it is sliding". */
  maxSlip: number;
}

const COLS: (keyof TraceRow)[] = [
  't', 'x_in', 'y_in', 'heading_deg', 'speed_ips',
  'turret_deg', 'turretTarget_deg', 'turretRate_dps', 'hood_deg',
  'rpm', 'rpmTarget', 'flywheelAmps', 'hopper', 'shots',
  'volts', 'amps',
  'hiveAngle_deg', 'hiveBallTorque_Nm', 'hiveGravity_Nm', 'ballsInUpCell',
  'maxSlip',
];

export class Trace {
  private buf: TraceRow[] = [];
  private head = 0;
  private filled = false;
  private nextT = 0;

  constructor(
    /** Samples per second of SIM time, so a fast-forwarded run records the same rows. */
    readonly hz = 20,
    /** Ring capacity. 24000 rows at 20 Hz is 20 minutes. */
    readonly capacity = 24000,
  ) {}

  get length(): number {
    return this.filled ? this.capacity : this.head;
  }

  /** Offer a snapshot. Ignored unless a sample is due, so callers can call it every frame. */
  sample(s: Snapshot, alliance: 'red' | 'blue'): void {
    if (s.t < this.nextT) return;
    this.nextT = s.t + 1 / this.hz;
    const r = s.robot;
    const h = s.hives.find((x) => x.alliance === alliance);
    const row: TraceRow = {
      t: s.t,
      x_in: r.ftc.x,
      y_in: r.ftc.y,
      heading_deg: r.ftc.heading,
      speed_ips: r.speed * M_TO_IN,
      turret_deg: r.turret.angleDeg,
      turretTarget_deg: r.turret.targetDeg,
      turretRate_dps: r.turret.omegaDps,
      hood_deg: r.hood.angleDeg,
      rpm: r.flywheel.rpm,
      rpmTarget: r.flywheel.targetRpm,
      flywheelAmps: r.flywheel.amps,
      hopper: r.hopper.count,
      shots: r.flywheel.shots,
      volts: r.battery.volts,
      amps: r.battery.amps,
      hiveAngle_deg: h?.angleDeg ?? 0,
      hiveBallTorque_Nm: h?.ballTorque_Nm ?? 0,
      hiveGravity_Nm: h?.gravityTorque_Nm ?? 0,
      ballsInUpCell: h?.ballsInUpCell ?? 0,
      maxSlip: r.wheels.reduce((a, w) => Math.max(a, Math.abs(w.slip)), 0),
    };
    if (this.filled) {
      this.buf[this.head] = row;
    } else {
      this.buf.push(row);
    }
    this.head = (this.head + 1) % this.capacity;
    if (this.head === 0) this.filled = true;
  }

  /** Oldest first. */
  rows(): TraceRow[] {
    return this.filled ? [...this.buf.slice(this.head), ...this.buf.slice(0, this.head)] : this.buf.slice(0, this.head);
  }

  clear(): void {
    this.buf = [];
    this.head = 0;
    this.filled = false;
    this.nextT = 0;
  }

  toCsv(): string {
    const fmt = (v: number) => (Number.isFinite(v) ? (Number.isInteger(v) ? String(v) : v.toFixed(4)) : '');
    return [COLS.join(','), ...this.rows().map((r) => COLS.map((c) => fmt(r[c])).join(','))].join('\n');
  }
}
