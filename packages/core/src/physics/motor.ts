/**
 * DC motor model. PLAN.md Appendix C:
 *   tau = tau_stall (1 - w / w_free(V)) (V / 12),   w_free(V) = w_free * V / 12
 *   I    = I_stall * tau / tau_stall
 */
import motorTable from '../../../../config/motors.json' with { type: 'json' };
import { rpmToRadS } from '../units.js';
import type { MotorSpec } from '../types.js';

export interface MotorModel {
  variant: string;
  /** Output-shaft free speed at 12 V, rad/s (after the gearbox and any external ratio). */
  freeOmega: number;
  /** Output-shaft stall torque at 12 V, N.m. */
  stallTorque: number;
  stallAmps: number;
  noLoadAmps: number;
  /** Encoder ticks per revolution of the output shaft, after the external ratio. */
  ticksPerRev: number;
  /** Inertia reflected to the output shaft, kg.m^2. */
  inertia: number;
  reversed: boolean;
}

const BARE = motorTable.bare;
const VARIANTS = motorTable.variants as Record<string, { ratio: number; ticksPerRev: number; freeRpm: number }>;

export function buildMotor(spec: MotorSpec): MotorModel {
  const v = VARIANTS[spec.variant];
  if (!v) throw new Error(`unknown motor variant "${spec.variant}" (see config/motors.json)`);
  const g = spec.gearRatio || 1;
  const eta = motorTable.gearboxEfficiency;
  return {
    variant: spec.variant,
    freeOmega: rpmToRadS(v.freeRpm) / g,
    stallTorque: BARE.stallTorque_Nm * v.ratio * eta * g,
    stallAmps: BARE.stallAmps,
    noLoadAmps: BARE.noLoadAmps,
    ticksPerRev: v.ticksPerRev * g,
    inertia: BARE.rotorInertia_kgm2 * v.ratio * v.ratio * g * g,
    reversed: spec.reversed ?? false,
  };
}

/**
 * Torque at the output shaft for a commanded duty in [-1, 1] at bus voltage V.
 * With duty 0 and BRAKE the terminals are shorted, so back-EMF alone still brakes;
 * with FLOAT the motor coasts. That is the whole of ZeroPowerBehavior.
 */
export function motorTorque(m: MotorModel, duty: number, omega: number, volts: number, brake = true): number {
  const d = duty < -1 ? -1 : duty > 1 ? 1 : duty;
  if (d === 0 && !brake) return 0;
  const vEff = (volts / 12) * d; // signed effective voltage fraction
  // Back-EMF limited: torque falls to zero at the free speed for this effective voltage.
  return m.stallTorque * (vEff - omega / m.freeOmega);
}

/** Current drawn for a given output torque, A (always positive). */
export function motorCurrent(m: MotorModel, torque: number): number {
  return m.noLoadAmps + (Math.abs(torque) / m.stallTorque) * m.stallAmps;
}

export const ticksToRad = (m: MotorModel, ticks: number) => (ticks / m.ticksPerRev) * 2 * Math.PI;
export const radToTicks = (m: MotorModel, rad: number) => (rad / (2 * Math.PI)) * m.ticksPerRev;
