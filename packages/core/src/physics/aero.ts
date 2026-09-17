/**
 * Drag and Magnus on a ball. PLAN.md Appendix C:
 *   Fd = -1/2 rho Cd A |v| v
 *   S  = |w| r / |v|,  Cl = clSlope * min(S, clSMax)
 *   Fm =  1/2 rho Cl A |v| (w_hat x v)
 *   w_dot = -spinDecay * w
 *
 * These balls are hollow and 26-hole, so Cd and Cl are the least trustworthy numbers in
 * the simulator. Both are config with a _source of "guess - CALIBRATE".
 */
import type { Params, Vec3 } from '../types.js';

export interface AeroResult {
  force: Vec3;
  /** Multiply the spin vector by this to decay it over dt. */
  spinScale: number;
}

const EPS = 1e-6;

export function aeroForce(p: Params['ball'], rho: number, radius: number, v: Vec3, spin: Vec3, dt: number): AeroResult {
  const speed = Math.hypot(v[0], v[1], v[2]);
  if (speed < EPS) return { force: [0, 0, 0], spinScale: Math.exp(-p.spinDecay * dt) };

  const area = Math.PI * radius * radius;
  const q = 0.5 * rho * area * speed;

  // drag, opposing v
  const fd = -q * p.Cd;
  const force: Vec3 = [fd * v[0], fd * v[1], fd * v[2]];

  const spinMag = Math.hypot(spin[0], spin[1], spin[2]);
  if (spinMag > EPS) {
    const S = Math.min((spinMag * radius) / speed, p.clSMax);
    const cl = p.clSlope * S;
    // w_hat x v
    const wx = spin[0] / spinMag, wy = spin[1] / spinMag, wz = spin[2] / spinMag;
    const cx = wy * v[2] - wz * v[1];
    const cy = wz * v[0] - wx * v[2];
    const cz = wx * v[1] - wy * v[0];
    const fm = q * cl;
    force[0] += fm * cx;
    force[1] += fm * cy;
    force[2] += fm * cz;
  }

  return { force, spinScale: Math.exp(-p.spinDecay * dt) };
}
