/** Unit conversions. SI everywhere inside the core; inches only at the CAD and UI boundaries. */

export const IN_TO_M = 0.0254;
export const M_TO_IN = 1 / IN_TO_M;
export const DEG = Math.PI / 180;
export const RAD = 180 / Math.PI;

export const inches = (v: number) => v * IN_TO_M;
export const toInches = (v: number) => v * M_TO_IN;

export const rpmToRadS = (rpm: number) => (rpm * 2 * Math.PI) / 60;
export const radSToRpm = (w: number) => (w * 60) / (2 * Math.PI);

export const clamp = (v: number, lo: number, hi: number) => (v < lo ? lo : v > hi ? hi : v);

/** Shortest signed difference a-b wrapped to [-pi, pi]. */
export function wrapPi(a: number): number {
  let x = a;
  while (x > Math.PI) x -= 2 * Math.PI;
  while (x < -Math.PI) x += 2 * Math.PI;
  return x;
}

export const lerp = (a: number, b: number, t: number) => a + (b - a) * t;
