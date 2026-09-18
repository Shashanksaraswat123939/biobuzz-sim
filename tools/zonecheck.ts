/**
 * Does the shot zone actually move when the robot does?
 *
 *   npm run tool -- tools/zonecheck.ts
 *
 * The green map is built from per-cell PARAMETERS, not a finished probability, so it can be
 * re-evaluated at any robot velocity. It was only ever evaluated at zero, so the overlay a
 * driver steered by was the standing-still map all match -- which is the wrong shape, because
 * the speed a shot has to LEAVE at depends on what the chassis is doing, and launch scatter
 * is a fraction of exit speed.
 *
 * This counts the cells that clear the gate at several speeds, using the same arithmetic the
 * renderer runs, so the claim "the zone moves" is a number rather than a look.
 */
import zone from '../config/shotzone.json' with { type: 'json' };
import { pThread } from '../packages/core/src/physics/ballistics.js';

interface Cell { x_in: number; z_in: number; k: { lo: number; hi: number; sigma: number; halfLat: number; pStay: number; ux: number; uz: number; dist: number; commanded: number; cosEl: number } | null }

function countAbove(cells: Cell[], vel: [number, number], threshold: number, maxSpeed: number, yawScatterDeg: number) {
  const sigmaYaw = Math.tan((yawScatterDeg * Math.PI) / 180);
  let good = 0;
  let any = 0;
  for (const c of cells) {
    const k = c.k;
    if (!k) continue;
    const horiz = k.commanded * k.cosEl;
    const required = Math.hypot(horiz * k.ux - vel[0], horiz * k.uz - vel[1]) / k.cosEl;
    if (required > maxSpeed) continue;
    const sigma = k.sigma * (required / Math.max(k.commanded, 1e-6));
    const p = pThread(k.lo, k.hi, k.commanded, sigma) * pThread(-k.halfLat, k.halfLat, 0, k.dist * sigmaYaw) * k.pStay;
    if (p > 0) any++;
    if (p >= threshold) good++;
  }
  return { good, any };
}

export async function main(): Promise<void> {
  const z = zone as unknown as { cells: Cell[]; threshold: number; maxSpeed: number; yawScatterDeg: number; step_in: number };
  const area = (n: number) => ((n * z.step_in * z.step_in) / 144).toFixed(1);
  console.log('\nHOW THE GREEN MOVES WITH THE ROBOT. Cells clearing the gate, of ' + z.cells.length + '.\n');
  console.log('   robot velocity (world x,z)     cells green    that is sq ft    any shot at all');
  const cases: [string, [number, number]][] = [
    ['standing still', [0, 0]],
    ['0.5 m/s along +Z', [0, 0.5]],
    ['1.0 m/s along +Z', [0, 1.0]],
    ['1.0 m/s along -Z', [0, -1.0]],
    ['1.0 m/s along +X', [1.0, 0]],
  ];
  for (const [name, v] of cases) {
    const r = countAbove(z.cells, v, z.threshold, z.maxSpeed, z.yawScatterDeg);
    console.log(`   ${name.padEnd(28)} ${String(r.good).padStart(8)}   ${area(r.good).padStart(12)}   ${String(r.any).padStart(12)}`);
  }
  console.log('');
  console.log('   Which sign helps depends where you are standing relative to the mouth. The point is');
  console.log('   that the shape CHANGES, so a map frozen at zero is the wrong map whenever you move.');
  console.log('');
}
