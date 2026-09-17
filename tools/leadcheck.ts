/**
 * Does the motion lead actually put the ball in the hole? Pure ballistics, no sim.
 *
 *   npm run tool -- tools/leadcheck.ts
 *
 * tools/movingfire.ts answers the same question by driving the robot, which means its answer
 * carries launch scatter, the readiness gate, the feeder and the field's edges. Useful, and
 * far too noisy to diagnose with: it was reporting "shooting while accelerating is limited by
 * the flywheel tachometer" for an aim that could not have landed a shot at ANY radial speed
 * from a perfect shooter.
 *
 * This takes the shot table's answer for a range, applies the lead for a given robot velocity,
 * integrates the resulting GROUND-FRAME launch through ballistics.ts, and reports where it
 * crosses the CELL mouth's height on the way down -- which is how world.ts scores arrival.
 * Nothing random, nothing timing-dependent: if a number here is not zero, the aim is wrong.
 *
 * It also prints what a FIXED-HOOD lead does, because that is what this used to do: solve the
 * horizontal triangle, keep the table's hood, and let the vertical fall where it may.
 */
import { readFileSync } from 'node:fs';
import params from '../config/params.json' with { type: 'json' };
import robotSpec from '../config/robot.json' with { type: 'json' };
import { ShotTable, leadShot } from '../packages/core/src/robot/builtinTeleOp.js';
import { simulateShot } from '../packages/core/src/physics/ballistics.js';
import { mouthLips } from './shottable.js';
import { DEG, rpmToRadS } from '../packages/core/src/units.js';
import type { Params, RobotSpec, Vec3 } from '../packages/core/src/types.js';

const table = ShotTable.fromCsv(readFileSync(new URL('../java/teamcode/assets/shottable.csv', import.meta.url), 'utf8'));

/** Horizontal distance at which a ground-frame launch crosses `mouthY` DESCENDING. */
function landRange(p: Params, from: Vec3, g: Vec3, spin: number, mouthY: number): number {
  const sp = Math.hypot(g[0], g[1], g[2]);
  const tr = simulateShot(p, {
    from, azimuth: Math.atan2(g[0], g[2]), elevation: Math.asin(g[1] / sp), speed: sp,
    radius: p.ball.pollen.d_m / 2, mass: p.ball.pollen.m_kg, spin,
  }, 100);
  let prev = tr.points[0];
  for (const q of tr.points) {
    if (prev[1] >= mouthY && q[1] < mouthY) {
      const f = (prev[1] - mouthY) / (prev[1] - q[1] || 1);
      return Math.hypot(prev[0] + (q[0] - prev[0]) * f - from[0], prev[2] + (q[2] - prev[2]) * f - from[2]);
    }
    prev = q;
  }
  return NaN;                                   // never got up there at all
}

export async function main(): Promise<void> {
  const p = structuredClone(params) as unknown as Params;
  const spec = robotSpec as unknown as RobotSpec;
  const f = spec.flywheel;
  const lips = mouthLips(p);
  const mouthY = (lips.near.y + lips.far.y) / 2;
  const from: Vec3 = [0, spec.turret.muzzleHeight_m, 0];
  const spinPerSpeed = 1 / (p.ball.pollen.d_m / 2);
  const [hoodLo, hoodHi] = spec.hood.angleRange_deg;

  console.log('THE MOTION LEAD, with no scatter, no gate and no feeder.');
  console.log(`  Mouth at ${mouthY.toFixed(2)} m, muzzle at ${from[1].toFixed(2)} m. Shot down +Z; +v_r closes.`);
  console.log('');
  console.log('                                 solving the hood too          keeping the table hood');
  console.log('  range   v_r  v_lat       rpm    hood   speed   error        hood   speed   error');
  for (const R of [40, 55, 70]) {
    const row = table.lookup(R);
    const elT = hoodLo + row.hoodPos * (hoodHi - hoodLo);
    const spT = f.k * f.r_fly_m * rpmToRadS(row.rpm);
    const base = landRange(p, from, [0, spT * Math.sin(elT * DEG), spT * Math.cos(elT * DEG)], spinPerSpeed * spT, mouthY);
    const cm = (v: number) => (Number.isFinite(v) ? `${((v - base) * 100).toFixed(0)} cm` : 'NEVER ARRIVES');

    for (const [vr, vl] of [[0, 0], [0.4, 0], [-0.4, 0], [0.8, 0], [-0.8, 0], [0, 0.6], [0, 1.2]] as const) {
      // leadShot is field-frame: with heading 0 and bearing 0 its +X is the shot line, so
      // vxField is the radial component and vyField the lateral one. Ground +Z is radial.
      const L = leadShot(0, spT, elT, vr, vl, 0, spec.hood.angleRange_deg);
      const at = (el: number, speed: number): Vec3 => [
        speed * Math.cos(el * DEG) * Math.sin(L.azimuthDeg * DEG) + vl,
        speed * Math.sin(el * DEG),
        speed * Math.cos(el * DEG) * Math.cos(L.azimuthDeg * DEG) + vr,
      ];
      const got = landRange(p, from, at(L.elevationDeg, L.speed), spinPerSpeed * L.speed, mouthY);
      // The old behaviour, for contrast: same azimuth, table's hood, speed solved so the
      // GROUND TRACK is right. The horizontal comes out exact and the vertical does not.
      const fixedSpeed = Math.hypot(L.speed * Math.cos(L.elevationDeg * DEG), 0) / Math.cos(elT * DEG);
      const old = landRange(p, from, at(elT, fixedSpeed), spinPerSpeed * fixedSpeed, mouthY);
      console.log(
        `  ${String(R).padStart(5)} ${vr.toFixed(1).padStart(5)} ${vl.toFixed(1).padStart(6)}   ` +
        `${(row.rpm * L.speed / spT).toFixed(0).padStart(7)}   ${L.elevationDeg.toFixed(1).padStart(5)} ${L.speed.toFixed(2).padStart(7)}` +
        `  ${cm(got).padStart(14)}      ${elT.toFixed(1).padStart(5)} ${fixedSpeed.toFixed(2).padStart(7)}  ${cm(old).padStart(14)}`,
      );
    }
  }
  console.log('');
  console.log('  Keeping the hood holds the ground track and breaks the hang time, and a ball');
  console.log('  with the wrong hang time does not reach the mouth AT ALL at radial speeds this');
  console.log('  robot can drive at. Solving the elevation as well is exact everywhere.');
}
