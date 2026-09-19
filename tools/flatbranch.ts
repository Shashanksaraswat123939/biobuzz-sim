/**
 * WHAT DOES A FLATTER SHOT COST? The horizontal-speed / landing-rate trade.
 *
 *   npm run tool -- tools/flatbranch.ts
 *
 * The shot table is solved for a robot standing still, and on that test the steep branch
 * always wins. A MOVING robot needs the opposite: the lead can only cancel as much sideways
 * motion as the ball's own horizontal speed S*cos(elevation), and the steep branch throws
 * that away. So the robot refuses shots from directly in front of the mouth -- not because
 * the aperture is shut (it is wide open) but because the arc it was handed cannot be led.
 *
 * This asks the same solver for the best shot at each range under a floor on horizontal
 * speed, and prints what each floor costs in landing probability. The floor a given chassis
 * speed needs is v / sin(leadCap): 1.7 m/s under a 45 deg cap wants 2.40 m/s across.
 */
import params from '../config/params.json' with { type: 'json' };
import robotSpec from '../config/robot.json' with { type: 'json' };
import { buildFieldGeometry } from '../packages/core/src/field/geometry.js';
import { bestShot, pThread, type Aperture } from '../packages/core/src/physics/ballistics.js';
import { loadEntry, mouthLips } from './shottable.js';
import { DEG, inches } from '../packages/core/src/units.js';
import type { Params, RobotSpec, Vec3 } from '../packages/core/src/types.js';

export async function main(): Promise<void> {
  const p = params as unknown as Params;
  const spec = robotSpec as unknown as RobotSpec;
  const entry = loadEntry();
  const lips = mouthLips(p);
  const ballR = p.ball.pollen.d_m / 2;
  const spinPerSpeed = spec.flywheel.type === 'single' ? 1 / ballR : 0;
  void buildFieldGeometry(p);

  const solve = (rIn: number, floor?: number) => {
    const centreZ = (lips.near.z + lips.far.z) / 2 + inches(rIn);
    const muzzle: Vec3 = [lips.x_m, spec.turret.muzzleHeight_m, centreZ - spec.turret.muzzleOffset_m];
    const aperture: Aperture = {
      nearRange: muzzle[2] - lips.near.z,
      nearHeight: lips.near.y + ballR * (p.hive.lipClearanceFrac ?? 1),
      farRange: muzzle[2] - lips.far.z,
      farHeight: lips.far.y - ballR * (p.hive.lipClearanceFrac ?? 1),
    };
    return bestShot(p, {
      muzzle, azimuth: Math.PI, aperture, radius: ballR, mass: p.ball.pollen.m_kg,
      hoodRange: spec.hood.angleRange_deg, hoodSteps: 31, spinPerSpeed,
      k: spec.flywheel.k, rFly: spec.flywheel.r_fly_m, maxRpm: spec.flywheel.maxRpm,
      range_in: rIn, minDescentDeg: 25, maxApex_m: 3.6, maxFlight_s: 2.0,
      objective: 'pLand',
      entryRate: entry ? (v: number, d: number) => entry.lookup(v, d) : undefined,
      scatter: { speedFrac: spec.flywheel.scatter.speedFrac, angle_deg: spec.flywheel.scatter.angle_deg },
      minHoriz_mps: floor,
    });
  };

  const pLand = (r: NonNullable<ReturnType<typeof solve>>) =>
    pThread(r.speedLo, r.speedHi, (r.speedLo + r.speedHi) / 2, r.sigmaSpeed) * (r.pStay ?? 1);

  console.log('\nWHAT A FLATTER SHOT COSTS. Same solver, with a floor on horizontal speed.');
  console.log('A chassis doing v m/s needs v/sin(leadCap) across: 1.7 under a 45 deg cap = 2.40 m/s.\n');
  const floors = [undefined, 2.0, 2.4, 2.8, 3.2];
  console.log('  range        ' + floors.map((f) => (f === undefined ? '  as shipped' : `  floor ${f.toFixed(1)}`)).join(''));
  for (const rIn of [30, 38, 46, 54, 62, 70, 82, 94]) {
    const cells = floors.map((f) => {
      const r = solve(rIn, f);
      if (!r) return '           -';
      const horiz = ((r.speedLo + r.speedHi) / 2) * Math.cos(r.hoodDeg * DEG);
      return `  ${r.hoodDeg.toFixed(0).padStart(2)}° ${horiz.toFixed(2)} ${(pLand(r) * 100).toFixed(0).padStart(3)}%`;
    });
    console.log(`  ${String(rIn).padStart(3)} in   ` + cells.join(''));
  }
  console.log('\n  each cell: hood angle, horizontal m/s, P(lands)   "-" = no arc threads under that floor\n');
}
