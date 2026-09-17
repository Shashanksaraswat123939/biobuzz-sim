/**
 * Is the table's own solution centred in the hole, or hugging one lip?
 *
 *   npm run tool -- tools/apercheck.ts
 *
 * No simulation: this replays each table row through the SOLVER at its own nominal speed and
 * hood angle and measures where that shot sits inside the mouth. If the table is honest, the
 * nominal shot clears the near lip with room and passes under the far lip with room, and its
 * crossing of the mouth's centre height lands near the mouth's centre.
 *
 * It exists because tools/trimsweep.ts found that asking the table for a shot 6 inches
 * SHORTER than the true range takes the land rate from 51% to 84%. A table that is right
 * cannot be improved by lying to it about the range, so one of the two is wrong, and this is
 * the half that costs nothing to check.
 */
import { readFileSync } from 'node:fs';
import params from '../config/params.json' with { type: 'json' };
import robotJson from '../config/robot.json' with { type: 'json' };
import { simulateShot } from '../packages/core/src/physics/ballistics.js';
import { ShotTable } from '../packages/core/src/robot/builtinTeleOp.js';
import { DEG, M_TO_IN, inches } from '../packages/core/src/units.js';
import { cmBare, m as fm } from './_units.js';
import { mouthLips } from './shottable.js';
import type { Params, RobotSpec } from '../packages/core/src/types.js';

export async function main(): Promise<void> {
  const p = params as unknown as Params;
  const spec = robotJson as unknown as RobotSpec;
  const lips = mouthLips(p);
  const ballR = p.ball.pollen.d_m / 2;
  const spinPerSpeed = spec.flywheel.type === 'single' ? 1 / ballR : 0;
  const table = ShotTable.fromCsv(readFileSync(new URL('../java/teamcode/assets/shottable.csv', import.meta.url), 'utf8'));
  const mouthY = (lips.near.y + lips.far.y) / 2;

  console.log('APERTURE CHECK — where the table\'s own shot sits in the mouth');
  console.log(`  near lip ${fm(lips.near.y * M_TO_IN)} up, far lip ${fm(lips.far.y * M_TO_IN)} up, ${fm((lips.near.z - lips.far.z) * M_TO_IN)} apart horizontally`);
  console.log('  clearance is how far ABOVE the near lip and BELOW the far lip the ball passes.');
  console.log('');
  console.log('  range   hood    rpm   near clear   far clear   centre crossing   (cm)');

  let worst = 0;
  for (const row of table.rows) {
    if (row.hoodDeg === undefined || row.speedLo === undefined || row.speedHi === undefined) continue;
    const speed = (row.speedLo + row.speedHi) / 2;
    const centreZ = (lips.near.z + lips.far.z) / 2 + inches(row.range_in);
    const muzzleZ = centreZ - spec.turret.muzzleOffset_m;
    const nearRange = muzzleZ - lips.near.z;
    const farRange = muzzleZ - lips.far.z;
    const centreRange = muzzleZ - (lips.near.z + lips.far.z) / 2;
    const gates: number[] = [];
    for (let g = centreRange - inches(18); g <= centreRange + inches(30); g += inches(0.5)) gates.push(g);
    const t = simulateShot(p, {
      from: [0, spec.turret.muzzleHeight_m, 0],
      azimuth: 0,
      elevation: row.hoodDeg * DEG,
      speed,
      radius: ballR,
      mass: p.ball.pollen.m_kg,
      spin: spinPerSpeed * speed,
    }, centreRange, 1 / 480, [nearRange, farRange, ...gates]);

    const nearClear = (t.gateHeights[0] - (lips.near.y + ballR)) * M_TO_IN;
    const farClear = ((lips.far.y - ballR) - t.gateHeights[1]) * M_TO_IN;
    // Where the arc crosses the mouth's centre height on the way down, as an offset from the
    // mouth's centre -- the same quantity the shot log reports as `long_in`.
    let cross = NaN;
    for (let i = 1; i < gates.length; i++) {
      const a = t.gateHeights[2 + i - 1];
      const b = t.gateHeights[2 + i];
      if (Number.isFinite(a) && Number.isFinite(b) && a >= mouthY && b < mouthY) {
        cross = (gates[i - 1] + (gates[i] - gates[i - 1]) * ((a - mouthY) / (a - b || 1)) - centreRange) * M_TO_IN;
        break;
      }
    }
    if (Number.isFinite(cross)) worst = Math.max(worst, Math.abs(cross));
    console.log(
      `  ${(row.range_in * 0.0254).toFixed(2).padStart(5)}  ${(row.hoodDeg).toFixed(0).padStart(4)}  ${row.rpm.toFixed(0).padStart(5)}  ` +
      `${cmBare(nearClear, 1, 9)}     ${cmBare(farClear, 1, 8)}    ${cmBare(cross, 1, 12)}`,
    );
  }
  console.log('');
  console.log(`  worst centre-crossing offset: ${cmBare(worst)} cm`);
  console.log('');

  // HOW TIGHT DOES THE CONTROL HAVE TO BE? The table being centred is only half the story:
  // the robot fires at whatever rpm and hood angle it actually has, and tools/aimbias.ts
  // measured the wheel running a steady 38 rpm ABOVE target at the moment of the shot. This
  // prices that, and the hood, in inches of range.
  const mid = table.rows[Math.floor(table.rows.length / 2)];
  if (mid.hoodDeg !== undefined && mid.speedLo !== undefined && mid.speedHi !== undefined) {
    const nominal = (mid.speedLo + mid.speedHi) / 2;
    const centreZ = (lips.near.z + lips.far.z) / 2 + inches(mid.range_in);
    const centreRange = centreZ - spec.turret.muzzleOffset_m - (lips.near.z + lips.far.z) / 2;
    const crossAt = (speed: number, hoodDeg: number): number => {
      const gates: number[] = [];
      for (let g = centreRange - inches(18); g <= centreRange + inches(30); g += inches(0.5)) gates.push(g);
      const t = simulateShot(p, {
        from: [0, spec.turret.muzzleHeight_m, 0], azimuth: 0, elevation: hoodDeg * DEG,
        speed, radius: ballR, mass: p.ball.pollen.m_kg, spin: spinPerSpeed * speed,
      }, centreRange, 1 / 480, gates);
      for (let i = 1; i < gates.length; i++) {
        const a = t.gateHeights[i - 1];
        const b = t.gateHeights[i];
        if (Number.isFinite(a) && Number.isFinite(b) && a >= mouthY && b < mouthY) {
          return (gates[i - 1] + (gates[i] - gates[i - 1]) * ((a - mouthY) / (a - b || 1)) - centreRange) * M_TO_IN;
        }
      }
      return NaN;
    };
    const base = crossAt(nominal, mid.hoodDeg);
    const perRpm = nominal / mid.rpm;
    console.log(`  SENSITIVITY at ${fm(mid.range_in)} (hood ${mid.hoodDeg.toFixed(0)} deg, ${mid.rpm.toFixed(0)} rpm), cm of crossing per unit:`);
    for (const d of [10, 38, 120]) {
      console.log(`    +${String(d).padStart(3)} rpm    ${cmBare(crossAt(nominal + d * perRpm, mid.hoodDeg) - base, 1, 6)} cm${d === 38 ? '   <- the measured standing error' : d === 120 ? '   <- the tolRpm window' : ''}`);
    }
    for (const d of [0.5, 1, 2]) {
      console.log(`    +${d.toFixed(1)} deg hood ${cmBare(crossAt(nominal, mid.hoodDeg + d) - base, 1, 6)} cm`);
    }
    console.log(`    the hole is ${cmBare((lips.near.z - lips.far.z) * M_TO_IN)} cm deep, so anything near half of that is a miss on its own.`);
  }
  console.log('  A negative clearance means the nominal shot does not fit through the hole AT ALL,');
  console.log('  which would make the whole table a solution to the wrong problem.');
}
