/**
 * WHAT IS THE MATHEMATICAL MAXIMUM? The per-shot accuracy ceiling and the time floor.
 *
 *   npm run tool -- tools/ceiling.ts
 *
 * Asked whether 89% at 1.71 s per ball is as good as it gets. It is not, and this says by
 * how much, from the physics rather than from a run.
 *
 * ACCURACY. A shot lands if the exit speed falls inside the band that threads the mouth, AND
 * it is pointed inside the mouth, AND it stays in once it arrives. With a PERFECT robot the
 * second is certain -- zero turret error -- so the ceiling per shot is
 *
 *     P(speed inside the band) x P(stays in)
 *
 * and the first is set by the launch scatter the shooter cannot help: 1.5% of exit speed and
 * 1 deg of elevation, baked into sigmaSpeed per row by tools/shottable.ts.
 *
 * TIME. transfer.cycleTime_s is the mechanism's floor between two balls, so the fastest any
 * gate can score is cycleTime / landRate seconds per ball, and only if the gate never closes.
 */
import { readFileSync } from 'node:fs';
import robotSpec from '../config/robot.json' with { type: 'json' };
import landcal from '../config/landcal.json' with { type: 'json' };
import { ShotTable } from '../packages/core/src/robot/builtinTeleOp.js';
import { pThread } from '../packages/core/src/physics/ballistics.js';
import type { RobotSpec } from '../packages/core/src/types.js';

export async function main(): Promise<void> {
  const spec = robotSpec as unknown as RobotSpec;
  const table = ShotTable.fromCsv(readFileSync(new URL('../java/teamcode/assets/shottable.csv', import.meta.url), 'utf8'));
  const cal = landcal as unknown as { ceiling: number; samples: number };

  console.log('\nTHE PER-SHOT CEILING, from the launch scatter alone. Perfect aim, wheel dead on.\n');
  console.log('  range   P(speed in band)   P(stays in)   P(land)  <- the best this shooter can do');
  let best = 0, bestR = 0;
  for (const r of table.rows) {
    if (r.speedLo === undefined || r.speedHi === undefined || r.sigmaSpeed === undefined) continue;
    const mid = (r.speedLo + r.speedHi) / 2;
    const pSpeed = pThread(r.speedLo, r.speedHi, mid, r.sigmaSpeed);
    const pStay = r.pStay ?? 1;
    const p = pSpeed * pStay;
    if (p > best) { best = p; bestR = r.range_in; }
    if (r.range_in % 12 === 6 || r.range_in === 30) {
      console.log(`  ${r.range_in.toFixed(0).padStart(4)} in   ${(pSpeed * 100).toFixed(1).padStart(14)}%   ${(pStay * 100).toFixed(1).padStart(10)}%   ${(p * 100).toFixed(1).padStart(6)}%`);
    }
  }
  console.log(`\n  BEST RANGE: ${bestR.toFixed(0)} in at ${(best * 100).toFixed(1)}% per shot, with nothing else going wrong.`);
  console.log(`  MEASURED CEILING: ${(cal.ceiling * 100).toFixed(1)}% (config/landcal.json, ${cal.samples} shots on the move).`);

  const cyc = spec.transfer.cycleTime_s;
  console.log(`\nTHE TIME FLOOR. transfer.cycleTime_s is ${cyc} s, so at best ${(1 / cyc).toFixed(2)} shots a second.\n`);
  console.log('  land rate   s per ball, gate never closing');
  for (const L of [1.0, cal.ceiling, 0.9, 0.85, 0.8]) {
    console.log(`  ${(L * 100).toFixed(0).padStart(8)}%   ${(cyc / L).toFixed(2).padStart(6)} s`);
  }
  console.log('\n  And with the gate only open some of the time -- which is the real situation:\n');
  console.log('  gate open   s per ball at 90% landed');
  for (const G of [1.0, 0.8, 0.6, 0.51, 0.4, 0.3]) {
    const s = cyc / (G * 0.9);
    console.log(`  ${(G * 100).toFixed(0).padStart(8)}%   ${s.toFixed(2).padStart(6)} s${s <= 1.3 ? '   <- meets 1.3 s' : ''}`);
  }
  console.log(`\n  So 90% at 1.3 s needs the gate open about ${((cyc / (1.3 * 0.9)) * 100).toFixed(0)}% of the drive.`);
  console.log('  It is currently open 27-56% at 1.57 m/s, and what shuts it is the camera.\n');
}
