/** Which lip clearance makes the solved band match the one tools/bandcheck.ts measured? */
import params from '../config/params.json' with { type: 'json' };
import robotSpec from '../config/robot.json' with { type: 'json' };
import { buildTable, loadEntry, loadMeasuredStay } from './shottable.js';
import { rpmToRadS } from '../packages/core/src/units.js';
import type { Params, RobotSpec } from '../packages/core/src/types.js';
export async function main(): Promise<void> {
  const spec = robotSpec as unknown as RobotSpec;
  const f = spec.flywheel;
  console.log('\n  measured (bandcheck, scatter off): 40 in -6..+5%, 55 in -6..+5%, 70 in -5..+5%\n');
  console.log('  clearance   40 in band        hood    56 in band        hood    70 in band        hood');
  for (const c of [1.0, 0.5, 0.3, 0.0]) {
    const p = structuredClone(params) as unknown as Params;
    p.hive.lipClearanceFrac = c;
    const rows = buildTable(p, spec, 30, 90, 4, loadEntry(), 'pLand', loadMeasuredStay());
    const cell = (r: number) => {
      const row = rows.find((x) => x.range_in === r)!;
      const nom = f.k * f.r_fly_m * rpmToRadS(row.rpm);
      return `${((row.speedLo / nom - 1) * 100).toFixed(1).padStart(5)}..+${((row.speedHi / nom - 1) * 100).toFixed(1)}%   ${row.hoodDeg.toFixed(0).padStart(3)}`;
    };
    console.log(`  ${c.toFixed(2).padStart(9)}   ${cell(42)}    ${cell(54)}    ${cell(70)}`);
  }
  console.log('');
}
