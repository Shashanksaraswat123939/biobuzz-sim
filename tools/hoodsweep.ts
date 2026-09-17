/**
 * Which hood range does this robot actually need?
 * For each candidate range, rebuild the shot table and report how much of the field is
 * usable and how the ball arrives. PLAN.md section 10.3 / section 13 phase 6.
 *
 *   npm run tool -- tools/hoodsweep.ts
 */
import params from '../config/params.json' with { type: 'json' };
import robotSpec from '../config/robot.json' with { type: 'json' };
import { buildTable } from './shottable.js';
import type { Params, RobotSpec } from '../packages/core/src/types.js';

export async function main(): Promise<void> {
  const p = params as unknown as Params;
  const candidates: [number, number][] = [
    [30, 60], [35, 65], [40, 70], [45, 75], [50, 80], [55, 85], [30, 75], [30, 85],
  ];
  console.log('hood range   usable  band(in)     best margin   arrival at best');
  for (const range of candidates) {
    const spec = structuredClone(robotSpec) as unknown as RobotSpec;
    spec.hood.angleRange_deg = range;
    const rows = buildTable(p, spec, 30, 150, 5);
    const usable = rows.filter((r) => r.margin >= 0.02);
    const best = rows.reduce((a, b) => (b.margin > a.margin ? b : a), rows[0] ?? { margin: 0, range_in: 0, descentDeg: 0, hoodDeg: 0, rpm: 0, hoodPos: 0 });
    const band = usable.length ? `${usable[0].range_in}-${usable[usable.length - 1].range_in}` : '-';
    const arrive = best.descentDeg >= 0 ? `${best.descentDeg.toFixed(0)} down` : `${(-best.descentDeg).toFixed(0)} up`;
    console.log(
      `${String(range[0]).padStart(3)}-${String(range[1]).padEnd(3)}     ${String(usable.length).padStart(3)}    ${band.padEnd(11)}  ${(best.margin * 100).toFixed(1).padStart(5)}% @ ${String(best.range_in).padStart(3)}in  ${arrive} (hood ${best.hoodDeg.toFixed(0)} deg)`,
    );
  }
}
