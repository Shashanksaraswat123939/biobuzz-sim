/**
 * The launch angle is the error amplifier, and 45 degrees is where it vanishes.
 *
 *   npm run tool -- tools/hoodangle.ts [--shots 14]
 *
 * Range against launch angle is R ~ v^2 sin(2 theta) / g, so dR/dtheta goes to ZERO at 45
 * degrees -- the maximum-range angle -- and blows up as the shot gets steeper:
 *
 *     50 deg   0.9 cm of range per degree of error, at 1.4 m
 *     70 deg   5.8 cm
 *     80 deg  13.4 cm
 *     85 deg  27.7 cm
 *
 * The shipped table chooses 50-85 deg, because the solver maximises tolerance to SPEED error
 * and a steep lob is very tolerant of that: at 80 degrees a speed error mostly changes height
 * rather than range. It buys that with angle sensitivity, and the launch elevation scatter is
 * 1 degree -- which is where the 13 cm spread floor comes from. Removing the exit-speed
 * scatter entirely does not move it.
 *
 * So: build the table again with the hood held near the insensitive angle and see whether the
 * trade is worth making.
 */
import params from '../config/params.json' with { type: 'json' };
import robotJson from '../config/robot.json' with { type: 'json' };
import { ShotTable } from '../packages/core/src/robot/builtinTeleOp.js';
import { buildTable, toCsv, loadEntry } from './shottable.js';
import { group } from './varcheck.js';
import type { Params, RobotSpec } from '../packages/core/src/types.js';

export async function main(argv: string[] = []): Promise<void> {
  const i = argv.indexOf('--shots');
  const shots = i >= 0 ? Number(argv[i + 1]) : 14;
  const p = params as unknown as Params;
  const entry = loadEntry();

  const bands: [string, [number, number]][] = [
    ['shipped 30-85', [30, 85]],
    ['flat 38-52', [38, 52]],
    ['flat 40-58', [40, 58]],
    ['mid 45-65', [45, 65]],
  ];

  console.log('LAUNCH ANGLE AGAINST EVERYTHING ELSE');
  console.log(`  ${shots} shots x 3 seeds a band, gate open, only the hood range differs`);
  console.log('');
  console.log('  hood range      mean hood   landed of fired   group spread');
  for (const [label, range] of bands) {
    const spec = structuredClone(robotJson) as unknown as RobotSpec;
    spec.hood.angleRange_deg = range;
    const rows = buildTable(p, spec, 30, 150, 4, entry, 'pLand');
    if (!rows.length) { console.log(`  ${label.padEnd(15)} no table`); continue; }
    const table = ShotTable.fromCsv(toCsv(rows));
    let fired = 0;
    let landed = 0;
    const spreads: number[] = [];
    for (const seed of [11, 29, 47]) {
      const pp = structuredClone(params) as unknown as Params;
      const rr = structuredClone(robotJson) as unknown as RobotSpec;
      rr.hood.angleRange_deg = range;
      const g = await group(pp, rr, shots, seed, table);
      fired += g.n;
      landed += g.landed;
      if (Number.isFinite(g.spread_cm)) spreads.push(g.spread_cm);
    }
    const mean = (x: number[]) => (x.length ? x.reduce((u, v) => u + v, 0) / x.length : NaN);
    console.log(
      `  ${label.padEnd(15)} ${mean(rows.map((r) => r.hoodDeg)).toFixed(0).padStart(7)} deg   ` +
      `${String(landed).padStart(6)} of ${String(fired).padStart(3)}   ${mean(spreads).toFixed(1).padStart(10)} cm`,
    );
  }
  console.log('');
  console.log('  A flatter table trades speed tolerance for angle tolerance. Which one wins is');
  console.log('  a question about THIS shooter’s scatter, and only the shooting decides it.');
}
