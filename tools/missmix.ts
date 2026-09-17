/**
 * The A/B said the flatter table lands no better than the steep one (48.5% vs 47.9%, 0.1
 * standard errors). The entry measurement said it should have been a rout. Both cannot be
 * right about what limits the shooter, and the answer decides where the next hour goes.
 *
 *   npm run tool -- tools/missmix.ts [--shots 12] [--seeds 4]
 *
 * Every shot already records WHERE IT CROSSED THE MOUTH PLANE on the way down -- `long_in`
 * along the shot line, `lat_in` across it -- separately from where it ended up. So the
 * misses split cleanly:
 *
 *   OVER THE HOLE   it arrived inside the opening and still did not stay. That is entry:
 *                   the arrival angle and speed are wrong and it bounced back out.
 *   SHORT / LONG    range error. Exit speed, or the table's RPM for this range.
 *   WIDE            bearing error. Turret aim.
 *
 * If almost nothing is in the first bucket, the entry model is optimising a term that is
 * not the binding constraint, and the flat table buying nothing is exactly what you would
 * expect rather than a puzzle.
 *
 * NOTE on "over the hole": the mouth is a slot tilted about 49 deg, and this tests the
 * ball's position against the slot's footprint at the instant it crosses the mouth's centre
 * height. That is the right test for being above the opening; it is not a claim that a ball
 * inside the footprint would necessarily have fitted through the tilted slot.
 */
import params from '../config/params.json' with { type: 'json' };
import robotJson from '../config/robot.json' with { type: 'json' };
import { buildFieldGeometry } from '../packages/core/src/field/geometry.js';
import { M_TO_IN } from '../packages/core/src/units.js';
import { landRate } from './landrate.js';
import { mouthLips } from './shottable.js';
import { cmBare } from './_units.js';
import type { Params, RobotSpec } from '../packages/core/src/types.js';

const RANGES = [40, 55, 70];

export async function main(argv: string[] = []): Promise<void> {
  const num = (k: string, d: number) => {
    const i = argv.indexOf(`--${k}`);
    return i >= 0 ? Number(argv[i + 1]) : d;
  };
  const shots = num('shots', 12);
  const seeds = Array.from({ length: num('seeds', 4) }, (_, i) => 11 + i * 18);

  const p = params as unknown as Params;
  const g = buildFieldGeometry(p);
  const lips = mouthLips(p);
  const r = p.ball.pollen.d_m / 2;
  // Half-extents of the opening the ball has to arrive inside, less its own radius.
  const halfLat = (g.cells[0].halfInterior[0] - r) * M_TO_IN;
  const halfLong = ((lips.near.z - lips.far.z) / 2 - r) * M_TO_IN;

  console.log('WHERE THE MISSES GO');
  console.log(`  opening at the mouth plane: +-${cmBare(halfLong)} cm along the shot line, +-${cmBare(halfLat)} cm across it`);
  console.log(`  ${seeds.length} seeds x ${RANGES.length} ranges x ${shots} shots, gate open`);
  console.log('');

  const before = (robotJson as unknown as RobotSpec).flywheel.minLandProb;
  (robotJson as unknown as RobotSpec).flywheel.minLandProb = 0;
  const rows: { range: number; n: number; cell: number; over: number; short: number; long: number; wide: number; longs: number[]; lats: number[] }[] = [];
  try {
    for (const range of RANGES) {
      const row = { range, n: 0, cell: 0, over: 0, short: 0, long: 0, wide: 0, longs: [] as number[], lats: [] as number[] };
      for (const seed of seeds) {
        const res = await landRate(range, shots, {}, seed);
        if (!res.placed) continue;
        for (const s of res.log) {
          if (s.result === 'flight' || !Number.isFinite(s.long_in)) continue;
          row.n++;
          row.longs.push(s.long_in);
          row.lats.push(s.lat_in);
          if (s.result === 'cell') { row.cell++; continue; }
          if (Math.abs(s.lat_in) > halfLat) row.wide++;
          else if (s.long_in < -halfLong) row.short++;
          else if (s.long_in > halfLong) row.long++;
          else row.over++;
        }
      }
      rows.push(row);
    }
  } finally {
    (robotJson as unknown as RobotSpec).flywheel.minLandProb = before;
  }

  const mean = (a: number[]) => (a.length ? a.reduce((x, y) => x + y, 0) / a.length : 0);
  const sd = (a: number[]) => {
    if (a.length < 2) return 0;
    const m = mean(a);
    return Math.sqrt(a.reduce((s, v) => s + (v - m) ** 2, 0) / (a.length - 1));
  };
  const pct = (k: number, n: number) => `${((k / Math.max(1, n)) * 100).toFixed(0)}%`.padStart(5);

  console.log('  range      n   landed   over the hole   short    long    wide    long bias (cm)  lat bias (cm)');
  for (const w of rows) {
    console.log(
      `  ${(w.range * 0.0254).toFixed(2).padStart(5)}  ${String(w.n).padStart(5)}   ${pct(w.cell, w.n)}   ${pct(w.over, w.n).padStart(13)}   ` +
      `${pct(w.short, w.n)}   ${pct(w.long, w.n)}   ${pct(w.wide, w.n)}   ` +
      `${cmBare(mean(w.longs), 1, 6)}+-${cmBare(sd(w.longs), 1, 5)}  ${cmBare(mean(w.lats), 1, 6)}+-${cmBare(sd(w.lats), 1, 5)}`,
    );
  }
  const tot = rows.reduce((a, w) => ({
    n: a.n + w.n, cell: a.cell + w.cell, over: a.over + w.over, short: a.short + w.short, long: a.long + w.long, wide: a.wide + w.wide,
  }), { n: 0, cell: 0, over: 0, short: 0, long: 0, wide: 0 });
  console.log('');
  console.log(`  ALL      ${String(tot.n).padStart(5)}   ${pct(tot.cell, tot.n)}   ${pct(tot.over, tot.n).padStart(13)}   ${pct(tot.short, tot.n)}   ${pct(tot.long, tot.n)}   ${pct(tot.wide, tot.n)}`);
  console.log('');
  const missed = tot.n - tot.cell;
  if (!missed) { console.log('  nothing missed.'); return; }
  const overShare = tot.over / missed;
  console.log('VERDICT');
  console.log(overShare > 0.5
    ? `  ENTRY. ${(overShare * 100).toFixed(0)}% of misses arrived over the opening and came back out. The entry model is`
      + ' aimed at the right term; the question is why the flatter table did not cash it in.'
    : `  AIM. Only ${(overShare * 100).toFixed(0)}% of misses were over the opening -- the rest never arrived there.`
      + ` Optimising how a ball behaves once it reaches the mouth cannot fix a shot that does not.`);
}
