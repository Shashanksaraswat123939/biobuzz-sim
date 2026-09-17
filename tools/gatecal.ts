/**
 * Two questions, one harness.
 *
 *   npm run tool -- tools/gatecal.ts [--seeds 3] [--shots 10]
 *
 * 1. DOES THE FLATTER TABLE ACTUALLY LAND MORE? The shot table's objective changed from
 *    "widest speed margin" to "P(threads the mouth) x P(stays in)", which dropped the apex
 *    from a mean of 119 in to 64 in and the arrival from 75 deg at 6.2 m/s to 34 deg at
 *    3.5 m/s. That is the right shape according to tools/entrycheck.ts. This checks it
 *    against the only thing that settles it, which is balls in the CELL.
 *
 * 2. IS THE 90% GATE HONEST? A robot that refuses to shoot unless it is "90% sure" and then
 *    lands 55% of what it takes is not being careful, it is lying. The gate is calibrated
 *    only if the measured land rate of the shots it ALLOWS is close to the threshold it was
 *    set to. This sweeps the threshold and plots predicted against actual, which is the
 *    check that makes the number mean something.
 *
 * Both arms run the same ranges, the same seeds and the same scripted firing, and the shot
 * table on disk is restored afterwards whatever happens.
 */
import { readFileSync, writeFileSync } from 'node:fs';
import params from '../config/params.json' with { type: 'json' };
import robotJson from '../config/robot.json' with { type: 'json' };
import { ShotTable } from '../packages/core/src/robot/builtinTeleOp.js';
import { landRate } from './landrate.js';
import { buildTable, toCsv, loadEntry } from './shottable.js';
import type { Params, RobotSpec } from '../packages/core/src/types.js';

const RANGES = [40, 55, 70];

export interface Arm {
  name: string;
  fired: number;
  landed: number;
  rate: number;
  se: number;
}

async function measure(table: ShotTable, name: string, seeds: number[], shots: number, minP: number): Promise<Arm> {
  const before = (robotJson as unknown as RobotSpec).flywheel.minLandProb;
  (robotJson as unknown as RobotSpec).flywheel.minLandProb = minP;
  let fired = 0;
  let landed = 0;
  try {
    for (const seed of seeds) {
      for (const r of RANGES) {
        const res = await landRate(r, shots, {}, seed, table);
        if (!res.placed) continue;
        fired += res.fired;
        landed += res.landed;
      }
    }
  } finally {
    (robotJson as unknown as RobotSpec).flywheel.minLandProb = before;
  }
  const rate = fired > 0 ? landed / fired : 0;
  return { name, fired, landed, rate, se: fired > 0 ? Math.sqrt((rate * (1 - rate)) / fired) : 0 };
}

export async function main(argv: string[] = []): Promise<void> {
  const num = (k: string, d: number) => {
    const i = argv.indexOf(`--${k}`);
    return i >= 0 ? Number(argv[i + 1]) : d;
  };
  const seeds = Array.from({ length: num('seeds', 3) }, (_, i) => 11 + i * 18);
  const shots = num('shots', 10);

  const p = params as unknown as Params;
  const spec = robotJson as unknown as RobotSpec;
  const tablePath = new URL('../java/teamcode/assets/shottable.csv', import.meta.url);
  const original = readFileSync(tablePath, 'utf8');

  // Build both tables from the same solver, differing only in whether the entry model is in
  // the objective. Rebuilding the old one rather than reading a saved copy keeps the two
  // arms honest: every other input is identical by construction.
  const entry = loadEntry();
  if (!entry) throw new Error('config/entry.json missing — run tools/entrycheck.ts first');
  const flatRows = buildTable(p, spec, 30, 150, 4, entry, 'pLand');
  // The ACTUAL old objective, not "the new one with the entry model switched off".
  const tallRows = buildTable(p, spec, 30, 150, 4, null, 'margin');
  const mean = (a: number[]) => a.reduce((x, y) => x + y, 0) / a.length;

  console.log('SHOT TABLE A/B AND GATE CALIBRATION');
  console.log('');
  console.log('  objective              apex (in)   flight   arrival        margin');
  for (const [nm, rows] of [['margin only (old)', tallRows], ['margin x entry (new)', flatRows]] as const) {
    console.log(
      `  ${nm.padEnd(22)} ${mean(rows.map((r) => r.apex_m)).toFixed(2)} m     ` +
      `${mean(rows.map((r) => r.flight_s)).toFixed(2)} s   ` +
      `${mean(rows.map((r) => r.descentDeg)).toFixed(0)} deg @ ${mean(rows.map((r) => r.arrival_mps)).toFixed(1)} m/s   ` +
      `${(mean(rows.map((r) => r.margin)) * 100).toFixed(1)}%`,
    );
  }
  console.log('');

  const flat = ShotTable.fromCsv(toCsv(flatRows));
  const tall = ShotTable.fromCsv(toCsv(tallRows));

  try {
    console.log(`  ${seeds.length} seeds x ${RANGES.length} ranges x ${shots} shots per arm, gate off (P >= 0)`);
    console.log('');
    console.log('  table                  fired   landed   land rate     95% interval');
    const arms: Arm[] = [];
    for (const [nm, t] of [['margin only (old)', tall], ['margin x entry (new)', flat]] as const) {
      writeFileSync(tablePath, nm.includes('old') ? toCsv(tallRows) : toCsv(flatRows));
      const a = await measure(t, nm, seeds, shots, 0);
      arms.push(a);
      console.log(
        `  ${a.name.padEnd(22)} ${String(a.fired).padStart(5)}   ${String(a.landed).padStart(6)}   ` +
        `${(a.rate * 100).toFixed(1).padStart(7)}%   ${((a.rate - 1.96 * a.se) * 100).toFixed(1).padStart(8)}% - ${((a.rate + 1.96 * a.se) * 100).toFixed(1)}%`,
      );
    }
    const gap = arms[1].rate - arms[0].rate;
    const gapSe = Math.hypot(arms[0].se, arms[1].se);
    console.log('');
    console.log(gap > 1.96 * gapSe
      ? `  The flatter table wins by ${(gap * 100).toFixed(1)} points, ${(gap / gapSe).toFixed(1)} standard errors. Real.`
      : `  Difference is ${(gap * 100).toFixed(1)} points, only ${(gap / Math.max(gapSe, 1e-9)).toFixed(1)} standard errors. NOT established.`);
    console.log('');

    // ---- is the gate honest?
    writeFileSync(tablePath, toCsv(flatRows));
    console.log('  GATE CALIBRATION — does the threshold mean what it says?');
    console.log('');
    // Every run gets the SAME frame budget whatever the threshold does, so `landed` is a
    // throughput as well as a count -- which is the number that actually decides the
    // setting. A gate that doubles the land rate and quarters the shots scores less.
    console.log(`  threshold   shots taken   landed   actual rate    per run    calibrated?`);
    for (const minP of [0, 0.3, 0.5, 0.7, 0.8, 0.86]) {
      const a = await measure(flat, `P>=${minP}`, seeds, shots, minP);
      const err = a.rate - minP;
      const runs = seeds.length * RANGES.length;
      console.log(
        `  ${(minP * 100).toFixed(0).padStart(7)}%   ${String(a.fired).padStart(11)}   ${String(a.landed).padStart(6)}   ` +
        `${(a.rate * 100).toFixed(1).padStart(9)}%   ${(a.landed / runs).toFixed(2).padStart(7)}    ` +
        `${minP === 0 ? '(ungated)' : Math.abs(err) < 0.1 ? 'yes' : err < 0 ? `OVERCONFIDENT by ${(-err * 100).toFixed(0)} pts` : `pessimistic by ${(err * 100).toFixed(0)} pts`}`,
      );
    }
    console.log('');
    console.log('  A gate is calibrated when the actual rate tracks the threshold. If it sits well');
    console.log('  BELOW, the robot is refusing shots on a promise it cannot keep, and the honest');
    console.log('  fix is in the model -- not in raising the threshold until the number looks good.');
  } finally {
    writeFileSync(tablePath, original);
    console.log('');
    console.log('  (shot table on disk restored to what it was before this run)');
  }
}
