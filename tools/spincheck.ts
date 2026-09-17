/**
 * Does the backspin off a single-wheel shooter help the ball STAY in the CELL?
 *
 *   npm run tool -- tools/spincheck.ts [--n 24]
 *
 * A single-wheel shooter squeezes the ball between one spinning wheel and a fixed hood, so
 * the ball leaves with heavy backspin. A dual-wheel shooter pinches it between two wheels
 * turning opposite ways and imparts almost none. That is the whole difference between the
 * two layouts as far as the ball is concerned, and it acts in two separate places:
 *
 *   IN FLIGHT, through Magnus lift (aero.ts): backspin pushes the ball UP, flattening the
 *   arc and extending the range for a given exit speed.
 *
 *   AT THE POCKET, through contact friction: a backspinning ball meeting the pocket floor
 *   has its surface moving BACKWARD at the contact, so friction there pushes it back toward
 *   the mouth it came from -- it digs in and kills its own forward speed instead of skipping
 *   off the back wall. Topspin does the opposite, which is why getting the sign wrong in
 *   entrycheck.ts once reported 0% entry at speeds the robot scores from perfectly well.
 *
 * This measures the second one alone, with no shooter and no flight in the way: inject balls
 * at the mouth with a given arrival speed and descent angle, once with the spin a single-wheel
 * shooter gives them and once with none, and count what is still in the CELL afterwards.
 */
import { readFileSync, writeFileSync } from 'node:fs';
import params from '../config/params.json' with { type: 'json' };
import robotJson from '../config/robot.json' with { type: 'json' };
import { ShotTable } from '../packages/core/src/robot/builtinTeleOp.js';
import { entryRate } from './entrycheck.js';
import { landRate } from './landrate.js';
import { buildTable, toCsv, loadEntry } from './shottable.js';
import type { Params, RobotSpec } from '../packages/core/src/types.js';

const tablePath = new URL('../java/teamcode/assets/shottable.csv', import.meta.url);

/**
 * THE WHOLE SHOOTER, both layouts, each with its OWN shot table.
 *
 * Comparing them against one table would be rigged: the table is solved against the spin the
 * layout produces (`spinPerSpeed` in tools/shottable.ts), so a dual-wheel robot flying a
 * single-wheel table is being asked to hit ranges that only a Magnus-lifted ball reaches. Each
 * side gets the table its own physics implies, and the table on disk is put back afterwards.
 */
async function wholeShooter(shots: number): Promise<void> {
  const p = structuredClone(params) as unknown as Params;
  const spec = robotJson as unknown as RobotSpec;
  const entry = loadEntry();
  const original = readFileSync(tablePath, 'utf8');
  const wasType = spec.flywheel.type;
  const wasGate = spec.flywheel.minLandProb;
  console.log('');
  console.log('  THE WHOLE SHOOTER, each layout with its own table, gate open');
  console.log('');
  console.log('  layout   solvable ranges   mean rpm   land rate at 40 / 55 / 70 in');
  try {
    for (const type of ['single', 'dual'] as const) {
      spec.flywheel.type = type;
      spec.flywheel.minLandProb = 0;             // measure the shooter, not the policy
      const rows = buildTable(p, spec, 30, 150, 4, entry, 'pLand');
      const usable = rows.filter((r) => r.margin > 0.02);
      const table = ShotTable.fromCsv(toCsv(rows));
      writeFileSync(tablePath, toCsv(rows));     // landRate's world reads the file
      const out: string[] = [];
      let fired = 0;
      let landed = 0;
      for (const r of [40, 55, 70]) {
        const res = await landRate(r, shots, {}, 11, table);
        fired += res.fired;
        landed += res.landed;
        out.push(res.fired ? `${((res.landed / res.fired) * 100).toFixed(0)}%` : '  -');
      }
      const rpm = usable.length ? usable.reduce((a, r) => a + r.rpm, 0) / usable.length : 0;
      console.log(
        `  ${type.padEnd(8)} ${String(usable.length).padStart(9)} of ${rows.length}   ${rpm.toFixed(0).padStart(8)}   ` +
        `${out.map((x) => x.padStart(6)).join('')}      pooled ${landed}/${fired}`,
      );
    }
  } finally {
    spec.flywheel.type = wasType;
    spec.flywheel.minLandProb = wasGate;
    writeFileSync(tablePath, original);
    console.log('  (shot table on disk restored)');
  }
}

export async function main(argv: string[] = []): Promise<void> {
  const i = argv.indexOf('--n');
  const n = i >= 0 ? Number(argv[i + 1]) : 24;

  console.log('BACKSPIN AT THE POCKET — of the balls that reach the mouth, how many stay?');
  console.log('');
  console.log(`  ${n} balls per cell, injected at the mouth. No shooter and no flight: this is`);
  console.log('  contact physics only, so any difference is the spin acting on the pocket.');
  console.log('');
  // The band the shot table actually arrives in, not the whole sweep.
  const speeds = [3, 4, 5];
  const descents = [35, 45, 55];
  console.log('              ' + speeds.map((s) => `${s} m/s`.padStart(16)).join(''));
  console.log('  descent     ' + speeds.map(() => '  spin   none  d'.padStart(16)).join(''));
  let spinWins = 0;
  let noneWins = 0;
  let spinTotal = 0;
  let noneTotal = 0;
  for (const d of descents) {
    let line = `  ${String(d).padStart(3)} deg down`;
    for (const sp of speeds) {
      const withSpin = await entryRate(sp, d, n, 7, 1);
      const without = await entryRate(sp, d, n, 7, 0);
      spinTotal += withSpin.stayed;
      noneTotal += without.stayed;
      if (withSpin.stayed > without.stayed) spinWins++;
      if (without.stayed > withSpin.stayed) noneWins++;
      const diff = (withSpin.rate - without.rate) * 100;
      line += `${(withSpin.rate * 100).toFixed(0).padStart(6)}%${(without.rate * 100).toFixed(0).padStart(6)}%${(diff >= 0 ? '+' : '') + diff.toFixed(0)}`.padStart(16);
    }
    console.log(line);
  }
  const cells = descents.length * speeds.length;
  const per = cells * n;
  const rate = (k: number) => k / per;
  const se = (k: number) => Math.sqrt((rate(k) * (1 - rate(k))) / per);
  const d = rate(spinTotal) - rate(noneTotal);
  const dse = Math.sqrt(se(spinTotal) ** 2 + se(noneTotal) ** 2);
  console.log('');
  console.log(`  backspin  ${(rate(spinTotal) * 100).toFixed(1)}% +-${(se(spinTotal) * 100).toFixed(1)}  (${spinTotal} of ${per})`);
  console.log(`  no spin   ${(rate(noneTotal) * 100).toFixed(1)}% +-${(se(noneTotal) * 100).toFixed(1)}  (${noneTotal} of ${per})`);
  console.log(`  difference ${(d * 100 >= 0 ? '+' : '')}${(d * 100).toFixed(1)} points, +-${(dse * 100).toFixed(1)} -- ${Math.abs(d) < 2 * dse ? 'INSIDE the noise, so this does not say which is better' : 'outside two standard errors'}`);
  console.log(`  better in ${spinWins} of ${cells} cells, worse in ${noneWins}. Per-cell counts are`);
  console.log(`  ${n} balls, so a single cell's swing is mostly sampling, not physics.`);
  console.log('');
  console.log('  That is entry only. The same spin also carries the ball in flight, through Magnus');
  console.log('  lift, and the shot table is solved against it -- so the layout has to be judged');
  console.log('  end to end, with each side given the table its own physics implies.');
  if (argv.includes('--full')) await wholeShooter(8);
}
