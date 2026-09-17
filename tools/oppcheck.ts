/**
 * Does the OPPONENT actually play? One line per seed.
 *
 *   npm run tool -- tools/oppcheck.ts [--seeds 3]
 *
 * Plays a whole match with the human robot standing still and the bot doing its thing, then
 * reports what the bot managed on its own: shots away, balls in its up CELL, tips, and score.
 * A bot that scores nothing is a bot that is not worth practising against, and that is not a
 * question the browser can answer quickly -- a match takes two and a half minutes to watch.
 */
import params from '../config/params.json' with { type: 'json' };
import robotSpec from '../config/robot.json' with { type: 'json' };
import staging from '../assets/staging.json' with { type: 'json' };
import { readFileSync } from 'node:fs';
import { World, initPhysics, emptyGamepad } from '../packages/core/src/physics/world.js';
import { BuiltinTeleOp, ShotTable } from '../packages/core/src/robot/builtinTeleOp.js';
import { OpponentBot } from '../packages/core/src/robot/opponentBot.js';
import { loadLandCal } from '../packages/core/src/robot/loadCal.js';
import { worldToFtc } from '../packages/core/src/field/ftcFrame.js';
import { M_TO_IN } from '../packages/core/src/units.js';
import type { BallKind, Params, RobotSpec, Vec3 } from '../packages/core/src/types.js';

const table = ShotTable.fromCsv(readFileSync(new URL('../java/teamcode/assets/shottable.csv', import.meta.url), 'utf8'));
const balls = (staging.balls as { kind: string; pos: number[] }[]).map((b) => ({ kind: b.kind as BallKind, pos: b.pos as Vec3 }));

const phaseT: Record<string, number> = {};
async function one(seed: number, trace = false) {
  const p = structuredClone(params) as unknown as Params;
  const spec = structuredClone(robotSpec) as unknown as RobotSpec;
  const world = new World({ params: p, robot: spec, staging: balls, alliance: 'red', seed, preload: spec.hopper.capacity, opponent: true });
  const oppBrain = new BuiltinTeleOp(spec, table, loadLandCal());
  const bot = new OpponentBot();
  const opp = world.opponent!;
  const hive = world.hives[opp.alliance];
  const zone = world.geom.zones.find((z) => z.name === 'LOADING' && z.alliance === opp.alliance)!;
  const ranges = table.rows.map((r) => r.range_in);
  const dt = p.sim.dt * p.sim.substepsPerFrame;
  world.clock.start();
  while (world.clock.period !== 'FINISHED') {
    const os = world.opponentSensors();
    const loose: [number, number][] = world.balls.balls
      .filter((b) => b.body.isEnabled() && b.state === 'free' && b.kind !== (opp.alliance === 'red' ? 'nectarBlue' : 'nectarRed'))
      .filter((b) => world.balls.pos(b)[1] * M_TO_IN < 12)
      .map((b) => { const f = worldToFtc(world.balls.pos(b)); return [f[0], f[1]] as [number, number]; });
    const g = bot.update(os, dt, {
      mouth: hive.upCellMouthWorld(),
      mouthNormal: hive.upCellMouthNormalWorld(),
      loading: [zone.min[0], zone.min[2], zone.max[0], zone.max[2]],
      halfWidth_m: world.geom.halfWidth_m,
      band_in: [Math.min(...ranges), Math.max(...ranges)],
    }, { loose, remaining: world.clock.remaining, period: world.clock.period, shotsTaken: opp.shots });
    world.setOpponentActuators(oppBrain.update(os, g, world.seq, dt));
    world.setGamepads(emptyGamepad(), emptyGamepad());
    world.step({ seq: world.seq, motors: {}, servos: {} });
    if (trace) {
      phaseT[bot.phase] = (phaseT[bot.phase] ?? 0) + dt;
      if (world.seq % 120 === 0) console.log(`    t=${world.t.toFixed(0)}s ${world.clock.period} ${bot.phase.padEnd(8)} hop=${os.game.hopper} rng=${os.game.upCellRangeIn.toFixed(0)} open=${os.game.upCellOpenDeg.toFixed(0)} rpm=${os.game.flywheelRpm.toFixed(0)} hold=${oppBrain.state.hold || '-'} at=(${os.localizer.x.toFixed(0)},${os.localizer.y.toFixed(0)}) want=(${bot.target[0].toFixed(0)},${bot.target[1].toFixed(0)}) world=(${opp.pos.map((v)=>(v*M_TO_IN).toFixed(0)).join(',')}) red=(${world.robot.pos.map((v)=>(v*M_TO_IN).toFixed(0)).join(',')}) v=${Math.hypot(opp.vel[0],opp.vel[2]).toFixed(3)}`);
    }
  }
  if (trace) console.log('    time per phase:', Object.entries(phaseT).map(([k, v]) => `${k} ${v.toFixed(0)}s`).join(', '));
  const sc = world.scorer.state[opp.alliance];
  return { seed, shots: opp.shots, inCell: world.landedInUpCell(opp.alliance), tips: sc.tips, score: sc.total, phase: bot.phase, note: bot.note };
}

export async function main(argv: string[] = []): Promise<void> {
  await initPhysics();
  const i = argv.indexOf('--seeds');
  const n = i >= 0 ? Number(argv[i + 1]) : 3;
  console.log('\nTHE OPPONENT BOT, a full match per seed, with the human robot standing still.\n');
  console.log('  seed   shots   in its CELL   tips   points   ended');
  const runs = [];
  for (let k = 0; k < n; k++) {
    const r = await one(11 + k * 17, argv.includes('--trace'));
    runs.push(r);
    console.log(`  ${String(r.seed).padStart(4)}   ${String(r.shots).padStart(5)}   ${String(r.inCell).padStart(11)}   ${String(r.tips).padStart(4)}   ${String(r.score).padStart(6)}   ${r.phase}`);
  }
  const mean = (a: number[]) => a.reduce((x, y) => x + y, 0) / (a.length || 1);
  console.log(`\n  mean: ${mean(runs.map((r) => r.shots)).toFixed(1)} shots, ${mean(runs.map((r) => r.tips)).toFixed(1)} tips, ${mean(runs.map((r) => r.score)).toFixed(1)} points`);
  console.log(`  last note: ${runs[runs.length - 1]?.note ?? '-'}\n`);
}
