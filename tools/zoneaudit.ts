/**
 * IS GREEN GREEN? Stand on the map's squares and shoot.
 *
 *   npm run tool -- tools/zoneaudit.ts [--cells 24] [--shots 4]
 *
 * The shot map is a promise to the driver: stand on a green square and the shot lands. This
 * keeps the promise honest by standing a robot on a sample of squares -- green ones and, for
 * contrast, the ones painted as too far or off the opening -- firing a few balls from each
 * with the real gate and real scatter, and reporting what landed against what the square
 * claimed. A green square that lands 60% is a map that lies; a grey one that lands 90% is a
 * map that hides a shot.
 */
import params from '../config/params.json' with { type: 'json' };
import robotSpec from '../config/robot.json' with { type: 'json' };
import zone from '../config/shotzone.json' with { type: 'json' };
import { readFileSync } from 'node:fs';
import { World, initPhysics, emptyGamepad } from '../packages/core/src/physics/world.js';
import { BuiltinTeleOp, ShotTable } from '../packages/core/src/robot/builtinTeleOp.js';
import { loadLandCal } from '../packages/core/src/robot/loadCal.js';
import { RAD, inches } from '../packages/core/src/units.js';
import type { GamepadState, Params, RobotSpec, Vec3 } from '../packages/core/src/types.js';

const table = ShotTable.fromCsv(readFileSync(new URL('../java/teamcode/assets/shottable.csv', import.meta.url), 'utf8'));

interface Cell { x_in: number; z_in: number; p: number; why?: string; range_in: number; offAxisDeg: number }

async function standAndShoot(c: Cell, shots: number, seed: number): Promise<{ shots: number; landed: number; hold: string; secs: number }> {
  await initPhysics();
  const p = structuredClone(params) as unknown as Params;
  const spec = structuredClone(robotSpec) as unknown as RobotSpec;
  const pool = Array.from({ length: 12 }, () => ({ kind: 'pollen' as const, pos: [0, -5, 0] as Vec3 }));
  const w = new World({ params: p, robot: spec, staging: pool, alliance: 'red', seed });
  for (const b of w.balls.balls) w.balls.park(b);
  const mouth = w.hives.red.upCellMouthWorld();
  const x = inches(c.x_in), z = inches(c.z_in);
  w.robot.place([x, spec.chassis.height_m / 2 + spec.chassis.clearance_m, z], Math.atan2(mouth[0] - x, mouth[2] - z) * RAD);
  const brain = new BuiltinTeleOp(spec, table, loadLandCal());
  brain.state.firing = true;
  let loaded = 0;
  const why: Record<string, number> = {};
  const step = (g: GamepadState) => {
    while (w.robot.heldBalls().length < 2 && loaded < pool.length) {
      if (!w.robot.preload(w.balls, w.balls.balls[loaded])) break;
      loaded++;
    }
    w.setGamepads(g, emptyGamepad());
    w.step(brain.update(w.sensors(), g, w.seq, 1 / 60));
    const h = brain.state.hold;
    const k = h ? h.replace(/-?[\d.]+/g, 'N') : 'clear';
    why[k] = (why[k] ?? 0) + 1;
  };
  // TIME, NOT JUST THE RATE. A gate that only fires when it is nearly certain reads as a
  // high percentage while scoring less, because every refused cycle is time spent not
  // scoring. Seconds per ball IN is the number a match is won on, so it travels with the
  // percentage everywhere.
  let f = 0;
  const t0 = w.t;
  let lastShotT = w.t;
  let seen = 0;
  while (w.robot.shots < shots && f++ < 60 * 8) {
    step(emptyGamepad());
    if (w.robot.shots > seen) { seen = w.robot.shots; lastShotT = w.t; }
  }
  brain.state.firing = false;
  for (let k = 0; k < 60 * 4; k++) step(emptyGamepad());
  const top = Object.entries(why).sort((a, b) => b[1] - a[1])[0]?.[0] ?? '';
  return { shots: w.robot.shots, landed: w.landedInUpCell('red'), hold: top, secs: lastShotT - t0 };
}

export async function main(argv: string[] = []): Promise<void> {
  const num = (k: string, d: number) => { const i = argv.indexOf(`--${k}`); return i >= 0 ? Number(argv[i + 1]) : d; };
  const nCells = num('cells', 24), shots = num('shots', 4);
  const z = zone as unknown as { cells: Cell[]; threshold: number };
  // A deterministic spread of squares: every k-th of each class, so the sample covers the
  // field rather than one corner of it.
  const pick = (cs: Cell[], n: number) => cs.filter((_, i) => i % Math.max(1, Math.floor(cs.length / n)) === 0).slice(0, n);
  const green = pick(z.cells.filter((c) => c.p >= z.threshold), nCells);
  const amber = pick(z.cells.filter((c) => c.p > 0 && c.p < z.threshold), Math.ceil(nCells / 3));
  const grey = pick(z.cells.filter((c) => c.p === 0 && c.why !== 'noRoom' && c.why !== 'behind'), Math.ceil(nCells / 3));
  console.log(`\nSTANDING ON THE MAP AND SHOOTING. ${shots} balls a square, real gate, real scatter, real rocker.\n`);
  for (const [name, cells] of [['GREEN (map says land)', green], ['AMBER (below the gate)', amber], ['GREY (too far / no shot)', grey]] as [string, Cell[]][]) {
    let S = 0, L = 0, fired = 0, T = 0;
    const holds: Record<string, number> = {};
    for (const [i, c] of cells.entries()) {
      const r = await standAndShoot(c, shots, 100 + i);
      S += r.shots; L += r.landed; if (r.shots > 0) fired++; T += r.secs;
      holds[r.hold] = (holds[r.hold] ?? 0) + 1;
    }
    const top = Object.entries(holds).sort((a, b) => b[1] - a[1]).slice(0, 2).map(([k, v]) => `${v}x ${k}`).join(', ');
    const perBall = L > 0 ? (T / L).toFixed(2) : '  -';
    console.log(`  ${name.padEnd(26)} ${String(cells.length).padStart(3)} squares   fired from ${String(fired).padStart(3)}   shots ${String(S).padStart(4)}   landed ${String(L).padStart(4)}   ${S ? ((L / S) * 100).toFixed(0).padStart(3) : '  -'}%   ${perBall.padStart(5)} s per ball IN`);
    console.log(`  ${' '.repeat(26)} held: ${top}`);
  }
  console.log('');
}
