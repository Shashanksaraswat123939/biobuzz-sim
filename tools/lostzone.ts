/**
 * THE SQUARES THE DEPTH TERM TOOK AWAY. Stand on them and shoot.
 *
 *   npm run tool -- tools/lostzone.ts [--shots 8]
 *
 * 84525e6 added the pocket's depth to the aperture -- width*cos(beta) - depth*sin(beta)
 * instead of width*cos(beta) -- and the green zone went from 52 squares to 32. That was
 * justified by tools/obliquity.ts, which COMPUTES THE FORMULA AND NEVER FIRES A BALL. This
 * fires the balls: it stands on each square the change took away, drops the probability gate
 * so the robot cannot refuse, and reports what actually lands.
 *
 * If they land like the green ones do, the depth term is too harsh and the zone is lying in
 * the expensive direction -- refusing shots that were there.
 */
import { execFileSync } from 'node:child_process';
import params from '../config/params.json' with { type: 'json' };
import robotSpec from '../config/robot.json' with { type: 'json' };
import zoneNow from '../config/shotzone.json' with { type: 'json' };
import { readFileSync } from 'node:fs';
import { World, initPhysics, emptyGamepad } from '../packages/core/src/physics/world.js';
import { BuiltinTeleOp, ShotTable } from '../packages/core/src/robot/builtinTeleOp.js';
import { loadLandCal } from '../packages/core/src/robot/loadCal.js';
import { RAD, inches } from '../packages/core/src/units.js';
import type { GamepadState, Params, RobotSpec, Vec3 } from '../packages/core/src/types.js';

const table = ShotTable.fromCsv(readFileSync(new URL('../java/teamcode/assets/shottable.csv', import.meta.url), 'utf8'));

interface Cell { x_in: number; z_in: number; p: number; why?: string }

/** Fire `shots` from this square with the gate wide open, and see what lands. */
async function standAndShoot(c: Cell, shots: number, seed: number) {
  await initPhysics();
  const p = structuredClone(params) as unknown as Params;
  const spec = structuredClone(robotSpec) as unknown as RobotSpec;
  // THE GATE'S OPINION IS THE THING UNDER TEST, so it does not get a vote. Everything else
  // -- scatter, the rocker, the lip, balls already in the pocket -- stays real.
  spec.flywheel.minLandProb = 0;
  spec.turret.fireOpenCap_deg = 180;
  const pool = Array.from({ length: shots + 4 }, () => ({ kind: 'pollen' as const, pos: [0, -5, 0] as Vec3 }));
  const w = new World({ params: p, robot: spec, staging: pool, alliance: 'red', seed });
  for (const b of w.balls.balls) w.balls.park(b);
  const mouth = w.hives.red.upCellMouthWorld();
  const x = inches(c.x_in), z = inches(c.z_in);
  w.robot.place([x, spec.chassis.height_m / 2 + spec.chassis.clearance_m, z], Math.atan2(mouth[0] - x, mouth[2] - z) * RAD);
  const brain = new BuiltinTeleOp(spec, table, loadLandCal());
  brain.state.firing = true;
  let loaded = 0;
  const step = () => {
    while (w.robot.heldBalls().length < 2 && loaded < pool.length) {
      if (!w.robot.preload(w.balls, w.balls.balls[loaded])) break;
      loaded++;
    }
    const g: GamepadState = emptyGamepad();
    w.setGamepads(g, emptyGamepad());
    w.step(brain.update(w.sensors(), g, w.seq, 1 / 60));
  };
  let f = 0;
  while (w.robot.shots < shots && f++ < 60 * 20) step();
  brain.state.firing = false;
  for (let k = 0; k < 60 * 4; k++) step();
  // Range and off-axis are not stored on the cell, so take them off the real geometry.
  const n = w.hives.red.upCellMouthNormalWorld();
  const dx = mouth[0] - w.robot.pos[0], dz = mouth[2] - w.robot.pos[2];
  const d = Math.hypot(dx, dz) || 1;
  const off = Math.acos(Math.max(-1, Math.min(1, -(dx * n[0] + dz * n[2]) / d))) * RAD;
  return { shots: w.robot.shots, landed: w.landedInUpCell('red'), off, range_in: d * 39.3700787 };
}

export async function main(argv: string[] = []): Promise<void> {
  const i = argv.indexOf('--shots');
  const shots = i >= 0 ? Number(argv[i + 1]) : 8;
  const now = zoneNow as unknown as { cells: Cell[]; threshold: number };
  const before = JSON.parse(execFileSync('git', ['show', 'a3e3701:config/shotzone.json'], { encoding: 'utf8' })) as { cells: Cell[]; threshold: number };
  const key = (c: Cell) => `${c.x_in},${c.z_in}`;
  const greenNow = new Set(now.cells.filter((c) => c.p >= now.threshold).map(key));
  const lost = before.cells.filter((c) => c.p >= before.threshold && !greenNow.has(key(c)));
  const kept = now.cells.filter((c) => c.p >= now.threshold);

  console.log(`\nWHAT THE DEPTH TERM TOOK AWAY. ${lost.length} squares were green before 84525e6 and are not now.`);
  console.log(`Firing ${shots} balls from each with the gate forced open, against ${Math.min(10, kept.length)} squares that stayed green.\n`);

  for (const [name, cells] of [['STILL GREEN (the control)', kept.filter((_, n) => n % 3 === 0).slice(0, 10)], ['LOST to the depth term', lost]] as [string, Cell[]][]) {
    let S = 0, L = 0;
    let worst = '';
    let worstRate = 2;
    const offSeen: number[] = [];
    for (const [n, c] of cells.entries()) {
      const r = await standAndShoot(c, shots, 500 + n);
      S += r.shots; L += r.landed;
      const rate = r.shots ? r.landed / r.shots : 0;
      offSeen.push(r.off);
      if (r.shots >= 3 && rate < worstRate) { worstRate = rate; worst = `${r.off.toFixed(0)}° off at ${r.range_in.toFixed(0)} in -> ${(rate * 100).toFixed(0)}%`; }
    }
    offSeen.sort((a, b) => a - b);
    console.log(`  ${name.padEnd(26)} ${String(cells.length).padStart(3)} squares   ${String(S).padStart(4)} fired   ${String(L).padStart(4)} in   ${S ? ((L / S) * 100).toFixed(0).padStart(3) : '  -'}%`);
    console.log(`  ${' '.repeat(26)} ${offSeen[0].toFixed(0)}-${offSeen[offSeen.length - 1].toFixed(0)}\u00b0 off the opening   worst: ${worst}`);
  }
  console.log('\n  The depth term claims the aperture is shut past 55 deg.\n');
}
