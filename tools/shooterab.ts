/**
 * The whole question, decided: does the fixed-speed shooter beat the speed-solving one?
 *
 *   npm run tool -- tools/shooterab.ts [--seconds 24]
 *
 * Two shooters, identical in every other respect -- same field, same seeds, same scripted
 * driving, same hopper, same gate threshold:
 *
 *   SPEED  the shipped design. Each range wants its own flywheel speed, so the wheel is in
 *          the control loop: it must reach a new speed for every shot, it lags on the way,
 *          and the readiness gate compares a MEASURED speed against a window. That window is
 *          +-60 rpm because one encoder count over 20 ms is 107 rpm, and +-60 rpm is +-21 cm
 *          of range into a pocket 22.5 cm deep.
 *
 *   HOOD   one wheel speed all match, hood does the aiming, and the robot's own velocity is
 *          an axis of the table rather than something to cancel. Readiness becomes "has the
 *          hood arrived", which is a commanded servo position.
 *
 * The cases are the ones that separated them before: standing still, steady motion, and the
 * accelerating case where the speed-solving shooter fires plenty and lands nothing.
 */
import params from '../config/params.json' with { type: 'json' };
import robotSpec from '../config/robot.json' with { type: 'json' };
import { readFileSync } from 'node:fs';
import { World, initPhysics, emptyGamepad } from '../packages/core/src/physics/world.js';
import { BuiltinTeleOp, ShotTable } from '../packages/core/src/robot/builtinTeleOp.js';
import { HoodTable } from '../packages/core/src/robot/hoodTable.js';
import { loadLandCal } from '../packages/core/src/robot/loadCal.js';
import { inches } from '../packages/core/src/units.js';
import type { GamepadState, Params, RobotSpec, Vec3 } from '../packages/core/src/types.js';

const shotCsv = readFileSync(new URL('../java/teamcode/assets/shottable.csv', import.meta.url), 'utf8');
const hoodCsv = readFileSync(new URL('../java/teamcode/assets/hoodtable.csv', import.meta.url), 'utf8');

interface Result { fired: number; landed: number; secs: number; longs: number[]; hold: Map<string, number> }

async function run(useHood: boolean, drive: [number, number], wobble: number, seconds: number, seed: number): Promise<Result> {
  await initPhysics();
  const p = structuredClone(params) as unknown as Params;
  const spec = structuredClone(robotSpec) as unknown as RobotSpec;
  // The gate is policy and the question here is the MECHANISM, so open it for both arms
  // equally -- otherwise a threshold calibrated against one shooter scores the other.
  spec.flywheel.minLandProb = 0;
  const staging = Array.from({ length: 90 }, () => ({ kind: 'pollen' as const, pos: [0, -5, 0] as Vec3 }));
  const world = new World({ params: p, robot: spec, staging, alliance: 'red', seed });
  for (const b of world.balls.balls) world.balls.park(b);

  const mouth = world.hives.red.upCellMouthWorld();
  const limit = world.geom.halfWidth_m - 0.35;
  const start: Vec3 = [
    mouth[0],
    spec.chassis.height_m / 2 + spec.chassis.clearance_m,
    Math.min(mouth[2] + inches(62), limit),
  ];
  world.robot.place(start, 180);

  const brain = new BuiltinTeleOp(
    spec,
    ShotTable.fromCsv(shotCsv),
    loadLandCal(),
    useHood ? HoodTable.fromCsv(hoodCsv) : null,
  );
  let loaded = 0;
  const step = (g: GamepadState) => {
    while (world.robot.heldBalls().length < 7 && loaded < staging.length) {
      if (!world.robot.preload(world.balls, world.balls.balls[loaded])) break;
      loaded++;
    }
    world.setGamepads(g, emptyGamepad());
    world.step(brain.update(world.sensors(), g, world.seq));
  };

  const arm = emptyGamepad();
  step(arm);
  const armed = emptyGamepad();
  armed.dpad_up = true;
  step(armed);

  const hold = (t: number): GamepadState => {
    const g = emptyGamepad();
    g.left_stick_x = drive[0];
    const stick = drive[1] + wobble * Math.sin(2 * Math.PI * 0.5 * t);
    g.left_stick_y = -Math.max(-1, Math.min(1, stick));
    return g;
  };
  for (let f = 0; f < 90; f++) step(hold(world.t));

  const t0 = world.t;
  const holds = new Map<string, number>();
  for (let f = 0; f < Math.round(seconds * 60); f++) {
    step({ ...hold(world.t), right_bumper: true });
    const h = brain.state.hold;
    if (h) holds.set(h.replace(/-?\d+(\.\d+)?/g, 'N'), (holds.get(h.replace(/-?\d+(\.\d+)?/g, 'N')) ?? 0) + 1);
    const r = world.robot.pos;
    if (Math.hypot(mouth[0] - r[0], mouth[2] - r[2]) < inches(30)) break;
  }
  for (let f = 0; f < 60 * 6; f++) step(emptyGamepad());

  const log = world.snapshot().shots.filter((x) => x.result !== 'flight');
  return {
    fired: world.robot.shots,
    landed: log.filter((x) => x.result === 'cell').length,
    secs: world.t - t0,
    longs: log.map((x) => x.long_in).filter(Number.isFinite),
    hold: holds,
  };
}

export async function main(argv: string[] = []): Promise<void> {
  const i = argv.indexOf('--seconds');
  const seconds = i >= 0 ? Number(argv[i + 1]) : 24;
  const seeds = [7, 25, 43, 61];
  const cases: [string, [number, number], number][] = [
    ['stopped', [0, 0], 0],
    ['steady closing', [0, 0.18], 0],
    ['steady strafing', [0.6, 0], 0],
    ['ACCELERATING', [0, 0.18], 0.30],
  ];

  const hood = HoodTable.fromCsv(hoodCsv);
  console.log('SHOOTER A/B — solving for speed, against holding one speed and aiming the hood');
  console.log(`  fixed wheel speed ${hood.fixedRpm} rpm; ${seeds.length} seeds a case; gate open for both arms`);
  console.log('');
  console.log('  case              shooter   shots/s   landed/s        landed   downrange bias');

  for (const [name, drive, wobble] of cases) {
    for (const useHood of [false, true]) {
      let secs = 0;
      let fired = 0;
      let landed = 0;
      const longs: number[] = [];
      const allHolds = new Map<string, number>();
      for (const seed of seeds) {
        const r = await run(useHood, drive, wobble, seconds, seed);
        secs += r.secs;
        fired += r.fired;
        landed += r.landed;
        longs.push(...r.longs);
        for (const [k, n] of r.hold) allHolds.set(k, (allHolds.get(k) ?? 0) + n);
      }
      const mean = (a: number[]) => (a.length ? a.reduce((x, y) => x + y, 0) / a.length : NaN);
      console.log(
        `  ${(useHood ? '' : name).padEnd(18)}${(useHood ? 'HOOD' : 'speed').padEnd(9)} ` +
        `${(fired / secs).toFixed(2).padStart(7)}   ${(landed / secs).toFixed(2).padStart(5)} ` +
        `+-${(Math.sqrt(Math.max(landed, 1)) / secs).toFixed(2)}   ${String(landed).padStart(6)}   ` +
        `${(mean(longs) * 2.54).toFixed(0).padStart(5)} cm`,
      );
      if (useHood && landed === 0) {
        // When an arm scores nothing, the reason it gave is worth more than the zero.
        const top = [...allHolds.entries()].sort((a, b) => b[1] - a[1]).slice(0, 2);
        if (top.length) console.log(`      held on: ${top.map(([k, n]) => `"${k}" x${n}`).join(', ')}`);
      }
    }
  }
  console.log('');
  console.log('  The accelerating case is the one that matters: the speed-solving shooter fires');
  console.log('  there and lands nothing, because a wheel chasing a falling target lags one way.');
}
