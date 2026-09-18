/**
 * Does the odometry drift, and does the tag pull it back?
 *
 *   npm run tool -- tools/driftcheck.ts [--secs 30] [--seeds 5]
 *
 * Two runs of the same driving on the same seeds: one with `sensors.localizer.fuse.gain` at
 * zero, so nothing corrects, and one with it at the shipped value. The number that matters is
 * the TRUE minus ESTIMATED position, which the robot can never see and the world always can.
 *
 * Without a correction this should grow without bound -- heading error rotates every inch
 * driven after it, so the error goes with DISTANCE and never comes back. With one it should
 * settle at whatever the camera's own error supports. If those two look the same, either the
 * drift model is not integrating or the fix is not being applied, and both have been true of
 * this file at some point.
 */
import params from '../config/params.json' with { type: 'json' };
import robotJson from '../config/robot.json' with { type: 'json' };
import { World, initPhysics, emptyGamepad } from '../packages/core/src/physics/world.js';
import { BuiltinTeleOp, ShotTable } from '../packages/core/src/robot/builtinTeleOp.js';
import { loadLandCal } from '../packages/core/src/robot/loadCal.js';
import { readFileSync } from 'node:fs';
import type { Params, RobotSpec, Vec3 } from '../packages/core/src/types.js';

const csv = readFileSync(new URL('../java/teamcode/assets/shottable.csv', import.meta.url), 'utf8');

/** Drive a lap of the field so the robot covers ground -- drift is paid for in inches moved. */
function stick(t: number) {
  const g = emptyGamepad();
  const phase = (t % 8) / 8;
  g.left_stick_y = phase < 0.5 ? -0.8 : 0.8;
  g.right_stick_x = phase < 0.25 || phase >= 0.75 ? 0.5 : -0.5;
  return g;
}

function run(seed: number, secs: number, gain: number): { finalIn: number; peakIn: number; rawIn: number; applied: number; rejected: number } {
  const p = structuredClone(params) as unknown as Params;
  const spec = structuredClone(robotJson) as unknown as RobotSpec;
  const base = spec.sensors.localizer.fuse ?? { gain: 0.15, headingGain: 0.05, rejectOver_in: 36 };
  spec.sensors.localizer.fuse = { ...base, gain, headingGain: gain > 0 ? base.headingGain : 0 };
  const staging = Array.from({ length: 8 }, () => ({ kind: 'pollen' as const, pos: [0, -5, 0] as Vec3 }));
  const world = new World({ params: p, robot: spec, staging, alliance: 'red', seed });
  for (const b of world.balls.balls) world.balls.park(b);
  const brain = new BuiltinTeleOp(spec, ShotTable.fromCsv(csv), loadLandCal(), null, 'red');

  const dt = p.sim.dt * p.sim.substepsPerFrame;
  // THE POSE THE ROBOT ACTUALLY USES, which is odometry PLUS the corrections the brain has
  // folded in. `world.odometryErrorIn()` is the raw dead reckoning and does not know the
  // fuser exists -- measuring that compared the same number against itself and reported,
  // very convincingly, that the correction did nothing.
  const errNow = (): number => {
    const ftc = world.snapshot().robot.ftc;
    const e = brain.pose_();
    return Math.hypot(ftc.x - e.x, ftc.y - e.y);
  };
  let peak = 0;
  for (let t = 0; t < secs; t += dt) {
    const g = stick(t);
    world.setGamepads(g, emptyGamepad());
    world.step(brain.update(world.sensors(), g, world.seq, dt));
    peak = Math.max(peak, errNow());
  }
  const st = brain.fuseStats();
  return { finalIn: errNow(), peakIn: peak, rawIn: world.odometryErrorIn(), applied: st.applied, rejected: st.rejected };
}

export async function main(args: string[] = []): Promise<void> {
  await initPhysics();
  const num = (k: string, d: number) => {
    const i = args.indexOf(`--${k}`);
    return i >= 0 ? Number(args[i + 1]) : d;
  };
  const secs = num('secs', 30);
  const seeds = Array.from({ length: num('seeds', 5) }, (_, i) => 7 + i * 18);

  console.log('DOES THE ODOMETRY DRIFT, AND DOES THE TAG PULL IT BACK?\n');
  console.log(`  ${secs} s of driving per run, true minus estimated position in inches.\n`);
  console.log('  seed    no correction            with tag fixes           fixes');
  console.log('              final    peak            final    peak     taken / thrown');

  const off: number[] = [];
  const on: number[] = [];
  for (const seed of seeds) {
    const a = run(seed, secs, 0);
    const b = run(seed, secs, robotJson.sensors.localizer.fuse?.gain ?? 0.15);
    off.push(a.finalIn);
    on.push(b.finalIn);
    console.log(
      `  ${String(seed).padStart(4)}  ${a.finalIn.toFixed(1).padStart(9)} ${a.peakIn.toFixed(1).padStart(7)}` +
        `  ${b.finalIn.toFixed(1).padStart(15)} ${b.peakIn.toFixed(1).padStart(7)}` +
        `  ${String(b.applied).padStart(10)} / ${String(b.rejected)}`,
    );
  }
  const mean = (a: number[]) => a.reduce((x, y) => x + y, 0) / a.length;
  console.log(`\n  mean final error: ${mean(off).toFixed(1)} in uncorrected, ${mean(on).toFixed(1)} in corrected.`);
  if (mean(off) > 0.9 && mean(on) < mean(off)) {
    console.log(`  The tag removes ${(100 * (1 - mean(on) / mean(off))).toFixed(0)}% of it.`);
  } else if (mean(off) <= 0.9) {
    console.log('  THE DRIFT MODEL IS NOT DRIFTING. Check sensors.localizer.drift.enabled.');
  } else {
    console.log('  THE FIX IS NOT BEING APPLIED. Check that the camera can see the tag on this path.');
  }
  console.log('\n  What is left after correction is the camera error the fix carries in, and the');
  console.log('  stretches of the lap where no tag is decodable at all -- 74% of the field, per');
  console.log('  tools/tagmap.ts. A robot that never sees the tag drifts exactly as the left column does.');
}
