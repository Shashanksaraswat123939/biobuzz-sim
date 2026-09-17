/**
 * Can this robot shoot while it is moving? Not "on a sweep" -- at a STEADY speed.
 *
 *   npm run tool -- tools/movingfire.ts [--seconds 20]
 *
 * tools/collect.ts --moving fires one shot in forty, and the obvious reading is that shooting
 * while driving is impossible. tools/slew.ts says otherwise, and says exactly what the limit
 * is: the lead moves the target rpm by 688-1431 per m/s of RADIAL speed, so
 *
 *     d(target rpm)/dt  =  (rpm per m/s)  x  (radial acceleration)
 *
 * Velocity is not in that expression. A steady 1.5 m/s holds the target rpm perfectly still;
 * only CHANGING the closing speed moves it. The wheel slews about 1100 rpm/s, which buys
 * roughly 1.2 m/s^2 of radial acceleration at the current hood -- and `collect --moving`
 * drives a range sweep, accelerating and braking the whole way, so it is over that budget
 * continuously and the gate refuses almost every shot.
 *
 * This holds the stick still instead, lets the speed settle, and then fires.
 */
import params from '../config/params.json' with { type: 'json' };
import robotSpec from '../config/robot.json' with { type: 'json' };
import { World, initPhysics, emptyGamepad } from '../packages/core/src/physics/world.js';
import { BuiltinTeleOp, ShotTable } from '../packages/core/src/robot/builtinTeleOp.js';
import { loadLandCal } from '../packages/core/src/robot/loadCal.js';
import { inches, M_TO_IN } from '../packages/core/src/units.js';
import { readFileSync } from 'node:fs';
import type { GamepadState, Params, RobotSpec, Vec3 } from '../packages/core/src/types.js';

const table = ShotTable.fromCsv(readFileSync(new URL('../java/teamcode/assets/shottable.csv', import.meta.url), 'utf8'));

interface Run {
  name: string; fired: number; radialMps: number; radialAccel: number; rpmErr: number;
  landed: number; secs: number;
  /** Where the misses actually went, at the mouth plane. This is the diagnostic. */
  longs: number[]; lats: number[];
}

/**
 * @param drive  what the stick is held at: [strafe, forward]. Forward is toward the hive.
 * @param wobble stick amplitude of a 0.5 Hz oscillation, to spend the acceleration budget.
 */
async function run(name: string, drive: [number, number], wobble: number, seconds: number, seed = 7, tau?: number, ramp = 0): Promise<Run> {
  await initPhysics();
  const p = structuredClone(params) as unknown as Params;
  const spec = structuredClone(robotSpec) as unknown as RobotSpec;
  // The QUESTION is whether the wheel and the gate can keep up with a moving lead, which is a
  // mechanism question. The land-probability threshold is policy, and with it closed the first
  // version of this test fired nothing even STANDING STILL -- the start was 2.4 m out, past
  // the range any shot clears 90% from, so every case read zero and proved nothing.
  spec.flywheel.minLandProb = 0;
  if (tau !== undefined) spec.transfer.leadLatency_s = tau;
  const staging = Array.from({ length: 80 }, () => ({ kind: 'pollen' as const, pos: [0, -5, 0] as Vec3 }));
  const world = new World({ params: p, robot: spec, staging, alliance: 'red', seed });
  for (const b of world.balls.balls) world.balls.park(b);

  const mouth = world.hives.red.upCellMouthWorld();
  // ON THE FIELD. The first version started at z = 2.38 m against a 1.80 m half-width --
  // outside the wall, where there is no floor. The robot fell, the balls fell with it, every
  // case read zero shots, and the balls' unchanged position relative to a departing robot
  // read as "the magazine empties itself". That was reported as a bug in the robot. It was a
  // bug in this line.
  const limit = world.geom.halfWidth_m - 0.35;
  const start: Vec3 = [
    mouth[0],
    spec.chassis.height_m / 2 + spec.chassis.clearance_m,
    Math.min(mouth[2] + inches(62), limit),
  ];
  world.robot.place(start, 180);
  const brain = new BuiltinTeleOp(spec, table, loadLandCal());
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
  arm.a = true;
  step(arm);

  // Let the wheel and the drive settle before anything is judged.
  const hold = (t: number): GamepadState => {
    const g = emptyGamepad();
    g.left_stick_x = drive[0];
    // `ramp` is a steady change of stick per second: constant acceleration, the case a
    // first-order predictor is actually built for. `wobble` oscillates instead, and an
    // oscillation defeats v + a*tau at exactly the moments the gate likes -- at a velocity
    // peak the acceleration is zero, so the predictor says "it will stay here" one instant
    // before it reverses.
    const stick = drive[1] + ramp * t + wobble * Math.sin(2 * Math.PI * 0.5 * t);
    g.left_stick_y = -Math.max(-1, Math.min(1, stick));                       // -y is forward
    return g;
  };
  for (let f = 0; f < 90; f++) step(hold(world.t));

  const fire = (t: number): GamepadState => ({ ...hold(t), right_bumper: true });
  const radial: number[] = [];
  let prevRadial = 0;
  const accels: number[] = [];
  const t0 = world.t;
  for (let f = 0; f < Math.round(seconds * 60); f++) {
    const before = world.robot.shots;
    step(fire(world.t));
    const r = world.robot.pos;
    const dx = mouth[0] - r[0];
    const dz = mouth[2] - r[2];
    const n = Math.hypot(dx, dz) || 1;
    const v = world.robot.body.linvel();
    const vr = (v.x * dx + v.z * dz) / n;             // closing speed, + is toward the mouth
    radial.push(vr);
    if (f > 0) accels.push(Math.abs(vr - prevRadial) * 60);
    prevRadial = vr;
    if (world.robot.shots > before) { /* fired this frame */ }
    if (n < inches(30)) break;                        // arrived; stop before it drives through
  }
  for (let f = 0; f < 60 * 6; f++) step(emptyGamepad());

  const log = world.snapshot().shots.filter((s) => s.result !== 'flight');
  const mean = (a: number[]) => (a.length ? a.reduce((x, y) => x + y, 0) / a.length : 0);
  return {
    name,
    longs: log.map((x) => x.long_in).filter(Number.isFinite),
    lats: log.map((x) => x.lat_in).filter(Number.isFinite),
    secs: world.t - t0,
    fired: world.robot.shots,
    radialMps: mean(radial),
    radialAccel: mean(accels),
    rpmErr: mean(log.map((s) => Math.abs(s.rpm - s.targetRpm))),
    landed: log.filter((s) => s.result === 'cell').length,
  };
}

export async function main(argv: string[] = []): Promise<void> {
  const i = argv.indexOf('--seconds');
  const seconds = i >= 0 ? Number(argv[i + 1]) : 20;

  console.log('SHOOTING ON THE MOVE — is it the speed, or the change in speed?');
  console.log(`  ${seconds} s per case, same start, hopper kept loaded, gate at its configured threshold.`);
  console.log('');
  // Shots per second, not shots: a closing run crosses the usable field in a couple of
  // seconds and then has to stop, so raw counts compare a short run against a long one.
  console.log('  case                     secs   shots/s   landed/s        landed   rpm err   where the shots went');
  const cases: [string, [number, number], number][] = [
    ['stopped', [0, 0], 0],
    ['steady closing, slow', [0, 0.18], 0],
    ['steady strafing', [0.6, 0], 0],
    ['closing, stick wobbling', [0, 0.18], 0.30],
  ];
  // SEEDS, because a closing run crosses the usable field in a few seconds and a single one
  // landed exactly one ball -- a rate quoted off one ball is not a rate.
  const seeds = [7, 25, 43, 61];
  for (const [name, drive, wobble] of cases) {
    let secs = 0;
    let fired = 0;
    let landed = 0;
    const rpmErrs: number[] = [];
    const radials: number[] = [];
    const longs: number[] = [];
    const lats: number[] = [];
    for (const seed of seeds) {
      const r = await run(name, drive, wobble, seconds, seed);
      secs += r.secs;
      fired += r.fired;
      landed += r.landed;
      if (r.fired) rpmErrs.push(r.rpmErr);
      radials.push(r.radialMps);
      longs.push(...r.longs);
      lats.push(...r.lats);
    }
    const mean = (a: number[]) => (a.length ? a.reduce((x, y) => x + y, 0) / a.length : 0);
    const rate = landed / secs;
    const se = Math.sqrt(Math.max(landed, 1)) / secs;   // Poisson on the count
    const sd = (a: number[]) => {
      if (a.length < 2) return NaN;
      const m2 = mean(a);
      return Math.sqrt(a.reduce((x, v) => x + (v - m2) ** 2, 0) / (a.length - 1));
    };
    const cm = (v: number) => (v * 2.54).toFixed(0);
    console.log(
      `  ${name.padEnd(24)} ${secs.toFixed(0).padStart(5)}   ${(fired / secs).toFixed(2).padStart(7)}   ` +
      `${rate.toFixed(2).padStart(5)} +-${se.toFixed(2)}   ${String(landed).padStart(6)}   ` +
      `${mean(rpmErrs).toFixed(0).padStart(4)} rpm   ` +
      `long ${cm(mean(longs)).padStart(4)}+-${cm(sd(longs)).padStart(3)}   lat ${cm(mean(lats)).padStart(4)}+-${cm(sd(lats)).padStart(3)} cm`,
    );
  }
  console.log('');
  console.log('  If steady motion fires like standing still and only the wobbling case starves,');
  console.log('  then "cannot shoot on the move" was never true -- the budget is acceleration.');
  console.log('');

  // THE LATENCY SWEEP. The wobbling case fires plenty and lands nothing with the wheel dead
  // on its target, which says the target is stale rather than unreachable. If that is right,
  // leading on v + a*tau should recover it, and the best tau should look like the feed delay
  // rather than like a free parameter.
  console.log('  LEADING ON THE PREDICTED RELEASE VELOCITY, accelerating case only');
  console.log(`  (feed pulse ${(robotSpec as unknown as RobotSpec).transfer.feedPulse_s} s + transit ${(robotSpec as unknown as RobotSpec).transfer.feedTransit_s} s)`);
  console.log('');
  console.log('  motion        tau (s)   shots/s   landed/s        landed   rpm err   long error');
  const accelCases: [string, number, number][] = [
    ['wobbling', 0.30, 0],
    ['steady ramp', 0, 0.06],
  ];
  for (const [label, wob, ramp] of accelCases) for (const tau of [0, 0.15, 0.3]) {
    let secs = 0;
    let fired = 0;
    let landed = 0;
    const errs: number[] = [];
    const lg: number[] = [];
    for (const seed of seeds) {
      const r = await run('accel', [0, 0.18], wob, seconds, seed, tau, ramp);
      lg.push(...r.longs);
      secs += r.secs;
      fired += r.fired;
      landed += r.landed;
      if (r.fired) errs.push(r.rpmErr);
    }
    const mean = (a: number[]) => (a.length ? a.reduce((x, y) => x + y, 0) / a.length : 0);
    console.log(
      `  ${label.padEnd(13)} ${tau.toFixed(2).padStart(6)}   ${(fired / secs).toFixed(2).padStart(7)}   ` +
      `${(landed / secs).toFixed(2).padStart(5)} +-${(Math.sqrt(Math.max(landed, 1)) / secs).toFixed(2)}   ` +
      `${String(landed).padStart(6)}   ${mean(errs).toFixed(0).padStart(4)} rpm   ${(mean(lg) * 2.54).toFixed(0).padStart(4)} cm`,
    );
  }
}
