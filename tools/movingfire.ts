/**
 * Can this robot shoot while it is moving, and while it is CHANGING speed?
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
 * only CHANGING the closing speed moves it. That budget mattered when the lead spent the whole
 * correction on the flywheel; it solves the hood too now, and the wheel's share of +-0.8 m/s
 * at 40 in is 2241-2477 rpm rather than 1283-3388.
 *
 * THE FIELD IS THIS HARNESS'S REAL CONSTRAINT, and two versions of it were beaten by that
 * before the numbers meant anything. It is 3.59 m square with the goal near the middle, so NO
 * direction sustains steady motion for twenty seconds: every long run ends against a wall,
 * and a robot pinned on a wall is a stopped robot being scored as a moving one. So a case
 * here is a series of short PASSES, pooled: place, settle, drive, and stop counting the
 * moment the robot leaves the band where the question makes sense. The band is the shot
 * table's own range -- its first row is 30 in, and below that every lookup returns the 30 in
 * answer, which is not the robot missing, it is the robot being asked a question the table
 * cannot answer -- and the field, less a margin, so a wall-pinned pass ends instead of
 * quietly turning into the stopped case.
 *
 * The accelerating case needs no runway at all: a fore/aft shuttle about one spot moves at
 * +-0.5 m/s and accelerates at +-1.6 m/s^2 indefinitely inside a 30 cm band.
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

/** The shot table's own coverage, less a margin at the near end. */
const BAND_IN: [number, number] = [33, 82];

interface Run {
  fired: number; radialMps: number; radialAccel: number; rpmErr: number;
  landed: number; secs: number;
  /** Where the misses actually went, at the mouth plane. This is the diagnostic. */
  longs: number[]; lats: number[]; ranges: number[];
}

interface Case {
  name: string;
  /** What the stick is held at: [strafe, forward]. Forward is toward the hive. */
  drive: [number, number];
  /** Stick amplitude of a 0.5 Hz oscillation on forward, to spend the acceleration budget. */
  wobble: number;
  /** A steady change of stick per second: constant acceleration. */
  ramp: number;
  /** Where the pass starts, in inches from the CELL mouth. */
  start_in: number;
}

/**
 * Stand the robot `range_in` from the up CELL's mouth, ON THE SIDE THE MOUTH OPENS TO, facing
 * it. The pocket faces +Z, so the far corner of the field is not "more range", it is the back
 * of the goal: placed there, the STOPPED control landed two shots in eighty seconds. Straight
 * down +Z the wall stops the robot at 41 in, so anything longer swings round in X, which is
 * where the room is.
 */
function placeAt(world: World, spec: RobotSpec, range_in: number): void {
  const mouth = world.hives.red.upCellMouthWorld();
  const lim = world.geom.halfWidth_m - 0.40;
  const R = inches(range_in);
  const dz = Math.min(R, lim - mouth[2]);
  const dx = Math.sqrt(Math.max(0, R * R - dz * dz));
  const x = Math.min(mouth[0] + dx, lim);
  const z = mouth[2] + dz;
  world.robot.place(
    [x, spec.chassis.height_m / 2 + spec.chassis.clearance_m, z],
    // Heading, FTC degrees: local forward is world +Z at yaw 0, so atan2 of the vector to the
    // mouth points the robot at it and "forward" on the stick closes.
    (Math.atan2(mouth[0] - x, mouth[2] - z) * 180) / Math.PI,
  );
}

/** One pass: place, settle, fire while in band. Returns only what happened in band. */
async function pass(c: Case, seed: number, maxSeconds: number, tau?: number): Promise<Run> {
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
  placeAt(world, spec, c.start_in);
  // Looser than placeAt's margin ON PURPOSE: placing the robot exactly on the band's edge
  // ended every pass on its first frame, and every case read zero.
  const wall = world.geom.halfWidth_m - 0.32;
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

  // SPIN THE WHEEL UP STANDING STILL, THEN settle the drive. One combined settle with the
  // case's own stick held spent the whole band before the first shot: the flywheel needs
  // 2400/1102 = 2.2 s to reach speed, and 2.2 s of strafing is 1.5 m, which is most of the
  // field. The strafing case read zero seconds because of exactly this.
  for (let f = 0; f < 150; f++) step(emptyGamepad());
  const t1 = world.t;
  const hold = (t: number): GamepadState => {
    const g = emptyGamepad();
    g.left_stick_x = c.drive[0];
    // `ramp` is a steady change of stick per second: constant acceleration, the case a
    // first-order predictor is actually built for. `wobble` oscillates instead, and an
    // oscillation defeats v + a*tau at exactly the moments the gate likes -- at a velocity
    // peak the acceleration is zero, so the predictor says "it will stay here" one instant
    // before it reverses.
    const stick = c.drive[1] + c.ramp * (t - t1) + c.wobble * Math.sin(2 * Math.PI * 0.5 * (t - t1));
    g.left_stick_y = -Math.max(-1, Math.min(1, stick));                       // -y is forward
    return g;
  };
  for (let f = 0; f < 45; f++) step(hold(world.t));

  const fire = (t: number): GamepadState => ({ ...hold(t), right_bumper: true });
  const radial: number[] = [];
  const ranges: number[] = [];
  let prevRadial = 0;
  const accels: number[] = [];
  const t0 = world.t;
  for (let f = 0; f < Math.round(maxSeconds * 60); f++) {
    step(fire(world.t));
    const r = world.robot.pos;
    const dx = mouth[0] - r[0];
    const dz = mouth[2] - r[2];
    const n = Math.hypot(dx, dz) || 1;
    const v = world.robot.body.linvel();
    const vr = (v.x * dx + v.z * dz) / n;             // closing speed, + is toward the mouth
    radial.push(vr);
    ranges.push(n * M_TO_IN);
    if (f > 0) accels.push(Math.abs(vr - prevRadial) * 60);
    prevRadial = vr;
    const inBand = n * M_TO_IN > BAND_IN[0] && n * M_TO_IN < BAND_IN[1]
      && Math.abs(r[0]) < wall && Math.abs(r[2]) < wall;
    if (!inBand) break;
  }
  // WHAT COUNTS IS WHAT WAS FIRED WHILE DRIVING. Fire is a latch, so leaving it set through
  // the settling tail let a closing pass take nearly all of its shots standing still after
  // the pass had ended, and be scored as though it had taken them on the move.
  const secs = world.t - t0;
  const firedMoving = world.robot.shots;
  step({ ...hold(world.t), right_bumper: true });     // same level as the loop: no edge
  step(hold(world.t));                               // release: the latch clears
  for (let f = 0; f < 60 * 6; f++) step(emptyGamepad());

  const log = world.snapshot().shots.slice(0, firedMoving).filter((s) => s.result !== 'flight');
  const avg = (a: number[]) => (a.length ? a.reduce((x, y) => x + y, 0) / a.length : 0);
  return {
    longs: log.map((x) => x.long_in).filter(Number.isFinite),
    lats: log.map((x) => x.lat_in).filter(Number.isFinite),
    ranges,
    secs,
    fired: firedMoving,
    radialMps: avg(radial),
    radialAccel: avg(accels),
    rpmErr: avg(log.map((s) => Math.abs(s.rpm - s.targetRpm))),
    landed: log.filter((s) => s.result === 'cell').length,
  };
}

const mean = (a: number[]) => (a.length ? a.reduce((x, y) => x + y, 0) / a.length : 0);
const sd = (a: number[]) => {
  if (a.length < 2) return NaN;
  const m = mean(a);
  return Math.sqrt(a.reduce((x, v) => x + (v - m) ** 2, 0) / (a.length - 1));
};
const cm = (v: number) => (v * 2.54).toFixed(0);

/** Pool passes, each on its own seed, until `seconds` of IN-BAND time has accumulated. */
async function pool(c: Case, seconds: number, tau?: number) {
  let secs = 0;
  let fired = 0;
  let landed = 0;
  const rpmErrs: number[] = [];
  const longs: number[] = [];
  const lats: number[] = [];
  const vr: number[] = [];
  const acc: number[] = [];
  const ranges: number[] = [];
  for (let i = 0; secs < seconds && i < 24; i++) {
    const r = await pass(c, 7 + i * 18, seconds - secs, tau);
    if (r.secs < 0.2) break;                          // a pass that cannot even start
    secs += r.secs;
    fired += r.fired;
    landed += r.landed;
    if (r.fired) rpmErrs.push(r.rpmErr);
    longs.push(...r.longs);
    lats.push(...r.lats);
    vr.push(r.radialMps);
    acc.push(r.radialAccel);
    ranges.push(...r.ranges);
  }
  return { secs, fired, landed, rpmErr: mean(rpmErrs), longs, lats, vr: mean(vr), acc: mean(acc), range: mean(ranges) };
}

export async function main(argv: string[] = []): Promise<void> {
  const i = argv.indexOf('--seconds');
  const seconds = i >= 0 ? Number(argv[i + 1]) : 20;

  console.log('SHOOTING ON THE MOVE — is it the speed, or the change in speed?');
  console.log(`  ${seconds} s of IN-BAND time per case, pooled over passes; band ${BAND_IN[0]}-${BAND_IN[1]} in.`);
  console.log('');
  console.log('  case                secs  range    v_r  |a_r|   shots/s   landed/s        landed   rpm err   where the shots went');
  const cases: Case[] = [
    { name: 'stopped', drive: [0, 0], wobble: 0, ramp: 0, start_in: 50 },
    { name: 'steady closing', drive: [0, 0.18], wobble: 0, ramp: 0, start_in: 76 },
    // Straight down +Z: the only pose with 1.7 m of lateral room either side.
    { name: 'steady strafing', drive: [0.6, 0], wobble: 0, ramp: 0, start_in: 41 },
    { name: 'fore/aft shuttle', drive: [0, 0], wobble: 0.35, ramp: 0, start_in: 50 },
    { name: 'closing, wobbling', drive: [0, 0.18], wobble: 0.30, ramp: 0, start_in: 76 },
  ];
  for (const c of cases) {
    const r = await pool(c, seconds);
    const se = Math.sqrt(Math.max(r.landed, 1)) / r.secs;   // Poisson on the count
    console.log(
      `  ${c.name.padEnd(19)} ${r.secs.toFixed(0).padStart(4)}  ${r.range.toFixed(0).padStart(5)}` +
      ` ${r.vr.toFixed(2).padStart(6)} ${r.acc.toFixed(2).padStart(6)}   ` +
      `${(r.fired / r.secs).toFixed(2).padStart(7)}   ` +
      `${(r.landed / r.secs).toFixed(2).padStart(5)} +-${se.toFixed(2)}   ${String(r.landed).padStart(6)}   ` +
      `${r.rpmErr.toFixed(0).padStart(4)} rpm   ` +
      `long ${cm(mean(r.longs)).padStart(4)}+-${cm(sd(r.longs)).padStart(3)}   lat ${cm(mean(r.lats)).padStart(4)}+-${cm(sd(r.lats)).padStart(3)} cm`,
    );
  }
  console.log('');
  console.log('  Every case should now land at about the stopped rate, and the long bias should be');
  console.log('  within a pocket depth. Before the lead solved the HOOD as well as the speed and the');
  console.log('  azimuth it was +38 to +104 cm on the move -- see tools/leadcheck.ts, which prices');
  console.log('  the same error with no scatter and no gate in the way.');
  console.log('');

  // THE LATENCY SWEEP. If an accelerating case fires plenty and lands nothing with the wheel
  // dead on its target, the target is stale rather than unreachable -- and leading on
  // v + a*tau should recover it, with the best tau looking like the feed delay rather than
  // like a free parameter.
  console.log('  LEADING ON THE PREDICTED RELEASE VELOCITY, accelerating cases only');
  console.log(`  (feed pulse ${(robotSpec as unknown as RobotSpec).transfer.feedPulse_s} s + transit ${(robotSpec as unknown as RobotSpec).transfer.feedTransit_s} s)`);
  console.log('');
  console.log('  motion            tau (s)   shots/s   landed/s        landed   rpm err   long error');
  const accelCases: Case[] = [
    { name: 'fore/aft shuttle', drive: [0, 0], wobble: 0.35, ramp: 0, start_in: 50 },
    { name: 'steady ramp', drive: [0, 0], wobble: 0, ramp: 0.06, start_in: 76 },
  ];
  for (const c of accelCases) for (const tau of [0, 0.15, 0.3]) {
    const r = await pool(c, seconds, tau);
    console.log(
      `  ${c.name.padEnd(17)} ${tau.toFixed(2).padStart(6)}   ${(r.fired / r.secs).toFixed(2).padStart(7)}   ` +
      `${(r.landed / r.secs).toFixed(2).padStart(5)} +-${(Math.sqrt(Math.max(r.landed, 1)) / r.secs).toFixed(2)}   ` +
      `${String(r.landed).padStart(6)}   ${r.rpmErr.toFixed(0).padStart(4)} rpm   ${cm(mean(r.longs)).padStart(4)} cm`,
    );
  }
}
