/**
 * THE MOVING SHOT, one ball at a time, with the reason each one missed.
 *
 *   npm run tool -- tools/movingtune.ts [--n 60] [--speed 0.5]
 *   npm run tool -- tools/movingtune.ts --sweep rangeTrim
 *
 * tools/movingfire.ts answers "how many land per second", which is the right question for a
 * design and the wrong one for tuning: it pools cases, ends passes on walls and tips, and a
 * number that moved could have moved for any of those reasons. This fires one ball at a time
 * from a robot that is DRIVING, records where it went and why, and prints the per-shot
 * breakdown -- so a change to a constant can be attributed.
 *
 * WHAT COUNTS AS LANDED is World.landedInUpCell: settled in OUR up CELL, not merely passing
 * through the volume, and not the down CELL or the opponent's hive.
 *
 * WHY A SHOT MISSED is taken at the frame it left, not reconstructed afterwards:
 *
 *   long / lat      where it crossed the mouth plane, signed, from the shot log
 *   rpm at release  the wheel against the target the lead asked for
 *   hood err        the hood against the elevation the lead solved
 *   turret err      the axis against the azimuth the lead solved
 *   v_r             closing speed, because the lead's whole job is to cancel it
 *
 * The robot is placed in front of the opening -- derived from the mouth normal, not typed in,
 * so it follows a TIP -- and driven with the stick held, which is what "shooting on the move"
 * means. It is robot-centric on purpose (the driver's default is field-centric).
 */
import { readFileSync } from 'node:fs';
import params from '../config/params.json' with { type: 'json' };
import robotSpec from '../config/robot.json' with { type: 'json' };
import { World, initPhysics, emptyGamepad } from '../packages/core/src/physics/world.js';
import { BuiltinTeleOp, ShotTable } from '../packages/core/src/robot/builtinTeleOp.js';
import { loadLandCal } from '../packages/core/src/robot/loadCal.js';
import { M_TO_IN, inches } from '../packages/core/src/units.js';
import type { GamepadState, Params, RobotSpec, Vec3 } from '../packages/core/src/types.js';

const table = ShotTable.fromCsv(readFileSync(new URL('../java/teamcode/assets/shottable.csv', import.meta.url), 'utf8'));

export interface Shot {
  long_in: number; lat_in: number; landed: boolean;
  rpmErr: number; hoodErr: number; turretErr: number; vr: number; range_in: number;
  /** Chassis yaw rate at the frame the ball left, deg/s, and the muzzle's own lateral speed. */
  omega: number; muzzleLat: number;
  /** Launch azimuth minus the true bearing to the mouth, degrees, measured at the muzzle. */
  aimErr: number;
  /** Which ball of the run this was, and how many were already sitting in the up CELL. */
  nth: number; inCell: number;
}

export interface Result { shots: Shot[]; fired: number; landed: number; secs: number; cycle_s: number }

/**
 * THE NUMBER THAT MATTERS: of the shots the robot COULD have taken in the time it spent
 * driving in range, how many ended up in the CELL.
 *
 * "96% of shots taken land" and "it shoots nine times out of ten" are different claims, and
 * only the second is what a driver holding the trigger experiences. A gate that refuses
 * nine cycles out of ten scores nothing while reporting a perfect land rate.
 */

const mean = (a: number[]) => (a.length ? a.reduce((x, y) => x + y, 0) / a.length : NaN);
const sd = (a: number[]) => {
  if (a.length < 2) return NaN;
  const m = mean(a);
  return Math.sqrt(a.reduce((x, v) => x + (v - m) ** 2, 0) / (a.length - 1));
};
const cm = (inch: number) => inch * 2.54;

/**
 * One run: place in the shooting sector at `range_in`, hold the stick, fire continuously.
 * `drive` is [strafe, forward] on the robot's own axes; forward is toward the mouth.
 */
export async function run(opts: {
  seed: number; range_in: number; drive: [number, number]; wobble: number; secs: number;
  /** Degrees off the mouth's normal to stand. 0 is square on; the room is off to the side. */
  bearing_deg?: number;
  mutate?: (p: Params, r: RobotSpec) => void;
}): Promise<Result> {
  const p = structuredClone(params) as unknown as Params;
  const spec = structuredClone(robotSpec) as unknown as RobotSpec;
  opts.mutate?.(p, spec);
  spec.flywheel.minLandProb = 0;          // mechanism, not policy: let every shot through
  // A bin of POLLEN and nothing else on the field, the same rig tools/movingfire.ts uses:
  // preloading straight out of the CAD staging pulls the NECTAR back out of the CELLs and
  // the first preload() that refuses one stops the top-up for good, so the hopper stays empty.
  const ammo = Array.from({ length: 80 }, () => ({ kind: 'pollen' as const, pos: [0, -5, 0] as Vec3 }));
  const world = new World({ params: p, robot: spec, staging: ammo, alliance: 'red', seed: opts.seed });
  for (const b of world.balls.balls) world.balls.park(b);
  const brain = new BuiltinTeleOp(spec, table, loadLandCal());

  // IN FRONT OF THE OPENING, AND ON THE FIELD. The mouth normal here is very nearly straight
  // down +Z and the mouth already sits 15.7 in that way, so the wall is only about 42 in out:
  // anything longer has to swing round in X, which is where the room is. Placing naively along
  // the normal put the robot past the wall and every pass ended on its first frame.
  const hive = world.hives.red;
  const mouth = hive.upCellMouthWorld();
  const nrm = hive.upCellMouthNormalWorld();
  const nh = Math.hypot(nrm[0], nrm[2]) || 1;
  const R = inches(opts.range_in);
  const lim = world.geom.halfWidth_m - 0.40;
  // Tangent to the normal, toward whichever side has more field.
  const tx = -nrm[2] / nh;
  const tz = nrm[0] / nh;
  const side = mouth[0] * tx + mouth[2] * tz > 0 ? -1 : 1;
  // A BEARING OFF THE NORMAL, because straight out of this mouth there is almost no field:
  // the opening points down +Z and the wall is 42 in away, so a receding case placed square
  // on has six inches before the pass ends. Off to the side it has the width of the field.
  const th = (opts.bearing_deg ?? 0) * Math.PI / 180;
  const alongN = Math.min(R * Math.cos(th), lim - (mouth[0] * (nrm[0] / nh) + mouth[2] * (nrm[2] / nh)));
  const rest = Math.sqrt(Math.max(0, R * R - alongN * alongN));
  const sx = mouth[0] + (nrm[0] / nh) * alongN + tx * rest * side;
  const sz = mouth[2] + (nrm[2] / nh) * alongN + tz * rest * side;
  world.robot.place([sx, spec.chassis.height_m / 2 + spec.chassis.clearance_m, sz],
    (Math.atan2(mouth[0] - sx, mouth[2] - sz) * 180) / Math.PI);

  let loaded = 0;
  const step = (g: GamepadState) => {
    while (world.robot.heldBalls().length < spec.hopper.capacity && loaded < ammo.length) {
      if (!world.robot.preload(world.balls, world.balls.balls[loaded])) break;
      loaded++;
    }
    world.setGamepads(g, emptyGamepad());
    world.step(brain.update(world.sensors(), g, world.seq));
  };

  const arm = emptyGamepad(); step(arm); arm.a = true; step(arm);
  for (let f = 0; f < 150; f++) step(emptyGamepad());   // spin the wheel up standing still

  const t1 = world.t;
  const hold = (): GamepadState => {
    const g = emptyGamepad();
    g.y = true;                              // robot-centric: drive is written on its own axes
    g.left_stick_x = opts.drive[0];
    const fwd = opts.drive[1] + opts.wobble * Math.sin(2 * Math.PI * 0.5 * (world.t - t1));
    g.left_stick_y = -Math.max(-1, Math.min(1, fwd));
    return g;
  };
  for (let f = 0; f < 30; f++) step(hold());

  // Per-shot state, sampled at the frame the ball leaves.
  const at: { rpmErr: number; hoodErr: number; turretErr: number; vr: number; range: number;
    omega: number; muzzleLat: number; aimErr: number; nth: number; inCell: number }[] = [];
  let seen = 0;
  const t0 = world.t;
  const tips0 = hive.tips;
  for (let f = 0; f < Math.round(opts.secs * 60); f++) {
    const g = hold();
    g.right_bumper = true;
    step(g);
    if (world.robot.shots > seen) {
      seen = world.robot.shots;
      const s = world.sensors();
      const bf = (s.imu.yaw + s.game.upCellAzimuthDeg) * Math.PI / 180;
      // The muzzle's own lateral velocity, and where it was actually POINTED, both measured
      // at the muzzle rather than inferred: turret error is the axis against its command and
      // says nothing about whether the command was right.
      const rp = world.robot.pos;
      const om = world.robot.body.angvel().y;
      const mz = world.robot.muzzle();
      const rx = mz.pos[0] - rp[0];
      const rz = mz.pos[2] - rp[2];
      const dx = mouth[0] - rp[0];
      const dz = mouth[2] - rp[2];
      const dn = Math.hypot(dx, dz) || 1;
      const ux = dx / dn;
      const uz = dz / dn;
      at.push({
        rpmErr: world.robot.lastShotRpm - brain.state.targetRpm,
        hoodErr: brain.state.hoodErrDeg,
        turretErr: brain.state.turretErrDeg,
        vr: s.localizer.vx * 0.0254 * Math.cos(bf) + s.localizer.vy * 0.0254 * Math.sin(bf),
        range: s.game.upCellRangeIn,
        omega: (om * 180) / Math.PI,
        muzzleLat: (-om * rz) * uz - (om * rx) * ux,
        aimErr: ((Math.atan2(mz.dir[0], mz.dir[2]) - Math.atan2(dx, dz)) * 180) / Math.PI,
        nth: seen,
        inCell: hive.ballsInUpCell,
      });
    }
    const r = world.robot.pos;
    const n = Math.hypot(mouth[0] - r[0], mouth[2] - r[2]) * M_TO_IN;
    const wall = world.geom.halfWidth_m - 0.32;
    if (n < 30 || n > 90 || Math.abs(r[0]) > wall || Math.abs(r[2]) > wall || hive.tips > tips0) break;
  }
  const secs = world.t - t0;
  const fired = world.robot.shots;
  for (let f = 0; f < 60 * 5; f++) step(emptyGamepad());   // let the last ball settle

  // PAIR ON THE SHOT'S OWN INDEX, not on its position in a filtered list. `at` has one entry
  // per ball that left; the log is then filtered to drop anything still in flight, and
  // mapping the filtered list against `at` by position shifts every later pairing by one for
  // each ball dropped. That is how a shot came to be reported with 0.3 deg of turret error
  // and 26 cm of lateral miss -- two different balls' numbers on one line.
  const shots: Shot[] = world.snapshot().shots.slice(0, fired)
    .map((x, i) => ({ x, a: at[i] }))
    .filter(({ x }) => x.result !== 'flight')
    .map(({ x, a }) => ({
      long_in: x.long_in, lat_in: x.lat_in, landed: x.result === 'cell',
      rpmErr: a?.rpmErr ?? NaN, hoodErr: a?.hoodErr ?? NaN,
      turretErr: a?.turretErr ?? NaN, vr: a?.vr ?? NaN, range_in: a?.range ?? NaN,
      omega: a?.omega ?? NaN, muzzleLat: a?.muzzleLat ?? NaN, aimErr: a?.aimErr ?? NaN,
      nth: a?.nth ?? NaN, inCell: a?.inCell ?? NaN,
    }));
  return { shots, fired, landed: shots.filter((s) => s.landed).length, secs, cycle_s: spec.transfer.cycleTime_s };
}

/** Pool several seeds of one case. */
export async function pool(
  name: string, drive: [number, number], wobble: number, range_in: number, seeds: number,
  mutate?: (p: Params, r: RobotSpec) => void, bearing_deg = 0,
): Promise<{ name: string; shots: Shot[]; secs: number; cycle_s: number }> {
  const shots: Shot[] = [];
  let secs = 0;
  let cycle_s = 1;
  for (let i = 0; i < seeds; i++) {
    const r = await run({ seed: 7 + i * 18, range_in, drive, wobble, secs: 14, bearing_deg, mutate });
    shots.push(...r.shots);
    secs += r.secs;
    cycle_s = r.cycle_s;
  }
  return { name, shots, secs, cycle_s };
}

function report(rows: { name: string; shots: Shot[]; secs: number; cycle_s: number }[]): void {
  console.log('  case              n   in%   wild  shots/s  landed/s     long cm        lat cm     rpm err');
  for (const r of rows) {
    const s = r.shots;
    if (!s.length) { console.log(`  ${r.name.padEnd(16)} ${String(0).padStart(3)}   (no shots)`); continue; }
    const lng = s.map((x) => cm(x.long_in)).filter(Number.isFinite);
    const lat = s.map((x) => cm(x.lat_in)).filter(Number.isFinite);
    const rate = s.filter((x) => x.landed).length / s.length;
    // WILD: outside the mouth by any reading. The mouth is 20 in across and 14 deep, so a
    // ball more than 20 cm off was never going in and its error is a different animal from
    // the group's -- a mean that mixes the two describes neither.
    const wild = s.filter((x) => Math.abs(x.lat_in) * 2.54 > 20 || Math.abs(x.long_in) * 2.54 > 30).length;
    console.log(
      `  ${r.name.padEnd(16)} ${String(s.length).padStart(3)}  ${(rate * 100).toFixed(0).padStart(4)}%  ${String(wild).padStart(5)}  ` +
      `${(s.length / Math.max(0.1, r.secs)).toFixed(2).padStart(7)}  ${(s.filter((x) => x.landed).length / Math.max(0.1, r.secs)).toFixed(2).padStart(8)}   ` +
      `${mean(lng).toFixed(0).padStart(5)} +-${sd(lng).toFixed(0).padStart(3)}   ` +
      `${mean(lat).toFixed(0).padStart(5)} +-${sd(lat).toFixed(0).padStart(3)}   ` +
      `${mean(s.map((x) => x.rpmErr)).toFixed(0).padStart(6)}`,
    );
  }
}

export async function main(argv: string[] = []): Promise<void> {
  await initPhysics();
  const num = (flag: string, d: number) => {
    const i = argv.indexOf(`--${flag}`);
    return i >= 0 ? Number(argv[i + 1]) : d;
  };
  const seeds = num('seeds', 4);

  console.log('\nTHE MOVING SHOT, per ball. Gate open: this is the mechanism, not the policy.\n');
  // EACH CASE STARTS WHERE IT HAS ROOM TO DO ITS THING. The band is 30-90 in and the field
  // runs out, so a closing case has to start long and a receding one short, or it leaves the
  // band in a second or two and the sample is three balls. The stick values are gentle for
  // the same reason: 0.35 is about 0.45 m/s, which crosses the whole band in nine seconds.
  // name, [strafe, forward], wobble, range_in, bearing off the normal
  const cases: [string, [number, number], number, number, number][] = [
    ['stopped 50in', [0, 0], 0, 50, 30],
    ['closing 0.25', [0, 0.25], 0, 78, 55],
    ['receding 0.25', [0, -0.25], 0, 38, 55],
    ['strafing 0.35', [0.35, 0], 0, 50, 30],
    ['wobbling', [0, 0], 0.35, 50, 30],
  ];
  const rows = [];
  for (const [name, drive, wobble, range, bear] of cases) rows.push(await pool(name, drive, wobble, range, seeds, undefined, bear));
  report(rows);

  const all = rows.flatMap((r) => r.shots);
  const moving = rows.filter((r) => r.name !== 'stopped 50in').flatMap((r) => r.shots);
  const st = rows[0].shots;
  console.log('');
  console.log(`  STOPPED  ${st.length} shots, ${(100 * st.filter((s) => s.landed).length / Math.max(1, st.length)).toFixed(0)}% in, ` +
    `long ${mean(st.map((s) => cm(s.long_in))).toFixed(1)} cm`);
  console.log(`  MOVING   ${moving.length} shots, ${(100 * moving.filter((s) => s.landed).length / Math.max(1, moving.length)).toFixed(0)}% in, ` +
    `long ${mean(moving.map((s) => cm(s.long_in))).toFixed(1)} cm`);
  console.log(`  ALL      ${all.length} shots, ${(100 * all.filter((s) => s.landed).length / Math.max(1, all.length)).toFixed(0)}% in`);
  console.log('');
  console.log('  If MOVING is worse than STOPPED the lead is not cancelling the motion; if both are');
  console.log('  long by the same amount it is a trim, and config/robot.json calibration.rangeTrim_in');
  console.log('  is the knob. Sweep it with --sweep before touching any physical constant.\n');

  // --wild: the shots that were never going in, with everything recorded at the frame they
  // left. A mean that mixes these with the group describes neither, and the useful question
  // is what they SHARE.
  if (argv.includes('--wild')) {
    console.log('WILD SHOTS (more than 20 cm lateral or 30 cm downrange off the mouth centre)');
    console.log('');
    console.log('  case              long cm   lat cm   AIM err   ball #   already in CELL   range');
    for (const r of rows) {
      for (const x of r.shots) {
        if (Math.abs(x.lat_in) * 2.54 <= 20 && Math.abs(x.long_in) * 2.54 <= 30) continue;
        console.log(
          `  ${r.name.padEnd(16)} ${cm(x.long_in).toFixed(0).padStart(6)}   ${cm(x.lat_in).toFixed(0).padStart(6)}   ` +
          `${x.aimErr.toFixed(2).padStart(7)}  ${String(x.nth).padStart(6)}  ${String(x.inCell).padStart(15)}   ${x.range_in.toFixed(0).padStart(5)}`);
      }
    }
    console.log('');
  }

  const sw = argv.indexOf('--sweep');
  if (sw >= 0) {
    console.log('RANGE TRIM SWEEP, inches. The brain looks the table up at (range - trim), so a');
    console.log('positive trim asks for a shorter shot -- which is what a group landing LONG needs.\n');
    console.log('  trim   n   landed      long cm      lat cm');
    for (const trim of [0, 1, 2, 3, 4, 5, 6]) {
      const rs = [];
      for (const [name, drive, wobble, range, bear] of cases) {
        rs.push(await pool(name, drive, wobble, range, Math.max(2, seeds - 1), (_p, r) => { r.calibration.rangeTrim_in = trim; }, bear));
      }
      const s = rs.flatMap((r) => r.shots);
      const lng = s.map((x) => cm(x.long_in)).filter(Number.isFinite);
      const lat = s.map((x) => cm(x.lat_in)).filter(Number.isFinite);
      console.log(`  ${String(trim).padStart(4)}  ${String(s.length).padStart(3)}   ${(100 * s.filter((x) => x.landed).length / Math.max(1, s.length)).toFixed(0).padStart(3)}%   ` +
        `${mean(lng).toFixed(0).padStart(5)} +-${sd(lng).toFixed(0).padStart(3)}   ${mean(lat).toFixed(0).padStart(5)} +-${sd(lat).toFixed(0).padStart(3)}`);
    }
    console.log('');
  }
}
