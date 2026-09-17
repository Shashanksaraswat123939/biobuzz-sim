/**
 * WHERE DOES A MOVING SHOT'S ERROR COME FROM? Per shot, one line per candidate.
 *
 *   npm run tool -- tools/shoterror.ts            # every driving case, broken down
 *   npm run tool -- tools/shoterror.ts --wild     # dump the badly wrong shots, shuttling
 *   npm run tool -- tools/shoterror.ts --wild --turn   # ... and while spinning
 *   npm run tool -- tools/shoterror.ts --tau      # does the release-velocity prediction help?
 *
 * tools/movingfire.ts counts what lands; this asks why the rest did not. It records, at the
 * frame each ball actually leaves, everything that could plausibly move it -- the wheel's
 * speed against its target, the hood's angle against the one the shot needed, the chassis yaw
 * rate, the turret's tracking error, how far past its end stop the aim wanted to be, the
 * muzzle's own velocity from the chassis rotating under it -- and correlates each against
 * where the ball came down.
 *
 * MEDIAN AND IQR SIT NEXT TO MEAN AND SD ON PURPOSE. A ball that never reaches the mouth's
 * height is scored where it stopped rolling, which can be metres away, and a handful of those
 * move a mean a long way while saying nothing about the aim. Three separate faults were found
 * by that column disagreeing with the mean: the shuttling case's typical shot was better than
 * a stationary one while a fifth of its shots were wild, and the wild ones all turned out to
 * share one cause.
 *
 * COUNT ONLY WHAT WAS FIRED WHILE DRIVING. Fire is a latch, so the settling tail at the end of
 * a pass will otherwise pad the sample with shots taken standing still at a range the run had
 * already left -- which is the very fault this tool exists to find, arriving through the back
 * door. It did exactly that in its first version.
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

interface S { rpmErr: number; long: number; lat: number; vr: number; hoodErr: number; sincePrev: number; range: number; omega: number; turretErr: number; aimErr: number; muzzleLat: number; clampedBy: number }

async function run(drive: [number, number], wobble: number, startIn: number, secs: number, seed: number, tau?: number, turn = 0): Promise<S[]> {
  await initPhysics();
  const p = structuredClone(params) as unknown as Params;
  const spec = structuredClone(robotSpec) as unknown as RobotSpec;
  if (tau !== undefined) spec.transfer.leadLatency_s = tau;
  const staging = Array.from({ length: 80 }, () => ({ kind: 'pollen' as const, pos: [0, -5, 0] as Vec3 }));
  const world = new World({ params: p, robot: spec, staging, alliance: 'red', seed });
  for (const b of world.balls.balls) world.balls.park(b);
  const mouth = world.hives.red.upCellMouthWorld();
  const lim = world.geom.halfWidth_m - 0.40;
  const R = inches(startIn);
  const dz = Math.min(R, lim - mouth[2]);
  const dx = Math.sqrt(Math.max(0, R * R - dz * dz));
  const x = Math.min(mouth[0] + dx, lim);
  const z = mouth[2] + dz;
  world.robot.place([x, spec.chassis.height_m / 2 + spec.chassis.clearance_m, z], (Math.atan2(mouth[0] - x, mouth[2] - z) * 180) / Math.PI);
  const brain = new BuiltinTeleOp(spec, table, loadLandCal());
  let loaded = 0;
  const at: Omit<S, 'long' | 'lat'>[] = [];
  let prevT = -99;
  const step = (g: GamepadState) => {
    while (world.robot.heldBalls().length < 7 && loaded < staging.length) {
      if (!world.robot.preload(world.balls, world.balls.balls[loaded])) break;
      loaded++;
    }
    const before = world.robot.shots;
    world.setGamepads(g, emptyGamepad());
    world.step(brain.update(world.sensors(), g, world.seq));
    if (world.robot.shots > before) {
      const r = world.robot.pos;
      const dxm = mouth[0] - r[0];
      const dzm = mouth[2] - r[2];
      const n = Math.hypot(dxm, dzm) || 1;
      const v = world.robot.body.linvel();
      // The muzzle is 0.12 m off the turret axis, so a ROTATING robot throws the ball with
      // v_cg + omega x r. The lead is handed the chassis CG velocity and knows nothing of it.
      // THE ERROR THAT MATTERS FOR A MOVING SHOT is not where the muzzle pointed -- the lead
      // deliberately points it off the bearing -- but where the ball's GROUND velocity pointed.
      // That is what the lead claims to put on the bearing, so if it is off, the lead is off.
      const ls = world.robot.lastShot!;
      const el = (ls.elevDeg * Math.PI) / 180;
      const az = (ls.azDeg * Math.PI) / 180;
      const gx = Math.sin(az) * Math.cos(el) * ls.v_exit + v.x;
      const gz = Math.cos(az) * Math.cos(el) * ls.v_exit + v.z;
      const groundAimErrDeg = ((Math.atan2(gx, gz) - Math.atan2(dxm, dzm)) * 180) / Math.PI;
      const om = world.robot.body.angvel().y;
      const mz = world.robot.muzzle();
      const rx = mz.pos[0] - r[0];
      const rz = mz.pos[2] - r[2];
      const mvx = -om * rz;                    // omega_y x r, horizontal part
      const mvz = om * rx;
      // across the shot line, which is where a lateral miss comes from
      const ux = dxm / n;
      const uz = dzm / n;
      at.push({
        rpmErr: world.robot.lastShotRpm - world.robot.lastTargetRpm,
        vr: (v.x * dxm + v.z * dzm) / n,
        hoodErr: brain.state.hoodErrDeg,
        sincePrev: world.t - prevT,
        range: n * M_TO_IN,
        omega: (om * 180) / Math.PI,
        turretErr: brain.state.turretErrDeg,
        aimErr: groundAimErrDeg,
        muzzleLat: mvx * uz - mvz * ux,
        // How far OUTSIDE its travel the lead wanted the turret. st.turretErrDeg is measured
        // against the CLAMPED command, so it reads zero on an axis pinned at its end stop and
        // the readiness gate cannot tell the difference between aimed and jammed.
        clampedBy: brain.state.turretPastStopDeg,
      });
      prevT = world.t;
    }
  };
  const arm = emptyGamepad(); step(arm); arm.dpad_up = true; step(arm);
  for (let f = 0; f < 150; f++) step(emptyGamepad());
  const t1 = world.t;
  const hold = (): GamepadState => {
    const g = emptyGamepad();
    // Robot-centric: these cases are written relative to the nose of a robot placed facing
    // the hive, not to the field. The driver's default is field-centric.
        g.left_stick_x = drive[0];
    g.left_stick_y = -Math.max(-1, Math.min(1, drive[1] + wobble * Math.sin(2 * Math.PI * 0.5 * (world.t - t1))));
    // A driver swinging the robot while the turret holds the goal: the most ordinary way to
    // be "moving" and the one no case here had ever covered.
    g.right_stick_x = turn * Math.sin(2 * Math.PI * 0.25 * (world.t - t1));
    return g;
  };
  for (let f = 0; f < 45; f++) step(hold());
  const wall = world.geom.halfWidth_m - 0.32;
  for (let f = 0; f < secs * 60; f++) {
    step({ ...hold(), right_bumper: true });
    const r = world.robot.pos;
    const n = Math.hypot(mouth[0] - r[0], mouth[2] - r[2]) * M_TO_IN;
    if (n < 33 || n > 82 || Math.abs(r[0]) > wall || Math.abs(r[2]) > wall) break;
  }
  // Only what was fired IN BAND while driving. The fire latch stays set through the
  // settling tail, so without this the sample is padded with shots taken while coasting to a
  // stop at a range the run had already left -- which is exactly the fault this tool exists
  // to look for, arriving through the back door.
  const firedMoving = world.robot.shots;
  for (let f = 0; f < 60 * 6; f++) step(emptyGamepad());
  const log = world.snapshot().shots.slice(0, firedMoving);
  const out: S[] = [];
  for (let i = 0; i < at.length && i < log.length; i++) {
    if (!Number.isFinite(log[i].long_in)) continue;
    out.push({ ...at[i], long: log[i].long_in * 2.54, lat: log[i].lat_in * 2.54 });
  }
  return out;
}

const mean = (a: number[]) => (a.length ? a.reduce((x, y) => x + y, 0) / a.length : 0);
const sd = (a: number[]) => { const m = mean(a); return a.length < 2 ? NaN : Math.sqrt(a.reduce((x, v) => x + (v - m) ** 2, 0) / (a.length - 1)); };
function fit(x: number[], y: number[]) {
  const mx = mean(x), my = mean(y);
  let sxy = 0, sxx = 0;
  for (let i = 0; i < x.length; i++) { sxy += (x[i] - mx) * (y[i] - my); sxx += (x[i] - mx) ** 2; }
  const slope = sxx ? sxy / sxx : 0;
  const resid = y.map((v, i) => v - (my + slope * (x[i] - mx)));
  return { slope, r: (sd(x) * slope) / (sd(y) || 1), residSd: sd(resid) };
}

export async function main(argv: string[] = []): Promise<void> {
  if (argv.includes('--tau')) {
    console.log('DOES THE RELEASE-VELOCITY PREDICTION THROW THE AIM? shuttling at 50 in.');
    console.log('');
    console.log('  tau     n   ground aim err      lateral miss        downrange');
    for (const t of [0, 0.075, 0.15, 0.3]) {
      const all: S[] = [];
      for (let i = 0; i < 8; i++) all.push(...await run([0, 0], 0.35, 50, 25, 7 + i * 18, t));
      const a = all.map((s) => s.aimErr);
      const l = all.map((s) => s.lat);
      const g = all.map((s) => s.long);
      console.log(
        `  ${t.toFixed(3)} ${String(all.length).padStart(4)}   ${mean(a).toFixed(2).padStart(6)} +-${sd(a).toFixed(2).padStart(5)} deg   ` +
        `${mean(l).toFixed(0).padStart(4)} +-${sd(l).toFixed(0).padStart(3)} cm   ${mean(g).toFixed(0).padStart(4)} +-${sd(g).toFixed(0).padStart(3)} cm`,
      );
    }
    return;
  }
  if (argv.includes('--wild')) {
    const all: S[] = [];
    const turning = argv.includes('--turn');
    for (let i = 0; i < 8; i++) {
      all.push(...(turning
        ? await run([0, 0], 0, 50, 25, 7 + i * 18, undefined, 0.5)
        : await run([0, 0], 0.35, 50, 25, 7 + i * 18, undefined, 0)));
    }
    const wild = all.filter((x) => Math.abs(x.long) > 60 || Math.abs(x.lat) > 60);
    console.log(`WILD ${turning ? 'TURNING' : 'SHUTTLING'} SHOTS: ${wild.length} of ${all.length}`);
      console.log('   long   lat   rpmErr  hoodErr    v_r   range  sincePrev  pastStop  yawRate  turretErr');
    for (const w of wild) {
      console.log(
        `  ${w.long.toFixed(0).padStart(5)} ${w.lat.toFixed(0).padStart(5)}  ${w.rpmErr.toFixed(0).padStart(6)}  ` +
        `${w.hoodErr.toFixed(2).padStart(6)} ${w.vr.toFixed(2).padStart(6)} ${w.range.toFixed(0).padStart(6)}  ` +
        `${w.sincePrev.toFixed(2).padStart(8)}  ${w.clampedBy.toFixed(1).padStart(7)}  ${w.omega.toFixed(0).padStart(7)}  ${w.turretErr.toFixed(2).padStart(9)}`,
      );
    }
    const ok = all.filter((x) => !wild.includes(x));
    const f = (a: S[], k: keyof S) => `${mean(a.map((x) => x[k] as number)).toFixed(2)} +-${sd(a.map((x) => x[k] as number)).toFixed(2)}`;
    console.log('');
    console.log(`  wild  rpmErr ${f(wild, 'rpmErr')}   yawRate ${f(wild, 'omega')}   turretErr ${f(wild, 'turretErr')}   pastStop ${f(wild, 'clampedBy')}`);
    console.log(`  good  rpmErr ${f(ok, 'rpmErr')}   yawRate ${f(ok, 'omega')}   turretErr ${f(ok, 'turretErr')}   pastStop ${f(ok, 'clampedBy')}`);
    return;
  }
  const cases: [string, [number, number], number, number, number][] = [
    ['stopped 50in', [0, 0], 0, 50, 0],
    ['shuttling 50in', [0, 0], 0.35, 50, 0],
    ['closing 76in', [0, 0.18], 0, 76, 0],
    ['turning 50in', [0, 0], 0, 50, 0.5],
    ['turn+drive 50in', [0, 0], 0.25, 50, 0.4],
  ];
  console.log('WHAT MOVES THE DOWNRANGE ERROR? per-shot, gate at its configured value.');
  console.log('');
  console.log('  case              n   rpm err at release   long cm     long vs rpm err     residual');
  for (const [name, drive, wobble, start, turn] of cases) {
    const all: S[] = [];
    for (let i = 0; i < 8; i++) all.push(...await run(drive, wobble, start, 25, 7 + i * 18, undefined, turn));
    if (all.length < 4) { console.log(`  ${name.padEnd(16)} ${String(all.length).padStart(3)}  (too few shots)`); continue; }
    const lng = all.map((s) => s.long);
    const lat = all.map((s) => s.lat);
    const cols: [string, number[], number[]][] = [
      ['rpmErr', all.map((s) => s.rpmErr), lng],
      ['hoodErr', all.map((s) => s.hoodErr), lng],
      ['range', all.map((s) => s.range), lng],
      ['yaw rate', all.map((s) => s.omega), lat],
      ['turretErr', all.map((s) => s.turretErr), lat],
      ['muzzle v_lat', all.map((s) => s.muzzleLat), lat],
      ['ground aim', all.map((s) => s.aimErr), lat],
      ['past end stop', all.map((s) => s.clampedBy), lat],
    ];
    // MEDIAN AND IQR ALONGSIDE, because a shot that never reaches the mouth's height is
    // scored where it STOPPED ROLLING, which can be metres out. A handful of those move a
    // mean and a standard deviation a long way and say nothing about the aim.
    const q = (a: number[], f: number) => { const b = [...a].sort((x, y) => x - y); return b[Math.min(b.length - 1, Math.floor(f * b.length))]; };
    const far = all.filter((s) => Math.abs(s.long) > 60 || Math.abs(s.lat) > 60).length;
    console.log(
      `  ${name.padEnd(16)} n=${String(all.length).padStart(3)}   long ${mean(lng).toFixed(0).padStart(4)} +-${sd(lng).toFixed(0).padStart(3)}  med ${q(lng, 0.5).toFixed(0).padStart(4)} [${q(lng, 0.25).toFixed(0)},${q(lng, 0.75).toFixed(0)}]   ` +
      `lat ${mean(all.map((s) => s.lat)).toFixed(0).padStart(4)} +-${sd(all.map((s) => s.lat)).toFixed(0).padStart(3)}  med ${q(all.map((s) => s.lat), 0.5).toFixed(0).padStart(4)} [${q(all.map((s) => s.lat), 0.25).toFixed(0)},${q(all.map((s) => s.lat), 0.75).toFixed(0)}]   ` +
      `wild ${far}`,
    );
    for (const [k, v, y] of cols) {
      const f = fit(v, y);
      const which = y === lng ? 'long' : 'lat ';
      console.log(`      ${k.padEnd(13)} ${mean(v).toFixed(2).padStart(8)} +-${sd(v).toFixed(2).padStart(7)}   vs ${which} r=${f.r.toFixed(2).padStart(5)}   leaves +-${f.residSd.toFixed(0)} cm`);
    }
  }
  console.log('');
  console.log('  If the slope is real and the residual is much smaller than the raw spread, the');
  console.log('  wheel speed at release IS the error and everything else is noise around it.');
}
