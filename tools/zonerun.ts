/**
 * DRIVING THE GREEN ZONE AT SPEED, with the latch on: does it shoot, and does it score?
 *
 *   npm run tool -- tools/zonerun.ts [--secs 60] [--seeds 3]
 *
 * The other moving harnesses cross the band once and stop, so at speed they get three balls
 * off and end on a wall. This is the thing a driver actually does: sweep back and forth
 * across the front of the mouth, inside the shooting sector, at a held speed, and let the
 * latch fire whenever the gate clears. The rocker is the real one -- it tips when it tips
 * and the tip's balls are credited, as in the game -- so this is the number that means
 * "shots per match", not "shots into a bucket".
 *
 * Reported per speed: how fast it actually went, shots taken, balls credited, the land rate,
 * balls per second, and why the gate held when it held.
 */
import params from '../config/params.json' with { type: 'json' };
import robotSpec from '../config/robot.json' with { type: 'json' };
import { readFileSync } from 'node:fs';
import { World, initPhysics, emptyGamepad } from '../packages/core/src/physics/world.js';
import { BuiltinTeleOp, ShotTable } from '../packages/core/src/robot/builtinTeleOp.js';
import { loadLandCal } from '../packages/core/src/robot/loadCal.js';
import { HoodTable } from '../packages/core/src/robot/hoodTable.js';
import { RAD, inches, wrapPi } from '../packages/core/src/units.js';
import type { GamepadState, Params, RobotSpec, Vec3 } from '../packages/core/src/types.js';

const table = ShotTable.fromCsv(readFileSync(new URL('../java/teamcode/assets/shottable.csv', import.meta.url), 'utf8'));

interface Run { speed: number; shots: number; credited: number; secs: number; why: Record<string, number>; long: number[]; lat: number[] }

async function run(stick: number, secs: number, seed: number, leadCap?: number): Promise<Run> {
  await initPhysics();
  const p = structuredClone(params) as unknown as Params;
  const spec = structuredClone(robotSpec) as unknown as RobotSpec;
  if (leadCap !== undefined) spec.turret.fireLeadCap_deg = leadCap;
  // What the remaining misses at speed are made of: the physics ceiling with the launch
  // scatter off, a stricter gate, and an aim point deeper in the pocket.
  const arg = (k: string) => { const i = process.argv.indexOf(`--${k}`); return i >= 0 ? Number(process.argv[i + 1]) : undefined; };
  if (process.argv.includes('--scatter0')) spec.flywheel.scatter = { angle_deg: 0, yaw_deg: 0, speedFrac: 0 };
  if (arg('minp') !== undefined) spec.flywheel.minLandProb = arg('minp')!;
  if (arg('depth') !== undefined) p.hive.aimDepthFrac = arg('depth')!;
  const pool = Array.from({ length: 300 }, () => ({ kind: 'pollen' as const, pos: [0, -5, 0] as Vec3 }));
  const w = new World({ params: p, robot: spec, staging: pool, alliance: 'red', seed });
  for (const b of w.balls.balls) w.balls.park(b);
  const mouth = w.hives.red.upCellMouthWorld();
  // The mouth opens toward +Z and the wall is 39 in behind it, so a +-30 deg sector -- the
  // one STRATEGY.md section 3 gives -- exists only inside about 45 in. 42 in, then patrol
  // ACROSS the mouth between +-30 deg of its axis.
  const nrm: Vec3 = [0, 0, 1];
  const R = inches(38);
  // A pass starts at the sector's far edge so the whole crossing is at speed.
  // -75 deg: outside the opening, so the first 0.5 s is a run-up and the robot enters the
  // sector at its full 1.56 m/s instead of accelerating through it.
  // A pass is a straight line 38 in in front of the mouth, x from -80 in to +80 in: the
  // first 15 in are outside the opening (a run-up to full speed), the middle is the sector.
  const isPass = process.argv.includes('--pass');
  const x0 = isPass ? mouth[0] - inches(80) : mouth[0];
  const z0 = mouth[2] + R;
  w.robot.place([x0, spec.chassis.height_m / 2 + spec.chassis.clearance_m, z0], process.argv.includes('--pass') ? 180 : Math.atan2(mouth[0] - x0, mouth[2] - z0) * RAD);
  // --hood: the FIXED-SPEED shooter. The wheel holds one speed all match and the hood
  // aims, so the robot's own velocity is an axis of the table instead of something the
  // flywheel has to chase -- which is the whole question at 1.57 m/s, where the wheel
  // needs +300 rpm while receding and slews at 1100 rpm/s.
  const hood = process.argv.includes('--hood')
    ? HoodTable.fromCsv(readFileSync(new URL('../java/teamcode/assets/hoodtable.csv', import.meta.url), 'utf8'))
    : null;
  const brain = new BuiltinTeleOp(spec, table, loadLandCal(), hood);
  brain.state.firing = true;
  let loaded = 0;
  let dir = 1;
  const why: Record<string, number> = {};
  let dist = 0;
  let prev: Vec3 = w.robot.pos;
  const dt = 1 / 60;
  let ran = 0;
  let inSector = 0;
  let shotsSeen = 0;
  for (let i = 0; i < secs * 60; i++) {
    // A TIP swaps which CELL is up and the new mouth faces the other way; the driver's next
    // job is to drive round, which is navigation, not aim. The pass ends there and the
    // rates are per second actually spent in front of an open mouth.
    if (w.hives.red.tips > 0) break;
    ran = i + 1;
    // Top up every loop from the under-floor pool: a 6 s pass cannot also be a test of
    // how fast four balls run out, and 'HOPPER EMPTY' was 28% of the full-stick loops.
    while (w.robot.heldBalls().length < spec.hopper.capacity && loaded < pool.length) {
      if (!w.robot.preload(w.balls, w.balls.balls[loaded])) break;
      loaded++;
    }
    // Field-frame patrol: hold the stand-off, move across the mouth, turn round at the
    // sector's edge. The chassis keeps facing the mouth so this is robot-centric strafe
    // plus a small radial correction, which is how a driver holds a range.
    const r = w.robot.pos;
    const dx = mouth[0] - r[0], dz = mouth[2] - r[2];
    const range = Math.hypot(dx, dz);
    const bearingOff = wrapPi(Math.atan2(dx, dz) - Math.atan2(nrm[0], nrm[2]) + Math.PI) * RAD;
    // Turn round at the sector's edge, once, with hysteresis: flipping every frame past the
    // edge is a robot that stands still and reads as 0.04 m/s.
    // --pass: ONE crossing at full stick, starting at the sector's edge and ending at the
    // other, which is the 0.7 s a driver actually gets at 1.2 m/s. A patrol cannot reach
    // that speed -- a mecanum takes half a second to reverse, so it oscillates at the edge.
    if (isPass && r[0] > mouth[0] + inches(80)) break;
    const edge = process.argv.includes('--nohold') ? 45 : 30;
    if (bearingOff < -edge) dir = -1;
    else if (bearingOff > edge) dir = 1;
    const g: GamepadState = emptyGamepad();
    // Face the mouth: a yaw correction the fire gate can live with.
    const yawErr = wrapPi(Math.atan2(dx, dz) - w.robot.yaw) * RAD;
    // In a pass the chassis does NOT turn: it faces the hive's side and strafes straight
    // across, which is what the turret is for, and the only way a mecanum reaches its full
    // 1.56 m/s -- any yaw share in the mixer scales the strafe down, and the arc-following
    // correction held the crossings at 0.97 m/s.
    g.right_stick_x = process.argv.includes('--pass') ? 0 : -Math.max(-0.25, Math.min(0.25, yawErr / 40));
    // A pass waits 1.5 s with the wheel spinning up before it moves, as a driver arriving
    // with the shooter already armed would; otherwise the first crossing is the spin-up.
    g.left_stick_x = isPass ? (i < 90 ? 0 : stick) : -dir * stick;   // strafe across (a pass: one way, +x)
    // --nohold: no range correction, so a full stick is a full stick (about 1.2 m/s), the
    // sector is +-45 deg, and each pass across it is the 0.7 s a driver actually gets.
    g.left_stick_y = process.argv.includes('--nohold') ? 0 : -Math.max(-0.3, Math.min(0.3, (range - R) / 0.5));
    w.setGamepads(g, emptyGamepad());
    w.step(brain.update(w.sensors(), g, w.seq, dt));
    const h = brain.state.hold;
    const k = h ? h.replace(/-?[\d.]+/g, 'N') : (w.sensors().game.hopper === 0 ? 'HOPPER EMPTY' : 'clear to fire');
    why[k] = (why[k] ?? 0) + 1;
    if (process.argv.includes('--shotlog') && w.robot.shots > shotsSeen) {
      shotsSeen = w.robot.shots;
      const st = brain.state;
      const ls = w.robot.lastShot!;
      console.log(`    shot ${shotsSeen}  rng ${w.sensors().game.upCellRangeIn.toFixed(0).padStart(3)}  open ${w.sensors().game.upCellOpenDeg.toFixed(0).padStart(3)}  v ${Math.hypot(w.robot.body.linvel().x, w.robot.body.linvel().z).toFixed(2)}  exit ${ls.v_exit.toFixed(2)} want ${st.leadSpeed.toFixed(2)}  hood ${w.robot.hoodAngle.toFixed(1)} want ${st.leadElevDeg.toFixed(1)}  rpm ${w.robot.lastShotRpm.toFixed(0)}/${st.targetRpm.toFixed(0)}  lead ${st.leadDeg.toFixed(0)}  pLand ${st.pLand.toFixed(2)} pSpeed ${st.pSpeed.toFixed(2)} pAim ${st.pAim.toFixed(2)}`);
    }
    const q = w.robot.pos;
    if (!process.argv.includes('--pass') || Math.abs(bearingOff) <= 60) { dist += Math.hypot(q[0] - prev[0], q[2] - prev[2]); inSector++; }
    if (process.argv.includes('--aimtrace') && i % 6 === 0 && i < 60 * 8) {
      const st = brain.state;
      console.log(`    t=${(i / 60).toFixed(2)} v=${Math.hypot(w.robot.body.linvel().x, w.robot.body.linvel().z).toFixed(2)} yaw=${(w.robot.body.angvel().y * RAD).toFixed(0).padStart(4)}dps bOff=${bearingOff.toFixed(0).padStart(4)} aimErr=${st.turretAimErrDeg.toFixed(1).padStart(5)} servoErr=${st.turretErrDeg.toFixed(1).padStart(5)} lead=${st.leadDeg.toFixed(1).padStart(5)} vr=${st.vRadial.toFixed(2)} hold=${st.hold || '-'}`);
    }
    if (process.argv.includes('--trace') && i % 60 === 0) {
      const sg = w.sensors().game;
      console.log(`    t=${(i / 60).toFixed(0)} at (${(q[0] * 39.37).toFixed(0)},${(q[2] * 39.37).toFixed(0)}) in  rng ${sg.upCellRangeIn.toFixed(0)}  open ${sg.upCellOpenDeg.toFixed(0)}  yaw ${(w.robot.yaw * RAD).toFixed(0)}  bearingOff ${bearingOff.toFixed(0)}  yawErr ${yawErr.toFixed(0)}  hold ${brain.state.hold || '-'}  shots ${w.robot.shots} cred ${w.landedInUpCell('red')}`);
    }
    prev = q;
  }
  for (let k = 0; k < 60 * 5; k++) { w.setGamepads(emptyGamepad(), emptyGamepad()); w.step(brain.update(w.sensors(), emptyGamepad(), w.seq, dt)); }
  const log = w.snapshot().shots;
  return {
    speed: dist / Math.max(1e-6, (process.argv.includes('--pass') ? inSector : ran) / 60), shots: w.robot.shots, credited: w.landedInUpCell('red'), secs: ran / 60, why,
    long: log.map((s) => s.long_in * 2.54).filter(Number.isFinite),
    lat: log.map((s) => s.lat_in * 2.54).filter(Number.isFinite),
  };
}

const mean = (a: number[]) => (a.length ? a.reduce((x, y) => x + y, 0) / a.length : NaN);
const sd = (a: number[]) => { const m = mean(a); return a.length < 2 ? NaN : Math.sqrt(a.reduce((x, v) => x + (v - m) ** 2, 0) / (a.length - 1)); };

export async function main(argv: string[] = []): Promise<void> {
  const num = (k: string, d: number) => { const i = argv.indexOf(`--${k}`); return i >= 0 ? Number(argv[i + 1]) : d; };
  const secs = num('secs', 60), seeds = num('seeds', 3);
  console.log(`\nPATROLLING THE SHOOTING SECTOR AT SPEED, latch on, ${secs} s x ${seeds} seeds per speed, real rocker.\n`);
  // --cap: sweep the motion-lead cap where it actually bites. tools/movingtune.ts --lead
  // never reaches 12 deg of lead in any of its cases, so it reads the same 106 shots at
  // every cap; the patrol at 0.6-0.85 m/s is refused 89% of the time by the cap at 20.
  const capSweep = argv.includes('--cap');
  const grid: [number, number | undefined][] = capSweep
    ? [0.5, 0.7, 1.0].flatMap((st) => [20, 30, 45, 90].map((c) => [st, c] as [number, number]))
    : (argv.includes('--only') ? [Number(argv[argv.indexOf('--only') + 1])] : [0, 0.3, 0.5, 0.7, 0.9, 1.0]).map((st) => [st, undefined] as [number, undefined]);
  console.log(`  stick ${capSweep ? ' cap ' : ''}  actual m/s   shots   credited   land%   balls/s   long cm        lat cm`);
  for (const [stick, cap] of grid) {
    const rs: Run[] = [];
    for (let s = 0; s < seeds; s++) rs.push(await run(stick, secs, 11 + s * 7, cap));
    const shots = rs.reduce((a, r) => a + r.shots, 0), cred = rs.reduce((a, r) => a + r.credited, 0);
    const long = rs.flatMap((r) => r.long), lat = rs.flatMap((r) => r.lat);
    const ranSecs = rs.reduce((a, r) => a + r.secs, 0);
    console.log(`  ${stick.toFixed(1).padStart(5)} ${cap === undefined ? '' : String(cap).padStart(4) + ' '}  ${mean(rs.map((r) => r.speed)).toFixed(2).padStart(10)}   ${String(shots).padStart(5)}   ${String(cred).padStart(8)}   ${((cred / Math.max(1, shots)) * 100).toFixed(0).padStart(4)}%   ${(cred / Math.max(1, ranSecs)).toFixed(2).padStart(7)}   ${mean(long).toFixed(0).padStart(4)} +-${sd(long).toFixed(0).padStart(3)}   ${mean(lat).toFixed(0).padStart(4)} +-${sd(lat).toFixed(0).padStart(3)}`);
    const agg: Record<string, number> = {};
    let tot = 0;
    for (const r of rs) for (const [k, v] of Object.entries(r.why)) { agg[k] = (agg[k] ?? 0) + v; tot += v; }
    const top = Object.entries(agg).sort((a, b) => b[1] - a[1]).slice(0, 3).map(([k, v]) => `${((v / tot) * 100).toFixed(0)}% ${k}`);
    console.log(`          ${top.join('   |   ')}`);
  }
  console.log('');
}
