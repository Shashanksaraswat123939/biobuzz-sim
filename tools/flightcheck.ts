/**
 * WHERE DOES THE HEIGHT ERROR COME FROM? The flight, split into its parts.
 *
 *   npm run tool -- tools/flightcheck.ts [--shots 6]
 *
 * A ball that arrives at the wrong height has passed through four places it could have gone
 * wrong, and every other tool measures the sum. This measures each one, with scatter and
 * ball variance switched OFF so the only thing left is systematic:
 *
 *   1. LAUNCH GEOMETRY  where the ball actually starts, against where the table assumes.
 *   2. INTEGRATOR       the world (Rapier, 1/240 s, its own angular damping) against
 *                       simulateShot (1/480 s, exp spin decay) FROM THE SAME INITIAL STATE.
 *                       Any difference here is the physics disagreeing with itself.
 *   3. EXECUTION        the launch the ball got (speed, elevation, ground velocity) against
 *                       the launch the brain solved for. Wheel, hood, and velocity estimate.
 *   4. AIM              the brain's solved launch, flown through simulateShot, against the
 *                       mouth. If this is off, the lead or the table is wrong on paper.
 *
 * Heights are reported at the descending crossing of the mouth's CENTRE height, as the world
 * scores arrival, and at the near and far lips, as the table solves. Every case runs
 * stationary head-on, stationary off-axis (which the table never solved for), and moving.
 */
import params from '../config/params.json' with { type: 'json' };
import robotSpec from '../config/robot.json' with { type: 'json' };
import { readFileSync } from 'node:fs';
import { World, initPhysics, emptyGamepad } from '../packages/core/src/physics/world.js';
import { BuiltinTeleOp, ShotTable } from '../packages/core/src/robot/builtinTeleOp.js';
import { simulateShot } from '../packages/core/src/physics/ballistics.js';
import { mouthLips } from './shottable.js';
import { DEG, RAD, M_TO_IN, inches, rpmToRadS } from '../packages/core/src/units.js';
import type { GamepadState, Params, RobotSpec, Vec3 } from '../packages/core/src/types.js';

const table = ShotTable.fromCsv(readFileSync(new URL('../java/teamcode/assets/shottable.csv', import.meta.url), 'utf8'));

interface Launch {
  /** The ball's state one frame after release: the same start for both integrators. */
  pos: Vec3; vel: Vec3; spin: Vec3;
  /** What the brain had solved for, ground frame. */
  wantSpeed: number; wantElev: number; wantAz: number; targetRpm: number;
  /** What the shooter actually did. */
  vExit: number; elev: number; rpm: number; hood: number;
  /** Chassis velocity the brain believed, and the truth, radial m/s. */
  vrBelieved: number; vrTrue: number;
  muzzleY: number; range_in: number;
  /** Balls already sitting in our up CELL when this one left. */
  inCell: number;
  /** Chassis yaw rate at release, deg/s, and the turret angle. */
  omegaDps: number; turretDeg: number;
  /** Chassis velocity at release, world frame, and the hood/rpm the table row held. */
  cv: Vec3; rowHood: number; rowRpm: number;
}

interface Cross { atMouth: number | null; nearLip: number | null; farLip: number | null; downrange: number | null }

/** Height where a track crosses each of three z planes going toward the hive (-Z). */
function crossings(pts: Vec3[], mouth: Vec3, near: { y: number; z: number }, far: { y: number; z: number }, from: Vec3): Cross {
  const atZ = (z: number): number | null => {
    for (let i = 1; i < pts.length; i++) {
      if (pts[i - 1][2] > z && pts[i][2] <= z) {
        const f = (pts[i - 1][2] - z) / (pts[i - 1][2] - pts[i][2] || 1);
        return pts[i - 1][1] + (pts[i][1] - pts[i - 1][1]) * f;
      }
    }
    return null;
  };
  // Descending crossing of the mouth's height: where the world scores long/short.
  let downrange: number | null = null;
  for (let i = 1; i < pts.length; i++) {
    if (pts[i - 1][1] >= mouth[1] && pts[i][1] < mouth[1]) {
      const f = (pts[i - 1][1] - mouth[1]) / (pts[i - 1][1] - pts[i][1] || 1);
      const x = pts[i - 1][0] + (pts[i][0] - pts[i - 1][0]) * f;
      const z = pts[i - 1][2] + (pts[i][2] - pts[i - 1][2]) * f;
      // Signed along the shot line from the launch point: past the mouth is positive.
      const dx = mouth[0] - from[0], dz = mouth[2] - from[2];
      const n = Math.hypot(dx, dz) || 1;
      downrange = ((x - mouth[0]) * dx + (z - mouth[2]) * dz) / n;
      break;
    }
  }
  return { atMouth: atZ(mouth[2]), nearLip: atZ(near.z), farLip: atZ(far.z), downrange };
}

function fly(p: Params, from: Vec3, vel: Vec3, spin: number): Vec3[] {
  const sp = Math.hypot(vel[0], vel[1], vel[2]);
  const t = simulateShot(p, {
    from, azimuth: Math.atan2(vel[0], vel[2]), elevation: Math.asin(vel[1] / sp), speed: sp,
    radius: p.ball.pollen.d_m / 2, mass: p.ball.pollen.m_kg, spin,
  }, 100);
  return t.points;
}

async function pass(range_in: number, drive: [number, number], shots: number, seed: number, headOn: boolean, rangeLead?: number, turn = 0) {
  await initPhysics();
  const p = structuredClone(params) as unknown as Params;
  const spec = structuredClone(robotSpec) as unknown as RobotSpec;
  // Nothing random: this is about the systematic part.
  if (!process.argv.includes('--scatter')) spec.flywheel.scatter = { angle_deg: 0, yaw_deg: 0, speedFrac: 0 };
  if (!process.argv.includes('--gate')) spec.flywheel.minLandProb = 0;
  if (rangeLead !== undefined) spec.flywheel.rangeLead_s = rangeLead;
  p.ball.pollen.dVar = 0;
  p.ball.pollen.mVar = 0;
  const staging = Array.from({ length: 60 }, () => ({ kind: 'pollen' as const, pos: [0, -5, 0] as Vec3 }));
  const w = new World({ params: p, robot: spec, staging, alliance: 'red', seed });
  for (const b of w.balls.balls) w.balls.park(b);
  const mouth = w.hives.red.upCellMouthWorld();
  const lips = mouthLips(p);
  // Head-on is straight down +Z from the mouth, which the wall allows only up to about 39 in.
  // Off-axis swings round in X the way tools/movingfire.ts places, and the way a driver stands.
  const lim = w.geom.halfWidth_m - 0.40;
  const R = inches(range_in);
  const dz = headOn ? R : Math.min(R, lim - mouth[2]);
  const dx = headOn ? 0 : Math.sqrt(Math.max(0, R * R - dz * dz));
  const x = mouth[0] + dx;
  const z = mouth[2] + dz;
  w.robot.place([x, spec.chassis.height_m / 2 + spec.chassis.clearance_m, z], Math.atan2(mouth[0] - x, mouth[2] - z) * RAD);
  const brain = new BuiltinTeleOp(spec, table);
  let loaded = 0;
  const out: { L: Launch; world: Cross; sim: Cross; aim: Cross; tableRow: Cross; rowHood: number; rowSpeed: number; offAxisDeg: number; apex: number; diverge: number; result: string; endLong: number; endLat: number }[] = [];
  const tracks: { L: Launch; id: number; pts: Vec3[]; born: number }[] = [];
  const step = (g: GamepadState) => {
    while (w.robot.heldBalls().length < 4 && loaded < staging.length) {
      if (!w.robot.preload(w.balls, w.balls.balls[loaded])) break;
      loaded++;
    }
    const before = w.robot.shots;
    const st = brain.state;
    // The lead's solution is overwritten by the update that fires, so read it first.
    const want = { speed: st.leadSpeed, elev: st.leadElevDeg, az: st.leadAzDeg, rpm: st.targetRpm, vr: st.leadVRadial, rowHood: st.rowHoodDeg ?? NaN, rowRpm: st.rowRpm ?? NaN };
    w.setGamepads(g, emptyGamepad());
    w.step(brain.update(w.sensors(), g, w.seq));
    if (w.robot.shots > before) {
      const id = w.robot.lastShotBallId;
      const b = w.balls.balls[id];
      const v = b.body.linvel(), s = b.body.angvel();
      const ls = w.robot.lastShot!;
      const cv = w.robot.body.linvel();
      const r = w.robot.pos;
      const dxm = mouth[0] - r[0], dzm = mouth[2] - r[2];
      const n = Math.hypot(dxm, dzm) || 1;
      const L: Launch = {
        pos: w.balls.pos(b), vel: [v.x, v.y, v.z], spin: [s.x, s.y, s.z],
        wantSpeed: want.speed, wantElev: want.elev, wantAz: want.az, targetRpm: want.rpm,
        vExit: ls.v_exit, elev: ls.elevDeg, rpm: w.robot.lastShotRpm, hood: w.robot.hoodAngle,
        vrBelieved: want.vr, vrTrue: (cv.x * dxm + cv.z * dzm) / n,
        muzzleY: w.robot.muzzle().pos[1], range_in: n * M_TO_IN,
        cv: [cv.x, cv.y, cv.z], rowHood: want.rowHood, rowRpm: want.rowRpm,
        omegaDps: w.robot.body.angvel().y * RAD, turretDeg: w.robot.turretAngle,
        inCell: w.landedInUpCell('red'),
      };
      tracks.push({ L, id, pts: [L.pos], born: w.t });
    }
    // Not the one born this frame: its first point is already the end of this frame.
    for (const tr of tracks) if (tr.born < w.t) tr.pts.push(w.balls.pos(w.balls.balls[tr.id]));
  };
  const arm = emptyGamepad(); step(arm); arm.dpad_up = true; step(arm);
  for (let f = 0; f < 150; f++) step(emptyGamepad());
  // A driver swinging the chassis while the turret holds the goal: tools/shoterror.ts's
  // turning case, where 7 of 62 shots went two metres long with every release term on target.
  const hold = (): GamepadState => ({ ...emptyGamepad(), left_stick_x: drive[0], left_stick_y: -drive[1], right_stick_x: turn * Math.sin(2 * Math.PI * 0.25 * w.t) });
  for (let f = 0; f < 45; f++) step(hold());
  let f = 0;
  while (tracks.length < shots && f++ < 60 * 60) step({ ...hold(), right_bumper: true });
  for (let k = 0; k < 60 * 5; k++) step(emptyGamepad());
  const log = w.snapshot().shots;

  for (const [ti, tr] of tracks.entries()) {
    const { L } = tr;
    const rec = log[ti];
    const world = crossings(tr.pts, mouth, lips.near, lips.far, L.pos);
    const sim = crossings(fly(p, L.pos, L.vel, Math.hypot(L.spin[0], L.spin[1], L.spin[2])), mouth, lips.near, lips.far, L.pos);
    // The brain's own answer, launched from where the TABLE thinks the muzzle is.
    const az = L.wantAz * DEG;
    const wantVel: Vec3 = [
      L.wantSpeed * Math.cos(L.wantElev * DEG) * Math.sin(az),
      L.wantSpeed * Math.sin(L.wantElev * DEG),
      L.wantSpeed * Math.cos(L.wantElev * DEG) * Math.cos(az),
    ];
    // Field azimuth: the lead's azimuth is turret-relative to the chassis heading.
    const yaw = w.robot.yaw;
    const azF = yaw + az;
    wantVel[0] = L.wantSpeed * Math.cos(L.wantElev * DEG) * Math.sin(azF);
    wantVel[2] = L.wantSpeed * Math.cos(L.wantElev * DEG) * Math.cos(azF);
    wantVel[0] += L.cv[0]; wantVel[1] += L.cv[1]; wantVel[2] += L.cv[2];
    const tableFrom: Vec3 = [L.pos[0], spec.turret.muzzleHeight_m, L.pos[2]];
    const aim = crossings(fly(p, tableFrom, wantVel, (1 / (p.ball.pollen.d_m / 2)) * L.wantSpeed), mouth, lips.near, lips.far, tableFrom);
    // THE TABLE'S OWN SHOT for the range the ball actually left from, standing still, from
    // the actual launch point. If this lands and the others do not, the row was right and
    // the lead or the range it was looked up at was wrong.
    const row = table.lookup(L.range_in - (spec.calibration?.rangeTrim_in ?? 0));
    const rowHood = spec.hood.angleRange_deg[0] + row.hoodPos * (spec.hood.angleRange_deg[1] - spec.hood.angleRange_deg[0]);
    const rowSpeed = spec.flywheel.k * spec.flywheel.r_fly_m * rpmToRadS(row.rpm);
    const bx = mouth[0] - L.pos[0], bz = mouth[2] - L.pos[2];
    const bn = Math.hypot(bx, bz) || 1;
    const rowVel: Vec3 = [rowSpeed * Math.cos(rowHood * DEG) * bx / bn, rowSpeed * Math.sin(rowHood * DEG), rowSpeed * Math.cos(rowHood * DEG) * bz / bn];
    const tableRow = crossings(fly(p, tableFrom, rowVel, (1 / (p.ball.pollen.d_m / 2)) * rowSpeed), mouth, lips.near, lips.far, tableFrom);
    // WHEN the world stops agreeing with the solver, in frames after release. A flight that
    // diverges in the first few frames hit something next to the muzzle; one that agrees
    // until the pocket hit the pocket.
    const simPts = fly(p, L.pos, L.vel, Math.hypot(L.spin[0], L.spin[1], L.spin[2]));
    let diverge = -1;
    for (let i = 1; i < tr.pts.length; i++) {
      const j = Math.min(simPts.length - 1, Math.round(i * 8));  // 1/60 s frames vs 1/480 s steps
      const d = Math.hypot(tr.pts[i][0] - simPts[j][0], tr.pts[i][1] - simPts[j][1], tr.pts[i][2] - simPts[j][2]);
      if (d > 0.05) { diverge = i; break; }
    }
    out.push({ L, world, sim, aim, tableRow, rowHood, rowSpeed, offAxisDeg: Math.atan2(dx, dz) * RAD, apex: Math.max(...tr.pts.map((q) => q[1])), diverge, result: rec?.result ?? '?', endLong: (rec?.long_in ?? NaN) * 2.54, endLat: (rec?.lat_in ?? NaN) * 2.54 });
  }
  return { out, mouth, lips };
}

const cm = (v: number | null) => (v === null ? '   --' : (v * 100).toFixed(1).padStart(5));
const mean = (a: number[]) => (a.length ? a.reduce((x, y) => x + y, 0) / a.length : NaN);

export async function main(argv: string[] = []): Promise<void> {
  const i = argv.indexOf('--shots');
  const shots = i >= 0 ? Number(argv[i + 1]) : 6;
  const p = params as unknown as Params;
  const spec = robotSpec as unknown as RobotSpec;
  const lips = mouthLips(p);
  console.log('\nTHE FLIGHT, SPLIT INTO ITS PARTS. Scatter off, ball variance off: what is left is systematic.');
  console.log(`  mouth lips: near y ${(lips.near.y * 100).toFixed(1)} cm z ${(lips.near.z * 100).toFixed(1)}, far y ${(lips.far.y * 100).toFixed(1)} z ${(lips.far.z * 100).toFixed(1)}`);
  console.log(`  table assumes the muzzle at ${(spec.turret.muzzleHeight_m * 100).toFixed(1)} cm\n`);

  const cases: [string, number, [number, number], boolean][] = [
    ['36 in head-on, still', 36, [0, 0], true],
    ['50 in off-axis, still', 50, [0, 0], false],
    ['70 in off-axis, still', 70, [0, 0], false],
    ['50 in closing 0.18', 50, [0, 0.18], false],
    ['70 in closing 0.35', 70, [0, 0.35], false],
    ['50 in receding 0.18', 50, [0, -0.18], false],
    ['50 in strafing 0.3', 50, [0.3, 0], false],
    ['50 in turning 0.5', 50, [0, 0], false],
  ];
  const only = argv.indexOf('--case') >= 0 ? argv[argv.indexOf('--case') + 1] : '';
  for (const [name, range, drive, headOn] of cases.filter((c) => c[0].includes(only))) {
    const rl = argv.indexOf('--rangeLead') >= 0 ? Number(argv[argv.indexOf('--rangeLead') + 1]) : undefined;
    const { out: all, mouth } = await pass(range, drive, shots, 5, headOn, rl, name.includes('turning') ? 0.5 : 0);
    // Inside the table's nearest row there is no solution to execute; those are a separate bug.
    const out = all.filter((o) => o.L.range_in >= 30);
    if (all.length > out.length) console.log(`  (${all.length - out.length} shots fired from INSIDE the table's 30 in minimum, not counted here)`);
    if (!out.length) { console.log(`  ${name}: no shots\n`); continue; }
    const L0 = out[0].L;
    console.log(`  ${name}   (${out.length} shots, ${out[0].offAxisDeg.toFixed(0)} deg off the mouth's axis, mouth centre ${(mouth[1] * 100).toFixed(1)} cm)`);
    console.log(`     launch: muzzle at ${(L0.muzzleY * 100).toFixed(1)} cm  (table ${(spec.turret.muzzleHeight_m * 100).toFixed(1)}: ${((L0.muzzleY - spec.turret.muzzleHeight_m) * 100).toFixed(1)} cm high)`);
    const d = (f: (o: typeof out[0]) => number) => mean(out.map(f).filter(Number.isFinite));
    console.log(`     rpm at release ${d((o) => o.L.rpm).toFixed(0)} vs target ${d((o) => o.L.targetRpm).toFixed(0)}  (${d((o) => o.L.rpm - o.L.targetRpm).toFixed(0)} rpm)     hood ${d((o) => o.L.hood).toFixed(2)} vs solved ${d((o) => o.L.wantElev).toFixed(2)} deg  (${d((o) => o.L.hood - o.L.wantElev).toFixed(2)})`);
    console.log(`     exit ${d((o) => o.L.vExit).toFixed(3)} m/s; lead wanted ${d((o) => o.L.wantSpeed).toFixed(3)} ground     v_radial believed ${d((o) => o.L.vrBelieved).toFixed(3)} true ${d((o) => o.L.vrTrue).toFixed(3)} m/s`);
    const gv = d((o) => Math.hypot(o.L.vel[0], o.L.vel[1], o.L.vel[2]));
    const ge = d((o) => Math.asin(o.L.vel[1] / Math.hypot(o.L.vel[0], o.L.vel[1], o.L.vel[2])) * RAD);
    console.log(`     ground launch actual ${gv.toFixed(3)} m/s at ${ge.toFixed(2)} deg; solved ${d((o) => o.L.wantSpeed).toFixed(3)} at ${d((o) => o.L.wantElev).toFixed(2)}`);
    console.log('                            height at near lip   at mouth z   at far lip    downrange at mouth height');
    const row = (label: string, f: (o: typeof out[0]) => Cross) =>
      console.log(`     ${label.padEnd(22)} ${cm(d((o) => f(o).nearLip ?? NaN))} cm          ${cm(d((o) => f(o).atMouth ?? NaN))} cm     ${cm(d((o) => f(o).farLip ?? NaN))} cm      ${cm(d((o) => f(o).downrange ?? NaN))} cm`);
    row('table row, from here', (o) => o.tableRow);
    row('world (Rapier)', (o) => o.world);
    row('sim, same start', (o) => o.sim);
    row('aim, as solved', (o) => o.aim);
    console.log('     per shot:  range  brainRow(hood,rpm)  hereRow(hood,speed)   ground launch h,v m/s      sim@mouthZ   world@mouthZ   apex(world)  yaw dps  turret deg  downrange world/sim   diverges   log: result long/lat cm   in CELL already');
    for (const o of out) {
      const gh = Math.hypot(o.L.vel[0], o.L.vel[2]), gv = o.L.vel[1];
      console.log(`               ${o.L.range_in.toFixed(1).padStart(5)}   ${o.L.rowHood.toFixed(1).padStart(5)} ${o.L.rowRpm.toFixed(0).padStart(5)}       ${o.rowHood.toFixed(1).padStart(5)} ${o.rowSpeed.toFixed(2).padStart(5)}          ${gh.toFixed(2)} ${gv.toFixed(2)}             ${cm(o.sim.atMouth)}        ${cm(o.world.atMouth)}      ${cm(o.apex)}    ${o.L.omegaDps.toFixed(0).padStart(5)}   ${o.L.turretDeg.toFixed(0).padStart(6)}     ${cm(o.world.downrange)} / ${cm(o.sim.downrange)}      ${(o.diverge < 0 ? 'never' : String(o.diverge)).padStart(6)}   ${o.result.padEnd(6)} ${o.endLong.toFixed(0).padStart(5)} / ${o.endLat.toFixed(0).padStart(4)}     ${String(o.L.inCell).padStart(4)}`);
    }
    console.log(`     lips to clear:         ${cm(lips.near.y + p.ball.pollen.d_m / 2)} cm (above)         ${cm(lips.far.y - p.ball.pollen.d_m / 2)} cm (below)`);
    console.log('');
  }
  console.log('  world - sim is the integrator. sim - aim is execution plus launch geometry. aim vs the');
  console.log('  lips is the solver. Whichever line is off is where the height goes wrong.\n');
}
