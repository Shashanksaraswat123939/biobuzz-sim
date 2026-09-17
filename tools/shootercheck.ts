/**
 * Three checks on the shooter, because all three are easy to get wrong silently:
 *   1. does the turret let the robot shoot from any bearing without turning the chassis?
 *   2. is the flywheel's moment of inertia a physically sensible number?
 *   3. does changing flywheel speed actually change exit speed (and does a shot cost it)?
 *
 *   npm run tool -- tools/shootercheck.ts
 */
import { readFileSync } from 'node:fs';
import params from '../config/params.json' with { type: 'json' };
import robotSpec from '../config/robot.json' with { type: 'json' };
import motors from '../config/motors.json' with { type: 'json' };
import { World, initPhysics, emptyGamepad } from '../packages/core/src/physics/world.js';
import { BuiltinTeleOp, ShotTable } from '../packages/core/src/robot/builtinTeleOp.js';
import { loadLandCal } from '../packages/core/src/robot/loadCal.js';
import { rpmToSpeed } from '../packages/core/src/physics/ballistics.js';
import { inches, M_TO_IN, rpmToRadS } from '../packages/core/src/units.js';
import type { ActuatorFrame, Params, RobotSpec, Vec3 } from '../packages/core/src/types.js';

const table = ShotTable.fromCsv(readFileSync(new URL('../java/teamcode/assets/shottable.csv', import.meta.url), 'utf8'));

function makeWorld(p: Params, spec: RobotSpec, n = 8) {
  const staging = Array.from({ length: n }, () => ({ kind: 'pollen' as const, pos: [0, -5, 0] as Vec3 }));
  const w = new World({ params: p, robot: spec, staging, alliance: 'red', seed: 4 });
  for (const b of w.balls.balls) w.balls.park(b);
  return w;
}

export async function main(): Promise<void> {
  await initPhysics();
  const p = structuredClone(params) as unknown as Params;
  const spec = robotSpec as unknown as RobotSpec;
  const f = spec.flywheel;

  // ---------------------------------------------------------------- 2. MOI
  const rotor = motors.bare.rotorInertia_kgm2 * (motors.variants[f.motor.variant as keyof typeof motors.variants] as { ratio: number }).ratio ** 2;
  const total = f.I_fly_kgm2 + rotor;
  const discMass = (2 * f.I_fly_kgm2) / (f.r_fly_m * f.r_fly_m);
  const ringMass = f.I_fly_kgm2 / (f.r_fly_m * f.r_fly_m);
  const stall = motors.bare.stallTorque_Nm * (motors.variants[f.motor.variant as keyof typeof motors.variants] as { ratio: number }).ratio * motors.gearboxEfficiency;
  const omegaFree = rpmToRadS((motors.variants[f.motor.variant as keyof typeof motors.variants] as { freeRpm: number }).freeRpm);
  console.log('--- flywheel inertia');
  console.log(`  wheel I      ${f.I_fly_kgm2.toExponential(3)} kg.m^2 at r = ${(f.r_fly_m * 2 * M_TO_IN).toFixed(1)} in diameter`);
  console.log(`  implies a wheel of ${(discMass * 1000).toFixed(0)} g as a solid disc, or ${(ringMass * 1000).toFixed(0)} g as a rim`);
  console.log(`  motor rotor  ${rotor.toExponential(3)} kg.m^2 reflected  ->  total ${total.toExponential(3)}`);
  console.log(`  spin-up time constant I*w_free/tau_stall = ${((total * omegaFree) / stall).toFixed(2)} s`);

  // ------------------------------------------------- 3. omega -> exit speed
  console.log('\n--- exit speed vs flywheel speed (v = k * omega * r)');
  console.log('   rpm    omega     predicted    measured    dip after the shot');
  for (const rpm of [1500, 2500, 3500, 4500, 5500]) {
    const w = makeWorld(p, spec);
    w.robot.place([0, 0.17, 0], 0);
    w.robot.preload(w.balls, w.balls.balls[0]);
    w.robot.flywheelOmega = rpmToRadS(rpm);
    w.robot.motors.get('flywheel')!.omega = w.robot.flywheelOmega;
    const before = w.robot.flywheelRpm;

    // Feed one ball with the flywheel held (no motor command), so only the shot changes it.
    const act: ActuatorFrame = { seq: 0, motors: { transfer: { mode: 'RUN_WITHOUT_ENCODER', power: 1 } }, servos: { gate: 1 } };
    for (let i = 0; i < 60 && w.robot.shots === 0; i++) w.step(act);
    const shot = w.robot.lastShot;
    const predicted = rpmToSpeed(before, f.k, f.r_fly_m);
    console.log(
      `  ${String(rpm).padStart(5)}  ${rpmToRadS(rpm).toFixed(0).padStart(5)}   ` +
        `${predicted.toFixed(2).padStart(8)} m/s  ${(shot ? shot.v_exit : NaN).toFixed(2).padStart(8)} m/s  ` +
        `${(before - w.robot.flywheelRpm).toFixed(0).padStart(5)} rpm`,
    );
  }

  // -------------------------------------- 1. any bearing, chassis not turned
  console.log('\n--- shooting from every bearing with a FIXED chassis heading');
  console.log('  the chassis always faces world +Z; only the turret moves');
  console.log('  bearing  range  turret cmd   reachable   landed in CELL');
  const mouth = makeWorld(p, spec).hives.red.upCellMouthWorld();
  for (const bearingDeg of [-150, -120, -90, -45, 0, 45, 90, 120, 150]) {
    const w = makeWorld(p, spec);
    const a = (bearingDeg * Math.PI) / 180;
    const limit = w.geom.halfWidth_m - 0.3;
    // Stand at `bearingDeg` around the CELL, chassis always yaw 0 (facing world +Z).
    // Back off until the spot is actually on the field -- the CELL is not in the middle.
    let pos: Vec3 | null = null;
    let usedIn = 0;
    for (let rIn = 100; rIn >= 35; rIn -= 5) {
      const d = inches(rIn);
      const c: Vec3 = [mouth[0] + d * Math.sin(a), spec.chassis.height_m / 2 + spec.chassis.clearance_m, mouth[2] + d * Math.cos(a)];
      if (Math.abs(c[0]) < limit && Math.abs(c[2]) < limit) { pos = c; usedIn = rIn; break; }
    }
    if (!pos) {
      console.log(`  ${String(bearingDeg).padStart(6)}    (no on-field spot)`);
      continue;
    }
    w.robot.place(pos, 0);
    for (let i = 0; i < 6; i++) w.robot.preload(w.balls, w.balls.balls[i]);
    const brain = new BuiltinTeleOp(spec, table, loadLandCal());
    const step = (g = emptyGamepad()) => {
      w.setGamepads(g, emptyGamepad());
      w.step(brain.update(w.sensors(), g, w.seq));
    };
    step();
    const press = emptyGamepad();
    press.a = true;
    step(press);
    for (let i = 0; i < 300; i++) step();
    const want = w.sensors().game.upCellAzimuthDeg;
    const fire = emptyGamepad();
    fire.right_bumper = true;
    for (let i = 0; i < 60 * 25 && w.robot.shots < 6; i++) step(fire);
    for (let i = 0; i < 300; i++) step(fire);
    const reach = want >= spec.turret.range_deg[0] && want <= spec.turret.range_deg[1];
    console.log(
      `  ${String(bearingDeg).padStart(6)}  ${String(usedIn).padStart(4)} in  ${want.toFixed(0).padStart(6)} deg    ${reach ? 'yes' : 'NO '}      ` +
        `${w.hives.red.ballsInUpCell} of ${w.robot.shots}` + (w.hives.red.tips ? `  (tipped)` : ''),
    );
  }

  // ------------------------------------------------- does the aim lead for motion?
  console.log('\n--- shooting while moving sideways (does the aim lead?)');
  for (const strafe of [0, 0.5, 1.0]) {
    const w = makeWorld(p, spec);
    w.robot.place([mouth[0] + inches(40), spec.chassis.height_m / 2 + spec.chassis.clearance_m, mouth[2] + inches(40)], 180);
    for (let i = 0; i < 6; i++) w.robot.preload(w.balls, w.balls.balls[i]);
    const brain = new BuiltinTeleOp(spec, table, loadLandCal());
    const g = emptyGamepad();
    const step = (gg = g) => {
      w.setGamepads(gg, emptyGamepad());
      w.step(brain.update(w.sensors(), gg, w.seq));
    };
    step();
    const press = emptyGamepad();
    press.a = true;
    step(press);
    for (let i = 0; i < 300; i++) step();
    // Push the robot sideways at a fixed speed and fire.
    const fire = emptyGamepad();
    fire.right_bumper = true;
    for (let i = 0; i < 60 * 20 && w.robot.shots < 4; i++) {
      w.robot.body.setLinvel({ x: strafe, y: w.robot.body.linvel().y, z: 0 }, true);
      step(fire);
    }
    for (let i = 0; i < 240; i++) step(fire);
    const sf = w.sensors();
    console.log(
      `  strafe ${strafe.toFixed(1)} m/s -> ${w.hives.red.ballsInUpCell} of ${w.robot.shots} landed, ` +
        `lead ${brain.state.leadDeg.toFixed(1)} deg, range ${sf.game.upCellRangeIn.toFixed(0)} in, ` +
        `az ${sf.game.upCellAzimuthDeg.toFixed(0)}, vx ${sf.localizer.vx.toFixed(1)} vy ${sf.localizer.vy.toFixed(1)} in/s, ` +
        `hopper ${w.robot.hopper.length}, note "${brain.state.note}"`,
    );
  }
}
