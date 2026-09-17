/**
 * Drive the robot around with the gamepad, and at each stop let the auto-aim work out the
 * shot from the robot's own state. Nothing is teleported: every move is the drivetrain.
 *
 *   npm run tool -- tools/drivedemo.ts
 *
 * IT STARTS IN THE SHOOTING SECTOR, which is not where a match starts. The up CELL opens
 * toward the audience, so the whole of the alliance-wall side of the field is BEHIND the
 * mouth -- from the match start pose it is 109 deg off its opening, and from the corners this
 * demo used to visit, up to 171 deg. No launch from there can enter.
 *
 * This demo used to start at the match pose and report "fired" from every one of those spots,
 * because nothing checked which way the mouth faced (PHYSICS 9.3 note in docs/DECISIONS.md);
 * the balls went into the back of the pocket and the old census counted some of them. With
 * `game.upCellOpenDeg` in the readiness gate it correctly refused all five and the demo read
 * 0 shots, which is the honest answer to the question it was asking. So it now asks a better
 * one: park in the sector a robot actually shoots from, and move around INSIDE it.
 */
import { readFileSync } from 'node:fs';
import params from '../config/params.json' with { type: 'json' };
import robotSpec from '../config/robot.json' with { type: 'json' };
import staging from '../assets/staging.json' with { type: 'json' };
import { World, initPhysics, emptyGamepad } from '../packages/core/src/physics/world.js';
import { BuiltinTeleOp, ShotTable } from '../packages/core/src/robot/builtinTeleOp.js';
import { loadLandCal } from '../packages/core/src/robot/loadCal.js';
import { M_TO_IN } from '../packages/core/src/units.js';
import type { BallKind, GamepadState, Params, RobotSpec, Vec3 } from '../packages/core/src/types.js';

const table = ShotTable.fromCsv(readFileSync(new URL('../java/teamcode/assets/shottable.csv', import.meta.url), 'utf8'));
const balls = (staging.balls as { kind: string; pos: number[] }[]).map((b) => ({ kind: b.kind as BallKind, pos: b.pos as Vec3 }));

export async function main(): Promise<void> {
  await initPhysics();
  const p = structuredClone(params) as unknown as Params;
  const spec = robotSpec as unknown as RobotSpec;
  const world = new World({ params: p, robot: spec, staging: balls, alliance: 'red', seed: 2, preload: spec.hopper.capacity });
  const brain = new BuiltinTeleOp(spec, table, loadLandCal());
  world.clock.startTeleOp();


  let topSpeed = 0;
  let maxTurret = 0;
  const step = (g: GamepadState) => {
    world.setGamepads(g, emptyGamepad());
    world.step(brain.update(world.sensors(), g, world.seq));
    const snap = world.snapshot().robot;
    topSpeed = Math.max(topSpeed, snap.speed * M_TO_IN);
    maxTurret = Math.max(maxTurret, Math.abs(snap.turret.angleDeg));
  };
  const hold = (g: GamepadState, seconds: number) => {
    for (let f = 0; f < Math.round(seconds * 60); f++) step(g);
  };
  const stick = (fwd: number, left: number, turn = 0): GamepadState => {
    const g = emptyGamepad();
    g.y = true;   // robot-centric: these moves are written relative to the robot's nose
    g.left_stick_y = -fwd;
    g.left_stick_x = -left;
    g.right_stick_x = -turn;
    return g;
  };

  // Toggle the flywheel on once (edge-detected, exactly like a real trigger press).
  step(emptyGamepad());
  const press = emptyGamepad();
  press.a = true;
  step(press);

  // Stand in front of the opening, at the middle of the shot table's usable range, derived
  // from where the mouth actually is rather than from a coordinate typed in here.
  const hive = world.hives.red;
  const mouth = hive.upCellMouthWorld();
  const nrm = hive.upCellMouthNormalWorld();
  const nh = Math.hypot(nrm[0], nrm[2]) || 1;
  const stand = 0.0254 * 55;
  const sx = mouth[0] + (nrm[0] / nh) * stand;
  const sz = mouth[2] + (nrm[2] / nh) * stand;
  world.robot.place(
    [sx, spec.chassis.height_m / 2 + spec.chassis.clearance_m, sz],
    (Math.atan2(mouth[0] - sx, mouth[2] - sz) * 180) / Math.PI,
  );
  for (let f = 0; f < 30; f++) step(emptyGamepad());


  const report = (label: string) => {
    const s = world.sensors();
    const r = world.snapshot().robot;
    console.log(
      `${label.padEnd(22)} pos ${s.localizer.x.toFixed(0).padStart(4)},${s.localizer.y.toFixed(0).padStart(4)} in ` +
        `@${s.imu.yaw.toFixed(0).padStart(4)}deg | range ${s.game.upCellRangeIn.toFixed(0).padStart(3)} in  ` +
        `bearing ${s.game.upCellAzimuthDeg.toFixed(0).padStart(4)}  turret ${r.turret.angleDeg.toFixed(0).padStart(4)}  ` +
        `hood ${r.hood.angleDeg.toFixed(0).padStart(2)}  rpm ${r.flywheel.rpm.toFixed(0).padStart(4)}/${brain.state.targetRpm.toFixed(0).padStart(4)} ` +
        `| ${brain.state.note}`,
    );
  };

  console.log('the chassis is never turned to aim -- only the turret moves\n');
  // Moves that stay in the sector: closing, strafing across the face, and backing off.
  const moves: [string, GamepadState, number][] = [
    ['start', stick(0, 0), 0.5],
    ['close in', stick(0.6, 0), 0.7],
    ['strafe across', stick(0, 0.7), 0.9],
    ['back off', stick(-0.6, 0), 0.6],
    ['strafe back', stick(0, -0.7), 1.4],
  ];

  let shotsBefore = 0;
  for (const [label, g, seconds] of moves) {
    hold(g, seconds);
    hold(stick(0, 0), 1.2); // settle, let the turret and flywheel catch up
    report(label);

    const fire = emptyGamepad();
    fire.right_bumper = true;
    hold(fire, 3.5);
    const landed = world.hives.red.ballsInUpCell;
    console.log(
      `${''.padEnd(22)}   fired ${world.robot.shots - shotsBefore}, ${landed} now in the up CELL` +
        (world.hives.red.tips ? `, HIVE TIPPED x${world.hives.red.tips}` : ''),
    );
    shotsBefore = world.robot.shots;

    // Top the hopper back up so each stop is a fair test of the aim, not of the intake.
    for (const b of world.balls.balls) {
      if (world.robot.heldBalls().length >= spec.hopper.capacity) break;
      if (b.kind === 'pollen' && b.state === 'free' && b.body.isEnabled() && world.balls.pos(b)[1] < 0.3) world.robot.preload(world.balls, b);
    }
  }

  const sc = world.snapshot().score.red;
  console.log(`\ntotal: ${world.robot.shots} shots, ${world.hives.red.tips} tips, ${sc.total} points`);
  console.log(`every move was the drivetrain: top speed ${topSpeed.toFixed(0)} in/s, turret swung out to ${maxTurret.toFixed(0)} deg`);
}
