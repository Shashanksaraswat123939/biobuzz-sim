import { describe, it, expect, beforeAll } from 'vitest';
import params from '../config/params.json';
import robotSpec from '../config/robot.json';
import { World, initPhysics, emptyGamepad } from '../packages/core/src/physics/world.js';
import { BuiltinTeleOp, ShotTable } from '../packages/core/src/robot/builtinTeleOp.js';
import { M_TO_IN } from '../packages/core/src/units.js';
import type { ActuatorFrame, GamepadState, Params, RobotSpec, Vec3 } from '../packages/core/src/types.js';

beforeAll(async () => {
  await initPhysics();
});

function rig() {
  const p = structuredClone(params) as unknown as Params;
  const spec = robotSpec as unknown as RobotSpec;
  const staging: { kind: 'pollen'; pos: Vec3 }[] = [];
  const world = new World({ params: p, robot: spec, staging, alliance: 'red', seed: 1 });
  world.robot.place([0.9, 0.17, -1.55], 0);
  return world;
}

/** What the wire carries: fl/bl are reversed in robot.json, so forward is (-1, 1, -1, 1). */
function wheels(fl: number, fr: number, bl: number, br: number): ActuatorFrame {
  return {
    seq: 0,
    motors: {
      fl: { mode: 'RUN_WITHOUT_ENCODER', power: fl },
      fr: { mode: 'RUN_WITHOUT_ENCODER', power: fr },
      bl: { mode: 'RUN_WITHOUT_ENCODER', power: bl },
      br: { mode: 'RUN_WITHOUT_ENCODER', power: br },
    },
    servos: {},
  };
}

describe('drivetrain (PLAN.md phase 2)', () => {
  it('the robot has the mass robot.json says it has', () => {
    const world = rig();
    expect(world.robot.body.mass()).toBeCloseTo(robotSpec.chassis.mass_kg, 2);
    expect(world.hives.red.body.mass()).toBeCloseTo(params.hive.massKg, 2);
  });

  it('reaches the free speed the motor curve predicts, within 10%', () => {
    const world = rig();
    const act = wheels(-1, 1, -1, 1);
    // Top speed, not the speed at the end: 2.5 s of full power crosses the whole field and
    // parks the robot against the far wall.
    let v = 0;
    for (let f = 0; f < 110; f++) {
      world.step(act);
      v = Math.max(v, world.snapshot().robot.speed * M_TO_IN);
    }
    // 312 rpm through 96 mm wheels = 62 in/s.
    const free = (312 / 60) * 2 * Math.PI * robotSpec.drivetrain.wheelRadius_m * M_TO_IN;
    expect(v).toBeGreaterThan(free * 0.9);
    expect(v).toBeLessThan(free * 1.1);
  });

  it('draws stall-ish current at rest and little when free-running, and the battery sags', () => {
    const world = rig();
    const act = wheels(-1, 1, -1, 1);
    world.step(act);
    const atRest = world.snapshot().robot.battery;
    for (let f = 0; f < 80; f++) world.step(act);
    const free = world.snapshot().robot.battery;
    expect(atRest.amps).toBeGreaterThan(free.amps * 3);
    expect(atRest.volts).toBeLessThan(free.volts);
  });

  it('strafes sideways and turns on the spot', () => {
    const strafe = rig();
    for (let f = 0; f < 90; f++) strafe.step(wheels(1, 1, -1, -1));
    const s = strafe.snapshot().robot;
    expect(Math.abs(s.p[0] - 0.9)).toBeGreaterThan(0.15);
    expect(Math.abs(s.yawDeg)).toBeLessThan(25);

    const spin = rig();
    for (let f = 0; f < 60; f++) spin.step(wheels(1, 1, 1, 1));
    expect(Math.abs(spin.snapshot().robot.omegaDps)).toBeGreaterThan(30);
  });

  /**
   * FIELD-CENTRIC IS THE DEFAULT, and this test used to assert the opposite.
   *
   * The old contract was robot-centric, on the argument that a turret means the chassis never
   * has to face the goal. That is true of the shot and false of the driver: with a turret the
   * chassis ends up pointing wherever it was last going, so 'forward' becomes a direction the
   * driver has to track in their head and cannot see. Push up, robot goes sideways, correct,
   * over-correct. Field-centric rotates the stick by the reported heading so up is always away
   * from the driver station.
   */
  const drive = (yawDeg: number, g: GamepadState) => {
    const w = rig();
    w.robot.place([0, 0.17, 0], yawDeg);
    const brain = new BuiltinTeleOp(robotSpec as unknown as RobotSpec, new ShotTable([]));
    for (let f = 0; f < 60; f++) {
      w.setGamepads(g, emptyGamepad());
      w.step(brain.update(w.sensors(), g, w.seq));
    }
    return w.robot.pos;
  };

  it('field-centric by default: the stick drives the FIELD, whatever way the robot points', () => {
    const g = emptyGamepad();
    g.left_stick_y = -1;          // stick up
    // Yaw 0 and yaw 90 must go the SAME way in the world. That is the whole property.
    const at0 = drive(0, g);
    const at90 = drive(90, g);
    expect(at0[2]).toBeGreaterThan(0.3);            // +Z, away from the driver station
    expect(at90[2]).toBeGreaterThan(0.3);            // and the same, nose sideways
    expect(Math.abs(at90[0])).toBeLessThan(0.15);    // not along its own nose
  });

  it('holding Y gives robot-centric back, for a drifted IMU', () => {
    const g = emptyGamepad();
    g.left_stick_y = -1;
    g.y = true;
    // Facing +X (yaw 90): robot-centric forward is along its own nose, +X.
    const p = drive(90, g);
    expect(p[0]).toBeGreaterThan(0.3);
    expect(Math.abs(p[2])).toBeLessThan(0.15);
  });

  it('strafing left and right are mirror images', () => {
    const spec = robotSpec as unknown as RobotSpec;
    const run = (dir: number) => {
      const w = rig();
      w.robot.place([0, 0.17, 0], 0);
      const brain = new BuiltinTeleOp(spec, new ShotTable([]));
      const g = emptyGamepad();
      g.left_stick_x = dir;
      for (let f = 0; f < 60; f++) {
        w.setGamepads(g, emptyGamepad());
        w.step(brain.update(w.sensors(), g, w.seq));
      }
      return w.robot.pos[0];
    };
    const left = run(-1);
    const right = run(1);
    expect(Math.sign(left)).toBe(-Math.sign(right));
    expect(Math.abs(Math.abs(left) - Math.abs(right))).toBeLessThan(0.05);
  });

  it('the built-in TeleOp drives the robot forward when the stick is pushed forward', () => {
    const world = rig();
    const brain = new BuiltinTeleOp(robotSpec as unknown as RobotSpec, new ShotTable([]));
    const g = emptyGamepad();
    g.left_stick_y = -1; // forward
    const start = world.robot.pos[2];
    for (let f = 0; f < 90; f++) {
      world.setGamepads(g, emptyGamepad());
      world.step(brain.update(world.sensors(), g, world.seq));
    }
    expect(world.robot.pos[2] - start).toBeGreaterThan(0.3);
  });
});
