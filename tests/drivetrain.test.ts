import { describe, it, expect, beforeAll } from 'vitest';
import params from '../config/params.json';
import robotSpec from '../config/robot.json';
import { World, initPhysics, emptyGamepad } from '../packages/core/src/physics/world.js';
import { BuiltinTeleOp, ShotTable, SPEED_STEP } from '../packages/core/src/robot/builtinTeleOp.js';
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

  it('robot-centric by default: forward is the nose, which is where the intake is', () => {
    const g = emptyGamepad();
    g.left_stick_y = -1;
    // Facing +X (yaw 90): the nose points at +X, so that is where "forward" goes.
    const p = drive(90, g);
    expect(p[0]).toBeGreaterThan(0.3);
    expect(Math.abs(p[2])).toBeLessThan(0.15);
  });

  it('holding R3 gives field-centric, measured from the re-zero heading', () => {
    const g = emptyGamepad();
    g.left_stick_y = -1;
    g.right_stick_button = true;
    // Same yaw 90, but the stick now points down the FIELD (+Z) whatever the robot does.
    const p = drive(90, g);
    expect(p[2]).toBeGreaterThan(0.3);
    expect(Math.abs(p[0])).toBeLessThan(0.15);
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

describe('the driver layout', () => {
  const step = (brain: BuiltinTeleOp, g: GamepadState, n = 3) => {
    const w = rig();
    for (let i = 0; i < n; i++) brain.update(w.sensors(), g, i, 1 / 60);
  };

  it('Y and A ratchet the speed gear between one detent and full, and nowhere outside it', () => {
    const brain = new BuiltinTeleOp(robotSpec as unknown as RobotSpec, new ShotTable([]));
    const tap = (k: 'y' | 'a') => {
      const g = emptyGamepad();
      (g as unknown as Record<string, boolean>)[k] = true;
      step(brain, g, 1);
      step(brain, emptyGamepad(), 1);
    };
    expect(brain.state.speedScale).toBeCloseTo(1, 6);
    for (let i = 0; i < 9; i++) tap('a');
    expect(brain.state.speedScale).toBeCloseTo(SPEED_STEP, 6);   // floors, never zero or negative
    for (let i = 0; i < 9; i++) tap('y');
    expect(brain.state.speedScale).toBeCloseTo(1, 6);            // and ceilings at full
  });

  it('the gear actually limits the robot, and a lower gear is slower', () => {
    const far = (scale: number) => {
      const w = rig();
      w.robot.place([0, 0.17, 0], 0);
      const brain = new BuiltinTeleOp(robotSpec as unknown as RobotSpec, new ShotTable([]));
      brain.state.speedScale = scale;
      const g = emptyGamepad();
      g.left_stick_y = -1;
      for (let i = 0; i < 90; i++) w.step(brain.update(w.sensors(), g, i, 1 / 60));
      return Math.hypot(w.robot.pos[0], w.robot.pos[2]);
    };
    const full = far(1);
    const low = far(SPEED_STEP);
    expect(low).toBeLessThan(full * 0.5);
    expect(low).toBeGreaterThan(0.02);   // still moves: a gear is a limit, not a brake
  });

  it('X turns one way and B the other, and neither translates', () => {
    const spin = (k: 'x' | 'b') => {
      const w = rig();
      w.robot.place([0, 0.17, 0], 0);
      const brain = new BuiltinTeleOp(robotSpec as unknown as RobotSpec, new ShotTable([]));
      const g = emptyGamepad();
      // The UI synthesises X/B into right_stick_x before the brain sees them; the brain's
      // own contract is still the yaw axis, so that is what is exercised here.
      g.right_stick_x = k === 'b' ? 1 : -1;
      for (let i = 0; i < 60; i++) w.step(brain.update(w.sensors(), g, i, 1 / 60));
      return { yaw: w.robot.yaw, d: Math.hypot(w.robot.pos[0], w.robot.pos[2]) };
    };
    const left = spin('x');
    const right = spin('b');
    expect(Math.sign(left.yaw)).toBe(-Math.sign(right.yaw));
    expect(Math.abs(left.yaw)).toBeGreaterThan(0.2);
    expect(left.d).toBeLessThan(0.08);
    expect(right.d).toBeLessThan(0.08);
  });
});
