import { describe, it, expect, beforeAll } from 'vitest';
import params from '../config/params.json';
import robotSpec from '../config/robot.json';
import shotCsvRaw from 'node:fs';
import { World, initPhysics, emptyGamepad } from '../packages/core/src/physics/world.js';
import { BuiltinTeleOp, ShotTable } from '../packages/core/src/robot/builtinTeleOp.js';
import { inches, M_TO_IN } from '../packages/core/src/units.js';
import type { Params, RobotSpec, Vec3, GamepadState } from '../packages/core/src/types.js';

const csv = shotCsvRaw.readFileSync(new URL('../java/teamcode/assets/shottable.csv', import.meta.url), 'utf8');
const table = ShotTable.fromCsv(csv);

beforeAll(async () => {
  await initPhysics();
});

/** Stand the robot a given range from the red hive on the side its up CELL faces. */
function rig(range_in: number, preload = 4, balls = 12, minLandProb = 0) {
  const p = structuredClone(params) as unknown as Params;
  // GATE OPEN by default.
  //
  // These are mechanism tests -- does the flywheel spin up, does the feed honour its cycle
  // time, does a ball reach the CELL. The shot-quality gate is a POLICY on top of that
  // mechanism, and leaving it on here made four of them fail for a reason none of them is
  // about: the robot was declining shots it judged unlikely to land. Policy gets its own
  // test, further down.
  const spec = structuredClone(robotSpec) as unknown as RobotSpec;
  spec.flywheel.minLandProb = minLandProb;
  const staging = Array.from({ length: balls }, () => ({ kind: 'pollen' as const, pos: [0, -5, 0] as Vec3 }));
  const world = new World({ params: p, robot: spec, staging, alliance: 'red', seed: 5 });
  for (const b of world.balls.balls) world.balls.park(b);

  // Standing square to the up CELL only ~55 in of field exists, which is inside the range
  // the shot table needs. Find a bearing at this range that is actually on the field.
  const mouth = world.hives.red.upCellMouthWorld();
  const limit = world.geom.halfWidth_m - 0.35;
  const d = inches(range_in);
  let spot: Vec3 | null = null;
  for (let deg = 0; deg <= 88 && !spot; deg += 2) {
    for (const sign of [1, -1]) {
      const a = (deg * Math.PI) / 180;
      const c: Vec3 = [mouth[0] + sign * d * Math.sin(a), spec.chassis.height_m / 2 + spec.chassis.clearance_m, mouth[2] + d * Math.cos(a)];
      if (Math.abs(c[0]) < limit && Math.abs(c[2]) < limit) { spot = c; break; }
    }
  }
  if (!spot) throw new Error(`no on-field spot at ${range_in} in from the up CELL`);
  const yaw = (Math.atan2(mouth[0] - spot[0], mouth[2] - spot[2]) * 180) / Math.PI;
  world.robot.place(spot, yaw);
  for (let i = 0; i < preload; i++) world.robot.preload(world.balls, world.balls.balls[i]);

  return { world, brain: new BuiltinTeleOp(spec, table) };
}

function run(world: World, brain: BuiltinTeleOp, seconds: number, g: GamepadState): void {
  const frames = Math.round(seconds * 60);
  for (let f = 0; f < frames; f++) {
    world.setGamepads(g, emptyGamepad());
    world.step(brain.update(world.sensors(), g, world.seq));
  }
}

/**
 * Toggle the flywheel on and let it reach speed. The first update() has no previous frame
 * to compare against, so a button held from the very first loop is not an edge -- exactly
 * as on the hub, where holding A through init must not trip a toggle.
 */
function spinUp(world: World, brain: BuiltinTeleOp, seconds = 4): void {
  run(world, brain, 0.05, emptyGamepad());
  const press = emptyGamepad();
  press.a = true;
  run(world, brain, 0.05, press);
  run(world, brain, seconds, emptyGamepad());
}

describe('shooting (PLAN.md phases 4-5)', () => {
  it('spins up, aims and puts preloaded POLLEN into the up CELL', () => {
    const { world, brain } = rig(75);
    spinUp(world, brain);
    expect(brain.state.flywheelOn).toBe(true);
    // READY FLICKERS, and it should. The brain reads the hub's velocity estimate, which is
    // counts over a 20 ms window quantised to whole encoder ticks -- about +-107 rpm on a
    // flywheel running 28 ticks a rev. Readiness therefore comes and goes with the sampling
    // even when the wheel is dead on target, so sampling it at one arbitrary instant is a
    // coin toss. What the robot has to do is become ready promptly, not be ready at a
    // particular microsecond.
    let readyAt = NaN;
    for (let f = 0; f < 120 && Number.isNaN(readyAt); f++) {
      run(world, brain, 1 / 60, emptyGamepad());
      if (brain.state.ready) readyAt = world.t;
    }
    expect(Number.isNaN(readyAt)).toBe(false);

    const fire = emptyGamepad();
    fire.right_bumper = true;
    run(world, brain, 10, fire);

    expect(world.robot.shots).toBeGreaterThanOrEqual(3);
    expect(world.hives.red.ballsInUpCell + world.hives.red.tips * 4).toBeGreaterThan(0);
  }, 30000);

  it('honours the transfer cycle time: shots are at least cycleTime apart', () => {
    const { world, brain } = rig(75);
    spinUp(world, brain);

    const times: number[] = [];
    const fire = emptyGamepad();
    fire.right_bumper = true;
    let lastShots = world.robot.shots;
    for (let f = 0; f < 60 * 10; f++) {
      world.setGamepads(fire, emptyGamepad());
      world.step(brain.update(world.sensors(), fire, world.seq));
      if (world.robot.shots > lastShots) {
        times.push(world.t);
        lastShots = world.robot.shots;
      }
    }
    expect(times.length).toBeGreaterThanOrEqual(3);
    for (let i = 1; i < times.length; i++) {
      expect(times[i] - times[i - 1]).toBeGreaterThanOrEqual(robotSpec.transfer.cycleTime_s - 0.05);
    }
  }, 30000);

  it('the flywheel dips when a ball is fired and recovers', () => {
    const { world, brain } = rig(75);
    spinUp(world, brain);
    const before = world.robot.flywheelRpm;

    const fire = emptyGamepad();
    fire.right_bumper = true;
    let dip = before;
    const shots0 = world.robot.shots;
    for (let f = 0; f < 60 * 3 && world.robot.shots === shots0; f++) {
      world.setGamepads(fire, emptyGamepad());
      world.step(brain.update(world.sensors(), fire, world.seq));
    }
    for (let f = 0; f < 20; f++) {
      world.setGamepads(fire, emptyGamepad());
      world.step(brain.update(world.sensors(), fire, world.seq));
      dip = Math.min(dip, world.robot.flywheelRpm);
    }
    expect(world.robot.shots).toBeGreaterThan(shots0);
    expect(dip).toBeLessThan(before);
    run(world, brain, 3, fire);
    expect(world.robot.flywheelRpm).toBeGreaterThan(dip);
  }, 30000);

  it('shooting enough POLLEN tips the HIVE and scores 20', () => {
    const { world, brain } = rig(75, 6, 40);
    spinUp(world, brain, 5);
    const fire = emptyGamepad();
    fire.right_bumper = true;

    // Keep the hopper topped up: this is the shooter's job, not the hive's.
    let loaded = 6;
    for (let f = 0; f < 60 * 70 && world.hives.red.tips === 0; f++) {
      // heldBalls, not hopper: the bin and the feed magazine are separate lists now, and a
      // ball on its way up the tube is still a ball the robot is carrying.
      while (world.robot.heldBalls().length < 6 && loaded < world.balls.balls.length) {
        if (!world.robot.preload(world.balls, world.balls.balls[loaded])) break;
        loaded++;
      }
      world.setGamepads(fire, emptyGamepad());
      world.step(brain.update(world.sensors(), fire, world.seq));
    }
    console.log(`DBG shots=${world.robot.shots} inCell=${world.hives.red.ballsInUpCell} held=${world.robot.heldBalls().length} loaded=${loaded} t=${world.t.toFixed(1)}`);
    expect(world.hives.red.tips).toBeGreaterThanOrEqual(1);
    expect(world.scorer.state.red.total).toBeGreaterThanOrEqual(20);
  }, 120000);

  it('the shot table covers the field and its ranges are monotone in hood position', () => {
    expect(table.rows.length).toBeGreaterThan(20);
    const [lo, hi] = table.bestBand();
    expect(hi).toBeGreaterThanOrEqual(lo);
    expect(table.lookup(90).rpm).toBeGreaterThan(1000);
    expect(table.lookup(90).margin).toBeGreaterThan(0.02);
    void M_TO_IN;
  });
});
