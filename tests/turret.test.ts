import { describe, it, expect, beforeAll } from 'vitest';
import params from '../config/params.json';
import robotSpec from '../config/robot.json';
import { World, initPhysics } from '../packages/core/src/physics/world.js';
import type { ActuatorFrame, Params, RobotSpec, Vec3 } from '../packages/core/src/types.js';

beforeAll(async () => {
  await initPhysics();
});

const TPD = robotSpec.turret.motor.ticksPerDeg;
const VMAX = robotSpec.turret.speed_dps;
const AMAX = robotSpec.turret.accel_dps2;

function rig() {
  const p = structuredClone(params) as unknown as Params;
  const staging: { kind: 'pollen'; pos: Vec3 }[] = [];
  return new World({ params: p, robot: robotSpec as unknown as RobotSpec, staging, alliance: 'red', seed: 1 });
}

const aim = (deg: number): ActuatorFrame => ({
  seq: 0,
  motors: { turret: { mode: 'RUN_TO_POSITION', target: Math.round(deg * TPD), power: 1 } },
  servos: {},
});

/** Step one frame and return the turret's state. */
function drive(world: World, act: ActuatorFrame, frames: number) {
  const trace: { deg: number; dps: number }[] = [];
  for (let f = 0; f < frames; f++) {
    world.step(act);
    const t = world.snapshot().robot.turret;
    trace.push({ deg: t.angleDeg, dps: t.omegaDps });
  }
  return trace;
}

describe('turret motion', () => {
  it('does not spin instantly: it has to accelerate', () => {
    const world = rig();
    const trace = drive(world, aim(90), 4);
    // One frame is 1/60 s, so the very first rate cannot exceed a*dt plus a little slop.
    expect(Math.abs(trace[0].dps)).toBeLessThan(AMAX * (1 / 60) * 1.5);
    expect(Math.abs(trace[0].dps)).toBeGreaterThan(0);
    // ...and it is still well short of the slew limit after one frame.
    expect(Math.abs(trace[0].dps)).toBeLessThan(VMAX * 0.5);
  });

  it('never exceeds the configured angular velocity or acceleration', () => {
    const world = rig();
    const trace = [...drive(world, aim(120), 90), ...drive(world, aim(-120), 180)];
    let prev = 0;
    for (const s of trace) {
      expect(Math.abs(s.dps)).toBeLessThanOrEqual(VMAX * 1.02);
      expect(Math.abs(s.dps - prev) * 60).toBeLessThanOrEqual(AMAX * 1.5);
      prev = s.dps;
    }
  });

  it('reaches the slew limit on a long move, in about v/a seconds', () => {
    const world = rig();
    const trace = drive(world, aim(120), 60);
    const peak = Math.max(...trace.map((s) => Math.abs(s.dps)));
    expect(peak).toBeGreaterThan(VMAX * 0.95);
    // ALLOW A FRAME EITHER SIDE. The trace samples at 60 Hz while the axis integrates at
    // 1/240, and a servo head reaches its slew limit in about 0.05 s -- three samples. A
    // percentage tolerance on a three-sample measurement is really a tolerance on where the
    // sampling grid happened to fall: at 261 deg/s and 5000 deg/s^2 the ideal ramp is 0.052 s
    // and one frame of quantisation is 0.017, a third of it. So the bound is stated in frames.
    const ideal = VMAX / AMAX;
    const rampFrames = trace.findIndex((s) => Math.abs(s.dps) >= VMAX * 0.95);
    expect(rampFrames / 60).toBeGreaterThan(ideal * 0.7 - 1 / 60);
    expect(rampFrames / 60).toBeLessThan(ideal * 1.6 + 1 / 60);
  });

  it('arrives on target and stops, without overshooting', () => {
    const world = rig();
    const trace = drive(world, aim(75), 180);
    const end = trace[trace.length - 1];
    expect(end.deg).toBeCloseTo(75, 0);
    expect(Math.abs(end.dps)).toBeLessThan(1);
    expect(Math.max(...trace.map((s) => s.deg))).toBeLessThan(75 + 2);
  });

  it('stops dead at its travel limits instead of driving through them', () => {
    const world = rig();
    const trace = drive(world, aim(400), 240);
    const end = trace[trace.length - 1];
    expect(end.deg).toBeCloseTo(robotSpec.turret.range_deg[1], 3);
    expect(Math.abs(end.dps)).toBeLessThan(1);
    expect(world.snapshot().robot.turret.atLimit).toBe(true);
  });

  it('a short move never reaches full speed: it is acceleration limited end to end', () => {
    const world = rig();
    const peak = Math.max(...drive(world, aim(5), 60).map((s) => Math.abs(s.dps)));
    // sqrt(a * travel) is the triangular-profile peak; nowhere near the slew limit.
    expect(peak).toBeLessThan(Math.sqrt(AMAX * 5) * 1.3);
    expect(peak).toBeLessThan(VMAX * 0.8);
  });
});
