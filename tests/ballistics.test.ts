/**
 * The flight, checked against things that are true whatever the constants are.
 *
 * PHYSICS_AND_SIMULATION.md section 11 asks for these and the repository had none of them:
 * a ball in vacuum on the closed-form parabola, the drag and lift laws at a known speed,
 * the world's integrator agreeing with the solver's from one initial state, a shot table
 * that stays on one branch, and a brain that refuses a range the table has no row for.
 * Every one of these was found wrong or unguarded on 2026-09-18 (docs/DECISIONS.md).
 */
import { describe, expect, it } from 'vitest';
import { readFileSync } from 'node:fs';
import params from '../config/params.json' with { type: 'json' };
import robotSpec from '../config/robot.json' with { type: 'json' };
import { simulateShot } from '../packages/core/src/physics/ballistics.js';
import { aeroForce } from '../packages/core/src/physics/aero.js';
import { World, initPhysics, emptyGamepad } from '../packages/core/src/physics/world.js';
import { BuiltinTeleOp, ShotTable } from '../packages/core/src/robot/builtinTeleOp.js';
import { DEG, RAD, inches, rpmToRadS } from '../packages/core/src/units.js';
import type { GamepadState, Params, RobotSpec, Vec3 } from '../packages/core/src/types.js';

const table = ShotTable.fromCsv(readFileSync(new URL('../java/teamcode/assets/shottable.csv', import.meta.url), 'utf8'));
const P = params as unknown as Params;
const SPEC = robotSpec as unknown as RobotSpec;

describe('the solver against closed forms', () => {
  it('flies the vacuum parabola when drag and lift are switched off', () => {
    const p = structuredClone(P);
    p.ball.Cd = 0;
    p.ball.clSlope = 0;
    const v = 6, el = 60 * DEG, h = 0.4;
    const t = simulateShot(p, { from: [0, h, 0], azimuth: 0, elevation: el, speed: v, radius: 0.0356, mass: 0.0249, spin: 0 }, 1.2, 1 / 480, [0.6, 1.2]);
    const y = (x: number) => h + x * Math.tan(el) - (p.env.g * x * x) / (2 * v * v * Math.cos(el) ** 2);
    // Semi-implicit Euler at 1/480 s is first order: a few millimetres over a metre, not 1e-6.
    expect(Math.abs(t.gateHeights[0] - y(0.6))).toBeLessThan(0.01);
    expect(Math.abs(t.gateHeights[1] - y(1.2))).toBeLessThan(0.01);
    expect(Math.abs(t.apex - (h + (v * Math.sin(el)) ** 2 / (2 * p.env.g)))).toBeLessThan(0.01);
  });

  it('applies quadratic drag against the velocity and Magnus lift up for backspin', () => {
    const r = P.ball.pollen.d_m / 2;
    const A = Math.PI * r * r;
    const { force } = aeroForce(P.ball, P.env.rho, r, [0, 0, 6], [0, 0, 0], 1 / 480);
    // F = 1/2 rho A Cd v^2, along -v.
    expect(force[2]).toBeCloseTo(-0.5 * P.env.rho * A * P.ball.Cd * 36, 6);
    expect(force[0]).toBeCloseTo(0, 12);
    expect(force[1]).toBeCloseTo(0, 12);
    // A ball moving +Z with backspin about -X (the launcher's convention) is lifted.
    const lift = aeroForce(P.ball, P.env.rho, r, [0, 0, 6], [-6 / r, 0, 0], 1 / 480).force;
    expect(lift[1]).toBeGreaterThan(0);
    expect(lift[2]).toBeLessThan(0);
  });
});

describe('the shot table', () => {
  it('stays on one branch: no adjacent rows more than 5 deg of hood apart', () => {
    // 60 deg / 2570 rpm at 74 in used to sit next to 70.7 deg / 2901 rpm at 78 in. The robot
    // interpolates between rows, so a driver crossing that boundary was handed a launch
    // halfway between two solutions, which solves nothing.
    const [lo, hi] = SPEC.hood.angleRange_deg;
    for (let i = 1; i < table.rows.length; i++) {
      const a = lo + table.rows[i - 1].hoodPos * (hi - lo);
      const b = lo + table.rows[i].hoodPos * (hi - lo);
      expect(Math.abs(a - b), `rows ${table.rows[i - 1].range_in} and ${table.rows[i].range_in} in`).toBeLessThanOrEqual(5.01);
    }
  });

  it('carries a retention column that rises with range, as measured, not the injected-ball model', () => {
    // The measured column is 0.5-0.7 close in and 1.0 from 70 in. The entry model it
    // replaced had the slope's sign wrong and regenerating the table used to restore it.
    const near = table.lookup(34).pStay as number;
    const far = table.lookup(70).pStay as number;
    expect(far).toBeGreaterThan(near);
    expect(far).toBeGreaterThan(0.9);
  });
});

describe('the world and the solver agree about a flight', () => {
  it('lands a stationary 36 in shot where simulateShot says, to 2 cm at the mouth plane', async () => {
    await initPhysics();
    const p = structuredClone(P);
    const spec = structuredClone(SPEC);
    spec.flywheel.scatter = { angle_deg: 0, yaw_deg: 0, speedFrac: 0 };
    spec.flywheel.minLandProb = 0;
    p.ball.pollen.dVar = 0;
    p.ball.pollen.mVar = 0;
    const staging = Array.from({ length: 12 }, () => ({ kind: 'pollen' as const, pos: [0, -5, 0] as Vec3 }));
    const w = new World({ params: p, robot: spec, staging, alliance: 'red', seed: 5 });
    for (const b of w.balls.balls) w.balls.park(b);
    const mouth = w.hives.red.upCellMouthWorld();
    const z = mouth[2] + inches(36);
    w.robot.place([mouth[0], spec.chassis.height_m / 2 + spec.chassis.clearance_m, z], Math.atan2(0, -1) * RAD);
    const brain = new BuiltinTeleOp(spec, table);
    let loaded = 0;
    // Boxed, because the assignment happens inside a closure and TypeScript's narrowing
    // would otherwise decide it is still null at the loop condition.
    const box: { track: { pts: Vec3[]; vel: Vec3; spin: number } | null } = { track: null };
    const step = (g: GamepadState) => {
      while (w.robot.heldBalls().length < 2 && loaded < staging.length) {
        if (!w.robot.preload(w.balls, w.balls.balls[loaded])) break;
        loaded++;
      }
      const before = w.robot.shots;
      w.setGamepads(g, emptyGamepad());
      w.step(brain.update(w.sensors(), g, w.seq));
      if (!box.track && w.robot.shots > before) {
        const b = w.balls.balls[w.robot.lastShotBallId];
        const v = b.body.linvel(), s = b.body.angvel();
        box.track = { pts: [w.balls.pos(b)], vel: [v.x, v.y, v.z], spin: Math.hypot(s.x, s.y, s.z) };
      } else if (box.track) {
        box.track.pts.push(w.balls.pos(w.balls.balls[w.robot.lastShotBallId]));
      }
    };
    const arm = emptyGamepad(); step(arm); arm.dpad_up = true; step(arm);
    for (let f = 0; f < 150; f++) step(emptyGamepad());
    for (let f = 0; f < 240 && (!box.track || box.track.pts.length < 90); f++) step({ ...emptyGamepad(), right_bumper: true });
    const tr = box.track;
    expect(tr, 'the robot fired').not.toBeNull();
    if (!tr) return;
    // Where each integrator crosses the mouth's z plane, from the same first state.
    const crossZ = (pts: Vec3[]): number => {
      for (let i = 1; i < pts.length; i++) {
        if (pts[i - 1][2] > mouth[2] && pts[i][2] <= mouth[2]) {
          const f = (pts[i - 1][2] - mouth[2]) / (pts[i - 1][2] - pts[i][2] || 1);
          return pts[i - 1][1] + (pts[i][1] - pts[i - 1][1]) * f;
        }
      }
      return NaN;
    };
    const sp = Math.hypot(tr.vel[0], tr.vel[1], tr.vel[2]);
    const sim = simulateShot(p, {
      from: tr.pts[0], azimuth: Math.atan2(tr.vel[0], tr.vel[2]), elevation: Math.asin(tr.vel[1] / sp), speed: sp,
      radius: p.ball.pollen.d_m / 2, mass: p.ball.pollen.m_kg, spin: tr.spin,
    }, 100);
    const world = crossZ(tr.pts);
    const solver = crossZ(sim.points);
    expect(Number.isFinite(world) && Number.isFinite(solver)).toBe(true);
    // Rapier at 1/240 s with its own angular damping against the solver at 1/480 s: measured
    // 0.4-0.7 cm apart on 2026-09-18 (tools/flightcheck.ts).
    expect(Math.abs(world - solver)).toBeLessThan(0.02);
    // And the table's answer for 36 in puts the ball inside the mouth, not merely near it.
    expect(world).toBeGreaterThan(mouth[1] - 0.10);
    expect(world).toBeLessThan(mouth[1] + 0.10);
  }, 60_000);
});

describe('the brain and the table', () => {
  it('refuses a range the table has no row for, as the Java usable() does', async () => {
    await initPhysics();
    const spec = structuredClone(SPEC);
    const w = new World({ params: P, robot: spec, staging: [], alliance: 'red', seed: 2 });
    const brain = new BuiltinTeleOp(spec, table);
    brain.state.firing = true;
    const s = w.sensors();
    // Lie about the range only: everything else about the frame is real.
    s.game.upCellRangeIn = table.rows[0].range_in - 8;
    s.game.upCellOpenDeg = 0;   // square on to the mouth, so the range is the only objection
    for (let i = 0; i < 5; i++) brain.update(s, emptyGamepad(), i, 1 / 60);
    expect(brain.state.ready).toBe(false);
    expect(brain.state.hold).toMatch(/outside the table/);
    // A tenth of an inch inside the first row is a shot again.
    s.game.upCellRangeIn = table.rows[0].range_in + (spec.calibration?.rangeTrim_in ?? 0) + 0.1;
    for (let i = 0; i < 5; i++) brain.update(s, emptyGamepad(), i + 5, 1 / 60);
    expect(brain.state.hold).not.toMatch(/outside the table/);
    void rpmToRadS;
  }, 30_000);
});
