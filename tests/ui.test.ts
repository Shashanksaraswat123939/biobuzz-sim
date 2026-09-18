import { describe, it, expect } from 'vitest';
import params from '../config/params.json';
import robotSpec from '../config/robot.json';
import { TUNABLES } from '../packages/ui/src/tune.js';
import { remap, type Paddles } from '../packages/ui/src/input.js';
import { emptyGamepad } from '../packages/core/src/physics/world.js';
import type { GamepadState, Params, RobotSpec } from '../packages/core/src/types.js';

const p = params as unknown as Params;
const r = robotSpec as unknown as RobotSpec;

/**
 * The Variables panel writes straight into the objects the physics runs on, so a slider that
 * cannot represent its own configured value is not a cosmetic problem: it renders pinned at
 * one end showing a number the simulator is not using, and the first drag rewrites the
 * constant to the nearest value the slider CAN say.
 *
 * Found that way: flywheel inertia was 0.000391 in config against a slider starting at
 * 0.0005, so touching it moved the wheel's inertia by 28% -- and inertia is what sets the
 * per-shot dip, which is what the last shooter decision turned on.
 */
describe('the Variables sliders', () => {
  it('can represent the value the simulator is actually running on', () => {
    for (const t of TUNABLES) {
      const v = t.get(p, r);
      expect(v, `${t.group} / ${t.label} reads ${v}`).toBeGreaterThanOrEqual(t.min);
      expect(v, `${t.group} / ${t.label} reads ${v}`).toBeLessThanOrEqual(t.max);
    }
  });

  it('can land ON that value, not merely bracket it', () => {
    // A step too coarse to hit the configured value moves it as soon as the slider is touched.
    for (const t of TUNABLES) {
      const v = t.get(p, r);
      const steps = (v - t.min) / t.step;
      expect(Math.abs(steps - Math.round(steps)), `${t.group} / ${t.label}`).toBeLessThan(0.51);
    }
  });

  it('round-trips: what it writes is what it reads back', () => {
    for (const t of TUNABLES) {
      const p2 = structuredClone(params) as unknown as Params;
      const r2 = structuredClone(robotSpec) as unknown as RobotSpec;
      const mid = t.min + Math.round(((t.max - t.min) / 2) / t.step) * t.step;
      t.set(p2, r2, mid);
      expect(t.get(p2, r2), `${t.group} / ${t.label}`).toBeCloseTo(mid, 6);
    }
  });

  it('never offers a setting the rules forbid', () => {
    // A cap written in a hint is a suggestion. G407 allows 4 SCORING ELEMENTS; the slider
    // used to go to 12, and the config it shipped with was already over.
    const hopper = TUNABLES.find((t) => t.label === 'Hopper capacity');
    expect(hopper).toBeDefined();
    expect(hopper!.max).toBeLessThanOrEqual(4);
  });

  it('gives every slider a range, a step and a reason to exist', () => {
    for (const t of TUNABLES) {
      expect(t.max, t.label).toBeGreaterThan(t.min);
      expect(t.step, t.label).toBeGreaterThan(0);
      expect(t.hint.length, t.label).toBeGreaterThan(20);
    }
  });
});

/**
 * THE DRIVER LAYOUT, as a mapping rather than as a comment.
 *
 * This is the layer that keeps breaking: the brain's `GamepadState` is the wire the Java
 * OpMode sees and its field names are fixed, so every time the driver's layout changes, some
 * caller is still talking about the old meaning of a button. `remap` is the single place the
 * translation happens, and these are the claims that matter.
 */
describe('the pad-to-brain remap', () => {
  const pad = (over: Partial<GamepadState> = {}): GamepadState => ({ ...emptyGamepad(), ...over });
  const none: Paddles = { m1: false, m2: false, rezero: false };

  it('X and B become the yaw axis, opposite ways round, and nothing else moves', () => {
    const left = remap(pad({ x: true }), none);
    const right = remap(pad({ b: true }), none);
    expect(left.right_stick_x).toBeLessThan(0);
    expect(right.right_stick_x).toBeGreaterThan(0);
    expect(left.left_stick_x).toBe(0);
    expect(left.left_stick_y).toBe(0);
  });

  it('R2 drives forward, L2 drives back, and they add to the stick rather than fight it', () => {
    // Stick up is negative, so forward is negative on left_stick_y.
    expect(remap(pad({ right_trigger: 1 }), none).left_stick_y).toBeLessThan(-0.9);
    expect(remap(pad({ left_trigger: 1 }), none).left_stick_y).toBeGreaterThan(0.9);
    expect(remap(pad({ right_trigger: 1, left_trigger: 1 }), none).left_stick_y, 'both cancel').toBeCloseTo(0, 6);
    // Half a trigger on top of half a stick is full throttle, not a replaced stick.
    expect(remap(pad({ left_stick_y: -0.5, right_trigger: 0.5 }), none).left_stick_y).toBeCloseTo(-1, 6);
    // And it never runs past what the drive can be told.
    expect(remap(pad({ left_stick_y: -1, right_trigger: 1 }), none).left_stick_y).toBeCloseTo(-1, 6);
  });

  it('R1 is the trigger the brain reads, and never the auto-fire latch', () => {
    const g = remap(pad({ right_bumper: true }), none);
    expect(g.b, 'R1 must arrive as the hold-to-fire input').toBe(true);
    expect(g.right_bumper, 'R1 must not toggle the latch as well').toBe(false);
  });

  it('L1 toggles auto-aim and L3 the auto-fire latch', () => {
    expect(remap(pad({ left_bumper: true }), none).x, 'L1 is auto-aim').toBe(true);
    expect(remap(pad({ left_stick_button: true }), none).right_bumper, 'L3 is the auto-fire latch').toBe(true);
    // And pressing the face buttons must not reach either of them.
    const plain = remap(pad({ x: true, b: true }), none);
    expect(plain.x).toBe(false);
    expect(plain.right_bumper).toBe(false);
  });

  it('M1 and M2 nudge the turret, which the brain reads as the D-pad', () => {
    expect(remap(pad(), { m1: true, m2: false, rezero: false }).dpad_left).toBe(true);
    expect(remap(pad(), { m1: false, m2: true, rezero: false }).dpad_right).toBe(true);
    // A real D-pad press is consumed as M1/M2's fallback upstream, so it must not arrive here
    // as a second, independent turret nudge.
    expect(remap(pad({ dpad_left: true }), none).dpad_left).toBe(false);
  });

  it('leaves Y and A alone: they are the speed gear the brain reads directly', () => {
    expect(remap(pad({ y: true }), none).y).toBe(true);
    expect(remap(pad({ a: true }), none).a).toBe(true);
  });

  it('every button the brain acts on is driven by exactly one physical control', () => {
    // One press, one action. The old layout had Y doing the auto-fill AND the drive frame,
    // and L3 doing pause AND the re-zero; both fired together and neither was discoverable.
    const sources: [string, GamepadState, Paddles][] = [
      ['X', pad({ x: true }), none], ['B', pad({ b: true }), none],
      ['Y', pad({ y: true }), none], ['A', pad({ a: true }), none],
      ['R1', pad({ right_bumper: true }), none],
      ['L1', pad({ left_bumper: true }), none],
      ['L3', pad({ left_stick_button: true }), none],
      ['R3', pad({ right_stick_button: true }), none],
      ['D-pad up', pad({ dpad_up: true }), none],
      ['D-pad down', pad({ dpad_down: true }), none],
      ['M1', pad(), { m1: true, m2: false, rezero: false }],
      ['M2', pad(), { m1: false, m2: true, rezero: false }],
      ['Backspace', pad(), { m1: false, m2: false, rezero: true }],
    ];
    const watched = ['x', 'b', 'y', 'a', 'right_bumper', 'left_stick_button', 'right_stick_button',
      'dpad_up', 'dpad_left', 'dpad_right'] as const;
    const hits: Record<string, string[]> = {};
    for (const [name, g, p] of sources) {
      const out = remap(g, p) as unknown as Record<string, unknown>;
      for (const f of watched) if (out[f] === true) (hits[f] ??= []).push(name);
      if ((out.left_trigger as number) > 0) (hits.left_trigger ??= []).push(name);
    }
    for (const [field, from] of Object.entries(hits)) {
      expect(from, `${field} is driven by ${from.join(' and ')}`).toHaveLength(1);
    }
  });
});
