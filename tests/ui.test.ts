import { describe, it, expect } from 'vitest';
import params from '../config/params.json';
import robotSpec from '../config/robot.json';
import { TUNABLES } from '../packages/ui/src/tune.js';
import type { Params, RobotSpec } from '../packages/core/src/types.js';

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
