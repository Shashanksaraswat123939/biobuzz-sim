import { describe, it, expect } from 'vitest';
import params from '../config/params.json';
import robotSpec from '../config/robot.json';
import { reachAt, knobs, predict } from '../packages/core/src/analysis/sensitivity.js';
import { analyse, toCsv } from '../packages/core/src/analysis/report.js';
import { DEG } from '../packages/core/src/units.js';
import type { ShotInput } from '../packages/core/src/physics/ballistics.js';
import type { Params, ShotRecord } from '../packages/core/src/types.js';

const p = params as unknown as Params;
const spec = robotSpec as unknown as { flywheel: { k: number; r_fly_m: number; tolRpm: number; scatter: { angle_deg: number; speedFrac: number } } };

const shot = (over: Partial<ShotInput> = {}): ShotInput => ({
  from: [0, 0.3, 0],
  azimuth: 0,
  elevation: 45 * DEG,
  speed: 8,
  radius: p.ball.pollen.d_m / 2,
  mass: p.ball.pollen.m_kg,
  spin: 0,
  ...over,
});

const ks = () => knobs({
  tolRpm: spec.flywheel.tolRpm,
  hoodSteps: 200,
  scatterDeg: spec.flywheel.scatter.angle_deg,
  scatterSpeedFrac: spec.flywheel.scatter.speedFrac,
  k: spec.flywheel.k,
  rFly: spec.flywheel.r_fly_m,
});

const rec = (over: Partial<ShotRecord>): ShotRecord => ({
  n: 1, t: 0, rangeIn: 72, bearingDeg: 0, hoodDeg: 60, rpm: 3000, targetRpm: 3000,
  exitSpeed: 8, result: 'miss', missBy: 0, long_in: 0, lat_in: 0, aimErrDeg: 0, ...over,
});

describe('reachAt', () => {
  it('finds the descending crossing, not the climbing one', () => {
    // A 45 deg 8 m/s shot from 0.3 m passes 1.0 m twice. The answer must be the second.
    const r = reachAt(p, shot(), 1.0);
    expect(r).toBeGreaterThan(20);
    // Drag pulls it in from the 260 in vacuum answer, but not below the climbing crossing.
    expect(r).toBeLessThan(300);
  });

  it('is monotonic in exit speed: a faster ball goes further', () => {
    const slow = reachAt(p, shot({ speed: 7 }), 1.0);
    const fast = reachAt(p, shot({ speed: 9 }), 1.0);
    expect(fast).toBeGreaterThan(slow + 10);
  });

  it('returns NaN when the ball never reaches that height', () => {
    expect(reachAt(p, shot({ speed: 2 }), 3.0)).toBeNaN();
  });
});

describe('the predictor', () => {
  it('signs the derivatives the way the physics does', () => {
    const pred = predict(p, shot(), 1.0, 100, ks());
    const by = Object.fromEntries(pred.rows.map((r) => [r.key, r]));
    // More speed is further; more drag is shorter. If either flips, the panel is lying.
    expect(by.rpm.perStep_in).toBeGreaterThan(0);
    expect(by.Cd.perStep_in).toBeLessThan(0);
    // Every knob's share is a fraction, and together they are the whole variance.
    expect(pred.rows.reduce((a, r) => a + r.share, 0)).toBeCloseTo(1, 6);
    expect(pred.spread_in).toBeGreaterThan(0);
  });

  it('ranks by contribution, biggest first', () => {
    const pred = predict(p, shot(), 1.0, 100, ks());
    for (let i = 1; i < pred.rows.length; i++) {
      expect(pred.rows[i - 1].contrib_in).toBeGreaterThanOrEqual(pred.rows[i].contrib_in);
    }
  });

  it('a knob with no uncertainty contributes nothing', () => {
    const zeroed = ks().map((k) => (k.key === 'Cd' ? { ...k, sigma: 0 } : k));
    const pred = predict(p, shot(), 1.0, 100, zeroed);
    expect(pred.rows.find((r) => r.key === 'Cd')!.contrib_in).toBe(0);
  });
});

describe('the run report', () => {
  it('separates bias from spread, and names the one that dominates', () => {
    // Ten shots all 20 in long and tightly grouped: a table error, not a scatter problem.
    const biased = Array.from({ length: 10 }, (_, i) => rec({ n: i + 1, long_in: 20 + (i % 2), lat_in: 0 }));
    const r = analyse(biased, 120);
    expect(r.bias_in).toBeGreaterThan(19);
    expect(r.sd_in).toBeLessThan(2);
    expect(r.verdict).toContain('BIAS');
    expect(r.faults.join(' ')).toMatch(/LONG/);

    // Ten shots centred but scattered: no offset helps.
    const wide = Array.from({ length: 10 }, (_, i) => rec({ n: i + 1, long_in: i % 2 ? 40 : -40, lat_in: 0 }));
    const w = analyse(wide, 120);
    expect(Math.abs(w.bias_in)).toBeLessThan(1);
    expect(w.sd_in).toBeGreaterThan(30);
    expect(w.verdict).toContain('SPREAD');
  });

  it('flags shots that left while the flywheel was out of band', () => {
    const off = Array.from({ length: 10 }, (_, i) => rec({ n: i + 1, rpm: 3000, targetRpm: 3400 }));
    const r = analyse(off, 120);
    expect(r.firedOffSpeed).toBe(10);
    expect(r.faults.join(' ')).toMatch(/outside its 120 rpm band/);
  });

  it('ignores shots still in the air', () => {
    expect(analyse([rec({ result: 'flight' })], 120).n).toBe(0);
  });

  it('bins by range so a range-dependent fault is visible', () => {
    const mixed = [
      ...Array.from({ length: 3 }, (_, i) => rec({ n: i + 1, rangeIn: 48, result: 'cell', missBy: 2 })),
      ...Array.from({ length: 3 }, (_, i) => rec({ n: i + 4, rangeIn: 96, result: 'miss', missBy: 40 })),
    ];
    const r = analyse(mixed, 120);
    expect(r.byRange.map((b) => b.range_in)).toEqual([48, 96]);
    expect(r.byRange[0].landed).toBe(3);
    expect(r.byRange[1].landed).toBe(0);
    expect(r.byRange[1].meanMiss_in).toBeCloseTo(40, 3);
  });

  it('exports a CSV row per shot with a header', () => {
    const csv = toCsv([rec({ n: 1 }), rec({ n: 2 })]).split('\n');
    expect(csv).toHaveLength(3);
    expect(csv[0]).toContain('long_in');
    expect(csv[1].split(',')).toHaveLength(csv[0].split(',').length);
  });
});
