import { describe, it, expect } from 'vitest';
import robotSpec from '../config/robot.json';

/**
 * The rules the CONFIGURATION can break on its own, checked against the configuration.
 *
 * Each of these was a real finding, and each was invisible because the number it violates was
 * written down and read by nothing: `limits.maxServos` and `limits.maxMotors` are in the spec
 * type and no code has ever looked at them, so a 12 went in unchallenged. The nine-motor robot
 * (flywheel `motorCount: 2` on top of an eight-entry hardware map) got as far as the working
 * tree the same way.
 *
 * A rule nobody tests is not a constraint, it is a comment. These are cheap; keep them here as
 * the configuration grows.
 */
const spec = robotSpec as unknown as {
  hardware: Record<string, string>;
  limits: { maxMotors: number; maxServos: number };
  hopper: { capacity: number };
  flywheel: { motorCount?: number };
  turret: { enabled: boolean; type: string };
};

/**
 * Every port the robot actually spends.
 *
 * DISTINCT PORTS, from the hardware map, which is the only place that says what is plugged
 * in where. A multi-motor mechanism has to appear once per motor -- `flywheelA` and
 * `flywheelB` -- because that is what it does to the hub, and the separate check below is
 * that `flywheel.motorCount` agrees with how many ports were actually set aside for it. The
 * first version of this added `motorCount - 1` to the map's length, which is right while the
 * map names the mechanism once and double-counts the moment it names both.
 */
function ports() {
  const used = Object.values(spec.hardware);
  return {
    motors: new Set(used.filter((v) => /^motor\d+$/.test(v))).size,
    servos: new Set(used.filter((v) => /^servo\d+$/.test(v))).size,
    flywheelPorts: Object.entries(spec.hardware)
      .filter(([k, v]) => /^flywheel/i.test(k) && /^motor\d+$/.test(v)).length,
  };
}

describe('the rules the config can break', () => {
  it('R503: at most 8 motors, counting every motor on a multi-motor mechanism', () => {
    // "ROBOTS are limited to a total of 8 motors and 8 servos." A second flywheel motor is
    // worth a lot (tools/shooterab.ts) and there is no port for it until something gives one
    // up -- making the turret a servo is the trade, and it has to land in the same change.
    expect(ports().motors).toBeLessThanOrEqual(spec.limits.maxMotors);
    expect(spec.limits.maxMotors).toBe(8);
  });

  it('sets aside one port per flywheel motor', () => {
    // The nine-motor robot was exactly this gap: `motorCount: 2` in the flywheel block with a
    // hardware map that still named one `flywheel` port. The physics honoured the 2, the
    // inspector would have counted 9, and nothing joined the two numbers up.
    const want = Math.max(1, Math.round(spec.flywheel.motorCount ?? 1));
    expect(ports().flywheelPorts).toBe(want);
  });

  it('R503: at most 8 servos, whatever the port count of two hubs', () => {
    expect(ports().servos).toBeLessThanOrEqual(spec.limits.maxServos);
    expect(spec.limits.maxServos).toBe(8);
  });

  it('G407: the hopper may not hold more than 4 SCORING ELEMENTS', () => {
    expect(spec.hopper.capacity).toBeLessThanOrEqual(4);
  });

  it('assigns every mechanism a distinct port', () => {
    const used = Object.values(spec.hardware).filter((v) => /^(motor|servo)\d+$/.test(v));
    expect(new Set(used).size).toBe(used.length);
  });
});

describe('the feed bore', () => {
  // Three audit findings turned out to be one number (PHYSICS 9.12, 9.13, 9.14). The bore has
  // to pass the larger game element and still be single file, and the constraint that decides
  // 'single file' is the DIAGONAL of two POLLEN, not their side-by-side width. Get this wrong
  // narrow and a ball wedges in the doorway below entryY, entryBusy latches true, and the
  // magazine deadlocks for the rest of the match with the belt still driving.
  const bore = (robotSpec as unknown as { transfer: { boreSize_m?: number } }).transfer.boreSize_m ?? 0;
  const IN = 0.0254;
  const POLLEN_D = 2.8 * IN;
  const NECTAR_D = 3.62 * IN;

  it('passes a NECTAR', () => {
    expect(bore).toBeGreaterThan(NECTAR_D);
  });

  it('cannot take two POLLEN in any orientation', () => {
    expect(bore).toBeLessThan(POLLEN_D * Math.SQRT2);
  });

  it('leaves a POLLEN enough clearance not to wedge in the doorway', () => {
    // 0.2 in a side was measurably too little: seed 43 jammed at -2.23 in and fired nothing.
    expect((bore - POLLEN_D) / 2).toBeGreaterThan(0.3 * IN);
  });
});
