import { describe, it, expect } from 'vitest';
import { leadShot } from '../packages/core/src/robot/builtinTeleOp.js';
import { DEG } from '../packages/core/src/units.js';

/** Where the ball actually ends up going once the robot's own velocity is added in. */
function resultingBearing(azimuthDeg: number, speed: number, elevDeg: number, vx: number, vy: number, headingDeg: number) {
  const horiz = speed * Math.cos(elevDeg * DEG);
  const a = (headingDeg + azimuthDeg) * DEG;
  const bx = horiz * Math.cos(a) + vx;
  const by = horiz * Math.sin(a) + vy;
  return { bearingDeg: (Math.atan2(by, bx) * 180) / Math.PI, horiz: Math.hypot(bx, by) };
}

describe('shooting on the move', () => {
  const elev = 45;
  const speed = 10;
  const heading = 30;
  const bearing = 20; // relative to the robot's heading

  it('a stationary robot needs no lead', () => {
    const r = leadShot(bearing, speed, elev, 0, 0, heading);
    expect(r.azimuthDeg).toBeCloseTo(bearing, 6);
    expect(r.speed).toBeCloseTo(speed, 6);
  });

  it('cancels a cross-track velocity: the ball flies at the target, not beside it', () => {
    // 2 m/s across the line of fire.
    const fieldBearing = (heading + bearing) * DEG;
    const cross = 2.0;
    const vx = -Math.sin(fieldBearing) * cross;
    const vy = Math.cos(fieldBearing) * cross;

    const naive = resultingBearing(bearing, speed, elev, vx, vy, heading);
    expect(Math.abs(naive.bearingDeg - (heading + bearing))).toBeGreaterThan(10); // it does miss

    const led = leadShot(bearing, speed, elev, vx, vy, heading);
    const actual = resultingBearing(led.azimuthDeg, led.speed, elev, vx, vy, heading);
    expect(actual.bearingDeg).toBeCloseTo(heading + bearing, 4);
    // and the ball still arrives with the horizontal speed the shot table asked for
    expect(actual.horiz).toBeCloseTo(speed * Math.cos(elev * DEG), 4);
  });

  it('turns the turret the opposite way for the opposite drift', () => {
    const fieldBearing = (heading + bearing) * DEG;
    const left = leadShot(bearing, speed, elev, -Math.sin(fieldBearing) * 2, Math.cos(fieldBearing) * 2, heading);
    const right = leadShot(bearing, speed, elev, Math.sin(fieldBearing) * 2, -Math.cos(fieldBearing) * 2, heading);
    expect(Math.sign(left.azimuthDeg - bearing)).toBe(-Math.sign(right.azimuthDeg - bearing));
    expect(Math.abs(left.azimuthDeg - bearing)).toBeGreaterThan(5);
  });

  it('always returns a wrapped azimuth, whatever the heading', () => {
    // The bug this guards: atan2 gives (-180, 180] and the heading is subtracted from it, so
    // a true bearing of +90 came back as -270 and clamped to the turret's end stop.
    for (const h of [-170, -135, -90, 0, 90, 135, 170]) {
      for (const b of [-170, -120, -45, 0, 45, 120, 170]) {
        const r = leadShot(b, 8, 80, 0, 0, h);
        expect(r.azimuthDeg).toBeGreaterThan(-180.001);
        expect(r.azimuthDeg).toBeLessThanOrEqual(180.001);
        // stationary robot: the lead must not move the aim at all
        expect(Math.abs(r.azimuthDeg - b)).toBeLessThan(0.01);
      }
    }
  });

  it('asks for less speed when closing and more when retreating', () => {
    const fieldBearing = (heading + bearing) * DEG;
    const closing = leadShot(bearing, speed, elev, Math.cos(fieldBearing) * 2, Math.sin(fieldBearing) * 2, heading);
    const away = leadShot(bearing, speed, elev, -Math.cos(fieldBearing) * 2, -Math.sin(fieldBearing) * 2, heading);
    expect(closing.speed).toBeLessThan(speed);
    expect(away.speed).toBeGreaterThan(speed);
    // Both still land on the bearing.
    for (const r of [closing, away]) {
      const v = r === closing ? 2 : -2;
      const actual = resultingBearing(r.azimuthDeg, r.speed, elev, Math.cos(fieldBearing) * v, Math.sin(fieldBearing) * v, heading);
      expect(actual.bearingDeg).toBeCloseTo(heading + bearing, 4);
    }
  });
});
