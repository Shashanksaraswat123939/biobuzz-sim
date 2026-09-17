import { describe, it, expect } from 'vitest';
import { leadShot, muzzleVelocity } from '../packages/core/src/robot/builtinTeleOp.js';
import { DEG } from '../packages/core/src/units.js';

type Lead = { azimuthDeg: number; speed: number; elevationDeg: number };

/**
 * The ball's velocity in the GROUND frame, once the robot's own is added in. Vertical too:
 * the lead solves the elevation as well as the azimuth and the speed, and a check that only
 * looks at the ground track cannot see the error that made radial motion miss entirely.
 */
function resulting(led: Lead, vx: number, vy: number, headingDeg: number) {
  const horiz = led.speed * Math.cos(led.elevationDeg * DEG);
  const a = (headingDeg + led.azimuthDeg) * DEG;
  const bx = horiz * Math.cos(a) + vx;
  const by = horiz * Math.sin(a) + vy;
  return {
    bearingDeg: (Math.atan2(by, bx) * 180) / Math.PI,
    horiz: Math.hypot(bx, by),
    vert: led.speed * Math.sin(led.elevationDeg * DEG),
  };
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

    const naive = resulting({ azimuthDeg: bearing, speed, elevationDeg: elev }, vx, vy, heading);
    expect(Math.abs(naive.bearingDeg - (heading + bearing))).toBeGreaterThan(10); // it does miss

    const led = leadShot(bearing, speed, elev, vx, vy, heading);
    const actual = resulting(led, vx, vy, heading);
    expect(actual.bearingDeg).toBeCloseTo(heading + bearing, 4);
    // and the ball still arrives with the horizontal speed the shot table asked for
    expect(actual.horiz).toBeCloseTo(speed * Math.cos(elev * DEG), 4);
  });

  it("leaves the ball with the table's FULL launch vector, vertical included", () => {
    // The invariant the lead exists to hold: whatever the robot is doing, the ball's
    // ground-frame velocity is the one a stationary robot would have given it. Fixing the
    // elevation and solving only the speed holds the horizontal and breaks the vertical --
    // closing at 0.4 m/s that dropped the hang time enough that the ball never reached the
    // mouth's height at all, in the sim and in tools/_lead.ts alike.
    const fieldBearing = (heading + bearing) * DEG;
    const wantHoriz = speed * Math.cos(elev * DEG);
    const wantVert = speed * Math.sin(elev * DEG);
    for (const [along, across] of [[0, 0], [2, 0], [-2, 0], [0, 2], [1.5, -1.5]] as const) {
      const vx = Math.cos(fieldBearing) * along - Math.sin(fieldBearing) * across;
      const vy = Math.sin(fieldBearing) * along + Math.cos(fieldBearing) * across;
      const a = resulting(leadShot(bearing, speed, elev, vx, vy, heading), vx, vy, heading);
      expect(a.bearingDeg).toBeCloseTo(heading + bearing, 4);
      expect(a.horiz).toBeCloseTo(wantHoriz, 4);
      expect(a.vert).toBeCloseTo(wantVert, 4);
    }
  });

  it('clamps the solved elevation to the hood travel', () => {
    // Closing almost as fast as the ball's horizontal leaves a near-vertical solution, and a
    // hood that cannot get there must say so by stopping at its stop -- not by dividing by
    // cos(85 deg) and asking for a speed no wheel has.
    const fieldBearing = (heading + bearing) * DEG;
    const v = speed * Math.cos(elev * DEG) - 0.05;
    const r = leadShot(bearing, speed, elev, Math.cos(fieldBearing) * v, Math.sin(fieldBearing) * v, heading, [30, 85]);
    expect(r.elevationDeg).toBeLessThanOrEqual(85);
    expect(r.elevationDeg).toBeGreaterThanOrEqual(30);
    expect(r.speed).toBeLessThan(speed * 1.2);
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
      const actual = resulting(r, Math.cos(fieldBearing) * v, Math.sin(fieldBearing) * v, heading);
      expect(actual.bearingDeg).toBeCloseTo(heading + bearing, 4);
    }
  });
});

/**
 * THE INVARIANT THE TWO HALVES EXIST TO HOLD. Robot.launch() gives the ball
 * v_exit*dir + v_cg + omega x r; muzzleVelocity() + leadShot() subtract the same
 * v_cg + omega x r before solving. So the ball's GROUND-frame launch vector is the one a
 * standing robot would have given it, whatever the chassis and the turret are doing.
 *
 * It is a round trip on purpose. Each half is separately plausible and wrong: leading on the
 * chassis velocity while the flight also uses the chassis velocity is self-consistent and
 * models a robot that does not exist, and fixing either half alone puts 4.7 in of lateral
 * error at 60 in straight into the shot. If someone moves the turret axis off the tracked
 * point in Robot.muzzle() and not here, this test is what fails.
 */
describe('the muzzle velocity, not the chassis velocity', () => {
  const OFFSET = 0.12;

  it('is the chassis velocity when nothing is rotating', () => {
    const mv = muzzleVelocity(1.5, -0.5, 0, 37, 21, OFFSET);
    expect(mv.vx).toBeCloseTo(1.5, 12);
    expect(mv.vy).toBeCloseTo(-0.5, 12);
  });

  it('swings with omega x r, and the lever arm turns with the TURRET', () => {
    const w = 90; // deg/s CCW
    const rate = (w * Math.PI) / 180;
    // heading 0, turret 0: r is +x, so omega x r is +y
    const a = muzzleVelocity(0, 0, w, 0, 0, OFFSET);
    expect(a.vx).toBeCloseTo(0, 12);
    expect(a.vy).toBeCloseTo(rate * OFFSET, 12);
    // same chassis, turret swung 90 deg: r is +y, so omega x r is -x
    const b = muzzleVelocity(0, 0, w, 0, 90, OFFSET);
    expect(b.vx).toBeCloseTo(-rate * OFFSET, 12);
    expect(b.vy).toBeCloseTo(0, 12);
    // a muzzle BEHIND the axis swings the other way
    const back = muzzleVelocity(0, 0, w, 0, 0, -0.043);
    expect(back.vy).toBeCloseTo(-rate * 0.043, 12);
  });

  it('leaves the ball with the table launch vector while the robot yaws', () => {
    const elev = 45;
    const speed = 10;
    const heading = 30;
    const bearing = 20;
    const wantHoriz = speed * Math.cos(elev * DEG);
    const wantVert = speed * Math.sin(elev * DEG);

    for (const omega of [-134, -90, -30, 0, 30, 90, 134]) {
      for (const [vx, vy] of [[0, 0], [1.2, 0], [0, -0.8], [0.9, 1.1]] as const) {
        // What the aim believes the muzzle is doing...
        const mv = muzzleVelocity(vx, vy, omega, heading, bearing, OFFSET);
        const led = leadShot(bearing, speed, elev, mv.vx, mv.vy, heading, [0, 90]);
        // ...and what Robot.launch() then adds to the ball: the SAME vector.
        const got = resulting(led, mv.vx, mv.vy, heading);
        expect(got.bearingDeg).toBeCloseTo(heading + bearing, 4);
        expect(got.horiz).toBeCloseTo(wantHoriz, 4);
        expect(got.vert).toBeCloseTo(wantVert, 4);
      }
    }
  });

  it('a yaw the aim ignores is a real miss: the term is worth inches, not rounding', () => {
    // The regression guard. Aiming on the chassis velocity while the ball leaves with the
    // muzzle's is the pre-fix behaviour; price it at the doc's own case.
    const elev = 45;
    const speed = 10;
    const heading = 0;
    const bearing = 0;
    const omega = 90;
    const mv = muzzleVelocity(0, 0, omega, heading, bearing, OFFSET);
    const naive = leadShot(bearing, speed, elev, 0, 0, heading, [0, 90]); // ignores it
    const off = resulting(naive, mv.vx, mv.vy, heading);
    const driftDeg = Math.abs(off.bearingDeg - (heading + bearing));
    expect(driftDeg).toBeGreaterThan(1.0);
    // 60 in of range, in inches across the mouth: the mouth gives 0.5 in of clearance a lip.
    expect(60 * Math.tan(driftDeg * DEG)).toBeGreaterThan(1.5);
  });
});
