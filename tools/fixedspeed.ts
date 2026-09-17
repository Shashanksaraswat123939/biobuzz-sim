/**
 * Take the flywheel out of the control loop: run it at ONE speed and let the hood aim.
 *
 *   npm run tool -- tools/fixedspeed.ts [--rpm 3000]
 *
 * Everything that makes shooting while accelerating hard is a property of the FLYWHEEL. It
 * has inertia, so it lags a moving target (1102 rpm/s against a lead that demands 912 per
 * m/s of radial speed). Its speed can only be measured to about 107 rpm a count, so the
 * firing window is +-60 rpm, which is +-21 cm of range into a pocket 22.5 cm deep. Leading on
 * the predicted release velocity halves the resulting bias and cannot do better, because you
 * cannot correct an error smaller than you can measure.
 *
 * None of that is true of the HOOD. It is a servo: commanded to a position, no speed to
 * measure, no inertia worth chasing. So hold the wheel at a constant rpm all match and solve
 * for range with the hood instead.
 *
 * The robot's own velocity then does not have to be cancelled at all -- it is simply part of
 * the ball's launch condition. Exit speed S at hood angle theta, plus a radial speed v, leaves
 * the ball with
 *
 *     horizontal  S*cos(theta) + v        vertical  S*sin(theta)
 *
 * which is a different launch speed AND a different launch angle, both known exactly the
 * instant the ball goes. This asks the only question that matters: for a fixed S, is there a
 * hood angle that puts the ball in the pocket, for every range and every speed the robot can
 * be doing?
 */
import params from '../config/params.json' with { type: 'json' };
import robotJson from '../config/robot.json' with { type: 'json' };
import { simulateShot, type Aperture } from '../packages/core/src/physics/ballistics.js';
import { DEG, M_TO_IN, inches, rpmToRadS } from '../packages/core/src/units.js';
import { mouthLips } from './shottable.js';
import type { Params, RobotSpec, Vec3 } from '../packages/core/src/types.js';

/** Does this launch thread the mouth? Same near-lip/far-lip test the shot table uses. */
function threads(p: Params, spec: RobotSpec, ap: Aperture, speed: number, elevDeg: number, ballR: number): boolean {
  const spinPerSpeed = spec.flywheel.type === 'single' ? 1 / ballR : 0;
  const t = simulateShot(p, {
    from: [0, spec.turret.muzzleHeight_m, 0],
    azimuth: 0,
    elevation: elevDeg * DEG,
    speed,
    radius: ballR,
    mass: p.ball.pollen.m_kg,
    // Spin is set by the wheel, which is at a FIXED speed -- it does not change with the hood.
    spin: spinPerSpeed * speed,
  }, ap.farRange, 1 / 480, [ap.nearRange, ap.farRange]);
  const overNear = t.gateHeights[0] > ap.nearHeight;
  const underFar = t.gateHeights[1] < ap.farHeight && Number.isFinite(t.gateHeights[1]);
  return overNear && underFar && t.descentAngle > 10 * DEG;
}

export async function main(argv: string[] = []): Promise<void> {
  const i = argv.indexOf('--rpm');
  const p = params as unknown as Params;
  const spec = robotJson as unknown as RobotSpec;
  const f = spec.flywheel;
  const ballR = p.ball.pollen.d_m / 2;
  const lips = mouthLips(p);
  const [hoodLo, hoodHi] = spec.hood.angleRange_deg;

  const rpms = i >= 0 ? [Number(argv[i + 1])] : [2600, 3000, 3400, 3800];
  const ranges = [40, 55, 70, 85, 100];
  const vels = [-1.5, -1.0, -0.5, 0, 0.5, 1.0, 1.5];   // + is closing on the hive

  console.log('ONE FLYWHEEL SPEED, HOOD DOES THE AIMING');
  console.log(`  hood range ${hoodLo}-${hoodHi} deg, searched every 0.5 deg`);
  console.log('  + closing, - retreating. A cell is the hood angle that lands it, or "--".');
  console.log('');

  for (const rpm of rpms) {
    const S = f.k * f.r_fly_m * rpmToRadS(rpm);
    console.log(`  ${rpm} rpm  =  ${S.toFixed(2)} m/s exit`);
    console.log(`    range     ${vels.map((v) => `${v > 0 ? '+' : ''}${v.toFixed(1)}`.padStart(6)).join('')}`);
    let solved = 0;
    let total = 0;
    for (const range_in of ranges) {
      const centreZ = (lips.near.z + lips.far.z) / 2 + inches(range_in);
      const muzzleZ = centreZ - spec.turret.muzzleOffset_m;
      const ap: Aperture = {
        nearRange: muzzleZ - lips.near.z,
        nearHeight: lips.near.y + ballR,
        farRange: muzzleZ - lips.far.z,
        farHeight: lips.far.y - ballR,
      };
      const cells = vels.map((v) => {
        total++;
        for (let hood = hoodLo; hood <= hoodHi; hood += 0.5) {
          // The robot's velocity is part of the launch, not something to cancel.
          const hx = S * Math.cos(hood * DEG) + v;
          const hy = S * Math.sin(hood * DEG);
          if (hx <= 0) continue;
          const ground = Math.hypot(hx, hy);
          const elev = (Math.atan2(hy, hx) * 180) / Math.PI;
          if (threads(p, spec, ap, ground, elev, ballR)) {
            solved++;
            return hood.toFixed(0).padStart(6);
          }
        }
        return '    --';
      });
      console.log(`    ${(range_in * 0.0254).toFixed(2)} m  ${cells.join('')}`);
    }
    console.log(`    -> ${solved} of ${total} solved`);
    console.log('');
  }
  console.log('  Every cell that solves is a shot the robot can take WITHOUT the flywheel');
  console.log('  knowing anything about how it is moving: same rpm, different hood.');
  console.log('');

  // THE POINT OF THE WHOLE EXERCISE: what does moving the compensation from the wheel to the
  // hood buy, in the two currencies that were the problem -- how fast it can follow, and how
  // precisely it can be set.
  const rpm = rpms[rpms.length - 1];
  const S = f.k * f.r_fly_m * rpmToRadS(rpm);
  const range_in = 70;
  const centreZ = (lips.near.z + lips.far.z) / 2 + inches(range_in);
  const muzzleZ = centreZ - spec.turret.muzzleOffset_m;
  const ap: Aperture = {
    nearRange: muzzleZ - lips.near.z,
    nearHeight: lips.near.y + ballR,
    farRange: muzzleZ - lips.far.z,
    farHeight: lips.far.y - ballR,
  };
  const hoodFor = (v: number): number => {
    for (let hood = hoodLo; hood <= hoodHi; hood += 0.25) {
      const hx = S * Math.cos(hood * DEG) + v;
      const hy = S * Math.sin(hood * DEG);
      if (hx <= 0) continue;
      if (threads(p, spec, ap, Math.hypot(hx, hy), (Math.atan2(hy, hx) * 180) / Math.PI, ballR)) return hood;
    }
    return NaN;
  };
  const a = hoodFor(-0.5);
  const b = hoodFor(0.5);
  const perMps = Math.abs(b - a);
  console.log('  WHAT IT BUYS, at 1.78 m:');
  console.log(`    the hood has to move ${perMps.toFixed(1)} deg per m/s of radial speed`);
  console.log(`    the hood servo does ${spec.hood.speed_dps} deg/s, so it tracks ${(spec.hood.speed_dps / perMps).toFixed(1)} m/s^2 of radial acceleration`);
  console.log('    the flywheel tracked 1.2 m/s^2 (tools/slew.ts), and no FTC robot accelerates at 14');
  console.log('');
  console.log('    precision: 1 deg of hood is 3.5 cm of range (tools/apercheck.ts),');
  console.log('    against +-21 cm for the +-60 rpm firing window the wheel is gated on.');
  console.log('    A servo is COMMANDED to a position; a flywheel speed has to be measured,');
  console.log('    and one encoder count over a 20 ms window is 107 rpm. That is the whole');
  console.log('    difference: the hood has no measurement in its loop to be wrong about.');
}
