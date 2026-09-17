/**
 * Can the flywheel actually follow the lead, or is shoot-on-the-move impossible?
 *
 *   npm run tool -- tools/slew.ts
 *
 * The lead moves the target rpm by about 900 per m/s of RADIAL speed, and the readiness gate
 * wants the wheel inside 60 rpm for three consecutive loops. That combination fires one shot
 * in forty on the move. Two different things could be to blame and they have opposite fixes:
 *
 *   THE WHEEL CANNOT KEEP UP. If the target moves faster than the flywheel can accelerate,
 *   no controller and no gate can help, and shooting while driving is off the table for this
 *   hardware.
 *
 *   THE GATE IS ASKING THE WRONG QUESTION. If the wheel tracks fine but the gate measures
 *   error against a target that has already moved on, the shot is refused for a lag that does
 *   not matter -- and that is a software problem with a software fix.
 *
 * This measures both sides of it: how fast the wheel can change speed under full power, and
 * how fast the target actually moves when a robot drives at the hive.
 */
import params from '../config/params.json' with { type: 'json' };
import robotSpec from '../config/robot.json' with { type: 'json' };
import { World, initPhysics } from '../packages/core/src/physics/world.js';
import type { ActuatorFrame, Params, RobotSpec, Vec3 } from '../packages/core/src/types.js';

/** rpm per second the wheel manages between two speeds, accelerating and decelerating. */
async function slewRate(from: number, to: number, motorCount = 1): Promise<number> {
  await initPhysics();
  const p = structuredClone(params) as unknown as Params;
  const spec = structuredClone(robotSpec) as unknown as RobotSpec;
  spec.flywheel.motorCount = motorCount;
  const staging: { kind: 'pollen'; pos: Vec3 }[] = [];
  const world = new World({ params: p, robot: spec, staging, alliance: 'red', seed: 1 });
  const ticksPerRev = 28;
  const cmd = (rpm: number): ActuatorFrame => ({
    seq: 0,
    motors: { flywheel: { mode: 'RUN_USING_ENCODER', velocity: (rpm * ticksPerRev) / 60 } },
    servos: {},
  });
  // Settle at the start speed first: a slew measured from rest includes the spin-up.
  for (let f = 0; f < 60 * 6; f++) world.step(cmd(from));
  const t0 = world.t;
  const rising = to > from;
  for (let f = 0; f < 60 * 4; f++) {
    world.step(cmd(to));
    const rpm = world.robot.flywheelRpm;
    if (rising ? rpm >= to - 20 : rpm <= to + 20) break;
  }
  return Math.abs(to - from) / Math.max(1e-6, world.t - t0);
}

export async function main(): Promise<void> {
  const f = (robotSpec as unknown as RobotSpec).flywheel;
  console.log('FLYWHEEL SLEW vs WHAT THE LEAD DEMANDS');
  console.log('');
  console.log('  what the wheel can do, rpm per second:');
  const up: number[] = [];
  for (const motors of [1, 2, 3]) {
    const a = await slewRate(2500, 3400, motors);
    const b = await slewRate(3400, 2500, motors);
    up.push(a);
    console.log(`    ${motors} motor${motors > 1 ? 's' : ' '}   speeding up ${a.toFixed(0).padStart(5)} rpm/s   slowing down ${b.toFixed(0).padStart(5)} rpm/s`);
  }
  console.log('    (slowing down is free -- drag helps. Speeding up is the one that has to keep up.)');
  console.log('');
  // How fast the target moves: d(rpm)/dt = d(rpm)/dv * dv/dt, and d(rpm)/dv is 442/cos(hood)
  // for this shooter -- exit speed is k*r*omega, so rpm per m/s is fixed by the gearing.
  const perMps = (hoodDeg: number) => (60 / (2 * Math.PI)) / (f.k * f.r_fly_m) / Math.cos((hoodDeg * Math.PI) / 180);
  console.log('  what the lead demands, for a robot ACCELERATING along the shot line:');
  console.log('    hood   rpm per m/s   at 1 m/s^2   at 2 m/s^2   at 4 m/s^2');
  for (const hood of [50, 61, 72]) {
    const k = perMps(hood);
    console.log(`    ${hood} deg  ${k.toFixed(0).padStart(11)}   ${k.toFixed(0).padStart(10)}   ${(k * 2).toFixed(0).padStart(10)}   ${(k * 4).toFixed(0).padStart(10)}`);
  }
  console.log('');
  console.log('  A wheel that slews faster than the target moves can TRACK it; the error then');
  console.log('  settles to lag, not to a runaway. If that holds, the gate is the problem.');
  console.log('');
  console.log('  HOW HARD THE ROBOT MAY ACCELERATE ALONG THE SHOT LINE and still be tracked:');
  console.log('    motors   hood 50    hood 61    hood 72');
  up.forEach((rate, i) => {
    const cell = (hood: number) => `${(rate / perMps(hood)).toFixed(1)} m/s^2`;
    console.log(`    ${String(i + 1).padStart(6)}   ${cell(50).padStart(8)}   ${cell(61).padStart(8)}   ${cell(72).padStart(8)}`);
  });
  console.log('');
  console.log('  Note what is NOT here: velocity. A steady speed holds the target rpm still,');
  console.log('  however fast it is -- only CHANGING it moves the target. Shooting on the move');
  console.log('  is an acceleration budget, not a speed limit.');
}
