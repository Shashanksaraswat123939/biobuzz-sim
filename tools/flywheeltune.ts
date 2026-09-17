/**
 * TuningFlywheel, sim side: hold a target RPM and report how the hub's velocity loop
 * behaves. Steady-state offset and ripple decide how tight the readiness window can be.
 *
 *   npm run tool -- tools/flywheeltune.ts [--target 3700]
 */
import params from '../config/params.json' with { type: 'json' };
import robotSpec from '../config/robot.json' with { type: 'json' };
import { World, initPhysics } from '../packages/core/src/physics/world.js';
import type { ActuatorFrame, Params, RobotSpec, Vec3 } from '../packages/core/src/types.js';

/**
 * Steady-state offset against COMMANDED SPEED, across the band the shot table actually uses.
 *
 * The loop settles dead on target at 2800 rpm and 64 rpm low at 3700, at every integral gain.
 * That is not tuning, it is the encoder: one count over a 20 ms window is 107 rpm, so the
 * closed loop can only park the REPORTED speed on the lattice of values the counter can
 * produce, and the true speed lands wherever the nearest lattice point puts it. The offset
 * should therefore sweep through zero with a 107 rpm period -- and it is worth up to half a
 * step, about 53 rpm or 7.5 in of range, which no single field trim can remove because it
 * changes with every range.
 */
async function scan(targets: number[], ticksPerRev: number): Promise<void> {
  console.log(' target   settled   offset   = inches');
  for (const targetRpm of targets) {
    const p = structuredClone(params) as unknown as Params;
    const spec = structuredClone(robotSpec) as unknown as RobotSpec;
    const staging: { kind: 'pollen'; pos: Vec3 }[] = [];
    const world = new World({ params: p, robot: spec, staging, alliance: 'red', seed: 1 });
    const act: ActuatorFrame = {
      seq: 0,
      motors: { flywheel: { mode: 'RUN_USING_ENCODER', velocity: (targetRpm * ticksPerRev) / 60 } },
      servos: {},
    };
    const tail: number[] = [];
    for (let f = 0; f < 60 * 6; f++) {
      world.step(act);
      if (f > 60 * 4) tail.push(world.robot.flywheelRpm);
    }
    const mean = tail.reduce((a, b) => a + b, 0) / tail.length;
    const off = mean - targetRpm;
    console.log(
      `${String(targetRpm).padStart(6)}   ${mean.toFixed(0).padStart(7)}   ${off.toFixed(0).padStart(6)}   ${(off * 0.14).toFixed(1).padStart(7)}`,
    );
  }
}

/**
 * FIT THE FEEDFORWARD, which is what TuningFlywheel exists to do on the robot.
 *
 * FlywheelGate drives the wheel open-loop -- power = (kS + kV*rpm)*(12/V) + kP*err -- and
 * ships with kS = 0.03 and kV = 1/freeRpm, which are guesses with a "re-fit with
 * TuningFlywheel" comment next to them. They are wrong for this wheel: the sim's motor comes
 * from published curves and the flywheel carries its own quadratic drag and coulomb friction,
 * none of which "one over the free speed" knows about. With the guessed gains the wheel
 * settles far enough off target that a 60 rpm readiness window never opens and the robot
 * fires NOTHING.
 *
 * So measure it the way a team would: hold a duty, wait for steady state, record the speed.
 * The fit is least squares on duty = kS + kV * rpm.
 */
async function fitFeedforward(): Promise<void> {
  const duties = [0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9];
  const pts: { duty: number; rpm: number }[] = [];
  console.log('FEEDFORWARD FIT (12 V, steady state)');
  console.log('  duty    rpm');
  for (const duty of duties) {
    const p = structuredClone(params) as unknown as Params;
    const spec = structuredClone(robotSpec) as unknown as RobotSpec;
    const staging: { kind: 'pollen'; pos: Vec3 }[] = [];
    const world = new World({ params: p, robot: spec, staging, alliance: 'red', seed: 1 });
    const act: ActuatorFrame = {
      seq: 0,
      motors: { flywheel: { mode: 'RUN_WITHOUT_ENCODER', power: duty, brake: false } },
      servos: {},
    };
    const tail: number[] = [];
    for (let f = 0; f < 60 * 10; f++) {
      world.step(act);
      if (f > 60 * 8) tail.push(world.robot.flywheelRpm);
    }
    const rpm = tail.reduce((a, b) => a + b, 0) / tail.length;
    pts.push({ duty, rpm });
    console.log(`  ${duty.toFixed(2)}   ${rpm.toFixed(0).padStart(5)}`);
  }
  const n = pts.length;
  const sx = pts.reduce((a, q) => a + q.rpm, 0);
  const sy = pts.reduce((a, q) => a + q.duty, 0);
  const sxx = pts.reduce((a, q) => a + q.rpm * q.rpm, 0);
  const sxy = pts.reduce((a, q) => a + q.rpm * q.duty, 0);
  const kV = (n * sxy - sx * sy) / (n * sxx - sx * sx);
  const kS = (sy - kV * sx) / n;
  const resid = Math.max(...pts.map((q) => Math.abs(q.duty - (kS + kV * q.rpm))));
  console.log('');
  console.log(`  kS = ${kS.toFixed(5)}   kV = ${kV.toExponential(5)}   worst residual ${resid.toFixed(4)} duty`);
  console.log(`  shipped guesses: kS = 0.03, kV = ${(1 / 6000).toExponential(5)}`);
  console.log(`  at 2800 rpm the fit asks for ${(kS + kV * 2800).toFixed(3)} duty, the guess ${(0.03 + 2800 / 6000).toFixed(3)}`);
}

export async function main(argv: string[] = []): Promise<void> {
  await initPhysics();
  if (argv.includes('--ff')) {
    await fitFeedforward();
    return;
  }
  if (argv.includes('--scan')) {
    const targets: number[] = [];
    for (let r = 2300; r <= 3320; r += 30) targets.push(r);
    await scan(targets, 28);
    return;
  }
  const i0 = argv.indexOf('--target');
  // 2800, not 3700: the shot table asks for 2293 to 3316 rpm and nothing else, so tuning at
  // a speed the robot never fires at tunes the wrong operating point.
  const targetRpm = i0 >= 0 ? Number(argv[i0 + 1]) : 2800;
  const ticksPerRev = 28;

  console.log(`target ${targetRpm} RPM`);
  // Offsets in INCHES as well as rpm. tools/apercheck.ts measures 1.4 in of range per 10 rpm
  // at 90 in against a hole 8.9 in deep, and that conversion is the whole reason this matters:
  // the i gain was picked for landing inside the readiness window, not for landing on target.
  const IN_PER_RPM = 0.14;
  console.log(' i     settle(s)  mean offset  = inches  ripple p-p   = inches');
  for (const iGain of [15, 25, 40, 60, 100, 150]) {
    const p = structuredClone(params) as unknown as Params;
    const spec = structuredClone(robotSpec) as unknown as RobotSpec;
    spec.hub.velocityPid.i = iGain;
    const staging: { kind: 'pollen'; pos: Vec3 }[] = [];
    const world = new World({ params: p, robot: spec, staging, alliance: 'red', seed: 1 });
    const act: ActuatorFrame = {
      seq: 0,
      motors: { flywheel: { mode: 'RUN_USING_ENCODER', velocity: (targetRpm * ticksPerRev) / 60 } },
      servos: {},
    };

    let settle = NaN;
    const tail: number[] = [];
    for (let f = 0; f < 60 * 12; f++) {
      world.step(act);
      const rpm = world.robot.flywheelRpm;
      if (Number.isNaN(settle) && Math.abs(rpm - targetRpm) < 120) settle = world.t;
      if (f > 60 * 8) tail.push(rpm);
    }
    const mean = tail.reduce((a, b) => a + b, 0) / tail.length;
    const ripple = Math.max(...tail) - Math.min(...tail);
    console.log(
      `${String(iGain).padStart(4)}   ${Number.isNaN(settle) ? '  never' : settle.toFixed(2).padStart(6)}    ${(mean - targetRpm).toFixed(0).padStart(8)}  ${((mean - targetRpm) * IN_PER_RPM).toFixed(1).padStart(7)}     ${ripple.toFixed(0).padStart(7)}  ${(ripple * IN_PER_RPM).toFixed(1).padStart(7)}`,
    );
  }
}
