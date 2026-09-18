/**
 * Does the turret ever point at the WRONG HIVE, and does turning stop you firing?
 *
 *   npm run tool -- tools/turretcheck.ts
 *
 * Two complaints that look alike from the driver's seat and have different causes:
 *
 *  1. "the turret sometimes goes to the other hive". The aim point is always our own hive, so
 *     if that is happening it is not a targeting mistake -- it is the UNWIND. The axis travels
 *     +-185 deg, and a chassis that keeps turning one way drives the required bearing off the
 *     end; the only way back is a 360 deg traverse, which sweeps the muzzle across the whole
 *     field, opponent's hive included, for as long as it takes. This measures how often that
 *     happens, how long it lasts, and whether the muzzle really does cross the other hive.
 *
 *  2. "sometimes it fires in the zone, sometimes it does not". The fire gate refuses above
 *     `turret.fireYawCap_dps` of chassis rotation, and X/B are BUTTONS: they command full
 *     rotation, not a feathered stick. This measures the rate a button actually produces
 *     against the cap that has to be met.
 */
import params from '../config/params.json' with { type: 'json' };
import robotSpec from '../config/robot.json' with { type: 'json' };
import { readFileSync } from 'node:fs';
import { World, initPhysics, emptyGamepad } from '../packages/core/src/physics/world.js';
import { BuiltinTeleOp, ShotTable } from '../packages/core/src/robot/builtinTeleOp.js';
import { loadLandCal } from '../packages/core/src/robot/loadCal.js';
import { readKeyboard, remap, type Keys } from '../packages/ui/src/input.js';
import { wrapPi, RAD, DEG } from '../packages/core/src/units.js';
import type { Params, RobotSpec } from '../packages/core/src/types.js';

const table = ShotTable.fromCsv(readFileSync(new URL('../java/teamcode/assets/shottable.csv', import.meta.url), 'utf8'));

function rig(seed = 3) {
  const p = structuredClone(params) as unknown as Params;
  const spec = structuredClone(robotSpec) as unknown as RobotSpec;
  const w = new World({ params: p, robot: spec, staging: [], alliance: 'red', seed, preload: spec.hopper.capacity });
  const brain = new BuiltinTeleOp(spec, table, loadLandCal());
  return { w, brain, spec };
}

/** World-frame bearing the muzzle is pointing, and the bearing to each hive's up CELL. */
function bearings(w: World) {
  const p = w.robot.pos;
  const muzzle = (w.robot.yaw * RAD + w.robot.turretAngle);
  const to = (a: 'red' | 'blue') => {
    const m = w.hives[a].upCellMouthWorld();
    return Math.atan2(m[0] - p[0], m[2] - p[2]) * RAD;
  };
  return { muzzle, red: to('red'), blue: to('blue') };
}

export async function main(): Promise<void> {
  await initPhysics();
  console.log('\n1. SPINNING ON THE SPOT FOR 12 s, auto-aim on, our hive is RED.\n');
  {
    const { w, brain } = rig();
    w.clock.start();
    const g = emptyGamepad();
    g.right_stick_x = -0.5;                      // a steady left turn
    let unwinding = 0, nearOpp = 0, n = 0, worstOff = 0, pastStop = 0;
    for (let i = 0; i < 12 * 60; i++) {
      w.step(brain.update(w.sensors(), g, i, 1 / 60));
      const b = bearings(w);
      const offRed = Math.abs(wrapPi((b.muzzle - b.red) * DEG) * RAD);
      const offBlue = Math.abs(wrapPi((b.muzzle - b.blue) * DEG) * RAD);
      n++;
      if (brain.state.turretPastStopDeg > 0.5) pastStop++;
      if (Math.abs(brain.state.turretErrDeg) > 15) unwinding++;
      if (offBlue < offRed) nearOpp++;
      worstOff = Math.max(worstOff, offRed);
      void unwinding;
    }
    const spin = Math.abs(w.sensors().localizer.omega);
    console.log(`   spinning ${spin.toFixed(0).padStart(3)} deg/s:  on the OPPONENT's side ${((nearOpp / n) * 100).toFixed(0).padStart(3)}%`
      + `   past its stop ${((pastStop / n) * 100).toFixed(0).padStart(3)}%   worst ${worstOff.toFixed(0).padStart(3)} deg off`);
  }

  console.log('');
  console.log(`2. WHAT A TURN BUTTON GIVES YOU. The fire gate refuses above ${robotSpec.turret.fireYawCap_dps} deg/s`);
  console.log('   of chassis rotation, so a turn you cannot feather is a shot you cannot take.');
  console.log('');
  for (const holdMs of [100, 150, 200, 300, 500, 1200]) {
    const { w, brain } = rig();
    w.clock.start();
    const kk: Keys = { down: new Set(['q']), pressed: new Set() };
    const idle: Keys = { down: new Set(), pressed: new Set() };
    let peak = 0;
    let rx = 0;
    const dt = 1 / 60;
    for (let i = 0; i < 180; i++) {
      const phys = readKeyboard(i * dt * 1000 < holdMs ? kk : idle);
      const want = remap(phys, phys.paddles).right_stick_x;
      // The UI's own yaw ramp, mirrored so this is the number a driver actually gets.
      const rate = Math.abs(want) > Math.abs(rx) ? 1.6 : 12;
      const step = rate * dt;
      rx = Math.abs(want - rx) <= step ? want : rx + Math.sign(want - rx) * step;
      const g = emptyGamepad();
      g.right_stick_x = rx;
      w.step(brain.update(w.sensors(), g, i, dt));
      peak = Math.max(peak, Math.abs(w.sensors().localizer.omega));
    }
    const cap = robotSpec.turret.fireYawCap_dps as number;
    console.log(`   ${String(holdMs).padStart(9)} ms     ${peak.toFixed(0).padStart(11)} deg/s   ${peak <= cap ? 'YES' : 'no, it refuses'}`);
  }
  console.log('');
}
