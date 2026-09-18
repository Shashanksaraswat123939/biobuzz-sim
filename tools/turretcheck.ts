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
import { wrapPi, RAD, DEG, M_TO_IN } from '../packages/core/src/units.js';
import type { Params, RobotSpec } from '../packages/core/src/types.js';

const table = ShotTable.fromCsv(readFileSync(new URL('../java/teamcode/assets/shottable.csv', import.meta.url), 'utf8'));

function rig(seed = 3, travelDeg?: number) {
  const p = structuredClone(params) as unknown as Params;
  const spec = structuredClone(robotSpec) as unknown as RobotSpec;
  // A SLIP RING is a real, buyable part, and it is the only thing that removes the traverse:
  // with more than 360 deg of travel there is always a reachable wrap, so the axis never has
  // to unwind. Modelled here as travel, not as a fudge -- nothing else changes.
  if (travelDeg) spec.turret.range_deg = [-travelDeg, travelDeg];
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

export async function main(argv: string[] = []): Promise<void> {
  await initPhysics();
  if (argv.includes('--tags')) { await tags(); return; }
  console.log('');
  console.log('1. TURNING FOR 12 s WITH AUTO-AIM ON, our hive is RED.');
  console.log(`   The turret slews at ${robotSpec.turret.speed_dps} deg/s over ${robotSpec.turret.range_deg[0]}..${robotSpec.turret.range_deg[1]} deg.`);
  console.log('   A chassis turning faster than the axis can counter-rotate cannot be tracked at all,');
  console.log('   and one that keeps turning eventually forces a 360 deg traverse whatever we do.');
  console.log('');
  console.log('   chassis rate   on the OPPONENT side   past its stop   worst off our hive');
  const cases: [number, number | undefined][] = [[-0.1, undefined], [-0.2, undefined], [-0.35, undefined], [-0.5, undefined], [-1, undefined]];
  const cases2: [number, number | undefined][] = [];
  for (const [rate, travel] of cases) {
    const { w, brain } = rig(3, travel);
    w.clock.start();
    const g = emptyGamepad();
    g.right_stick_x = rate;
    let nearOpp = 0, n = 0, worstOff = 0, pastStop = 0;
    for (let i = 0; i < 12 * 60; i++) {
      w.step(brain.update(w.sensors(), g, i, 1 / 60));
      const b = bearings(w);
      const offRed = Math.abs(wrapPi((b.muzzle - b.red) * DEG) * RAD);
      const offBlue = Math.abs(wrapPi((b.muzzle - b.blue) * DEG) * RAD);
      n++;
      if (brain.state.turretPastStopDeg > 0.5) pastStop++;
      if (offBlue < offRed) nearOpp++;
      worstOff = Math.max(worstOff, offRed);
    }
    const spin = Math.abs(w.sensors().localizer.omega);
    console.log(`   ${spin.toFixed(0).padStart(9)} deg/s ${((nearOpp / n) * 100).toFixed(0).padStart(20)}% ${((pastStop / n) * 100).toFixed(0).padStart(14)}% ${worstOff.toFixed(0).padStart(18)} deg${travel ? '   (slip ring, +-' + travel + ' deg)' : ''}`);
  }
  console.log('');
  console.log('   The same runs with a SLIP RING, which is the only thing that removes the traverse:');
  for (const [rate] of cases) cases2.push([rate, 900]);
  for (const [rate, travel] of cases2) {
    const { w, brain } = rig(3, travel);
    w.clock.start();
    const g = emptyGamepad();
    g.right_stick_x = rate;
    let nearOpp = 0, n = 0, worstOff = 0, pastStop = 0;
    for (let i = 0; i < 12 * 60; i++) {
      w.step(brain.update(w.sensors(), g, i, 1 / 60));
      const b = bearings(w);
      const offRed = Math.abs(wrapPi((b.muzzle - b.red) * DEG) * RAD);
      const offBlue = Math.abs(wrapPi((b.muzzle - b.blue) * DEG) * RAD);
      n++;
      if (brain.state.turretPastStopDeg > 0.5) pastStop++;
      if (offBlue < offRed) nearOpp++;
      worstOff = Math.max(worstOff, offRed);
    }
    const spin = Math.abs(w.sensors().localizer.omega);
    console.log(`   ${spin.toFixed(0).padStart(9)} deg/s ${((nearOpp / n) * 100).toFixed(0).padStart(20)}% ${((pastStop / n) * 100).toFixed(0).padStart(14)}% ${worstOff.toFixed(0).padStart(18)} deg`);
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

/**
 * Section 3 lives in its own entry point so it can be run alone:
 *   npm run tool -- tools/turretcheck.ts --tags
 */
export async function tags(): Promise<void> {
  await initPhysics();
  const cam = robotSpec.sensors.camera;
  console.log('\n3. THE APRILTAG ON OUR OWN CELL, as the camera actually sees it.');
  console.log(`   ${cam.widthPx}x${cam.heightPx} at ${cam.hfov_deg} deg hfov, ${cam.mount}-mounted, `
    + `${cam.tagSize_in} in tag, decode floor ${cam.minTagPx} px, obliquity limit ${cam.maxObliquity_deg} deg.\n`);

  // Sweep the field: from each spot, aim the turret at the CELL and ask whether the tag reads.
  const p = structuredClone(params) as unknown as Params;
  const w = new World({ params: p, robot: robotSpec as unknown as RobotSpec, staging: [], alliance: 'red', seed: 4 });
  const hw = w.geom.halfWidth_m;
  let seen = 0, total = 0;
  const byRange = new Map<number, { seen: number; n: number }>();
  for (let x = -hw + 0.3; x < hw - 0.3; x += 0.25) {
    for (let z = -hw + 0.3; z < hw - 0.3; z += 0.25) {
      w.robot.place([x, 0.17, z], 0);
      // Point the turret at the CELL, which is what auto-aim does and what the camera rides.
      const m = w.hives.red.upCellMouthWorld();
      w.robot.setTurretForTest(Math.atan2(m[0] - x, m[2] - z) * RAD);
      const t = w.tagSighting();
      total++;
      const r = Math.round((Math.hypot(m[0] - x, m[2] - z) * M_TO_IN) / 20) * 20;
      const b = byRange.get(r) ?? { seen: 0, n: 0 };
      b.n++;
      if (t) { seen++; b.seen++; }
      byRange.set(r, b);
    }
  }
  console.log(`   over the whole field, turret pointed at the CELL: readable from ${((seen / total) * 100).toFixed(0)}% of spots`);
  console.log('\n   range (in)   readable');
  for (const r of [...byRange.keys()].sort((a, b) => a - b)) {
    const b = byRange.get(r)!;
    if (b.n < 8) continue;
    console.log(`   ${String(r).padStart(9)}   ${((b.seen / b.n) * 100).toFixed(0).padStart(6)}%   (${b.n} spots)`);
  }
  console.log('');
}
