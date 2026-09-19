/**
 * WHAT DOES HOLDING COST? Time from the gate clearing to the ball leaving.
 *
 *   npm run tool -- tools/holdcost.ts
 *
 * The question this answers: while the robot is waiting for P(land) to come good, is it ALSO
 * paying the feed cycle and the climb up the tube afterwards, or do those run underneath the
 * wait? The code says underneath -- the belt is commanded on whenever the flywheel is on, not
 * when the shot is approved, so the tube stays loaded against the gate; and cycleTime is
 * measured from the last SHOT, so it ticks while holding. This measures it rather than
 * trusting the reading.
 *
 * Two numbers matter and they are different:
 *   - CLEAR -> AWAY: from the first loop the hold string is empty to the ball leaving. This
 *     is what a wait actually costs once readiness arrives.
 *   - the same thing for a shot taken back-to-back, where the cycle timer has NOT run out
 *     underneath and really does have to be waited for.
 */
import { readFileSync } from 'node:fs';
import params from '../config/params.json' with { type: 'json' };
import robotSpec from '../config/robot.json' with { type: 'json' };
import { World, initPhysics, emptyGamepad } from '../packages/core/src/physics/world.js';
import { BuiltinTeleOp, ShotTable } from '../packages/core/src/robot/builtinTeleOp.js';
import { loadLandCal } from '../packages/core/src/robot/loadCal.js';
import { RAD, inches } from '../packages/core/src/units.js';
import type { GamepadState, Params, RobotSpec, Vec3 } from '../packages/core/src/types.js';

const table = ShotTable.fromCsv(readFileSync(new URL('../java/teamcode/assets/shottable.csv', import.meta.url), 'utf8'));

export async function main(): Promise<void> {
  await initPhysics();
  const p = structuredClone(params) as unknown as Params;
  const spec = structuredClone(robotSpec) as unknown as RobotSpec;
  const pool = Array.from({ length: 40 }, () => ({ kind: 'pollen' as const, pos: [0, -5, 0] as Vec3 }));
  const w = new World({ params: p, robot: spec, staging: pool, alliance: 'red', seed: 31 });
  for (const b of w.balls.balls) w.balls.park(b);
  const mouth = w.hives.red.upCellMouthWorld();
  const n = w.hives.red.upCellMouthNormalWorld();
  const d = inches(60);
  const at: Vec3 = [mouth[0] + n[0] * d, spec.chassis.height_m / 2 + spec.chassis.clearance_m, mouth[2] + n[2] * d];
  w.robot.place(at, Math.atan2(mouth[0] - at[0], mouth[2] - at[2]) * RAD);
  const brain = new BuiltinTeleOp(spec, table, loadLandCal());

  let loaded = 0;
  let shots = 0;
  let clearSince = NaN;      // when the hold last went empty
  let lastShotT = NaN;
  const rows: { clearToAway: number; sinceLast: number; heldFor: number }[] = [];
  let holdStart = 0;
  let holdInjected = false, holdDone = false, holdFrom = 0;

  // Arm the shooter only after a few seconds, so the FIRST shot is one that has been waiting
  // on readiness with the cycle timer long since expired -- the case in question.
  for (let i = 0; i < 60 * 30; i++) {
    while (w.robot.heldBalls().length < spec.hopper.capacity && loaded < pool.length) {
      if (!w.robot.preload(w.balls, w.balls.balls[loaded])) break;
      loaded++;
    }
    if (i === 60 * 3) brain.state.firing = true;
    // A DELIBERATE MID-MATCH HOLD. Once it is shooting steadily, make the probability gate
    // impossible for three seconds and then let it go. This is the case being asked about:
    // the wheel is spinning, the belt is running, and only the SHOT is being refused.
    if (shots === 4 && !holdInjected) { spec.flywheel.minLandProb = 1.1; holdInjected = true; holdFrom = w.t; }
    if (holdInjected && !holdDone && w.t - holdFrom >= 3) { spec.flywheel.minLandProb = 0.7; holdDone = true; }
    const g: GamepadState = emptyGamepad();
    w.setGamepads(g, emptyGamepad());
    w.step(brain.update(w.sensors(), g, w.seq, 1 / 60));
    const t = w.t;
    // state.ready, NOT an empty hold string. `st.hold` is also '' when the wheel is OFF
    // (the chain starts `!wheelOn ? ''`), so an empty hold counted spin-up as clear and the
    // first shot read 5.77 s. `ready` is the thing the feed actually asks.
    const clear = brain.state.ready;
    if (clear && Number.isNaN(clearSince)) clearSince = t;
    if (!clear) { clearSince = NaN; holdStart = holdStart || t; }
    if (w.robot.shots > shots) {
      shots = w.robot.shots;
      rows.push({
        clearToAway: Number.isNaN(clearSince) ? NaN : t - clearSince,
        sinceLast: Number.isNaN(lastShotT) ? NaN : t - lastShotT,
        heldFor: holdDone && rows.length === 4 ? 3 : NaN,
      });
      lastShotT = t;
      clearSince = NaN;
    }
    if (shots >= 8) break;
  }

  console.log('\nWHAT HOLDING COSTS. Standing at 60 in, gate clearing normally.\n');
  console.log(`  cycleTime_s ${spec.transfer.cycleTime_s}, feedPulse_s ${spec.transfer.feedPulse_s}, readySteps ${spec.flywheel.readySteps}\n`);
  console.log('  shot   ready -> away   since the last shot   note');
  rows.forEach((r, i) => {
    const note = i === 0 ? 'first: tube priming from cold'
      : !Number.isNaN(r.heldFor) ? 'FIRST SHOT AFTER A 3 s FORCED HOLD'
      : '';
    console.log(`  ${String(i + 1).padStart(4)}   ${(Number.isNaN(r.clearToAway) ? '   -' : r.clearToAway.toFixed(2)).padStart(10)} s   ${(Number.isNaN(r.sinceLast) ? '     -' : `${r.sinceLast.toFixed(2)} s`).padStart(12)}   ${note}`);
  });
  console.log('');
  console.log('  The wheel and the belt run whenever the shooter is ARMED, not when the shot is');
  console.log('  approved, so a hold keeps the tube loaded underneath itself; and cycleTime is');
  console.log('  measured from the last SHOT, so it expires during the wait too. A hold therefore');
  console.log('  costs what the row says and not that plus a fresh cycle -- once the tube is primed.');
  console.log('');
}
