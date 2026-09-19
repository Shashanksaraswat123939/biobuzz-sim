/**
 * IS "OFF THE OPENING" A POLICY OR A WALL? The aperture, solved at every bearing.
 *
 *   npm run tool -- tools/obliquity.ts
 *
 * The fire gate refuses past turret.fireOpenCap_deg, and on a straight full-speed pass that
 * one rule holds 45% of the drive -- by far the biggest single loss. A cap is a suspicious
 * thing to lose half your time to, so this asks whether a shot EXISTS out there at all.
 *
 * A CELL is a slot, not a hole in a plane: 20 in wide along the pivot axis and 12.04 in
 * DEEP. Stand off its normal by beta and two things happen at once -- the width you can use
 * shrinks with cos(beta), and the pocket's own depth cuts across the opening as
 * depth*sin(beta). What is left is
 *
 *     clear = width*cos(beta) - depth*sin(beta)
 *
 * which does not taper gently to zero: it CROSSES zero, and past that there is no aperture
 * to aim a ball through however the flywheel and hood are set. That is the difference
 * between a limit a better controller can push and one it cannot.
 */
import params from '../config/params.json' with { type: 'json' };
import robotSpec from '../config/robot.json' with { type: 'json' };
import { buildFieldGeometry } from '../packages/core/src/field/geometry.js';
import { M_TO_IN, DEG } from '../packages/core/src/units.js';
import type { Params, RobotSpec } from '../packages/core/src/types.js';

export async function main(): Promise<void> {
  const p = params as unknown as Params;
  const spec = robotSpec as unknown as RobotSpec;
  const g = buildFieldGeometry(p);
  const cell = g.cells[0];
  const width_in = cell.halfInterior[0] * 2 * M_TO_IN;
  const depth_in = cell.halfInterior[1] * 2 * M_TO_IN;
  const ball_in = p.ball.pollen.d_m * M_TO_IN;
  const cap = spec.turret.fireOpenCap_deg ?? 75;

  console.log('\nTHE CELL SEEN FROM AN ANGLE. Width %s in, depth %s in, ball %s in.',
    width_in.toFixed(1), depth_in.toFixed(1), ball_in.toFixed(2));
  console.log('  clear = width*cos(beta) - depth*sin(beta); a ball needs more than its own diameter.\n');
  console.log('  off-axis   clear width   room for a POLLEN   verdict');
  let closes = NaN, marginal = NaN;
  for (let b = 0; b <= 85; b += 5) {
    const clear = width_in * Math.cos(b * DEG) - depth_in * Math.sin(b * DEG);
    const room = clear - ball_in;
    const verdict = room > 4 ? 'comfortable' : room > 1.5 ? 'ok' : room > 0 ? 'marginal' : 'NO APERTURE';
    if (Number.isNaN(marginal) && room <= 1.5) marginal = b;
    if (Number.isNaN(closes) && room <= 0) closes = b;
    console.log(`  ${String(b).padStart(6)}\u00b0   ${clear.toFixed(1).padStart(9)} in   ${room.toFixed(1).padStart(16)} in   ${verdict}`);
  }
  console.log('');
  console.log(`  A POLLEN stops fitting at about ${closes}\u00b0 and is marginal from about ${marginal}\u00b0.`);
  console.log(`  The gate's cap is ${cap}\u00b0.`);
  console.log('');
  console.log('  So the cap is not holding back a shot that a cleverer solver could take: past');
  console.log('  this angle the opening has NO AREA. No exit speed and no hood angle put a ball');
  console.log('  through a hole that is not there -- the only fix is to drive round, which is');
  console.log('  what the hold string says.');
  console.log('');
}
