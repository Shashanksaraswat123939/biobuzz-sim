/**
 * A ball that goes through the mouth has not scored. Does it STAY?
 *
 *   npm run tool -- tools/entrycheck.ts [--n 24]
 *
 * The shot table is built by `bestShot`, which scores a candidate on its SPEED MARGIN: how
 * much exit-speed error still threads the mouth. Threading the mouth is the only thing it
 * checks, and threading the mouth is not scoring. A ball that arrives fast keeps most of
 * that speed through its first bounce off the pocket floor and can come straight back out.
 *
 * So the real quantity is a product of two independent things:
 *
 *     P(land) = P(thread the aperture) x P(stay in | how it arrived)
 *
 * `bestShot` optimises the first and is blind to the second, which is why it chose 81 deg
 * lobs arriving at 6.2 m/s: near-vertical shots have enormous speed margin, because at 81
 * degrees a speed error mostly changes HEIGHT rather than range, so the ball still falls
 * through the hole. It just does not necessarily stay there.
 *
 * This measures the second factor on its own, with no shooter in the loop: inject balls at
 * the mouth with a chosen arrival speed and descent angle, let them settle, count what is
 * still in the CELL. That isolates entry physics from aiming physics, so the result can be
 * used as an objective without dragging the whole shooter through every evaluation.
 */
import { writeFileSync } from 'node:fs';
import { World, initPhysics } from '../packages/core/src/physics/world.js';
import { DEG, inches } from '../packages/core/src/units.js';
import params from '../config/params.json' with { type: 'json' };
import robotJson from '../config/robot.json' with { type: 'json' };
import type { Params, RobotSpec, Vec3 } from '../packages/core/src/types.js';

export interface EntryResult {
  speed_mps: number;
  descent_deg: number;
  fired: number;
  stayed: number;
  rate: number;
}

/**
 * Inject `n` balls at the up-CELL mouth travelling inward at (speed, descent) and count how
 * many are still in the CELL once everything has settled.
 *
 * The lateral and vertical aim is jittered by a ball radius so this measures entry over the
 * mouth rather than down one perfect line -- a shot that only works dead-centre is not a
 * shot. It deliberately does NOT jitter speed or angle: those are the axes being swept.
 */
export async function entryRate(
  speed: number, descent_deg: number, n = 24, seed = 7,
  /**
   * How much of a single-wheel shooter's backspin the ball carries in. 1 is what
   * `Robot.launch` gives it; 0 is a dual-wheel shooter, which imparts none. The axis matters
   * more than the magnitude -- see the note at the release below.
   */
  spinFactor = 1,
): Promise<EntryResult> {
  await initPhysics();
  const p = structuredClone(params) as unknown as Params;
  // PIN THE ROCKER. 12 POLLEN tip the hive, and a tip empties the CELL -- so a 16-ball run
  // returned exactly 12/16 = 75% in EVERY cell of the sweep, which is a counting artefact
  // wearing the costume of a result. A rocker this heavy cannot tip, so `ballsInUpCell` is
  // just a count. (params.hive.ballsToTipOverride exists in the types and is read by
  // NOTHING, so it is not an option.)
  //
  // The cost: a heavier rocker also recoils less when a ball hits it. That is second order
  // next to the artefact being removed -- the pocket is resting on a hard stop either way --
  // but it is a real difference and worth knowing when reading these numbers.
  p.hive.massKg = 200;
  const spec = robotJson as unknown as RobotSpec;
  const staging = Array.from({ length: n + 2 }, () => ({ kind: 'pollen' as const, pos: [0, -5, 0] as Vec3 }));
  const world = new World({ params: p, robot: spec, staging, alliance: 'red', seed });
  for (const b of world.balls.balls) world.balls.park(b);
  // Park the robot far away so it cannot be hit by anything.
  world.robot.place([0, 0.17, 0], 0);

  const mouth = world.hives.red.upCellMouthWorld();
  const r = p.ball.pollen.d_m / 2;
  const th = descent_deg * DEG;

  // Approach from the +Z side, descending: the direction a shot from the field arrives on.
  const dir: Vec3 = [0, -Math.sin(th), -Math.cos(th)];
  // Start one ball-diameter back up the incoming ray so the ball is clear of the lip.
  const back = inches(10);

  let rng = seed * 2654435761;
  const rand = () => {
    rng = (rng * 1103515245 + 12345) & 0x7fffffff;
    return rng / 0x7fffffff - 0.5;
  };

  let fired = 0;
  for (let i = 0; i < n; i++) {
    const b = world.balls.balls[i];
    const jitterX = rand() * 2 * r;
    const jitterY = rand() * 2 * r;
    const at: Vec3 = [
      mouth[0] + jitterX - dir[0] * back,
      mouth[1] + jitterY - dir[1] * back,
      mouth[2] - dir[2] * back,
    ];
    const vel: Vec3 = [dir[0] * speed, dir[1] * speed, dir[2] * speed];
    // Spin taken EXACTLY as Robot.launch computes it, from the same expression, so this
    // cannot drift away from what the shooter really does:
    //     spin = [(-dir.z/n)*v/r, 0, (dir.x/n)*v/r]
    // Writing it out by hand instead got the sign backwards and gave every ball TOPSPIN,
    // which grabs the pocket floor and kicks it straight back out -- the tool then reported
    // 0% entry at speeds the real robot scores from perfectly well.
    const n = Math.hypot(dir[0], dir[2]) || 1;
    const mag = (speed / r) * spinFactor;
    const spin: Vec3 = [(-dir[2] / n) * mag, 0, (dir[0] / n) * mag];
    world.balls.release(b, at, vel, spin, 'flight');
    fired++;
    // Space them out so they are not colliding with each other on the way in.
    for (let f = 0; f < 45; f++) world.step({ seq: f, motors: {}, servos: {} });
  }
  for (let f = 0; f < 60 * 4; f++) world.step({ seq: f, motors: {}, servos: {} });

  if (world.hives.red.tips > 0) throw new Error('the rocker tipped despite being pinned; the count is not trustworthy');
  const stayed = world.hives.red.ballsInUpCell;
  return { speed_mps: speed, descent_deg, fired, stayed, rate: stayed / fired };
}

export async function main(argv: string[] = []): Promise<void> {
  const i = argv.indexOf('--n');
  const n = i >= 0 ? Number(argv[i + 1]) : 24;

  console.log('ENTRY PHYSICS — of the balls that reach the mouth, how many stay?');
  console.log('');
  console.log(`  ${n} balls per cell, injected at the mouth, no shooter involved.`);
  console.log('  Aim is jittered by a ball radius so this is entry over the whole mouth, not down');
  console.log('  one perfect line. Speed and descent angle are the axes, so they are NOT jittered.');
  console.log('');

  const speeds = [2, 3, 4, 5, 6, 7, 8, 9];
  const descents = [25, 35, 45, 55, 65, 75, 85];
  const grid: EntryResult[] = [];

  process.stdout.write('  arrival speed ->' + speeds.map((s) => `${s} m/s`.padStart(8)).join('') + '\n');
  for (const d of descents) {
    process.stdout.write(`  ${String(d).padStart(3)} deg down   `);
    for (const s of speeds) {
      const res = await entryRate(s, d, n);
      grid.push(res);
      process.stdout.write(`${(res.rate * 100).toFixed(0).padStart(7)}%`);
    }
    process.stdout.write('\n');
  }
  console.log('');

  const best = [...grid].sort((a, b) => b.rate - a.rate)[0];
  const cur = grid.reduce((acc, g) =>
    Math.abs(g.speed_mps - 6.2) + Math.abs(g.descent_deg - 75) / 20 <
    Math.abs(acc.speed_mps - 6.2) + Math.abs(acc.descent_deg - 75) / 20 ? g : acc, grid[0]);

  console.log(`  best of the grid:   ${best.speed_mps} m/s at ${best.descent_deg} deg down -> ${(best.rate * 100).toFixed(0)}%`);
  console.log(`  what the table asks for today (about 6.2 m/s at 75 deg): ${(cur.rate * 100).toFixed(0)}%`);
  console.log('');
  console.log('  If that gap is real, it is free points: the shot table is choosing arrival');
  console.log('  conditions on SPEED MARGIN alone and is blind to this entire axis.');

  writeFileSync(new URL('../config/entry.json', import.meta.url), JSON.stringify({
    _about: 'MEASURED by tools/entrycheck.ts: of the balls that reach the up-CELL mouth at a given arrival speed and descent angle, what fraction STAY IN. tools/shottable.ts multiplies this into its objective, so the table stops choosing shots that thread the mouth and bounce straight back out.',
    _method: `${n} balls per cell injected at the mouth with the launcher's own spin convention, aim jittered by a ball radius, rocker pinned at ${200} kg so a tip cannot empty the CELL mid-count.`,
    _caveat: 'Binomial noise at this sample size is about +-10 points per cell. Read the shape, not individual cells.',
    generated: new Date().toISOString(),
    ballsPerCell: n,
    speeds_mps: speeds,
    descents_deg: descents,
    // rate[descentIndex][speedIndex]
    rate: descents.map((d) => speeds.map((sp) => grid.find((g) => g.descent_deg === d && g.speed_mps === sp)!.rate)),
  }, null, 1) + String.fromCharCode(10));
  console.log('  wrote config/entry.json');
}
