/**
 * Where on the field can this robot actually shoot from?
 *
 *   npm run tool -- tools/shotzone.ts [--step 6]
 *
 * Every land-rate measurement in this repo was taken from ONE spot per range, and all three
 * of them are square in front of the mouth: `placeAt` sweeps the stand-off angle up from zero
 * and takes the first position that fits on the field, which at 40, 55 and 70 in is zero. So
 * the robot has only ever been asked to shoot down the hive's own centre line, and the CELL
 * mouth is a SLOT -- approaching it from an angle changes the target in two opposite ways:
 *
 *   the slot gets DEEPER along the shot line. Its lips are lines running across the field, so
 *   a shot crossing them at an angle passes between them over a longer distance: the range
 *   window widens by 1/cos(angle).
 *
 *   the slot gets NARROWER across it. The opening's width is fixed in the field frame, and
 *   what matters is its extent perpendicular to the shot, which shrinks by cos(angle).
 *
 * So standing off-axis trades range tolerance for bearing tolerance, and nothing in the shot
 * table knows about it -- the table is solved head-on. This walks a grid over the field, runs
 * the solver against the aperture as seen FROM THAT SPOT with the hood and RPM the table would
 * actually command there, and reports the chance a perfectly aimed shot lands.
 *
 * It is a MODEL map, not a measurement. It says where the physics is forgiving; the spots it
 * likes still have to be shot from before they can be believed.
 */
import { readFileSync, writeFileSync } from 'node:fs';
import params from '../config/params.json' with { type: 'json' };
import robotJson from '../config/robot.json' with { type: 'json' };
import { buildFieldGeometry } from '../packages/core/src/field/geometry.js';
import { bestShot, pThread, type Aperture } from '../packages/core/src/physics/ballistics.js';
import { ShotTable } from '../packages/core/src/robot/builtinTeleOp.js';
import { loadLandCal } from '../packages/core/src/robot/loadCal.js';
import { DEG, M_TO_IN, inches, rpmToRadS } from '../packages/core/src/units.js';
import { loadEntry, mouthLips } from './shottable.js';
import { cmBare, m as fm } from './_units.js';
import type { Params, RobotSpec, Vec3 } from '../packages/core/src/types.js';

export interface ZoneCell {
  x_in: number; z_in: number; range_in: number; offAxisDeg: number;
  /** P(land) for a STATIONARY robot, which is what the tool's own printout reports. */
  p: number;
  /**
   * Everything needed to redo that probability at a different robot velocity WITHOUT
   * re-running the solver.
   *
   * The expensive half of a cell is the trajectory integration, and velocity does not change
   * it: the lead keeps the ball's ground path identical, so the aperture, the speed band and
   * the entry rate are all properties of the SPOT. What motion changes is the exit speed the
   * shot has to be taken at, and that is arithmetic. Shipping these lets the overlay follow
   * the robot in real time instead of being a picture of standing still.
   */
  k?: {
    lo: number; hi: number; sigma: number; pStay: number; halfLat: number;
    ux: number; uz: number; dist: number; commanded: number; cosEl: number;
  };
}

/**
 * Field-frame velocity of the robot, m/s, for asking whether the zone MOVES when it drives.
 *
 * It mostly should not, and that is a claim worth testing rather than assuming. The brain
 * leads the shot (`leadShot`): it subtracts the robot's own velocity from the ball's required
 * ground velocity, so a perfectly led shot flies the same ground path as a stationary one and
 * threads the same aperture. What motion costs is not the aim, it is the SPEED it has to be
 * taken at -- driving away needs a faster shot, and the launch scatter is a FRACTION of that
 * speed, so the absolute error at the mouth grows with it. Driving in is the reverse.
 */
export function buildZone(
  step_in = 6,
  vel: [number, number] = [0, 0],
  /** Which stop the rocker is on. It flips on every TIP and takes the mouth with it. */
  side: -1 | 1 = -1,
): { cells: ZoneCell[]; threshold: number } {
  const p = params as unknown as Params;
  const spec = robotJson as unknown as RobotSpec;
  const g = buildFieldGeometry(p);
  const lips = mouthLips(p, side);
  const entry = loadEntry();
  const cal = loadLandCal();
  const table = ShotTable.fromCsv(
    readFileSync(new URL('../java/teamcode/assets/shottable.csv', import.meta.url), 'utf8'),
  );
  const ballR = p.ball.pollen.d_m / 2;
  const spinPerSpeed = spec.flywheel.type === 'single' ? 1 / ballR : 0;
  const f = spec.flywheel;
  const halfLat0 = g.cells[0].halfInterior[0] - ballR;
  const mouthZ = (lips.near.z + lips.far.z) / 2;
  const minR = table.rows[0].range_in;
  const maxR = table.rows[table.rows.length - 1].range_in;

  const limit = g.halfWidth_m - 0.35;
  const cells: ZoneCell[] = [];
  for (let z = -limit; z <= limit; z += inches(step_in)) {
    for (let x = -limit; x <= limit; x += inches(step_in)) {
      const dx = lips.x_m - x;
      const dz = mouthZ - z;
      const dist = Math.hypot(dx, dz);
      const range_in = dist * M_TO_IN;
      // EVERY square gets a cell, even the hopeless ones.
      //
      // Skipping them left the map full of holes, and a hole reads as "no information" when
      // what it means is "you cannot shoot from here" -- which is the single most useful
      // thing the map can tell a driver. A zero is drawn; a gap is not.
      const blank = (x_in: number, z_in: number): ZoneCell =>
        ({ x_in, z_in, range_in, offAxisDeg: 90, p: 0 });
      if (range_in < minR || range_in > maxR) { cells.push(blank(x * M_TO_IN, z * M_TO_IN)); continue; }
      const row = table.lookup(range_in);
      if (row.hoodDeg === undefined || row.rpm <= 0) { cells.push(blank(x * M_TO_IN, z * M_TO_IN)); continue; }

      // |uz| is the cosine of the angle off the hive's normal: 1 straight in front of it.
      const uz = dz / dist;
      const cosOff = Math.abs(uz);
      if (cosOff < 0.15) { cells.push(blank(x * M_TO_IN, z * M_TO_IN)); continue; }
      const offAxisDeg = Math.acos(Math.min(1, cosOff)) / DEG;

      const muzzleZ = z + spec.turret.muzzleOffset_m * uz;
      // Ranges are measured along the shot, on the side the mouth actually opens toward.
      // `facing` carries that, so a shooter standing behind the pocket gets a negative near
      // range and is dropped rather than being scored against the wrong lip.
      const w = lips.facing;
      const aperture: Aperture = {
        nearRange: (w * (muzzleZ - lips.near.z)) / cosOff,
        nearHeight: lips.near.y + ballR,
        farRange: (w * (muzzleZ - lips.far.z)) / cosOff,
        farHeight: lips.far.y - ballR,
      };
      const here = { x_in: x * M_TO_IN, z_in: z * M_TO_IN, range_in, offAxisDeg };
      if (aperture.nearRange <= 0 || aperture.farRange <= aperture.nearRange) { cells.push({ ...here, p: 0 }); continue; }

      // The shot the TABLE commands here, tested against the aperture as seen from here.
      const solved = bestShot(p, {
        muzzle: [0, spec.turret.muzzleHeight_m, 0] as Vec3,
        azimuth: 0,
        aperture,
        radius: ballR,
        mass: p.ball.pollen.m_kg,
        hoodRange: [row.hoodDeg, row.hoodDeg],
        hoodSteps: 1,
        spinPerSpeed,
        k: f.k,
        rFly: f.r_fly_m,
        maxRpm: f.maxRpm,
        range_in,
        minDescentDeg: 0,
        entryRate: entry ? (v, d) => entry.lookup(v, d) : undefined,
        scatter: { speedFrac: f.scatter.speedFrac, angle_deg: f.scatter.angle_deg },
      });
      if (!solved) { cells.push({ ...here, p: 0 }); continue; }

      const commanded = f.k * f.r_fly_m * rpmToRadS(row.rpm);
      // What the shot has to LEAVE at once the robot's own velocity is taken out of it. The
      // ball's ground speed stays `commanded` -- that is what the lead is for -- so the
      // aperture test is unchanged and only the scatter moves, because scatter is a fraction
      // of the exit speed. Scaling the whole sigma by the speed ratio is an approximation:
      // the elevation-scatter half of it does not scale quite like the speed half.
      const [vx, vz] = vel;
      const cosEl = Math.cos((row.hoodDeg as number) * DEG);
      const horiz = commanded * cosEl;
      const required = Math.hypot(horiz * (dx / dist) - vx, horiz * (dz / dist) - vz) / cosEl;
      const maxSpeed = f.k * f.r_fly_m * rpmToRadS(f.maxRpm);
      if (required > maxSpeed) { cells.push({ ...here, p: 0 }); continue; }
      const sigma = solved.sigmaSpeed * (required / Math.max(commanded, 1e-6));
      const speed = pThread(solved.speedLo, solved.speedHi, commanded, sigma);
      // Across the shot line, the opening is the mouth's width foreshortened by the approach.
      const halfLat = halfLat0 * cosOff;
      const aim = pThread(-halfLat, halfLat, 0, dist * Math.tan(f.scatter.yaw_deg * DEG));
      const raw = speed * aim * solved.pStay;
      cells.push({
        ...here,
        p: cal ? cal.apply(raw) : raw,
        k: {
          lo: solved.speedLo, hi: solved.speedHi, sigma: solved.sigmaSpeed, pStay: solved.pStay,
          halfLat, ux: dx / dist, uz: dz / dist, dist, commanded, cosEl,
        },
      });
    }
  }
  return { cells, threshold: f.minLandProb ?? 0.9 };
}

/** Round for the wire: four decimals is well past what a 512 px texture can show. */
const trim = (c: ZoneCell) => ({
  x_in: +c.x_in.toFixed(1),
  z_in: +c.z_in.toFixed(1),
  p: +c.p.toFixed(4),
  k: c.k && {
    lo: +c.k.lo.toFixed(4), hi: +c.k.hi.toFixed(4), sigma: +c.k.sigma.toFixed(5),
    pStay: +c.k.pStay.toFixed(4), halfLat: +c.k.halfLat.toFixed(4),
    ux: +c.k.ux.toFixed(4), uz: +c.k.uz.toFixed(4), dist: +c.k.dist.toFixed(3),
    commanded: +c.k.commanded.toFixed(4), cosEl: +c.k.cosEl.toFixed(4),
  },
});

export async function main(argv: string[] = []): Promise<void> {
  const i = argv.indexOf('--step');
  const step = i >= 0 ? Number(argv[i + 1]) : 6;
  const { cells, threshold } = buildZone(step);
  const cal = loadLandCal();

  console.log('SHOT ZONE — where a perfectly aimed shot is worth taking');
  console.log(`  ${(step * 2.54).toFixed(0)} cm grid, hood and rpm from the shipped table, aperture as seen from each spot.`);
  console.log(`  ${cal ? 'calibrated against config/landcal.json' : 'RAW model score, nothing measured'}; the gate fires at ${(threshold * 100).toFixed(0)}%.`);
  console.log('');
  console.log('  HIVE at the top. # clears the gate, + is over half of it, . is refused, space has no solution.');

  const zs = [...new Set(cells.map((c) => Math.round(c.z_in)))].sort((a, b) => a - b);
  const xs = [...new Set(cells.map((c) => Math.round(c.x_in)))].sort((a, b) => a - b);
  const at = new Map(cells.map((c) => [`${Math.round(c.x_in)},${Math.round(c.z_in)}`, c]));
  for (const z of zs) {
    let line = '';
    for (const x of xs) {
      const c = at.get(`${x},${z}`);
      line += !c || c.p <= 0 ? ' ' : c.p >= threshold ? '#' : c.p >= threshold / 2 ? '+' : '.';
    }
    console.log(`  ${String(Math.round(z)).padStart(5)} |${line}|`);
  }
  console.log('');

  const live = cells.filter((c) => c.p >= threshold);
  console.log(`  ${live.length} of ${cells.length} grid squares clear the ${(threshold * 100).toFixed(0)}% gate.`);
  if (live.length) {
    const r = live.map((c) => c.range_in);
    console.log(
      `  ranges ${fm(Math.min(...r))}-${fm(Math.max(...r))}, ` +
      `off-axis up to ${Math.max(...live.map((c) => c.offAxisDeg)).toFixed(0)} deg`,
    );
  }
  const mean = (a: number[]) => (a.length ? a.reduce((x, y) => x + y, 0) / a.length : 0);
  const band = (lo: number, hi: number) => cells.filter((c) => c.offAxisDeg >= lo && c.offAxisDeg < hi);
  console.log('');
  console.log('  off-axis    squares   mean P   clears the gate');
  for (const [lo, hi] of [[0, 15], [15, 30], [30, 45], [45, 90]]) {
    const b = band(lo, hi);
    if (!b.length) continue;
    const ok = b.filter((c) => c.p >= threshold).length;
    console.log(
      `  ${String(lo).padStart(4)}-${String(hi).padEnd(3)} deg  ${String(b.length).padStart(7)}   ` +
      `${(mean(b.map((c) => c.p)) * 100).toFixed(0).padStart(5)}%   ${((ok / b.length) * 100).toFixed(0).padStart(6)}%`,
    );
  }

  // DOES IT MOVE WHEN THE ROBOT DOES? One number per case, same grid, same everything else.
  console.log('');
  console.log('  driving        squares clearing the gate   mean P');
  for (const [name, v] of [
    ['stationary', [0, 0]],
    ['1 m/s at the hive', [0, -1]],
    ['1 m/s away', [0, 1]],
    ['1 m/s across', [1, 0]],
    ['2 m/s away', [0, 2]],
  ] as const) {
    const z = buildZone(step, [...v] as [number, number]);
    const ok = z.cells.filter((c) => c.p >= z.threshold).length;
    const m = z.cells.reduce((a, c) => a + c.p, 0) / Math.max(1, z.cells.length);
    console.log(`  ${name.padEnd(20)} ${String(ok).padStart(5)} of ${z.cells.length}          ${(m * 100).toFixed(0).padStart(4)}%`);
  }

  // AND THE OTHER HALF OF THE MATCH. A tip puts the rocker on its other stop, the up CELL
  // becomes the other one, and the mouth swings across the pivot -- so the zone does too.
  const flipped = buildZone(step, [0, 0], 1);
  const flippedLive = flipped.cells.filter((c) => c.p >= flipped.threshold);
  console.log('');
  console.log(`  after a TIP the rocker rests on its other stop: ${flippedLive.length} of ${flipped.cells.length} squares clear the gate there,`);
  const meanZ = (cs: ZoneCell[]) => (cs.length ? cs.reduce((a, c) => a + c.z_in, 0) / cs.length : NaN);
  console.log(`  centred at z = ${fm(meanZ(flippedLive))} against z = ${fm(meanZ(live))} before it. The zone MOVES.`);

  writeFileSync(new URL('../config/shotzone.json', import.meta.url), JSON.stringify({
    _about: 'MODEL map from tools/shotzone.ts: the chance a perfectly aimed shot lands, per field position, using the hood and rpm the shipped table commands at that range and the CELL mouth as seen from that spot. Not a measurement -- it says where the physics is forgiving, and the spots it likes still have to be shot from.',
    generated: new Date().toISOString(),
    step_in: step,
    threshold,
    /** Constants the live recompute needs, so the renderer does not import the robot spec. */
    maxSpeed: (robotJson as unknown as RobotSpec).flywheel.k
      * (robotJson as unknown as RobotSpec).flywheel.r_fly_m
      * rpmToRadS((robotJson as unknown as RobotSpec).flywheel.maxRpm),
    yawScatterDeg: (robotJson as unknown as RobotSpec).flywheel.scatter.yaw_deg,
    cells: cells.map(trim),
    /** The same map for the rocker's other stop, which is where it sits after a TIP. */
    cellsTipped: flipped.cells.map(trim),
  }, null, 1) + String.fromCharCode(10));
  console.log('');
  console.log('  wrote config/shotzone.json');
}
