/**
 * Generate java/teamcode/assets/shottable.csv for the current robot: for each range from
 * the robot's centre to the up CELL, the hood position and flywheel RPM with the widest
 * band of exit speeds that still put the ball through the mouth.
 *
 *   npm run tool -- tools/shottable.ts [--min 30] [--max 150] [--step 4]
 *
 * Range is measured centre-to-target because that is what the robot can actually sense;
 * the muzzle offset is applied here so the table and the robot agree.
 */
import { writeFileSync, mkdirSync, existsSync, readFileSync } from 'node:fs';
import params from '../config/params.json' with { type: 'json' };
import robotSpec from '../config/robot.json' with { type: 'json' };
import { buildFieldGeometry } from '../packages/core/src/field/geometry.js';
import { bestShot, type Aperture, type ShotTableRow } from '../packages/core/src/physics/ballistics.js';
import { inches, M_TO_IN } from '../packages/core/src/units.js';
import { EntryModel, type EntryTable } from '../packages/core/src/robot/entryModel.js';
import type { Params, RobotSpec, Vec3 } from '../packages/core/src/types.js';

/**
 * The measured entry model, if tools/entrycheck.ts has been run. Without it the table falls
 * back to the old margin-only objective and SAYS SO, rather than silently optimising for the
 * wrong thing.
 */
export function loadEntry(): EntryModel | null {
  const url = new URL('../config/entry.json', import.meta.url);
  if (!existsSync(url)) return null;
  return new EntryModel(JSON.parse(readFileSync(url, 'utf8')) as EntryTable);
}

/**
 * The up CELL's mouth at rest, as the two lips a ball must thread between.
 * The mouth is a line in the (Y, Z) plane tilted about 49 deg; a shooter standing off in
 * +Z meets the low lip first and must clear it, then must be below the high lip.
 */
export function mouthLips(
  p: Params,
  /**
   * Which stop the rocker is resting on. -1 is where a RED hive starts; +1 is where it lands
   * after a TIP, and the two are not cosmetic -- the up CELL becomes the other one, so the
   * mouth swings to the far side of the pivot and the whole shooting position moves with it.
   * `Hive.side` is this number at run time.
   */
  side: -1 | 1 = -1,
): { x_m: number; near: { y: number; z: number }; far: { y: number; z: number }; facing: -1 | 1 } {
  const g = buildFieldGeometry(p);
  const theta = side * g.restAngle_rad;
  // The same test Hive.upCell uses: the cell whose radius points up at this angle.
  const cell = Math.cos(g.cells[0].bodyAngle_rad + theta) > 0 ? g.cells[0] : g.cells[1];
  const phi = cell.bodyAngle_rad + theta;
  const r = cell.radius_m + cell.halfInterior[1];
  const ht = cell.halfInterior[2];
  const lip = (sign: number) => ({
    y: g.pivotY_m + r * Math.cos(phi) - sign * ht * Math.sin(phi),
    z: r * Math.sin(phi) + sign * ht * Math.cos(phi),
  });
  const a = lip(+1);
  const b = lip(-1);
  // WHICH WAY THE MOUTH OPENS, which is not a constant. At rest it leans toward +Z and the
  // shooter stands in +Z; after a TIP the rocker is on its other stop, the up CELL is the
  // other one, and the mouth leans the other way -- so "the near lip" is the opposite one and
  // a shooter on the old side is looking at the back of the pocket. Assuming +Z made the
  // tipped map come out with zero shootable squares out of 418, which is what a sign error
  // looks like when it is polite enough to be obvious.
  const facing: -1 | 1 = Math.sin(phi) >= 0 ? 1 : -1;
  const near = facing * a.z > facing * b.z ? a : b;
  const far = near === a ? b : a;
  return { x_m: g.hiveX_m.red, near, far, facing };
}

export function buildTable(
  p: Params, spec: RobotSpec, min = 30, max = 150, step = 4,
  entry: EntryModel | null = null,
  objective: 'margin' | 'pLand' = 'pLand',
): ShotTableRow[] {
  const g = buildFieldGeometry(p);
  const lips = mouthLips(p);
  const rows: ShotTableRow[] = [];
  const ballR = p.ball.pollen.d_m / 2;
  const spinPerSpeed = spec.flywheel.type === 'single' ? 1 / ballR : 0;

  let prevHood: number | undefined;
  for (let rIn = min; rIn <= max; rIn += step) {
    // Robot centre at `rIn` from the mouth centre, on the +Z side, facing the hive.
    const centreZ = (lips.near.z + lips.far.z) / 2 + inches(rIn);
    const muzzle: Vec3 = [lips.x_m, spec.turret.muzzleHeight_m, centreZ - spec.turret.muzzleOffset_m];
    const aperture: Aperture = {
      nearRange: muzzle[2] - lips.near.z,
      nearHeight: lips.near.y + ballR,
      farRange: muzzle[2] - lips.far.z,
      farHeight: lips.far.y - ballR,
    };
    if (aperture.nearRange <= 0) continue;

    const row = bestShot(p, {
      muzzle,
      azimuth: Math.PI, // facing -Z, toward the hive
      aperture,
      radius: ballR,
      mass: p.ball.pollen.m_kg,
      hoodRange: spec.hood.enabled ? spec.hood.angleRange_deg : [spec.hood.fixedAngle_deg, spec.hood.fixedAngle_deg],
      hoodSteps: spec.hood.enabled ? 31 : 1,
      spinPerSpeed,
      k: spec.flywheel.k,
      rFly: spec.flywheel.r_fly_m,
      maxRpm: spec.flywheel.maxRpm,
      range_in: rIn,
      minDescentDeg: 15,
      maxApex_m: 3.6,
      maxFlight_s: 2.0,
      preferHoodPos: prevHood,
      objective,
      entryRate: entry ? (v, d) => entry.lookup(v, d) : undefined,
      scatter: { speedFrac: spec.flywheel.scatter.speedFrac, angle_deg: spec.flywheel.scatter.angle_deg },
    });
    if (row) {
      // The mouth's half-width across the shot line. The solver's aperture is a 2-D slice
      // down the shot line and says nothing about width, so this is measured off the CELL
      // and carried so the robot can work out whether it is pointing well enough.
      row.halfLat_m = g.cells[0].halfInterior[0] - ballR;
      rows.push(row);
      prevHood = row.hoodPos;
    }
  }
  return rows;
}

/**
 * Columns past `margin` are additions and are appended, never inserted: tools/genconstants.mjs
 * reads this file by column INDEX to bake it into Java, so moving an existing column silently
 * corrupts the hub's copy of the table.
 */
export function toCsv(rows: ShotTableRow[]): string {
  const head = 'range_in,hoodPos,rpm,margin,hoodDeg,speedLo,speedHi,sigmaSpeed,pStay,halfLat_m';
  return [head, ...rows.map((r) => [
    r.range_in.toFixed(1), r.hoodPos.toFixed(4), r.rpm.toFixed(0), r.margin.toFixed(4),
    r.hoodDeg.toFixed(2), r.speedLo.toFixed(4), r.speedHi.toFixed(4), r.sigmaSpeed.toFixed(5), r.pStay.toFixed(4), (r.halfLat_m ?? 0).toFixed(4),
  ].join(','))].join('\n') + '\n';
}

export async function main(argv: string[] = []): Promise<void> {
  const num = (k: string, d: number) => {
    const i = argv.indexOf(`--${k}`);
    return i >= 0 ? Number(argv[i + 1]) : d;
  };
  const p = params as unknown as Params;
  const spec = robotSpec as unknown as RobotSpec;
  const lips = mouthLips(p);
  console.log(
    `mouth lips (in): near Y ${(lips.near.y * M_TO_IN).toFixed(1)} Z ${(lips.near.z * M_TO_IN).toFixed(1)}  |  far Y ${(lips.far.y * M_TO_IN).toFixed(1)} Z ${(lips.far.z * M_TO_IN).toFixed(1)}`,
  );

  const entry = loadEntry();
  console.log(entry
    ? `entry model: config/entry.json, best cell ${entry.best().speed_mps} m/s at ${entry.best().descent_deg} deg -> ${(entry.best().rate * 100).toFixed(0)}%`
    : 'entry model: NONE (run tools/entrycheck.ts). Falling back to the margin-only objective, ' +
      'which optimises for threading the mouth and is blind to whether the ball stays in.');
  const rows = buildTable(p, spec, num('min', 30), num('max', 150), num('step', 4), entry);
  const dir = new URL('../java/teamcode/assets/', import.meta.url);
  mkdirSync(dir, { recursive: true });
  writeFileSync(new URL('shottable.csv', dir), toCsv(rows));

  console.log(`\nrange_in  hood  hoodDeg   rpm   margin  arrival    apex  flight  arr m/s`);
  for (const r of rows) {
    const arrive = r.descentDeg >= 0 ? `${r.descentDeg.toFixed(0)} down` : `${(-r.descentDeg).toFixed(0)} up`;
    console.log(
      `${r.range_in.toFixed(0).padStart(7)}  ${r.hoodPos.toFixed(2)}  ${r.hoodDeg.toFixed(0).padStart(5)}  ` +
      `${r.rpm.toFixed(0).padStart(5)}  ${(r.margin * 100).toFixed(1).padStart(5)}%  ${arrive.padStart(8)}  ` +
      `${(r.apex_m * M_TO_IN).toFixed(0).padStart(4)} in  ${r.flight_s.toFixed(2)} s  ${r.arrival_mps.toFixed(1).padStart(5)}`,
    );
  }
  const apexes = rows.map((r) => r.apex_m * M_TO_IN);
  const mean = (a: number[]) => a.reduce((x, y) => x + y, 0) / a.length;
  console.log(
    `\napex ${Math.min(...apexes).toFixed(0)}-${Math.max(...apexes).toFixed(0)} in (mean ${mean(apexes).toFixed(0)}), ` +
    `flight ${mean(rows.map((r) => r.flight_s)).toFixed(2)} s mean, ` +
    `arrival ${mean(rows.map((r) => r.descentDeg)).toFixed(0)} deg down at ${mean(rows.map((r) => r.arrival_mps)).toFixed(1)} m/s`,
  );
  const usable = rows.filter((r) => r.margin >= 0.02);
  console.log(`\n${rows.length} ranges solved, ${usable.length} with >=2% speed margin`);
  if (usable.length) console.log(`usable band: ${usable[0].range_in.toFixed(0)}..${usable[usable.length - 1].range_in.toFixed(0)} in`);
  console.log('wrote java/teamcode/assets/shottable.csv');
}
