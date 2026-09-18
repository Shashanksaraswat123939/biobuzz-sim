/**
 * The FLOWER table: hood angle and wheel speed to drop a ball into the top of the tube.
 *
 *   npm run tool -- tools/flowertable.ts [--step 2]
 *
 * A FLOWER cannot be loaded through its retrieval opening -- tools/flowerprobe.ts stood a
 * loaded robot in front of all four at four stand-offs, reversed the roller for six seconds,
 * and not one ball went in, because the intake only touches a ball at the MOUTH and a ball in
 * the bin has no path back to it. The opening is a way OUT of a flower.
 *
 * The way in is over the top: PHYSICS_AND_SIMULATION.md 7.2's trough lob, a soft steep shot
 * into the tube's mouth. This solves it properly rather than by the doc's three sample rows --
 * for each stand-off, every (hood, speed) pair is flown and the one that clears the rim by the
 * biggest margin while still descending wins. That margin IS the tolerance: the tube is a
 * 4.0 in hole for a 2.80 in ball, so there is 0.6 in of room either side and the answer has to
 * be centred in it rather than merely inside it.
 *
 * Writes java/teamcode/assets/flowertable.csv, which BuiltinTeleOp reads in FLOWER mode.
 */
import { writeFileSync, mkdirSync } from 'node:fs';
import params from '../config/params.json' with { type: 'json' };
import robotSpec from '../config/robot.json' with { type: 'json' };
import { buildFieldGeometry } from '../packages/core/src/field/geometry.js';
import { worldToFtc } from '../packages/core/src/field/ftcFrame.js';
import { simulateShot } from '../packages/core/src/physics/ballistics.js';
import { DEG, M_TO_IN, inches, rpmToRadS, radSToRpm } from '../packages/core/src/units.js';
import type { Params, RobotSpec } from '../packages/core/src/types.js';

export interface FlowerRow {
  range_in: number;
  hoodDeg: number;
  rpm: number;
  /** Height it crosses the tube's rim at, inches, and how far that is above the rim. */
  /** Where it comes down through the rim, inches from the muzzle. */
  arriveY_in: number;
  /** Room to spare inside the hole, inches. 0 means it grazed the rim. */
  clearance_in: number;
  /** Degrees below horizontal at the rim. A ball still climbing bounces off the far lip. */
  descentDeg: number;
  /** Half-width of the band of speeds that still drop in, as a fraction. The tolerance. */
  margin: number;
}

export function buildFlowerTable(p: Params, spec: RobotSpec, step = 2): FlowerRow[] {
  const g = buildFieldGeometry(p);
  const f = g.flowers[0];
  const r = p.ball.pollen.d_m / 2;
  const k = spec.flywheel.k * spec.flywheel.r_fly_m;
  const [hoodLo, hoodHi] = spec.hood.angleRange_deg;
  const muzzleY = spec.turret.muzzleHeight_m + spec.chassis.clearance_m;
  // A ball is in when its CENTRE crosses inside the rim: the hole's radius less the ball's.
  const rimR = f.openingR_m - r;
  const rows: FlowerRow[] = [];

  /**
   * WHERE IT COMES DOWN THROUGH THE RIM'S HEIGHT, not how high it is over the tube.
   *
   * The first version asked for the height at the tube's own distance and took anything
   * above the rim -- which selected shots passing 4 in over the hole, still travelling, that
   * land somewhere behind the flower. A ball goes IN when its descending crossing of the rim
   * height happens within the hole, so the quantity is a DISTANCE, and the tolerance is the
   * hole's radius less the ball's.
   */
  const crossing = (hood: number, v: number) => {
    const t = simulateShot(p, {
      from: [0, muzzleY, 0], azimuth: 0, elevation: hood * DEG, speed: v,
      radius: r, mass: p.ball.pollen.m_kg, spin: v / r,
    }, 3.0, 1 / 480);
    const pts = t.points;
    for (let i = 1; i < pts.length; i++) {
      if (pts[i - 1][1] >= f.topY_m && pts[i][1] < f.topY_m) {
        const fr = (pts[i - 1][1] - f.topY_m) / (pts[i - 1][1] - pts[i][1] || 1);
        const z = pts[i - 1][2] + (pts[i][2] - pts[i - 1][2]) * fr;
        const vy = pts[i][1] - pts[i - 1][1];
        const vz = pts[i][2] - pts[i - 1][2];
        return { at: z, descent: Math.atan2(-vy, Math.abs(vz)) * (180 / Math.PI) };
      }
    }
    return null;
  };
  const drops = (hood: number, v: number, d_m: number) => {
    const a = crossing(hood, v);
    // Inside the hole when it comes down through the rim, and steep enough to go down the
    // tube rather than skip across it.
    return !!a && Math.abs(a.at - d_m) <= rimR && a.descent > 25;
  };
  const fly = (hood: number, v: number, d_m: number) => {
    const a = crossing(hood, v);
    void d_m;
    return { y: f.topY_m, descent: a ? a.descent : NaN, at: a ? a.at : NaN };
  };

  for (let d_in = 8; d_in <= 26; d_in += step) {
    const d_m = inches(d_in);
    let best: FlowerRow | null = null;
    let bestScore = -Infinity;
    for (let hood = hoodLo; hood <= hoodHi; hood += 0.5) {
      // The contiguous band of speeds that drop in at this hood angle.
      let lo = NaN, hi = NaN;
      for (let v = 1.6; v <= 5.0; v += 0.02) {
        if (drops(hood, v, d_m)) { if (Number.isNaN(lo)) lo = v; hi = v; }
        else if (!Number.isNaN(lo)) break;
      }
      if (Number.isNaN(lo)) continue;
      const mid = (lo + hi) / 2;
      const margin = (hi - lo) / 2 / mid;
      const a = fly(hood, mid, d_m);
      // Widest speed band wins: that is the tolerance to a wheel that is not quite on speed,
      // which is the only error this shot has left once the hood is a commanded servo.
      if (margin > bestScore) {
        bestScore = margin;
        best = {
          range_in: d_in, hoodDeg: hood, rpm: radSToRpm(mid / k),
          // Where it actually comes down through the rim, and how far that is from the
          // tube's axis. Zero is dead centre; the hole allows +-rimR.
          arriveY_in: a.at * M_TO_IN, clearance_in: (rimR - Math.abs(a.at - d_m)) * M_TO_IN,
          descentDeg: a.descent, margin,
        };
      }
    }
    if (best) rows.push(best);
  }
  return rows;
}

export function toCsv(rows: FlowerRow[], flowers: { x: number; y: number }[] = []): string {
  // THE TARGETS TRAVEL WITH THE SOLUTION. The robot needs to know where the flowers ARE as
  // well as how to reach one, and a second file to keep in step is a second file to get out
  // of step. FTC inches, from the same geometry the rows were solved against.
  const where = flowers.map((f, i) => `# flower${i}=${f.x.toFixed(2)},${f.y.toFixed(2)}`);
  const head = 'range_in,hoodDeg,rpm,arriveY_in,clearance_in,descentDeg,margin';
  return [...where, head, ...rows.map((r) => [
    r.range_in.toFixed(1), r.hoodDeg.toFixed(2), r.rpm.toFixed(0), r.arriveY_in.toFixed(2),
    r.clearance_in.toFixed(2), r.descentDeg.toFixed(1), r.margin.toFixed(4),
  ].join(','))].join('\n') + '\n';
}

export async function main(argv: string[] = []): Promise<void> {
  const i = argv.indexOf('--step');
  const step = i >= 0 ? Number(argv[i + 1]) : 2;
  const p = params as unknown as Params;
  const spec = robotSpec as unknown as RobotSpec;
  const g = buildFieldGeometry(p);
  const f = g.flowers[0];
  console.log('\nTHE FLOWER TABLE — dropping a ball into the top of the tube.');
  console.log(`  The hole is ${(f.openingR_m * 2 * M_TO_IN).toFixed(1)} in across at ${(f.topY_m * M_TO_IN).toFixed(1)} in, for a ${(p.ball.pollen.d_m * M_TO_IN).toFixed(2)} in ball.`);
  console.log(`  Muzzle at ${((spec.turret.muzzleHeight_m + spec.chassis.clearance_m) * M_TO_IN).toFixed(1)} in. Wheel speed is ${(spec.flywheel.k * spec.flywheel.r_fly_m * rpmToRadS(1000)).toFixed(2)} m/s per 1000 rpm.\n`);
  const rows = buildFlowerTable(p, spec, step);
  if (!rows.length) {
    console.log('  NO SOLUTION AT ANY STAND-OFF. The shooter cannot reach the tube top.');
    return;
  }
  console.log('  stand-off   hood    rpm   comes down at   room to spare   descent   speed margin');
  for (const r of rows) {
    console.log(`  ${r.range_in.toFixed(0).padStart(7)} in  ${r.hoodDeg.toFixed(1).padStart(5)}  ${r.rpm.toFixed(0).padStart(5)}  ${r.arriveY_in.toFixed(1).padStart(9)} in  ${r.clearance_in.toFixed(1).padStart(12)} in  ${r.descentDeg.toFixed(0).padStart(7)}°  ${(r.margin * 100).toFixed(1).padStart(11)}%`);
  }
  const dir = new URL('../java/teamcode/assets/', import.meta.url);
  mkdirSync(dir, { recursive: true });
  const ftc = g.flowers.map((fl) => {
    const w = worldToFtc([fl.x_m, 0, fl.z_m]);
    return { x: w[0], y: w[1] };
  });
  writeFileSync(new URL('flowertable.csv', dir), toCsv(rows, ftc));
  const mean = rows.reduce((a, r) => a + r.margin, 0) / rows.length;
  console.log(`\n  ${rows.length} stand-offs solved, mean speed margin ${(mean * 100).toFixed(1)}%.`);
  console.log('  wrote java/teamcode/assets/flowertable.csv\n');
}
