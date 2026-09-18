/**
 * Of the balls that reach the CELL mouth, how many stay in?
 *
 * `tools/entrycheck.ts` measures it by injecting balls at the mouth at a chosen arrival
 * speed and descent angle with no shooter in the loop, and writes `config/entry.json`.
 * This reads that grid and interpolates it.
 *
 * WHY IT EXISTS. The shot table used to be built by maximising SPEED MARGIN -- how much
 * exit-speed error still threads the mouth. Threading the mouth is not scoring. A ball
 * arriving fast keeps most of that speed through its first bounce off the pocket floor and
 * can come straight back out, and the solver could not see that at all. It therefore chose
 * near-vertical 81 degree lobs arriving at 6.2 m/s, because a shot that steep has enormous
 * speed tolerance -- at 81 degrees a speed error mostly changes HEIGHT rather than range,
 * so the ball still falls through the hole.
 *
 * The measurement says that arrival stays in about a quarter of the time, and a flatter one
 * stays in most of the time. That is the entire gap between the two, and it was invisible.
 */

export interface EntryTable {
  generated: string;
  ballsPerCell: number;
  speeds_mps: number[];
  descents_deg: number[];
  /** rate[descentIndex][speedIndex] */
  rate: number[][];
}

export class EntryModel {
  constructor(readonly table: EntryTable) {
    if (!table.speeds_mps?.length || !table.descents_deg?.length) {
      throw new Error('entry table has no axes');
    }
  }

  /**
   * Fraction of balls arriving at (speed, descent) that stay in. Bilinear, clamped at the
   * edges -- clamping is the honest choice here because the grid spans the whole range the
   * shooter can actually produce, so anything outside it is a shot that will not be chosen.
   */
  lookup(arrival_mps: number, descentDeg: number): number {
    const [i0, i1, ts] = span(this.table.speeds_mps, arrival_mps);
    const [j0, j1, td] = span(this.table.descents_deg, descentDeg);
    const r = this.table.rate;
    return (
      (1 - td) * ((1 - ts) * r[j0][i0] + ts * r[j0][i1]) +
      td * ((1 - ts) * r[j1][i0] + ts * r[j1][i1])
    );
  }

  /** The best cell in the grid, for reporting what the shooter should be aiming to produce. */
  best(): { speed_mps: number; descent_deg: number; rate: number } {
    let out = { speed_mps: 0, descent_deg: 0, rate: -1 };
    this.table.descents_deg.forEach((d, j) => {
      this.table.speeds_mps.forEach((s, i) => {
        if (this.table.rate[j][i] > out.rate) out = { speed_mps: s, descent_deg: d, rate: this.table.rate[j][i] };
      });
    });
    return out;
  }
}

/** Bracketing indices and the blend, clamped at both ends. */
function span(axis: number[], v: number): [number, number, number] {
  if (axis.length === 1) return [0, 0, 0];
  if (v <= axis[0]) return [0, 1, 0];
  if (v >= axis[axis.length - 1]) return [axis.length - 2, axis.length - 1, 1];
  for (let i = 1; i < axis.length; i++) {
    if (v <= axis[i]) return [i - 1, i, (v - axis[i - 1]) / (axis[i] - axis[i - 1])];
  }
  return [axis.length - 2, axis.length - 1, 1];
}

/**
 * The measured mapping from the model's raw P(land) score to how often shots at that score
 * ACTUALLY land (`config/landcal.json`, written by tools/landcal.ts).
 *
 * Without it the threshold is a score with a percent sign on it: tools/gatecal.ts measured
 * the raw model overconfident by about 20 points at every setting, so asking for 90% got
 * 71%. The two factors in the raw score are both real -- they just do not account for the
 * ribs and churros inside the pocket, or for the fact that entry was measured with balls
 * arriving at the middle of the mouth rather than spread across it.
 */
export interface LandCalPoint { score: number; observed: number; n: number }

export class LandCalibration {
  /** The highest frequency any bin of shots actually achieved. */
  readonly ceiling: number;

  constructor(readonly points: LandCalPoint[]) {
    if (!points.length) throw new Error('calibration has no points');
    this.points = [...points].sort((a, b) => a.score - b.score);
    this.ceiling = Math.max(...points.map((p) => p.observed));
  }

  /**
   * Raw model score -> calibrated probability. Piecewise linear, and ANCHORED AT THE ORIGIN
   * below the lowest bin that was actually measured.
   *
   * It used to clamp: anything at or under the lowest fitted score returned that bin's
   * observed rate. The fit's lowest bin is score 0.49 -> 0.839, because the gate was open but
   * the SHOOTER still only produced shots it thought were decent, so nothing below 0.49 was
   * ever sampled. Clamping therefore asserted that a shot the model rates at 5% lands 84% of
   * the time -- an 84% floor under every number in the system.
   *
   * Two things fell out of that floor, and they looked unrelated until this was found. The
   * shot-zone map came out a uniform green blob: every solvable square on the field scored
   * between 0.839 and 0.869, a three point spread, because almost all of them were below the
   * fit and all got the same answer. And `flywheel.minLandProb` stopped being a gate at all
   * -- no shot could ever score under 0.839, so 0.70, 0.80 and 0.85 were measured to give
   * byte-identical results.
   *
   * Below the measured range there is no data, so the honest curve is the conservative one
   * that is still monotone: a straight line from (0, 0) up to the first bin. It says "we did
   * not measure here, and we assume it gets worse", which is what not measuring means.
   */
  apply(score: number): number {
    const p = this.points;
    if (score <= 0) return 0;
    if (score <= p[0].score) return (score / p[0].score) * p[0].observed;
    if (score >= p[p.length - 1].score) return p[p.length - 1].observed;
    for (let i = 1; i < p.length; i++) {
      if (score <= p[i].score) {
        const t = (score - p[i - 1].score) / (p[i].score - p[i - 1].score);
        return p[i - 1].observed + t * (p[i].observed - p[i - 1].observed);
      }
    }
    return p[p.length - 1].observed;
  }
}
