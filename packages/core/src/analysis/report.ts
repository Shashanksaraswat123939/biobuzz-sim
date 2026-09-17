/**
 * Turn a run of shots into the two numbers that matter -- where the group sits, and how
 * big it is -- plus a plain statement of which of the two is the problem.
 *
 * Bias and spread are different faults with different fixes. A group that is tight and 20
 * inches long is a TABLE error: the shot table says one thing and the world does another,
 * and one offset fixes every shot at once. A group that is centred but 40 inches wide is a
 * REPEATABILITY error: no offset helps, and the fix is whichever knob owns the biggest
 * share of the sensitivity budget. Reporting one number for "accuracy" hides that, which
 * is why this reports them separately and says which one dominates.
 */
import type { ShotRecord } from '../types.js';

export interface RangeBin {
  range_in: number;
  n: number;
  landed: number;
  meanMiss_in: number;
}

export interface Report {
  n: number;
  landed: number;
  landRate: number;
  /** Mean distance from the CELL mouth where shots ended, inches. */
  meanMiss_in: number;
  sdMiss_in: number;
  /** Mean signed downrange error: positive is LONG. One offset fixes all of this. */
  bias_in: number;
  /** Spread of the downrange error: the part no single offset can fix. */
  sd_in: number;
  /** Mean signed lateral error: positive is left of the shot line. */
  latBias_in: number;
  latSd_in: number;
  /** Mean absolute RPM error at the moment of firing. */
  rpmErr_rpm: number;
  /** Shots fired while the wheel was outside its tolerance band. */
  firedOffSpeed: number;
  byRange: RangeBin[];
  /** Which fault dominates, and what to do about it. */
  verdict: string;
  /** One line per real problem found, worst first. Empty means nothing stood out. */
  faults: string[];
  /** Mean range the run was fired from, inches. The trims are only valid near it. */
  meanRange_in: number;
  /**
   * The correction this run implies, in the units `robot.json -> calibration` takes.
   * Applying it should move the group onto the mouth; it cannot do anything about spread.
   */
  fix: { rangeTrim_in: number; turretTrim_deg: number; worthIt: boolean; why: string };
}

const mean = (a: number[]) => (a.length ? a.reduce((x, y) => x + y, 0) / a.length : 0);
const sd = (a: number[]) => {
  if (a.length < 2) return 0;
  const m = mean(a);
  return Math.sqrt(a.reduce((x, y) => x + (y - m) ** 2, 0) / (a.length - 1));
};

/**
 * `tolRpm` is the flywheel's own tolerance band, used to count shots that went out while
 * the wheel was not up to speed -- a gate fault, not an aiming fault.
 */
export function analyse(shots: ShotRecord[], tolRpm: number): Report {
  const done = shots.filter((s) => s.result !== 'flight');
  const miss = done.map((s) => s.missBy).filter(Number.isFinite);
  const landed = done.filter((s) => s.result === 'cell').length;

  // The world writes the error resolved along the shot line it was actually fired down,
  // so long/short and left/right are separate faults with separate fixes.
  const long = done.map((s) => s.long_in).filter(Number.isFinite);
  const lat = done.map((s) => s.lat_in).filter(Number.isFinite);

  const rpmErrs = done.map((s) => Math.abs(s.rpm - s.targetRpm)).filter(Number.isFinite);
  const offSpeed = done.filter((s) => Math.abs(s.rpm - s.targetRpm) > tolRpm).length;

  // 12 in range bins, so "it only misses past 90 in" is visible instead of averaged away.
  const bins = new Map<number, ShotRecord[]>();
  for (const s of done) {
    const b = Math.round(s.rangeIn / 12) * 12;
    (bins.get(b) ?? bins.set(b, []).get(b)!).push(s);
  }
  const byRange: RangeBin[] = [...bins.entries()]
    .sort((a, b) => a[0] - b[0])
    .map(([range_in, rs]) => ({
      range_in,
      n: rs.length,
      landed: rs.filter((r) => r.result === 'cell').length,
      meanMiss_in: mean(rs.map((r) => r.missBy).filter(Number.isFinite)),
    }));

  const bias = mean(long);
  const spread = sd(long);
  const latBias = mean(lat);
  const latSpread = sd(lat);
  const rate = done.length ? landed / done.length : 0;

  const faults: string[] = [];
  if (done.length < 8) faults.push(`Only ${done.length} shots have landed. Anything below about 20 is noise, not a measurement.`);
  if (offSpeed > done.length * 0.1) {
    faults.push(`${offSpeed} of ${done.length} shots left while the flywheel was outside its ${tolRpm} rpm band. The readiness gate is letting shots through early — raise flywheel.readySteps or lower the cycle rate.`);
  }
  if (Math.abs(bias) > 6) {
    faults.push(`The group lands ${Math.abs(bias).toFixed(0)} in ${bias > 0 ? 'LONG' : 'SHORT'} on average. That is a shot-table error and one offset fixes every shot: regenerate with tools/shottable.ts, or trim flywheel.k by about ${((-bias / Math.max(1, mean(done.map((s) => s.rangeIn)))) * 100).toFixed(1)}%.`);
  }
  if (Math.abs(latBias) > 5) {
    faults.push(`The group sits ${Math.abs(latBias).toFixed(0)} in to the ${latBias > 0 ? 'LEFT' : 'RIGHT'} of the line. Lateral bias is an aiming fault, not a ballistic one: check the turret encoder zero and the motion lead.`);
  }
  if (spread > 18) faults.push(`Downrange spread is ${spread.toFixed(0)} in, 1 sigma. No offset helps this. Open the Predictor and fix whichever knob owns the biggest share.`);
  if (latSpread > 12) faults.push(`Lateral spread is ${latSpread.toFixed(0)} in, 1 sigma. That is turret settling or yaw scatter — check flywheel.scatter.yaw_deg and whether shots go out while the turret is still slewing.`);
  if (rate > 0 && rate < 0.35 && spread < 18 && Math.abs(bias) < 6) {
    faults.push('Aim is good but the balls are not staying in. That is the pocket, not the shot — check ball.e_poly, which is still a guess.');
  }

  const verdict = !done.length
    ? 'No shots have finished yet.'
    : Math.abs(bias) > spread
      ? `Dominated by BIAS: the group is ${bias.toFixed(0)} in ${bias > 0 ? 'long' : 'short'} but only ${spread.toFixed(0)} in wide. One table offset recovers most of it.`
      : `Dominated by SPREAD: the group is centred to ${bias.toFixed(0)} in but ${spread.toFixed(0)} in wide. An offset will not help; reduce the top contributor in the Predictor.`;

  // The correction. Downrange: the table's answer for R actually reached R + bias, so ask
  // it for R - bias instead. Lateral: the group sits latBias inches off at meanRange, which
  // is atan(latBias / meanRange) of azimuth error, whatever caused it.
  const meanRange = mean(done.map((s) => s.rangeIn)) || 0;
  const turretTrim = meanRange > 1 ? Math.atan2(latBias, meanRange) * (180 / Math.PI) : 0;
  // Below the noise floor there is nothing to correct: with n shots, the standard error of
  // the mean is sd/sqrt(n), and trimming by less than that is chasing the sample, not the
  // robot.
  const stderr = done.length ? spread / Math.sqrt(done.length) : Infinity;
  const worthIt = done.length >= 8 && Math.abs(bias) > Math.max(2, stderr);
  const fixWhy = !done.length
    ? 'No finished shots.'
    : done.length < 8
      ? `Only ${done.length} shots. Collect at least 8 before trimming, or you will calibrate out a sample and not a fault.`
      : worthIt
        ? `Trim the range by ${bias >= 0 ? '+' : ''}${bias.toFixed(1)} in and the turret by ${turretTrim >= 0 ? '+' : ''}${turretTrim.toFixed(2)}°. Measured at a mean range of ${meanRange.toFixed(0)} in over ${done.length} shots; the standard error of the mean is ${stderr.toFixed(1)} in.`
        : `The bias (${bias.toFixed(1)} in) is inside the noise of this run (±${stderr.toFixed(1)} in standard error over ${done.length} shots). Nothing to trim — the group is where it should be, it is just wide.`;

  return {
    n: done.length,
    landed,
    landRate: rate,
    meanMiss_in: mean(miss),
    sdMiss_in: sd(miss),
    bias_in: bias,
    sd_in: spread,
    latBias_in: latBias,
    latSd_in: latSpread,
    rpmErr_rpm: mean(rpmErrs),
    firedOffSpeed: offSpeed,
    byRange,
    verdict,
    faults,
    meanRange_in: meanRange,
    fix: { rangeTrim_in: bias, turretTrim_deg: turretTrim, worthIt, why: fixWhy },
  };
}

/** The shot log as CSV, so a run can leave the browser and go into a spreadsheet. */
export function toCsv(shots: ShotRecord[]): string {
  const head = 'n,t_s,range_in,bearing_deg,hood_deg,rpm,target_rpm,exit_mps,result,miss_in,long_in,lat_in';
  const body = shots.map((s) =>
    [s.n, s.t.toFixed(2), s.rangeIn.toFixed(1), s.bearingDeg.toFixed(1), s.hoodDeg.toFixed(1),
      s.rpm.toFixed(0), s.targetRpm.toFixed(0), s.exitSpeed.toFixed(2), s.result,
      Number.isFinite(s.missBy) ? s.missBy.toFixed(1) : '',
      Number.isFinite(s.long_in) ? s.long_in.toFixed(1) : '',
      Number.isFinite(s.lat_in) ? s.lat_in.toFixed(1) : ''].join(','),
  );
  return [head, ...body].join('\n');
}
