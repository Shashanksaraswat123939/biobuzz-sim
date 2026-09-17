package org.firstinspires.ftc.teamcode.control;

import org.firstinspires.ftc.teamcode.config.ShotTableData;

/**
 * Will this shot land? A probability, not a tolerance.
 *
 * The hub solves nothing here either: tools/shottable.ts bakes the exit-speed band, its
 * scatter and the measured entry rate into ShotTableData per range, and this turns them into
 * a number using the speed the wheel is ACTUALLY doing and the bearing it is ACTUALLY
 * pointing. Three things have to go right and each one is a factor:
 *
 *   P(land) = P(exit speed threads the mouth) x P(pointing inside the mouth) x P(it stays in)
 *
 * The last factor is measured by tools/entrycheck.ts. The first two are normal integrals over
 * what the shooter can currently do, which is why the answer moves every loop rather than
 * flipping when some tolerance is crossed.
 *
 * The raw product is then passed through a MEASURED score-to-frequency curve
 * (tools/landcal.ts), because the product on its own is optimistic -- it knows nothing about
 * the ribs inside the pocket, and it cannot see anything the sim's own measurement could not.
 * Without the curve the threshold is a score with a percent sign on it.
 */
public final class LandProbability {
    private LandProbability() { }

    /** Abramowitz & Stegun 26.2.17, good to about 1e-7 -- plenty against a measured curve. */
    public static double normalCdf(double z) {
        double sign = z < 0 ? -1 : 1;
        double x = Math.abs(z) / Math.sqrt(2);
        double t = 1 / (1 + 0.3275911 * x);
        double y = 1 - (((((1.061405429 * t - 1.453152027) * t) + 1.421413741) * t - 0.284496736) * t
                + 0.254829592) * t * Math.exp(-x * x);
        return 0.5 * (1 + sign * y);
    }

    /** The chance a normal(mean, sigma) draw lands inside [lo, hi]. */
    public static double pThread(double lo, double hi, double mean, double sigma) {
        if (!(sigma > 0)) return mean >= lo && mean <= hi ? 1 : 0;
        return Math.max(0, normalCdf((hi - mean) / sigma) - normalCdf((lo - mean) / sigma));
    }

    /** Raw model score -> the frequency shots at that score actually landed. Piecewise linear. */
    public static double calibrate(double score) {
        double[] s = ShotTableData.CAL_SCORE;
        double[] o = ShotTableData.CAL_OBSERVED;
        if (s.length == 0) return score;
        if (score <= s[0]) return o[0];
        if (score >= s[s.length - 1]) return o[o.length - 1];
        for (int i = 1; i < s.length; i++) {
            if (score <= s[i]) {
                double t = (score - s[i - 1]) / (s[i] - s[i - 1]);
                return o[i - 1] + t * (o[i] - o[i - 1]);
            }
        }
        return o[o.length - 1];
    }

    /**
     * @param exitSpeed    what k * omega * r says the ball will leave at, from the FILTERED rpm
     * @param rangeM       muzzle to mouth, metres
     * @param turretErrDeg how far off the commanded bearing the turret currently is
     * @param yawScatterDeg one sigma of launch bearing scatter
     * @return calibrated probability in [0, 1], or -1 when the table row carries no model
     */
    public static double pLand(ShotTable table, double rangeIn, double exitSpeed,
                               double rangeM, double turretErrDeg, double yawScatterDeg) {
        double lo = table.lerpAt(ShotTableData.SPEED_LO, rangeIn);
        double hi = table.lerpAt(ShotTableData.SPEED_HI, rangeIn);
        double sigma = table.lerpAt(ShotTableData.SIGMA_SPEED, rangeIn);
        double pStay = table.lerpAt(ShotTableData.P_STAY, rangeIn);
        double halfLat = table.lerpAt(ShotTableData.HALF_LAT_M, rangeIn);
        if (!(hi > lo) || !(sigma > 0)) return -1;

        double speed = pThread(lo, hi, exitSpeed, sigma);
        // Bearing error at the target is range * tan(error), and the launch adds its own
        // scatter, so the arrival across the shot line is normal about where it is pointing.
        double aim = halfLat > 0
                ? pThread(-halfLat, halfLat,
                          rangeM * Math.tan(Math.toRadians(turretErrDeg)),
                          rangeM * Math.tan(Math.toRadians(yawScatterDeg)))
                : 1;
        return calibrate(speed * aim * pStay);
    }
}
