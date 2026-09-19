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
    /**
     * P(a ball stays in) against how many are already in the CELL. MEASURED,
     * tools/whatmisses.ts, 385 settled shots split by the pocket's contents as each left:
     * 0 in 95%, 3 in 88%, 5 in 85%, 8 in 60%. A 35 point spread -- twice the next strongest
     * feature and five times either of the two this model was built on.
     *
     * tools/landcal.ts could not find it: it fires ten shots into an empty CELL and stops,
     * so every calibration sample ever taken was of an empty pocket and the fitted curve
     * came out flat. With this term the same fit spans 0.84 to 1.00 and the score finally
     * separates a good shot from a bad one (74% low vs 89% high, against -3 points before).
     */
    private static final double[] FILL_N = { 0, 3, 5, 8 };
    private static final double[] FILL_P = { 0.95, 0.88, 0.85, 0.60 };

    public static double fillFactor(double fill) {
        if (fill <= FILL_N[0]) return FILL_P[0];
        if (fill >= FILL_N[FILL_N.length - 1]) return FILL_P[FILL_P.length - 1];
        for (int i = 1; i < FILL_N.length; i++) {
            if (fill <= FILL_N[i]) {
                double t = (fill - FILL_N[i - 1]) / (FILL_N[i] - FILL_N[i - 1]);
                return FILL_P[i - 1] + t * (FILL_P[i] - FILL_P[i - 1]);
            }
        }
        return FILL_P[FILL_P.length - 1];
    }

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
     * @param openAngleDeg how far off the mouth's opening the shot comes in: 0 is square on,
     *                     and the mouth's usable width falls away as its cosine
     * @return calibrated probability in [0, 1], or -1 when the table row carries no model
     */
    /** An empty CELL. Kept so existing callers and the self-check read unchanged. */
    public static double pLand(ShotTable table, double rangeIn, double exitSpeed,
                               double rangeM, double turretErrDeg, double yawScatterDeg,
                               double openAngleDeg) {
        return pLand(table, rangeIn, exitSpeed, rangeM, turretErrDeg, yawScatterDeg, openAngleDeg, 0);
    }

    public static double pLand(ShotTable table, double rangeIn, double exitSpeed,
                               double rangeM, double turretErrDeg, double yawScatterDeg,
                               double openAngleDeg, double cellFill) {
        double lo = table.lerpAt(ShotTableData.SPEED_LO, rangeIn);
        double hi = table.lerpAt(ShotTableData.SPEED_HI, rangeIn);
        double sigma = table.lerpAt(ShotTableData.SIGMA_SPEED, rangeIn);
        double pStay = table.lerpAt(ShotTableData.P_STAY, rangeIn);
        // THE MOUTH IS A SLOT, AND OFF TO ONE SIDE IT IS A NARROWER ONE.
        //
        // halfLat is solved head-on, because the shot table is. What matters to a shot is the
        // mouth's extent PERPENDICULAR TO THE SHOT LINE, and that shrinks by cos(off-axis) --
        // at 60 deg round the side the target is half as wide as the table thinks, and the
        // robot was scoring its chances against the full width from everywhere on the field.
        // tools/shotzone.ts has always modelled this; the robot never did, which is why the
        // painted zone and the robot's own gate could disagree about the same spot.
        //
        // ponytail: the same geometry makes the slot DEEPER along the shot by 1/cos, which
        // widens the speed band and is the forgiving half of the trade. Left out because it
        // flatters the shot and this factor is the one that refuses bad ones; add it by
        // scaling speedLo/speedHi about their centre if the zone map and the gate ever need
        // to agree to better than a few percent.
        // THE MOUTH'S EXTENT PERPENDICULAR TO THE SHOT. The pocket's depth was subtracted
        // here for one commit (84525e6) -- width*cos(beta) - depth*sin(beta) -- on the
        // argument that a slot seen off its normal has its own depth cutting across the
        // opening. That is the clear straight line THROUGH a slot, which is what a ball
        // would need if it had to reach the back wall untouched, and a ball does not: it
        // crosses the mouth and stays in the pocket, and the depth is behind the mouth.
        // MEASURED, tools/lostzone.ts: the 20 shot-zone squares the term deleted land 156
        // of 160 with the gate forced open, 98%, against 89% from the ones it kept, and the
        // worst sits 61 deg off -- past the 55 deg where the term says there is no hole.
        double beta = Math.toRadians(openAngleDeg);
        double halfLat = Math.max(0,
                table.lerpAt(ShotTableData.HALF_LAT_M, rangeIn) * Math.cos(beta));
        if (!(hi > lo) || !(sigma > 0)) return -1;

        double speed = pThread(lo, hi, exitSpeed, sigma);
        // Bearing error at the target is range * tan(error), and the launch adds its own
        // scatter, so the arrival across the shot line is normal about where it is pointing.
        // NO OPENING IS NO CHANCE, not a certainty. This fell through to 1 -- a mouth with
        // zero usable width scored as a PERFECT aim -- which was unreachable while the width
        // was cos(beta) alone, because a cosine only vanishes at 90 deg and nothing shoots
        // from there. Adding the depth term made it reachable at 55, where the slot really
        // does close, and the fallback then scored every impossible shot as a sure thing.
        // BuiltinTeleOp never had this: pThread(-0, 0, ..) integrates a zero-width band to 0.
        double aim = halfLat > 0
                ? pThread(-halfLat, halfLat,
                          rangeM * Math.tan(Math.toRadians(turretErrDeg)),
                          rangeM * Math.tan(Math.toRadians(yawScatterDeg)))
                : 0;
        return calibrate(speed * aim * pStay * fillFactor(cellFill));
    }
}
