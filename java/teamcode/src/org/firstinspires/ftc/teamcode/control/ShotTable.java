package org.firstinspires.ftc.teamcode.control;

/**
 * range -> (hood position, flywheel RPM, margin), generated offline by
 * tools/shottable.ts and baked in by tools/genconstants.mjs.
 *
 * The solver runs on a laptop; the hub only interpolates. That is what makes the port
 * cheap (PLAN.md section 10.3).
 */
public class ShotTable {
    private final double[] range;
    private final double[] hood;
    private final double[] rpm;
    private final double[] margin;

    public ShotTable(double[] range, double[] hood, double[] rpm, double[] margin) {
        this.range = range; this.hood = hood; this.rpm = rpm; this.margin = margin;
    }

    public boolean isEmpty() { return range.length == 0; }
    public double minRange() { return range.length == 0 ? 0 : range[0]; }
    public double maxRange() { return range.length == 0 ? 0 : range[range.length - 1]; }

    private int index(double r) {
        if (r <= range[0]) return 0;
        for (int i = 1; i < range.length; i++) if (r <= range[i]) return i;
        return range.length - 1;
    }

    private double lerp(double[] a, double r) {
        if (range.length == 0) return 0;
        if (r <= range[0]) return a[0];
        if (r >= range[range.length - 1]) return a[range.length - 1];
        int i = index(r);
        double t = (r - range[i - 1]) / (range[i] - range[i - 1]);
        return a[i - 1] + t * (a[i] - a[i - 1]);
    }

    /**
     * Interpolate any column that shares this table's ranges -- the land-probability inputs
     * live in ShotTableData as parallel arrays rather than being copied into every row.
     */
    public double lerpAt(double[] column, double rangeIn) {
        return column.length == range.length ? lerp(column, rangeIn) : 0;
    }

    public double hoodFor(double rangeIn) { return lerp(hood, rangeIn); }
    public double rpmFor(double rangeIn) { return lerp(rpm, rangeIn); }
    public double marginFor(double rangeIn) { return lerp(margin, rangeIn); }

    /**
     * The range with the widest tolerance: where DriveToRange wants to be.
     *
     * THE MIDDLE OF THE GOOD BAND, not the single best row, and that is not a refinement --
     * it is the difference between the autonomous scoring and not.
     *
     * Taking the argmax returned 30.0 in for the shipped table, which is also the table's
     * FIRST row. DriveToRange then parked the robot exactly on the table's floor, where an
     * inch of overshoot or a noisy range reading puts it at 28 in and `usable()` is false. So
     * AutoOneTip's "back off until the table has an answer" loop could never satisfy its exit
     * condition, ran out its twelve seconds, and fell through to PARK having fired nothing.
     * That is the unexplained "Auto One Tip does not currently score its shots" in the README.
     *
     * Every row within 95% of the best margin is good enough to shoot from -- mirroring
     * ShotTable.bestBand() in the TypeScript -- so aim at the centre of that band and keep the
     * edges as tolerance instead of spending them on arriving.
     */
    public double bestRange() {
        if (range.length == 0) return 0;
        double best = 0;
        for (int i = 0; i < range.length; i++) if (margin[i] > best) best = margin[i];
        int lo = -1;
        int hi = -1;
        for (int i = 0; i < range.length; i++) {
            if (margin[i] >= best * 0.95) {
                if (lo < 0) lo = i;
                hi = i;
            }
        }
        if (lo < 0) return range[0];
        return (range[lo] + range[hi]) / 2;
    }

    /** Is this range worth shooting from at all? */
    public boolean usable(double rangeIn) {
        return !isEmpty() && rangeIn >= minRange() && rangeIn <= maxRange() && marginFor(rangeIn) >= 0.02;
    }
}
