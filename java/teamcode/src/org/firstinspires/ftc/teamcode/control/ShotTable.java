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

    /** The range with the widest tolerance: where DriveToRange wants to be. */
    public double bestRange() {
        double best = 0;
        double bestR = range.length == 0 ? 0 : range[0];
        for (int i = 0; i < range.length; i++) {
            if (margin[i] > best) { best = margin[i]; bestR = range[i]; }
        }
        return bestR;
    }

    /** Is this range worth shooting from at all? */
    public boolean usable(double rangeIn) {
        return !isEmpty() && rangeIn >= minRange() && rangeIn <= maxRange() && marginFor(rangeIn) >= 0.02;
    }
}
