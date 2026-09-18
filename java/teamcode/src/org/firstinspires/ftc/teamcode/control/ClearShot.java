package org.firstinspires.ftc.teamcode.control;

import org.firstinspires.ftc.teamcode.config.RobotConstants;

/**
 * Is the OTHER HIVE in the way?
 *
 * THE MIRROR of packages/core/src/robot/clearShot.ts.
 *
 * The two HIVEs sit 25.5 in apart along the pivot axis and each CELL is 20 in wide, so they
 * very nearly touch. A shot at your own CELL taken from the far side of theirs has to cross
 * their structure, and nothing in the shot solver ever knew that: it checks the near and far
 * lip of the TARGET mouth and integrates a ball through empty air. The physics knows -- both
 * rockers carry colliders -- so the ball really does hit, and the aim really does choose to
 * throw it there. Measured over the shot zone, 5% of the positions the table is happy with
 * are blocked.
 *
 * THE OPPONENT'S ROCKER STATE DOES NOT MATTER, which is the piece of luck that makes this
 * cheap on a hub. Both of their CELLs swing about their pivot at a fixed radius, so the
 * volume the structure can occupy is a DISC, and a disc is the same in both states. No need
 * to see their tag, track their tips, or take the worst of two cases.
 *
 * A GROUND-PLANE TEST, refusing any shot whose path crosses the footprint, rather than asking
 * whether the arc clears the top. Conservative and knowingly so: the table's apex is 59-61 in
 * and the obstacle's top runs 60 in at the edges of the disc to 66 at its centre, so a shot
 * over the shallow end would sometimes get through. Buying those back means carrying the
 * arc's height profile onto the hub -- a shot table with a second axis -- against a few
 * positions on one side of the field. Four baked numbers and a segment test instead.
 */
public final class ClearShot {
    private ClearShot() { }

    /**
     * Does the ground path from the robot to the mouth cross the opposing HIVE's footprint?
     *
     * A box: `radius` either side of their pivot along FTC X, `halfWidth` along FTC Y, tested
     * against the segment by slab clipping.
     */
    public static boolean blocked(double fromX, double fromY, double toX, double toY) {
        final double minX = RobotConstants.OBSTACLE_X_IN - RobotConstants.OBSTACLE_RADIUS_IN;
        final double maxX = RobotConstants.OBSTACLE_X_IN + RobotConstants.OBSTACLE_RADIUS_IN;
        final double minY = RobotConstants.OBSTACLE_Y_IN - RobotConstants.OBSTACLE_HALF_WIDTH_IN;
        final double maxY = RobotConstants.OBSTACLE_Y_IN + RobotConstants.OBSTACLE_HALF_WIDTH_IN;

        if (inside(fromX, fromY, minX, maxX, minY, maxY)) return true;
        if (inside(toX, toY, minX, maxX, minY, maxY)) return true;

        double dx = toX - fromX;
        double dy = toY - fromY;
        double[] t = { 0, 1 };                       // the segment, as a parameter range
        if (!slab(fromX, dx, minX, maxX, t)) return false;
        if (!slab(fromY, dy, minY, maxY, t)) return false;
        return t[1] >= t[0];
    }

    private static boolean inside(double x, double y, double minX, double maxX, double minY, double maxY) {
        return x >= minX && x <= maxX && y >= minY && y <= maxY;
    }

    /** Clip the parameter range to the interval this axis is inside the box. */
    private static boolean slab(double p, double d, double lo, double hi, double[] t) {
        if (Math.abs(d) < 1e-9) return p >= lo && p <= hi;   // parallel: in or out for good
        double a = (lo - p) / d;
        double b = (hi - p) / d;
        t[0] = Math.max(t[0], Math.min(a, b));
        t[1] = Math.min(t[1], Math.max(a, b));
        return t[1] >= t[0];
    }
}
