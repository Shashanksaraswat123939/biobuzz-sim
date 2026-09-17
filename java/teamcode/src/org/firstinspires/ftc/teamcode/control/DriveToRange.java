package org.firstinspires.ftc.teamcode.control;

import org.firstinspires.ftc.teamcode.util.Units;

/**
 * Nudge the robot along the line to the goal until the range is one the shot table can
 * actually solve. Outputs a forward request in [-1, 1]; the caller mixes it with the
 * driver's.
 */
public class DriveToRange {
    private final ShotTable table;
    private final double tolIn, kP;

    public DriveToRange(ShotTable table, double toleranceIn, double kP) {
        this.table = table;
        this.tolIn = toleranceIn;
        this.kP = kP;
    }

    /**
     * @param rangeIn    current range to the up CELL
     * @param bearingDeg bearing to it, relative to the robot's heading
     * @return forward power; positive drives toward the goal
     */
    public double update(double rangeIn, double bearingDeg) {
        double want = table.bestRange();
        double err = rangeIn - want;

        // The deadband is for fine trim INSIDE the table's band only. Outside it the robot
        // must keep moving: 24 in is within 6 in of a 30 in floor, but no hood angle and no
        // RPM will score from there, and a robot that stops there stops for good.
        if (table.usable(rangeIn) && Math.abs(err) < tolIn) return 0;

        // Only close range along the bearing; if the goal is off to the side, turning is the
        // driver's job (or the turret's).
        double along = Math.cos(Math.toRadians(Units.wrapDeg(bearingDeg)));
        double drive = kP * err * along;

        // Below the table's floor `err` alone would be a feeble nudge, so guarantee enough
        // power to actually back the robot off.
        if (!table.usable(rangeIn) && Math.abs(drive) < 0.25) {
            drive = Math.signum(err == 0 ? -1 : err) * 0.25 * (along >= 0 ? 1 : -1);
        }
        return Units.clamp(drive, -0.6, 0.6);
    }

    /** True once the shot table has a usable answer for this range. */
    public boolean inBand(double rangeIn) {
        return table.usable(rangeIn);
    }
}
