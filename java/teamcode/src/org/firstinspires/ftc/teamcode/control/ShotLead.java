package org.firstinspires.ftc.teamcode.control;

import org.firstinspires.ftc.teamcode.util.Units;

/**
 * Aim that accounts for the robot's own motion.
 *
 * A ball leaves the shooter with v_exit*dir + v_robot, so a robot moving across the shot
 * throws it off line by exactly its own cross-track speed. Solving the triangle is both
 * exact and shorter than approximating it: the horizontal velocity the ball must leave
 * with is (the speed the shot table wants, along the bearing) minus (the robot's own
 * velocity), and the turret points wherever that vector points.
 *
 * Only velocity is compensated, not acceleration. Over a one-second flight the a*t^2 term
 * is small next to the one or two degrees of launch scatter, and a lead that differentiates
 * a noisy velocity estimate is worse than no lead at all.
 */
public class ShotLead {
    /** Lead-corrected turret bearing, relative to the robot's heading, degrees. */
    public double azimuthDeg;
    /** Exit speed the shot now needs, m/s. */
    public double speed;
    /** How far the correction moved the aim, degrees. Telemetry only. */
    public double correctionDeg;

    /**
     * @param bearingDeg   geometric bearing to the target, relative to the robot's heading
     * @param tableSpeed   exit speed the shot table asks for, m/s
     * @param elevationDeg hood angle
     * @param vxField      robot velocity along FTC +X, m/s
     * @param vyField      robot velocity along FTC +Y, m/s
     * @param headingDeg   robot heading, degrees CCW from FTC +X
     */
    public ShotLead solve(double bearingDeg, double tableSpeed, double elevationDeg,
                          double vxField, double vyField, double headingDeg) {
        azimuthDeg = bearingDeg;
        speed = tableSpeed;
        correctionDeg = 0;

        double horiz = tableSpeed * Math.cos(Math.toRadians(elevationDeg));
        if (horiz < 1e-3) return this;

        double bearingField = Math.toRadians(headingDeg + bearingDeg);
        double wantX = horiz * Math.cos(bearingField) - vxField;
        double wantY = horiz * Math.sin(bearingField) - vyField;
        double mag = Math.hypot(wantX, wantY);
        if (mag < 1e-3) return this;

        // WRAP. atan2 gives (-180, 180] and the heading comes off it, so the raw result can
        // land outside a turret's travel even when the true bearing is well inside it: a
        // bearing of +90 came out as -270 and clamped to the -120 end stop.
        azimuthDeg = Units.wrapDeg(Math.toDegrees(Math.atan2(wantY, wantX)) - headingDeg);
        speed = mag / Math.cos(Math.toRadians(elevationDeg));
        correctionDeg = Units.wrapDeg(azimuthDeg - bearingDeg);
        return this;
    }
}
