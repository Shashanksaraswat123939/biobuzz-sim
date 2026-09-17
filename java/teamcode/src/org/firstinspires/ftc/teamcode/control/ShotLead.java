package org.firstinspires.ftc.teamcode.control;

import org.firstinspires.ftc.teamcode.util.Units;

/**
 * Aim that accounts for the robot's own motion.
 *
 * A ball leaves the shooter with v_exit*dir + v_robot, so the shot is solved for the
 * velocity the BALL must have in the field frame -- which is the shot table's answer, as a
 * vector -- with the robot's own velocity subtracted from it.
 *
 *     ground horizontal   S*cos(el) along the bearing        vertical   S*sin(el)
 *
 * THE VERTICAL IS PART OF THE ANSWER. The first version of this solved the horizontal
 * triangle only and left the hood where the table put it, so the ball went out with a
 * vertical of mag*tan(el) instead of S*sin(el): the ground track was exact and the hang time
 * was not. Closing at 0.4 m/s from 40 in that takes the exit speed from 5.28 to 4.09 m/s at
 * a fixed 70 deg hood, the vertical from 4.97 to 3.85, and the ball NEVER REACHES the CELL
 * mouth's 1.46 m. Not a miss -- a shot that cannot arrive. Retreating sailed over it the
 * same way, and the simulator's own harness read the whole thing as "the flywheel cannot
 * track the lead while accelerating".
 *
 * Three components, three unknowns -- azimuth, ELEVATION and speed -- so solve all three:
 *
 *     el = atan2(vert, mag)        speed = hypot(mag, vert)
 *
 * The second prize is that this all but removes the flywheel from the problem. Over +-0.8
 * m/s of closing speed at 40 in the old lead swung the target between 1283 and 3388 RPM,
 * against a wheel that slews 1102 RPM/s; this asks for 2242 to 2477, nine times less, and
 * hands the rest to a hood servo that is COMMANDED rather than measured.
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
    /** Hood angle the shot now needs, degrees, clamped to the hood's travel. */
    public double elevationDeg;
    /** How far the correction moved the aim, degrees. Telemetry only. */
    public double correctionDeg;

    /**
     * @param bearingDeg   geometric bearing to the target, relative to the robot's heading
     * @param tableSpeed   exit speed the shot table asks for, m/s
     * @param tableElevDeg hood angle the shot table asks for, degrees
     * @param vxField      robot velocity along FTC +X, m/s
     * @param vyField      robot velocity along FTC +Y, m/s
     * @param headingDeg   robot heading, degrees CCW from FTC +X
     * @param hoodMinDeg   hood travel, low end
     * @param hoodMaxDeg   hood travel, high end
     */
    public ShotLead solve(double bearingDeg, double tableSpeed, double tableElevDeg,
                          double vxField, double vyField, double headingDeg,
                          double hoodMinDeg, double hoodMaxDeg) {
        azimuthDeg = bearingDeg;
        speed = tableSpeed;
        elevationDeg = tableElevDeg;
        correctionDeg = 0;

        double horiz = tableSpeed * Math.cos(Math.toRadians(tableElevDeg));
        double vert = tableSpeed * Math.sin(Math.toRadians(tableElevDeg));
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
        // Clamped, with the speed left as the magnitude of the vector we wanted rather than
        // re-solved for the clamped angle: past the hood's travel no launch matches the
        // table's vector at all, and this degrades smoothly instead of dividing by
        // cos(85 deg). It bites only when charging the goal at most of top speed from close
        // in.
        elevationDeg = Units.clamp(Math.toDegrees(Math.atan2(vert, mag)), hoodMinDeg, hoodMaxDeg);
        speed = Math.hypot(mag, vert);
        correctionDeg = Units.wrapDeg(azimuthDeg - bearingDeg);
        return this;
    }

    /**
     * Velocity of the ball's EXIT POINT in the FTC field frame, m/s: v_cg + omega x r.
     *
     * solve() subtracts the velocity the ball inherits from the robot. The velocity it
     * inherits is the MUZZLE's, and on a yawing robot that is not the chassis's: the muzzle
     * sits muzzleOffsetM out along the shot line from the turret axis, so it swings at
     * omega*r of its own. At 0.12 m and 90 deg/s that is 0.19 m/s, which puts a 60 in shot
     * 4.7 in sideways -- wider than the clearance to the lip, and Localizer.getOmegaDps()
     * had been implemented on both sides and read by nobody.
     *
     * @param turretActualDeg the turret's MEASURED angle, not its target: the muzzle is where
     *                        the turret IS, and the axis is acceleration limited.
     * @param muzzleOffsetM   signed along the shot line: + ahead of the turret axis, - behind.
     * @return {vx, vy} in the FTC field frame.
     */
    public static double[] muzzleVelocity(double vxField, double vyField, double omegaDps,
                                          double headingDeg, double turretActualDeg,
                                          double muzzleOffsetM) {
        double w = Math.toRadians(omegaDps);
        double shot = Math.toRadians(headingDeg + turretActualDeg);
        double rx = muzzleOffsetM * Math.cos(shot);
        double ry = muzzleOffsetM * Math.sin(shot);
        return new double[] { vxField - w * ry, vyField + w * rx };
    }
}
