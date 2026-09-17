package org.firstinspires.ftc.teamcode.control;

/**
 * Mecanum inverse kinematics (PLAN.md Appendix C).
 *   fl = vx - vy - w,  fr = vx + vy + w,  bl = vx + vy - w,  br = vx - vy + w
 * with vx forward, vy left, w counter-clockwise, all normalised to [-1, 1].
 */
public class MecanumKinematics {
    public double fl, fr, bl, br;

    public void compute(double vx, double vy, double omega) {
        fl = vx - vy - omega;
        fr = vx + vy + omega;
        bl = vx + vy - omega;
        br = vx - vy + omega;
        double max = Math.max(Math.max(Math.abs(fl), Math.abs(fr)), Math.max(Math.abs(bl), Math.abs(br)));
        if (max > 1.0) {
            fl /= max; fr /= max; bl /= max; br /= max;
        }
    }

    /** Rotate a field-frame request into the robot frame before computing. */
    public void computeFieldCentric(double fieldForward, double fieldLeft, double omega, double headingDeg) {
        double h = Math.toRadians(headingDeg);
        double c = Math.cos(h);
        double s = Math.sin(h);
        compute(fieldForward * c + fieldLeft * s, -fieldForward * s + fieldLeft * c, omega);
    }
}
