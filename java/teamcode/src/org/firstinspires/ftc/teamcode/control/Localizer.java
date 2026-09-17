package org.firstinspires.ftc.teamcode.control;

/**
 * Where the robot is, in FTC field coordinates (inches, degrees CCW from +X).
 *
 * In the sim this is satisfied by ground truth handed over the bridge; on the hub it is
 * a Pinpoint / odometry / Limelight implementation. Only the constructor changes at port
 * time, which is the whole point of the interface existing.
 */
public interface Localizer {
    double getX();
    double getY();
    double getHeadingDeg();
    double getVx();
    double getVy();
    double getOmegaDps();
    void update();
}
