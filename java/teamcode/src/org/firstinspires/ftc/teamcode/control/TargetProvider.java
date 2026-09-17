package org.firstinspires.ftc.teamcode.control;

/**
 * Where to aim. In the sim this comes from the world's ground truth; on the hub it will
 * be a Limelight or an AprilTag pose plus the known HIVE position.
 */
public interface TargetProvider {
    /** Bearing to the up CELL relative to the robot's heading, degrees, CCW positive. */
    double getAzimuthDeg();
    /** Ground range from the robot's centre to the up CELL mouth, inches. */
    double getRangeIn();
    /** True while the HIVE is mid-tip: do not shoot at a moving goal (G417). */
    boolean isTipping();
    boolean isValid();
}
