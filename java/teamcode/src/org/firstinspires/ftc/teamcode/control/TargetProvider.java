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

    /**
     * How far off the up CELL's opening we are, degrees: 0 is square onto the mouth, 90 is
     * edge-on to its plane, 180 is standing behind the goal.
     *
     * A TIP swaps which CELL is up and the new one opens the OTHER WAY, so a robot that was
     * lined up is suddenly behind the target. Nothing used to know that: in the simulator the
     * stopped control case tipped the HIVE and then spent the rest of its run putting balls
     * into the back of the pocket, all counted as misses. On a real robot this is the
     * AprilTag going out of view; until there is a vision stack it comes from the world's
     * `game` block like the bearing and the range do.
     */
    double getOpenAngleDeg();

    boolean isValid();
}
