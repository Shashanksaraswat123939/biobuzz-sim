package org.firstinspires.ftc.teamcode.control;

/**
 * The most recent AprilTag detection, as the pipeline hands it over.
 *
 * Same shape as {@link Localizer} and {@link BallCounter}: the sim supplies one, and on the
 * hub this is a thin wrapper over `AprilTagProcessor.getDetections()` -- `detection.id`,
 * `ftcPose.bearing`, `ftcPose.range`, `ftcPose.yaw` and `frameAcquisitionNanoTime`. Every
 * field below is something that call already returns, which is the point: none of this needs
 * a vision stack that does not exist yet.
 *
 * A camera never reports "I cannot see it". It simply stops updating -- out of the lens, too
 * far, too far round the side, or the tag moving too fast to decode -- so the detection's AGE
 * is the only signal there is, and {@link TagTargetProvider} is what turns it into a
 * decision. That is why the timestamp is part of the interface and not an afterthought.
 */
public interface TagCamera {
    /** True once the camera has ever decoded the tag. False means there is nothing at all. */
    boolean hasFix();

    /** Which CELL's tag: 1 = A, 2 = B. It changes when the HIVE tips, and that is the tell. */
    int getId();

    /** Bearing to the tag relative to the ROBOT's heading, degrees CCW. */
    double getBearingDeg();

    double getRangeIn();

    /** Tag yaw: how far off square its face is, degrees. 0 is dead on, 90 is edge-on. */
    double getOpenDeg();

    /**
     * When the photons behind this detection left, on the same clock as {@link #getTimeS()}.
     * On the hub: `frameAcquisitionNanoTime * 1e-9`, NOT the time the loop read it.
     */
    double getSampleTimeS();

    /** Now, on that same clock. */
    double getTimeS();
}
