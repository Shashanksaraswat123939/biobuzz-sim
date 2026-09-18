package org.firstinspires.ftc.teamcode.control;

import org.firstinspires.ftc.teamcode.config.RobotConstants;
import org.firstinspires.ftc.teamcode.util.Units;

/**
 * Odometry drifts. The tag says where you are. This closes the loop.
 *
 * THE MIRROR of packages/core/src/robot/poseFuser.ts.
 *
 * A fiducial at a SURVEYED position is, first and last, a measurement of the observer. The
 * robot sees the tag at a bearing and a range; the tag's field position is a known constant;
 * so the robot's position follows by subtraction. That bounds an error which otherwise grows
 * with every inch driven and never comes back -- measured in the simulator, 3.6 to 25 in over
 * 120 s of driving, of which the tag removes about 62%.
 *
 * It wraps a raw {@link Localizer} and IS one, so everything downstream gets the corrected
 * pose without knowing this exists. On the hub the raw one is a Pinpoint or a pod stack.
 *
 * WHICH WAY THE INFORMATION FLOWS IS A CHOICE, AND YOU ONLY GET ONE. A single sighting is one
 * relative measurement between two unknowns -- where the robot is and where the HIVE is -- and
 * it cannot pin both. The field is the reference and the robot is what gets estimated, because
 * the field is the one that is built to a drawing.
 *
 * THREE THINGS HAD TO BE RIGHT, and each was wrong first in the TypeScript. Every one of them
 * looked like it worked and measurably made the robot WORSE, so they are worth stating:
 *
 *   1. HEADING HAS TO BE CORRECTED TOO. A bearing-and-range fix is measured along a line drawn
 *      at the heading the robot believes, so correcting position under a heading 4 deg out
 *      drives the pose to a place consistent with that 4 deg. Bearing and range are two
 *      constraints for three unknowns; the panel's YAW supplies the third. Leaving it out took
 *      0.8 in of error to 7.2.
 *
 *   2. THE FIX IS COMPARED AGAINST THE POSE THE ROBOT WAS IN WHEN IT LOOKED. A detection is
 *      75-108 ms old; a robot turning at 100 deg/s has moved 10 deg by then, and a driving
 *      robot turns in a correlated way, so that lands as a BIAS and not as noise.
 *
 *   3. THIS IS AN ESTIMATOR, NOT AN OFFSET. Adding a correction to whatever odometry reports
 *      leaves odometry INTEGRATING in its own uncorrected heading, so error accrues at the old
 *      rate and a constant offset cannot chase it. Carrying its own pose and integrating
 *      DELTAS is what makes a heading fix change the frame every later step is added in. That
 *      one took 39 in back down to 4.5.
 *
 * And a fourth the unit tests caught: a correction is a rigid transform of the whole trajectory
 * estimate, so the back-dating history has to move with it. Left stale, every fix is compared
 * against a pose that has seen none of the corrections before it, and the same correction is
 * applied over and over -- 60 identical sightings drove an 8 deg error to 16 deg, steadily
 * away from the truth.
 */
public class FusedLocalizer implements Localizer {
    private final Localizer raw;
    private final TagCamera cam;
    private final double gain;
    private final double headingGain;
    private final double rejectOver;

    private double x, y, heading;
    private boolean started = false;
    private double lastRawX, lastRawY, lastRawHeading;
    private double lastSampleT = -1e9;

    /** A short history of the FUSED pose, so a detection can be back-dated onto it. */
    private static final int HIST = 64;
    private final double[] hT = new double[HIST];
    private final double[] hX = new double[HIST];
    private final double[] hY = new double[HIST];
    private final double[] hH = new double[HIST];
    private int hCount = 0;
    private int hNext = 0;

    public int applied = 0;
    public int rejected = 0;
    public double lastCorrectionIn = 0;

    public FusedLocalizer(Localizer raw, TagCamera cam,
                          double gain, double headingGain, double rejectOver_in) {
        this.raw = raw;
        this.cam = cam;
        this.gain = gain;
        this.headingGain = headingGain;
        this.rejectOver = rejectOver_in;
    }

    /**
     * Call once per loop, before anything reads the pose. SAFE TO CALL TWICE.
     *
     * The aim needs the pose before robot.update() runs, and a tuning OpMode that never
     * touches the aim still needs somebody to step it -- so this gets called from two places
     * and must not double-integrate when both fire. The guard is that nothing has moved:
     * within one loop the hub's bulk cache returns the identical reading, and in lockstep the
     * sensor frame is the same object, so an unchanged raw pose means this already ran.
     */
    @Override
    public void update() {
        if (raw == null) return;
        raw.update();
        if (started
                && raw.getX() == lastRawX
                && raw.getY() == lastRawY
                && raw.getHeadingDeg() == lastRawHeading) {
            return;
        }
        propagate();
        if (cam != null && cam.hasFix() && cam.getSampleTimeS() > lastSampleT) {
            lastSampleT = cam.getSampleTimeS();
            observe();
        }
    }

    /**
     * Fold in this loop's odometry. DELTAS, not the absolute reading: the raw pose carries all
     * of its accumulated drift, and taking it whole would throw away every correction so far.
     */
    private void propagate() {
        if (!started) {
            x = raw.getX();
            y = raw.getY();
            heading = raw.getHeadingDeg();
            started = true;
        } else {
            double dx = raw.getX() - lastRawX;
            double dy = raw.getY() - lastRawY;
            double dHead = Units.wrapDeg(raw.getHeadingDeg() - lastRawHeading);
            // The step, rotated out of odometry's heading frame and into the corrected one.
            // This is where a heading fix earns its keep: every inch after it is integrated
            // the right way round.
            double rot = Math.toRadians(Units.wrapDeg(heading - lastRawHeading));
            double c = Math.cos(rot);
            double s = Math.sin(rot);
            x += dx * c - dy * s;
            y += dx * s + dy * c;
            heading = Units.wrapDeg(heading + dHead);
        }
        lastRawX = raw.getX();
        lastRawY = raw.getY();
        lastRawHeading = raw.getHeadingDeg();

        double t = cam == null ? 0 : cam.getTimeS();
        hT[hNext] = t;
        hX[hNext] = x;
        hY[hNext] = y;
        hH[hNext] = heading;
        hNext = (hNext + 1) % HIST;
        if (hCount < HIST) hCount++;
    }

    /** Index of the remembered pose nearest `t`, or -1 when nothing is remembered. */
    private int nearest(double t) {
        int best = -1;
        double bestGap = Double.MAX_VALUE;
        for (int i = 0; i < hCount; i++) {
            double gap = Math.abs(hT[i] - t);
            if (gap < bestGap) { bestGap = gap; best = i; }
        }
        return best;
    }

    /** Fold in one sighting of a tag whose field position is surveyed. */
    private void observe() {
        int i = nearest(cam.getSampleTimeS());
        double thenX = i < 0 ? x : hX[i];
        double thenY = i < 0 ? y : hY[i];
        double thenH = i < 0 ? heading : hH[i];

        int id = cam.getId();
        boolean isB = id == RobotConstants.TAG_ID_CELL_B;
        // The TAG's surveyed position: pivot, out to the CELL, then back off the rigid
        // tag -> mouth correction. Anchored on the prior, not on the robot's own estimate of
        // the pivot -- see the class comment on which way the information is allowed to flow.
        double tagX = RobotConstants.HIVE_ANCHOR_X_IN
                + (isB ? RobotConstants.MOUTH_FROM_ANCHOR_X_B_IN : RobotConstants.MOUTH_FROM_ANCHOR_X_A_IN)
                - (isB ? RobotConstants.TAG_MOUTH_DX_B_IN : RobotConstants.TAG_MOUTH_DX_A_IN);
        double tagY = RobotConstants.HIVE_ANCHOR_Y_IN;
        double facingX = isB ? RobotConstants.MOUTH_FACING_X_B : RobotConstants.MOUTH_FACING_X_A;

        // HEADING FIRST, because the position fix is measured along a line drawn at it.
        if (headingGain > 0 && facingX != 0) {
            double psi = facingX > 0 ? 0 : 180;              // the panel normal, as a field angle
            double phiEst = Math.toDegrees(Math.atan2(thenY - tagY, thenX - tagX));
            double a = psi + cam.getOpenDeg();               // yaw loses the left/right sign,
            double b = psi - cam.getOpenDeg();               // so the estimate picks a side
            double phi = Math.abs(Units.wrapDeg(a - phiEst)) <= Math.abs(Units.wrapDeg(b - phiEst)) ? a : b;
            double implied = Units.wrapDeg(phi + 180 - cam.getBearingDeg());
            double hg = Units.clamp(headingGain, 0, 1);
            double dH = Units.wrapDeg(implied - thenH) * hg;
            thenH = Units.wrapDeg(thenH + dH);
            heading = Units.wrapDeg(heading + dH);
            // Rebase the history: a correction is a rigid transform of the whole trajectory,
            // and the history is what the NEXT detection gets back-dated onto.
            for (int k = 0; k < hCount; k++) hH[k] = Units.wrapDeg(hH[k] + dH);
        }

        double bearing = Math.toRadians(thenH + cam.getBearingDeg());
        double dx = tagX - cam.getRangeIn() * Math.cos(bearing) - thenX;
        double dy = tagY - cam.getRangeIn() * Math.sin(bearing) - thenY;
        double miss = Math.hypot(dx, dy);
        lastCorrectionIn = 0;
        if (!(miss <= rejectOver)) {
            // Three feet of disagreement between two frames is a misread or the wrong tag, not
            // drift -- drift arrives an inch at a time.
            rejected++;
            return;
        }
        double g = Units.clamp(gain, 0, 1);
        x += dx * g;
        y += dy * g;
        for (int k = 0; k < hCount; k++) { hX[k] += dx * g; hY[k] += dy * g; }
        applied++;
        lastCorrectionIn = miss * g;
    }

    /** The fused pose at `t`, for anything else that has to back-date a measurement. */
    public double poseXAt(double t) { int i = nearest(t); return i < 0 ? x : hX[i]; }
    public double poseYAt(double t) { int i = nearest(t); return i < 0 ? y : hY[i]; }
    public double poseHeadingAt(double t) { int i = nearest(t); return i < 0 ? heading : hH[i]; }

    @Override public double getX() { return x; }
    @Override public double getY() { return y; }
    @Override public double getHeadingDeg() { return heading; }

    /**
     * Velocity is NOT corrected, and passes straight through.
     *
     * It is differentiated locally, so it never inherited the accumulated position error in
     * the first place -- only the scale error and its own noise. That is why the motion lead
     * survives a drifted pose when the aim does not, and there is nothing here for a tag fix
     * to improve.
     */
    @Override public double getVx() { return raw == null ? 0 : raw.getVx(); }
    @Override public double getVy() { return raw == null ? 0 : raw.getVy(); }
    @Override public double getOmegaDps() { return raw == null ? 0 : raw.getOmegaDps(); }
}
