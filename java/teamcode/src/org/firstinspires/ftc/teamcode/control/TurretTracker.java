package org.firstinspires.ftc.teamcode.control;

import org.firstinspires.ftc.teamcode.util.Units;

/** Turns a bearing into a turret command, clamped to travel and rate limited. */
public class TurretTracker {
    private final double minDeg, maxDeg, maxRateDps;
    private double commandDeg = 0;

    public TurretTracker(double minDeg, double maxDeg, double maxRateDps) {
        this.minDeg = minDeg; this.maxDeg = maxDeg; this.maxRateDps = maxRateDps;
    }

    /** bearingDeg is relative to the robot's heading. dt in seconds. */
    public double update(double bearingDeg, double dt) {
        double want = Units.clamp(Units.wrapDeg(bearingDeg), minDeg, maxDeg);
        double step = maxRateDps * Math.max(dt, 1e-3);
        commandDeg += Units.clamp(want - commandDeg, -step, step);
        return commandDeg;
    }

    public double getCommandDeg() { return commandDeg; }
    public void reset(double deg) { commandDeg = Units.clamp(deg, minDeg, maxDeg); }

    /** True when the turret can actually point where it is being asked to. */
    public boolean canReach(double bearingDeg) {
        double b = Units.wrapDeg(bearingDeg);
        return b >= minDeg && b <= maxDeg;
    }

    public boolean onTarget(double bearingDeg, double toleranceDeg) {
        return Math.abs(Units.wrapDeg(bearingDeg) - commandDeg) < toleranceDeg;
    }
}
