package org.firstinspires.ftc.teamcode.control;

import org.firstinspires.ftc.teamcode.util.Units;

/**
 * Voltage-compensated flywheel control plus the readiness gate.
 *
 *   power = (kS + kV * targetRpm) * (12 / V) + kP * (targetRpm - rpm)
 *
 * isReady() only goes true after the wheel has been inside the tolerance band for
 * readySteps consecutive loops, and fired() resets it so the next feed waits for
 * recovery. This is the piece that keeps a shooter honest, and it ships to the hub.
 */
public class FlywheelGate {
    private final double kS, kV, kP, tolRpm, minRpmFrac;
    private final int readySteps;
    /**
     * Loops of moving average over the measured RPM before it is compared with anything.
     *
     * getVelocity() returns whole encoder counts over the hub's own 20 ms window, and a
     * flywheel direct-driven on a 28-tick encoder turns about 26 counts in that window at
     * 2800 RPM -- so ONE COUNT is roughly 107 RPM, which is 15 in of range. Comparing a
     * single reading against a 30 RPM tolerance compares against noise. Averaging costs lag
     * and buys resolution; tools/trimsweep.ts measures the trade against balls in the goal.
     */
    private final int filterFrames;
    private final double[] history;
    private int filled = 0;
    private int head = 0;

    private double targetRpm = 0;
    private int inBand = 0;
    private double lastRpm = 0;

    public FlywheelGate(double kS, double kV, double kP, double tolRpm, double minRpmFrac, int readySteps) {
        this(kS, kV, kP, tolRpm, minRpmFrac, readySteps, 1);
    }

    public FlywheelGate(double kS, double kV, double kP, double tolRpm, double minRpmFrac,
                        int readySteps, int filterFrames) {
        this.kS = kS; this.kV = kV; this.kP = kP;
        this.tolRpm = tolRpm; this.minRpmFrac = minRpmFrac;
        this.readySteps = readySteps;
        this.filterFrames = Math.max(1, filterFrames);
        this.history = new double[this.filterFrames];
    }

    private double filter(double rpm) {
        history[head] = rpm;
        head = (head + 1) % filterFrames;
        if (filled < filterFrames) filled++;
        double sum = 0;
        for (int i = 0; i < filled; i++) sum += history[i];
        return sum / filled;
    }

    /**
     * A live target is a continuously varying double -- range jitter and the motion lead
     * move it by a fraction of an RPM every loop. Resetting readiness on any change at all
     * means it is reset every loop and the gate never opens. Only a change big enough to
     * matter against the tolerance counts as a new target.
     */
    public void setTargetRpm(double rpm) {
        if (Math.abs(rpm - targetRpm) > tolRpm * 0.5) inBand = 0;
        targetRpm = rpm;
    }

    public double getTargetRpm() { return targetRpm; }
    public double getRpm() { return lastRpm; }

    /** Returns the open-loop power to command. Call once per loop. */
    public double update(double rpmRaw, double volts) {
        double rpmMeasured = filter(rpmRaw);
        lastRpm = rpmMeasured;
        if (targetRpm <= 0) {
            inBand = 0;
            filled = 0;
            head = 0;
            return 0;
        }
        boolean ok = Math.abs(rpmMeasured - targetRpm) < tolRpm && rpmMeasured > targetRpm * minRpmFrac;
        // LEAKY, NOT A HARD RESET. Mirrors BuiltinTeleOp. One loop outside the window used to
        // throw the whole settle away and start from zero, so a single dipped reading cost
        // three more loops -- and while the robot is moving the target rpm is moving too, so
        // it dips often. MEASURED by tools/zonerun.ts, patrolling at 1.05 m/s, 6 runs of 40 s:
        // "settling" went from 24.4% of the drive to 1.8%, the land rate from 70% to 75%, and
        // the time per ball IN from 1.64 s to 1.37 s. A real loss of readiness still walks the
        // counter to zero in three loops, which is what the settle was for.
        inBand = ok ? inBand + 1 : Math.max(0, inBand - 1);

        double v = volts < 6 ? 12 : volts;
        double ff = (kS + kV * targetRpm) * (12.0 / v);
        return Units.clamp(ff + kP * (targetRpm - rpmMeasured), -1, 1);
    }

    public boolean isReady() { return targetRpm > 0 && inBand >= readySteps; }

    /** Call the moment a ball is fed: readiness must be re-earned. */
    public void fired() { inBand = 0; }

    /** Rough seconds until ready, for feeding predictively. */
    public double predictedReadyIn(double rpmPerSecond) {
        if (isReady()) return 0;
        double gap = targetRpm - lastRpm;
        if (gap <= 0 || rpmPerSecond <= 0) return 0.05;
        return gap / rpmPerSecond;
    }
}
