package org.firstinspires.ftc.teamcode.util;

/** Conversions. FTC code speaks inches, degrees, ticks and RPM; keep it in one place. */
public final class Units {
    private Units() { }

    public static final double IN_PER_M = 39.3700787;

    public static double ticksToRevs(double ticks, double ticksPerRev) { return ticks / ticksPerRev; }
    public static double revsToTicks(double revs, double ticksPerRev) { return revs * ticksPerRev; }
    public static double ticksPerSecToRpm(double tps, double ticksPerRev) { return tps * 60.0 / ticksPerRev; }
    public static double rpmToTicksPerSec(double rpm, double ticksPerRev) { return rpm * ticksPerRev / 60.0; }

    /** Wrap to (-180, 180]. */
    public static double wrapDeg(double deg) {
        double d = deg;
        while (d > 180) d -= 360;
        while (d <= -180) d += 360;
        return d;
    }

    public static double clamp(double v, double lo, double hi) {
        return v < lo ? lo : (v > hi ? hi : v);
    }
}
