package com.qualcomm.robotcore.util;

/** SDK shim. In the sim the clock is the world's, not the wall's. */
public class ElapsedTime {
    public enum Resolution { SECONDS, MILLISECONDS }

    private static volatile double simNow = 0.0;
    private static volatile boolean useSim = false;

    /** Called by the runner once per loop. Not part of the real SDK. */
    public static void setSimTime(double seconds) { simNow = seconds; useSim = true; }
    private static double now() { return useSim ? simNow : System.nanoTime() / 1.0e9; }

    private double start;
    public ElapsedTime() { start = now(); }
    public ElapsedTime(long startNanos) { start = startNanos / 1.0e9; }
    public void reset() { start = now(); }
    public double seconds() { return now() - start; }
    public double milliseconds() { return seconds() * 1000.0; }
    public double time() { return seconds(); }
    public long startTime() { return (long) (start * 1e9); }
}
