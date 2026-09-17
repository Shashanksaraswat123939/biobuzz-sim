package org.firstinspires.ftc.robotcore.external.navigation;

public enum CurrentUnit {
    AMPS(1.0), MILLIAMPS(0.001);
    private final double amps;
    CurrentUnit(double a) { this.amps = a; }
    public double fromAmps(double a) { return a / amps; }
}
