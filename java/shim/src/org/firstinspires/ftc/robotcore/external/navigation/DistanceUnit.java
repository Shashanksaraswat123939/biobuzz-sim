package org.firstinspires.ftc.robotcore.external.navigation;

public enum DistanceUnit {
    METER(1.0), CM(0.01), MM(0.001), INCH(0.0254);
    private final double metres;
    DistanceUnit(double m) { this.metres = m; }
    public double fromMeters(double m) { return m / metres; }
    public double fromInches(double in) { return (in * 0.0254) / metres; }
    public double toInches(double v) { return (v * metres) / 0.0254; }
    public double toMeters(double v) { return v * metres; }
}
