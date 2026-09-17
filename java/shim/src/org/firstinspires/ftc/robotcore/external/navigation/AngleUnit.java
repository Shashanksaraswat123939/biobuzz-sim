package org.firstinspires.ftc.robotcore.external.navigation;

public enum AngleUnit {
    DEGREES, RADIANS;
    public double fromDegrees(double deg) { return this == DEGREES ? deg : Math.toRadians(deg); }
    public double fromRadians(double rad) { return this == RADIANS ? rad : Math.toDegrees(rad); }
    public double toDegrees(double v) { return this == DEGREES ? v : Math.toDegrees(v); }
    public double toRadians(double v) { return this == RADIANS ? v : Math.toRadians(v); }
    public double normalize(double v) {
        double limit = this == DEGREES ? 180.0 : Math.PI;
        double full = 2 * limit;
        double r = v;
        while (r > limit) r -= full;
        while (r <= -limit) r += full;
        return r;
    }
}
