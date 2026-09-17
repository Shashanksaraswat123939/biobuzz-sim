package org.firstinspires.ftc.robotcore.external.navigation;

public class YawPitchRollAngles {
    private final double yawDeg, pitchDeg, rollDeg;
    private final long acquisitionTime;

    public YawPitchRollAngles(AngleUnit unit, double yaw, double pitch, double roll, long acquisitionTime) {
        this.yawDeg = unit.toDegrees(yaw);
        this.pitchDeg = unit.toDegrees(pitch);
        this.rollDeg = unit.toDegrees(roll);
        this.acquisitionTime = acquisitionTime;
    }
    public double getYaw(AngleUnit unit) { return unit.fromDegrees(yawDeg); }
    public double getPitch(AngleUnit unit) { return unit.fromDegrees(pitchDeg); }
    public double getRoll(AngleUnit unit) { return unit.fromDegrees(rollDeg); }
    public double getYaw() { return yawDeg; }
    public double getPitch() { return pitchDeg; }
    public double getRoll() { return rollDeg; }
    public long getAcquisitionTime() { return acquisitionTime; }
}
