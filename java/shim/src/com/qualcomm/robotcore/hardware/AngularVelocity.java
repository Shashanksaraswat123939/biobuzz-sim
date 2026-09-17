package com.qualcomm.robotcore.hardware;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class AngularVelocity {
    public final AngleUnit unit;
    public final float xRotationRate, yRotationRate, zRotationRate;
    public final long acquisitionTime;

    public AngularVelocity(AngleUnit unit, float x, float y, float z, long acquisitionTime) {
        this.unit = unit;
        this.xRotationRate = x; this.yRotationRate = y; this.zRotationRate = z;
        this.acquisitionTime = acquisitionTime;
    }
}
