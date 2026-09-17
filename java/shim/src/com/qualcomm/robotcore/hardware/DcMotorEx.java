package com.qualcomm.robotcore.hardware;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public interface DcMotorEx extends DcMotor {
    void setVelocity(double ticksPerSecond);
    void setVelocity(double angularRate, AngleUnit unit);
    double getVelocity();
    double getVelocity(AngleUnit unit);
    void setPIDFCoefficients(DcMotor.RunMode mode, PIDFCoefficients pidf);
    PIDFCoefficients getPIDFCoefficients(DcMotor.RunMode mode);
    double getCurrent(CurrentUnit unit);
    void setMotorEnable();
    void setMotorDisable();
    boolean isMotorEnabled();
}
