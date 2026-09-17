package com.qualcomm.robotcore.hardware;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public interface IMU {
    class Parameters {
        public final RevHubOrientationOnRobot imuOrientationOnRobot;
        public Parameters(RevHubOrientationOnRobot orientation) { this.imuOrientationOnRobot = orientation; }
    }
    boolean initialize(Parameters parameters);
    void resetYaw();
    YawPitchRollAngles getRobotYawPitchRollAngles();
    AngularVelocity getRobotAngularVelocity(AngleUnit unit);
}
