package com.qualcomm.robotcore.hardware;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public interface DistanceSensor {
    double getDistance(DistanceUnit unit);
}
