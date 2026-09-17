package com.qualcomm.robotcore.hardware;

public interface Servo {
    enum Direction { FORWARD, REVERSE }
    void setPosition(double position);
    double getPosition();
    void setDirection(Direction direction);
    Direction getDirection();
    void scaleRange(double min, double max);
}
