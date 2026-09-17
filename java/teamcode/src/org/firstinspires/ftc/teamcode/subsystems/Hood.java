package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.config.RobotConfig;
import org.firstinspires.ftc.teamcode.util.Units;

/** Shot elevation. The shot table speaks in servo positions, so this is thin on purpose. */
public class Hood {
    private Servo servo;
    private RobotConfig cfg;
    private double position = 0.5;

    public void init(HardwareMap hw, RobotConfig config) {
        cfg = config;
        servo = hw.tryGet(Servo.class, config.HOOD);
    }

    public void setPosition(double p) { position = Units.clamp(p, 0, 1); }
    public double getPosition() { return position; }

    public double getAngleDeg() { return cfg.hoodMinDeg + position * (cfg.hoodMaxDeg - cfg.hoodMinDeg); }

    public void update() { if (servo != null) servo.setPosition(position); }

    public void telemetry(Telemetry t) { t.addData("hood", "%.2f (%.0f deg)", position, getAngleDeg()); }
}
