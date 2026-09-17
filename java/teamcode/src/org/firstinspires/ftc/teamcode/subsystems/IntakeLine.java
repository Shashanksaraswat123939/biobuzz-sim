package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.config.RobotConfig;

/** The line-in intake: run it forward to collect, backward to spit. */
public class IntakeLine {
    private DcMotorEx motor;
    private double power = 0;

    public void init(HardwareMap hw, RobotConfig cfg) {
        motor = hw.get(DcMotorEx.class, cfg.INTAKE);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public void collect() { power = 1.0; }
    public void eject() { power = -1.0; }
    public void stop() { power = 0.0; }
    public void set(double p) { power = p; }

    public void update() { motor.setPower(power); }

    public void telemetry(Telemetry t) { t.addData("intake", "%.2f", power); }
}
