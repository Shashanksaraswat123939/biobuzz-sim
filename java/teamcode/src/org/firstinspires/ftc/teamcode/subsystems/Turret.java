package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.config.RobotConfig;
import org.firstinspires.ftc.teamcode.control.TurretTracker;
import org.firstinspires.ftc.teamcode.util.Units;

/** Shooter yaw. RUN_TO_POSITION on the turret motor, rate limited by the tracker. */
public class Turret {
    private DcMotorEx motor;
    private RobotConfig cfg;
    private TurretTracker tracker;
    private double commandDeg = 0;

    public void init(HardwareMap hw, RobotConfig config) {
        cfg = config;
        motor = hw.tryGet(DcMotorEx.class, config.TURRET);
        tracker = new TurretTracker(config.turretMinDeg, config.turretMaxDeg, config.turretSpeedDps);
        if (motor != null) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setTargetPosition(0);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor.setPower(1.0);
        }
    }

    public boolean isPresent() { return motor != null; }
    public TurretTracker tracker() { return tracker; }

    /** Point at a bearing relative to the robot's heading. */
    public void aimAt(double bearingDeg, double dt) {
        commandDeg = tracker.update(bearingDeg, dt);
    }

    public void setAngleDeg(double deg) {
        commandDeg = Units.clamp(deg, cfg.turretMinDeg, cfg.turretMaxDeg);
        tracker.reset(commandDeg);
    }

    public double getAngleDeg() {
        if (motor == null) return 0;
        return motor.getCurrentPosition() / cfg.turretTicksPerDeg;
    }

    public boolean onTarget(double toleranceDeg) {
        return Math.abs(errorDeg()) < toleranceDeg;
    }

    /** Where it is minus where it was told to be, degrees. The land model needs the number. */
    public double errorDeg() { return Units.wrapDeg(getAngleDeg() - commandDeg); }

    public void update() {
        if (motor == null) return;
        motor.setTargetPosition((int) Math.round(commandDeg * cfg.turretTicksPerDeg));
        motor.setPower(1.0);
    }

    public void telemetry(Telemetry t) {
        t.addData("turret", "%.1f -> %.1f deg", getAngleDeg(), commandDeg);
    }
}
