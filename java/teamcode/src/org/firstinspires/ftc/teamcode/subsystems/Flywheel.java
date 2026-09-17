package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.config.RobotConfig;
import org.firstinspires.ftc.teamcode.control.FlywheelGate;
import org.firstinspires.ftc.teamcode.util.Units;

/**
 * The shooter wheel. The readiness gate lives in FlywheelGate because that is the piece
 * that has to ship to the hub; this class just wires it to hardware.
 */
public class Flywheel {
    private DcMotorEx motor;
    private VoltageSensor battery;
    private RobotConfig cfg;
    private FlywheelGate gate;

    public void init(HardwareMap hw, RobotConfig config) {
        cfg = config;
        motor = hw.get(DcMotorEx.class, config.FLYWHEEL);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // A flywheel must coast, never brake: braking on zero power throws away the
        // stored energy that the next shot needs.
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        gate = new FlywheelGate(config.flywheelKs, config.flywheelKv, config.flywheelKp,
                config.flywheelTolRpm, config.flywheelMinRpmFrac, config.flywheelReadySteps,
                config.flywheelRpmFilterFrames);
        for (VoltageSensor v : hw.voltageSensor) { battery = v; break; }
    }

    public FlywheelGate gate() { return gate; }

    public void setTargetRpm(double rpm) {
        gate.setTargetRpm(Units.clamp(rpm, 0, cfg.flywheelMaxRpm));
    }

    public void stop() { gate.setTargetRpm(0); }

    public double getRpm() {
        return Units.ticksPerSecToRpm(motor.getVelocity(), cfg.flywheelTicksPerRev);
    }

    public boolean isReady() { return gate.isReady(); }

    public void fired() { gate.fired(); }

    public void update() {
        double volts = battery == null ? 12.0 : battery.getVoltage();
        motor.setPower(gate.update(getRpm(), volts));
    }

    public void telemetry(Telemetry t) {
        t.addData("flywheel", "%.0f / %.0f rpm%s", getRpm(), gate.getTargetRpm(), gate.isReady() ? "  READY" : "");
    }
}
