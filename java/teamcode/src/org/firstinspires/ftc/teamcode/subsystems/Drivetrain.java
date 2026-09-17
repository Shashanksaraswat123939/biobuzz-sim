package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.config.RobotConfig;
import org.firstinspires.ftc.teamcode.control.Localizer;
import org.firstinspires.ftc.teamcode.control.MecanumKinematics;
import org.firstinspires.ftc.teamcode.util.Units;

/** Mecanum drive, field- or robot-centric, with the IMU for heading. */
public class Drivetrain {
    private final MecanumKinematics ik = new MecanumKinematics();
    private DcMotorEx fl, fr, bl, br;
    private IMU imu;
    private Localizer localizer;
    private RobotConfig cfg;
    private double headingOffsetDeg = 0;

    public void init(HardwareMap hw, RobotConfig config) {
        cfg = config;
        fl = hw.get(DcMotorEx.class, config.FL);
        fr = hw.get(DcMotorEx.class, config.FR);
        bl = hw.get(DcMotorEx.class, config.BL);
        br = hw.get(DcMotorEx.class, config.BR);

        fl.setDirection(config.flReversed ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        fr.setDirection(config.frReversed ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        bl.setDirection(config.blReversed ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        br.setDirection(config.brReversed ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);

        setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        setZeroPower(DcMotor.ZeroPowerBehavior.BRAKE);

        imu = hw.tryGet(IMU.class, config.IMU);
        if (imu != null) {
            imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.UP,
                    RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));
        }
        // Sim provides ground truth under this name; on the hub tryGet returns null and
        // the drivetrain falls back to the IMU alone.
        localizer = hw.tryGet(Localizer.class, "localizer");
    }

    public void setMode(DcMotor.RunMode mode) {
        fl.setMode(mode); fr.setMode(mode); bl.setMode(mode); br.setMode(mode);
    }

    public void setZeroPower(DcMotor.ZeroPowerBehavior b) {
        fl.setZeroPowerBehavior(b); fr.setZeroPowerBehavior(b);
        bl.setZeroPowerBehavior(b); br.setZeroPowerBehavior(b);
    }

    public Localizer getLocalizer() { return localizer; }

    public double getHeadingDeg() {
        if (imu != null) {
            return Units.wrapDeg(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) - headingOffsetDeg);
        }
        return localizer != null ? Units.wrapDeg(localizer.getHeadingDeg() - headingOffsetDeg) : 0;
    }

    public void resetHeading() {
        headingOffsetDeg = 0;
        if (imu != null) imu.resetYaw();
    }

    public void driveRobotCentric(double forward, double left, double turn) {
        ik.compute(forward, left, turn);
        apply();
    }

    public void driveFieldCentric(double fieldForward, double fieldLeft, double turn) {
        ik.computeFieldCentric(fieldForward, fieldLeft, turn, getHeadingDeg());
        apply();
    }

    private void apply() {
        fl.setPower(ik.fl); fr.setPower(ik.fr); bl.setPower(ik.bl); br.setPower(ik.br);
    }

    public void stop() { driveRobotCentric(0, 0, 0); }

    /** Average forward travel since the encoders were last reset, in inches. */
    public double getForwardIn() {
        double ticks = (fl.getCurrentPosition() + fr.getCurrentPosition()
                + bl.getCurrentPosition() + br.getCurrentPosition()) / 4.0;
        return ticks / cfg.driveTicksPerRev * 2 * Math.PI * cfg.wheelRadiusIn;
    }

    public void resetEncoders() {
        setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void update() {
        if (localizer != null) localizer.update();
    }

    public void telemetry(Telemetry t) {
        t.addData("heading", "%.1f deg", getHeadingDeg());
        t.addData("forward", "%.1f in", getForwardIn());
        if (localizer != null) {
            t.addData("pose", "%.1f, %.1f @ %.1f", localizer.getX(), localizer.getY(), localizer.getHeadingDeg());
        }
    }
}
