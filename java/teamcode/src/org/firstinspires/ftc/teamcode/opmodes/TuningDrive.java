package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

/**
 * Full-power step response, then coast. Gives acceleration, top speed and stopping
 * distance -- the three numbers that pin down mu, eta and rolling resistance.
 */
@TeleOp(name = "Tuning: Drive", group = "tuning")
public class TuningDrive extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot();
        robot.init(hardwareMap);

        telemetry.addLine("Tuning: Drive. A = 1.5 s full power, then coast.");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.a) {
                robot.drive.resetEncoders();
                ElapsedTime t = new ElapsedTime();
                double last = 0, topSpeed = 0;

                while (opModeIsActive() && t.seconds() < 1.5) {
                    robot.drive.driveRobotCentric(1.0, 0, 0);
                    robot.update();
                    double now = robot.drive.getForwardIn();
                    double dt = robot.dt();
                    if (dt > 0) topSpeed = Math.max(topSpeed, (now - last) / dt);
                    last = now;
                    telemetry.addData("accelerating", "%.1f in  %.0f in/s", now, topSpeed);
                    telemetry.update();
                    idle();
                }

                double atCut = robot.drive.getForwardIn();
                ElapsedTime coast = new ElapsedTime();
                while (opModeIsActive() && coast.seconds() < 2.0) {
                    robot.drive.stop();
                    robot.update();
                    idle();
                }
                telemetry.addData("top speed", "%.0f in/s", topSpeed);
                telemetry.addData("coast", "%.1f in", robot.drive.getForwardIn() - atCut);
                telemetry.addLine("release A to run again");
                telemetry.update();
                while (opModeIsActive() && gamepad1.a) idle();
            } else {
                robot.drive.driveRobotCentric(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);
                robot.update();
                robot.telemetry(telemetry);
                telemetry.update();
                idle();
            }
        }
        robot.stop();
    }
}
