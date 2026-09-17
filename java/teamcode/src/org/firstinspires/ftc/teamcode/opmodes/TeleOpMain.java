package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.control.AimController;
import org.firstinspires.ftc.teamcode.control.DriveToRange;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

/** Driver practice: field-centric drive, intake on the triggers, auto-aim on X. */
@TeleOp(name = "TeleOp Main", group = "match")
public class TeleOpMain extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot();
        robot.init(hardwareMap);
        AimController aim = new AimController(robot);
        DriveToRange toRange = new DriveToRange(robot.shots, 6.0, 0.02);

        boolean autoAim = true;
        boolean lastX = false;
        boolean spinUp = false;
        boolean lastA = false;

        telemetry.addLine("TeleOpMain ready");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.x && !lastX) autoAim = !autoAim;
            lastX = gamepad1.x;
            if (gamepad1.a && !lastA) spinUp = !spinUp;
            lastA = gamepad1.a;

            double slow = gamepad1.left_bumper ? 0.35 : 1.0;
            double forward = -gamepad1.left_stick_y * slow;
            double left = -gamepad1.left_stick_x * slow;
            double turn = -gamepad1.right_stick_x * slow;

            boolean canShoot = aim.update(spinUp);

            if (autoAim && spinUp && Math.abs(forward) < 0.15 && Math.abs(left) < 0.15) {
                // Hands off the sticks: close the range to the table's best band.
                forward = toRange.update(aim.rangeIn(), robot.target() == null ? 0 : robot.target().getAzimuthDeg());
            }
            robot.drive.driveFieldCentric(forward, left, turn);

            if (gamepad1.right_trigger > 0.1) {
                robot.intake.collect();
                robot.hopper.setCount(robot.hopper.getCount()); // real counter goes here
            } else if (gamepad1.left_trigger > 0.1) {
                robot.intake.eject();
            } else {
                robot.intake.stop();
            }

            if ((gamepad1.right_bumper || gamepad1.b) && canShoot) aim.fireIfReady();
            robot.transfer.setGate(gamepad1.y);

            robot.update();

            telemetry.addData("status", aim.status());
            telemetry.addData("auto-aim", autoAim);
            telemetry.addData("range", "%.1f in", aim.rangeIn());
            robot.telemetry(telemetry);
            telemetry.update();
        }
        robot.stop();
    }
}
