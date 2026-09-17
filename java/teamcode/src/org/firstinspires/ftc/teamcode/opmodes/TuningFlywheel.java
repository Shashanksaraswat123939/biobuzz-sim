package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

/**
 * Step the flywheel through a set of RPMs and print the spin-up and the dip after each
 * shot. Run this on the sim, then run the identical file on the hub in November and
 * compare -- that is how kS, kV and lossFactor stop being guesses.
 */
@TeleOp(name = "Tuning: Flywheel", group = "tuning")
public class TuningFlywheel extends LinearOpMode {

    private static final double[] STEPS = { 2500, 3000, 3500, 4000, 4500 };

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot();
        robot.init(hardwareMap);

        telemetry.addLine("Tuning: Flywheel. dpad up/down steps the target.");
        telemetry.update();
        waitForStart();

        int index = 0;
        boolean lastUp = false, lastDown = false;
        ElapsedTime sinceChange = new ElapsedTime();
        double peak = 0;
        double settleTime = -1;

        while (opModeIsActive()) {
            if (gamepad1.dpad_up && !lastUp && index < STEPS.length - 1) {
                index++; sinceChange.reset(); peak = 0; settleTime = -1;
            }
            if (gamepad1.dpad_down && !lastDown && index > 0) {
                index--; sinceChange.reset(); peak = 0; settleTime = -1;
            }
            lastUp = gamepad1.dpad_up;
            lastDown = gamepad1.dpad_down;

            double target = STEPS[index];
            robot.flywheel.setTargetRpm(target);
            double rpm = robot.flywheel.getRpm();
            peak = Math.max(peak, rpm);
            if (settleTime < 0 && robot.flywheel.isReady()) settleTime = sinceChange.seconds();

            if (gamepad1.right_bumper && robot.flywheel.isReady() && robot.transfer.feedOne()) {
                robot.flywheel.fired();
            }

            robot.update();
            telemetry.addData("target", "%.0f rpm", target);
            telemetry.addData("rpm", "%.0f", rpm);
            telemetry.addData("overshoot", "%.0f", peak - target);
            telemetry.addData("settle", settleTime < 0 ? "-" : String.format("%.2f s", settleTime));
            telemetry.addData("ready", robot.flywheel.isReady());
            telemetry.addLine("dpad = step, RB = fire");
            telemetry.update();
            idle();
        }
        robot.stop();
    }
}
