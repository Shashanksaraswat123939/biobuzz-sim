package com.qualcomm.robotcore.eventloop.opmode;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * SDK shim. The runner drives start and stop exactly as the hub does, and `sleep` and
 * `opModeIsActive` run on the SIM clock so lockstep runs repeat exactly.
 */
public abstract class LinearOpMode {
    /** The runner implements this. TeamCode never references it. */
    public interface LinearOpModeHost {
        boolean isStarted();
        boolean isStopRequested();
        void idle() throws InterruptedException;
        void sleep(long ms) throws InterruptedException;
    }

    public HardwareMap hardwareMap;
    public Telemetry telemetry;
    public Gamepad gamepad1;
    public Gamepad gamepad2;

    /** Set by the runner. Not part of the SDK surface TeamCode may touch. */
    public volatile LinearOpModeHost host;

    private final ElapsedTime runtime = new ElapsedTime();

    public abstract void runOpMode() throws InterruptedException;

    public void waitForStart() throws InterruptedException {
        while (!isStarted() && !isStopRequested()) idle();
    }
    public boolean opModeIsActive() { return isStarted() && !isStopRequested(); }
    public boolean opModeInInit() { return !isStarted() && !isStopRequested(); }
    public boolean isStarted() { return host != null && host.isStarted(); }
    public boolean isStopRequested() { return host == null || host.isStopRequested(); }
    public void idle() throws InterruptedException { if (host != null) host.idle(); }
    public void sleep(long milliseconds) throws InterruptedException {
        if (host != null) host.sleep(milliseconds);
    }
    public double getRuntime() { return runtime.seconds(); }
    public void resetRuntime() { runtime.reset(); }
}
