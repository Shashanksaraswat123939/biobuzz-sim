package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.config.RobotConfig;
import org.firstinspires.ftc.teamcode.control.Localizer;
import org.firstinspires.ftc.teamcode.control.ShotTable;
import org.firstinspires.ftc.teamcode.control.TargetProvider;

import java.util.List;

/**
 * Everything the robot is, assembled once. OpModes compose this and call update().
 * No threads, no static mutable state: the SDK does not forgive either.
 */
public class Robot {
    public final RobotConfig cfg = new RobotConfig();
    public final Drivetrain drive = new Drivetrain();
    public final IntakeLine intake = new IntakeLine();
    public final Transfer transfer = new Transfer();
    public final Turret turret = new Turret();
    public final Hood hood = new Hood();
    public final Flywheel flywheel = new Flywheel();
    public final Hopper hopper = new Hopper();
    public final ShotTable shots = cfg.shotTable();

    private TargetProvider target;
    private final ElapsedTime loop = new ElapsedTime();
    private double dt = 0.02;

    public void init(HardwareMap hw) {
        // Bulk caching AUTO: one I/O round trip per loop instead of one per read. On the
        // hub this is the difference between a 10 ms loop and a 40 ms one.
        List<LynxModule> hubs = hw.getAll(LynxModule.class);
        for (int i = 0; i < hubs.size(); i++) {
            hubs.get(i).setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        drive.init(hw, cfg);
        intake.init(hw, cfg);
        transfer.init(hw, cfg);
        turret.init(hw, cfg);
        hood.init(hw, cfg);
        flywheel.init(hw, cfg);
        hopper.init(hw, cfg);

        // Sim registers these; on the hub tryGet returns null and the caller falls back.
        target = hw.tryGet(TargetProvider.class, "target");
        loop.reset();
    }

    public TargetProvider target() { return target; }
    public Localizer localizer() { return drive.getLocalizer(); }
    public double dt() { return dt; }

    public void update() {
        dt = Math.max(1e-3, Math.min(0.2, loop.seconds()));
        loop.reset();
        drive.update();
        intake.update();
        transfer.update();
        turret.update();
        hood.update();
        flywheel.update();
    }

    public void telemetry(Telemetry t) {
        drive.telemetry(t);
        intake.telemetry(t);
        transfer.telemetry(t);
        turret.telemetry(t);
        hood.telemetry(t);
        flywheel.telemetry(t);
        hopper.telemetry(t);
    }

    public void stop() {
        drive.stop();
        intake.stop();
        transfer.hold();
        flywheel.stop();
        update();
    }
}
