package org.firstinspires.ftc.teamcode.control;

import org.firstinspires.ftc.teamcode.subsystems.Robot;

/**
 * Turret for azimuth, shot table for hood and RPM, gate for permission to fire.
 * Shared by TeleOp and the autonomous routines so there is only one aiming behaviour
 * to tune -- and only one to port.
 */
public class AimController {
    private final Robot robot;
    private final ShotLead lead = new ShotLead();
    private boolean aimed = false;
    private double lastRangeIn = 0;
    private double lastLeadDeg = 0;

    public AimController(Robot robot) { this.robot = robot; }

    public boolean isAimed() { return aimed; }
    public double rangeIn() { return lastRangeIn; }
    /** How many degrees of the aim are motion lead. Telemetry only. */
    public double leadDeg() { return lastLeadDeg; }

    /** Call once per loop while aiming. Returns true when a shot is allowed. */
    public boolean update(boolean wantShot) {
        TargetProvider tp = robot.target();
        if (tp == null || !tp.isValid()) {
            aimed = false;
            robot.flywheel.stop();
            return false;
        }
        double range = tp.getRangeIn();
        lastRangeIn = range;
        double bearing = tp.getAzimuthDeg();

        robot.turret.aimAt(bearing, robot.dt());

        if (!wantShot) {
            robot.flywheel.stop();
            aimed = false;
            return false;
        }

        robot.hood.setPosition(robot.shots.hoodFor(range));
        robot.flywheel.setTargetRpm(robot.shots.rpmFor(range));

        boolean pointing = robot.turret.onTarget(3.0) && robot.turret.tracker().canReach(bearing);
        boolean inRange = robot.shots.usable(range);
        // G417: only a shot into the up CELL may move the HIVE, and a hive already
        // tipping is not a target. Hold fire.
        aimed = pointing && inRange && robot.flywheel.isReady() && !tp.isTipping();
        return aimed;
    }

    /** Fire if everything says yes. Returns true if a ball was actually released. */
    public boolean fireIfReady() {
        if (!aimed || robot.hopper.isEmpty()) return false;
        if (!robot.transfer.feedOne()) return false;
        robot.flywheel.fired();
        robot.hopper.onFeed();
        return true;
    }

    public String status() {
        TargetProvider tp = robot.target();
        if (tp == null || !tp.isValid()) return "no target";
        if (tp.isTipping()) return "hive tipping - hold";
        if (!robot.shots.usable(tp.getRangeIn())) return "out of range";
        if (!robot.turret.tracker().canReach(tp.getAzimuthDeg())) return "turret cannot reach";
        if (!robot.turret.onTarget(3.0)) return "turret slewing";
        if (!robot.flywheel.isReady()) return "spinning up";
        if (robot.hopper.isEmpty()) return "hopper empty";
        return "READY";
    }
}
