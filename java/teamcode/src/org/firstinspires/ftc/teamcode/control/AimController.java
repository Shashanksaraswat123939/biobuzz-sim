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

        // LEAD ON THE ROBOT'S OWN MOTION, all three axes of it.
        //
        // ShotLead used to be constructed here and never called: the hub aimed at the raw
        // bearing and took the table's hood and RPM straight, so every shot taken while
        // driving left with the robot's velocity added to it and went wherever that put it.
        // The simulator's mirror led on azimuth and speed; the deliverable led on nothing.
        //
        // With no localizer -- which is what tryGet returns on a bare hub -- there is no
        // velocity to lead on and solve() falls through to the table's own answer, so this
        // is exactly the old behaviour until odometry is fitted.
        double hoodSpan = robot.cfg.hoodMaxDeg - robot.cfg.hoodMinDeg;
        double tableRpm = robot.shots.rpmFor(range);
        double tableSpeed = robot.cfg.exitSpeedFor(tableRpm);
        double tableElev = robot.cfg.hoodMinDeg + robot.shots.hoodFor(range) * hoodSpan;
        Localizer loc = robot.localizer();
        lead.solve(
            bearing, tableSpeed, tableElev,
            // The localizer reports FTC inches; the shot is solved in metres.
            loc == null ? 0 : loc.getVx() * 0.0254,
            loc == null ? 0 : loc.getVy() * 0.0254,
            loc == null ? 0 : loc.getHeadingDeg(),
            robot.cfg.hoodMinDeg, robot.cfg.hoodMaxDeg);
        lastLeadDeg = lead.correctionDeg;

        robot.turret.aimAt(lead.azimuthDeg, robot.dt());

        if (!wantShot) {
            robot.flywheel.stop();
            aimed = false;
            return false;
        }

        // Both from the lead, not from the table: the hood carries most of the correction
        // and the wheel is left with what is left, which is little.
        robot.hood.setPosition(hoodSpan == 0 ? 0.5 : (lead.elevationDeg - robot.cfg.hoodMinDeg) / hoodSpan);
        robot.flywheel.setTargetRpm(tableSpeed > 1e-6 ? tableRpm * lead.speed / tableSpeed : tableRpm);

        // Against where the turret is actually POINTED, which is the led bearing.
        boolean pointing = robot.turret.onTarget(3.0) && robot.turret.tracker().canReach(lead.azimuthDeg);
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
        if (!robot.turret.tracker().canReach(tp.getAzimuthDeg() + lead.correctionDeg)) return "turret cannot reach";
        if (!robot.turret.onTarget(3.0)) return "turret slewing";
        if (!robot.flywheel.isReady()) return "spinning up";
        if (robot.hopper.isEmpty()) return "hopper empty";
        return "READY";
    }
}
