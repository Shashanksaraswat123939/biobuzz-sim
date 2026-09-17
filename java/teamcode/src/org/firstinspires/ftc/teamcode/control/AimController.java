package org.firstinspires.ftc.teamcode.control;

import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.util.Units;

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
    private double lastHoodErrDeg = 0;
    private double lastPLand = -1;
    /** One-pole filtered localizer velocity, m/s. The lead is only as good as this. */
    private double velX = 0;
    private double velY = 0;

    public AimController(Robot robot) { this.robot = robot; }

    public boolean isAimed() { return aimed; }
    public double rangeIn() { return lastRangeIn; }
    /** How many degrees of the aim are motion lead. Telemetry only. */
    public double leadDeg() { return lastLeadDeg; }
    /** Hood angle minus the angle this shot needs, degrees. Telemetry only. */
    public double hoodErrDeg() { return lastHoodErrDeg; }
    /** Calibrated P(this shot lands), or -1 if the table carries no model. Telemetry only. */
    public double pLand() { return lastPLand; }

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
        // FILTER THE REPORTED VELOCITY BEFORE AIMING ON IT. The whole lead hangs off this
        // number and on a real robot it is a differentiated encoder, which is the noisiest
        // thing on the machine. One pole: the quantity being estimated moves on the timescale
        // of the robot's own acceleration, so a few tens of milliseconds of lag is cheap and
        // the noise it removes would otherwise jitter the lead azimuth, drag the turret around
        // chasing it, and hold the readiness gate open on an axis that never settles.
        Localizer loc = robot.localizer();
        double rawVx = loc == null ? 0 : loc.getVx() * 0.0254;
        double rawVy = loc == null ? 0 : loc.getVy() * 0.0254;
        double a = Units.clamp(robot.cfg.velFilterAlpha, 0.01, 1);
        velX += a * (rawVx - velX);
        velY += a * (rawVy - velY);
        // THE BALL INHERITS THE MUZZLE'S VELOCITY, NOT THE CHASSIS'S: v_cg + omega x r. The
        // turret's MEASURED angle, because that is where the muzzle is. With no localizer
        // omega is 0 and this returns the chassis velocity unchanged.
        double[] mv = ShotLead.muzzleVelocity(velX, velY,
            loc == null ? 0 : loc.getOmegaDps(),
            loc == null ? 0 : loc.getHeadingDeg(),
            robot.turret.getAngleDeg(),
            robot.cfg.muzzleOffsetM);
        lead.solve(
            bearing, tableSpeed, tableElev, mv[0], mv[1],
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

        // WAIT FOR THE HOOD. It used to hold the table's stationary angle and arrive long
        // before the wheel did, so nothing checked it. The lead solves it now -- that is what
        // makes shooting on the move work -- so it moves every loop and a shot taken mid-slew
        // leaves at an elevation belonging to a velocity the robot has already left. A servo
        // is commanded rather than measured, so this is a readiness question and not another
        // factor in the probability: once the hood IS there, the ball leaves with the table's
        // launch vector and the band the table measured standing still is valid again.
        lastHoodErrDeg = robot.hood.getAngleDeg() - lead.elevationDeg;
        boolean hoodThere = Math.abs(lastHoodErrDeg) <= robot.cfg.hoodTolDeg;

        // WILL IT LAND? LandProbability has been sitting here fully implemented, self-checked
        // and called by nothing, so the hub's only gate was the RPM window -- a fixed band at
        // every range, blind to whether the robot was pointing into the mouth and blind to
        // whether a ball arriving like this stays in. This is the same product the simulator's
        // mirror computes, against the same baked calibration.
        //
        // The measured speed is mapped back into the frame the table's band was solved in:
        // speedLo/speedHi thread the mouth from a STANDING robot at the table's own angle, and
        // the lead moves both the angle and the speed, so the band moves with them.
        double exitRel = robot.cfg.exitSpeedFor(robot.flywheel.getRpm()) - (lead.speed - tableSpeed);
        lastPLand = LandProbability.pLand(
                robot.shots, range, exitRel, range * 0.0254,
                robot.turret.errorDeg(), robot.cfg.flywheelYawScatterDeg);
        // -1 means the table predates the columns the model needs; fall through to the window
        // rather than refusing every shot forever.
        boolean likely = lastPLand < 0 || lastPLand >= robot.cfg.flywheelMinLandProb;

        // IS THE MOUTH STILL OPEN TOWARDS US? A TIP swaps which CELL is up and the new one
        // opens the other way, so a robot that was square onto the goal is now behind it and
        // no launch can enter. 75 deg rather than 90 because at 90 the opening is exactly
        // edge-on and has no area at all. The mirror gates on the same number.
        boolean mouthOpen = tp.getOpenAngleDeg() <= 75.0;

        // G417: only a shot into the up CELL may move the HIVE, and a hive already
        // tipping is not a target. Hold fire.
        aimed = pointing && inRange && hoodThere && likely && robot.flywheel.isReady()
                && !tp.isTipping() && mouthOpen;

        // AND KEEP CHECKING AFTER THE DECISION. A feed already in flight is cancelled if the
        // things that can go bad inside those four tenths do: the turret running out of travel,
        // or falling behind under a turning chassis.
        //
        // THE TWO TURRET CONDITIONS AND NOTHING ELSE, which is also what the mirror re-checks
        // (builtinTeleOp: turretPastStopDeg < 0.5 && |turretErrDeg| < 3). The two halves had
        // drifted apart, each keeping a different term that the measurement rejects: a WHEEL
        // FLOOR here, and P(land) in the mirror. The floor is unmeetable for a robot whose
        // range is growing -- it took the strafing case to zero -- and re-checking P(land)
        // rides the tachometer's quantisation and flickers (stopped 0.36 -> 0.12). Not the
        // full readiness latch either: that needs three consecutive good loops, and demanding
        // an unbroken run of them while the gate servo travels stops the robot shooting almost
        // entirely. tests/shotlead.test.ts asserts the two expressions agree.
        robot.transfer.stillGood(
                robot.turret.tracker().canReach(lead.azimuthDeg)
                && robot.turret.onTarget(3.0));
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
        if (Math.abs(lastHoodErrDeg) > robot.cfg.hoodTolDeg) {
            return String.format("hood %.0f off", lastHoodErrDeg);
        }
        if (lastPLand >= 0 && lastPLand < robot.cfg.flywheelMinLandProb) {
            return String.format("P(land) %.0f%% < %.0f%%",
                    lastPLand * 100, robot.cfg.flywheelMinLandProb * 100);
        }
        if (robot.hopper.isEmpty()) return "hopper empty";
        return "READY";
    }
}
