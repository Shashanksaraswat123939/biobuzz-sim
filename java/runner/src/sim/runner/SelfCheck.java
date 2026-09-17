package sim.runner;

import org.firstinspires.ftc.teamcode.control.LandProbability;
import org.firstinspires.ftc.teamcode.control.ShotLead;
import org.firstinspires.ftc.teamcode.control.ShotTable;
import org.firstinspires.ftc.teamcode.control.MecanumKinematics;
import org.firstinspires.ftc.teamcode.config.RobotConfig;
import org.firstinspires.ftc.teamcode.config.ShotTableData;

/**
 * Asserts on the TeamCode maths that has no other way of being wrong loudly.
 * Run by java/build.sh; a failure here means the Java and the TypeScript have drifted.
 */
public class SelfCheck {

    private static int checks = 0;

    private static void near(String what, double got, double want, double tol) {
        checks++;
        if (Math.abs(got - want) > tol) {
            throw new AssertionError(what + ": got " + got + ", wanted " + want);
        }
    }

    private static void that(String what, boolean ok) {
        checks++;
        if (!ok) throw new AssertionError(what);
    }

    /** Where the ball really goes once the robot's own velocity is added. */
    private static double resultingBearing(double azimuthDeg, double speed, double elevDeg,
                                           double vx, double vy, double headingDeg) {
        double horiz = speed * Math.cos(Math.toRadians(elevDeg));
        double a = Math.toRadians(headingDeg + azimuthDeg);
        return Math.toDegrees(Math.atan2(horiz * Math.sin(a) + vy, horiz * Math.cos(a) + vx));
    }

    public static void main(String[] args) {
        final double elev = 45, speed = 10, heading = 30, bearing = 20;
        ShotLead lead = new ShotLead();

        lead.solve(bearing, speed, elev, 0, 0, heading);
        near("stationary needs no lead", lead.azimuthDeg, bearing, 1e-6);
        near("stationary keeps the table speed", lead.speed, speed, 1e-6);

        double fb = Math.toRadians(heading + bearing);
        double vx = -Math.sin(fb) * 2.0;
        double vy = Math.cos(fb) * 2.0;
        that("uncorrected aim actually misses",
                Math.abs(resultingBearing(bearing, speed, elev, vx, vy, heading) - (heading + bearing)) > 10);
        lead.solve(bearing, speed, elev, vx, vy, heading);
        near("lead puts the ball on the bearing",
                resultingBearing(lead.azimuthDeg, lead.speed, elev, vx, vy, heading), heading + bearing, 1e-4);

        lead.solve(bearing, speed, elev, Math.cos(fb) * 2, Math.sin(fb) * 2, heading);
        that("closing needs less speed", lead.speed < speed);
        lead.solve(bearing, speed, elev, -Math.cos(fb) * 2, -Math.sin(fb) * 2, heading);
        that("retreating needs more speed", lead.speed > speed);

        MecanumKinematics ik = new MecanumKinematics();
        ik.compute(1, 0, 0);
        near("forward drives all four the same", ik.fl, ik.fr, 1e-9);
        near("forward drives all four the same", ik.fl, ik.br, 1e-9);
        ik.compute(0, 1, 0);
        that("strafe opposes the diagonals", ik.fl * ik.fr < 0);
        ik.compute(2, 2, 2);
        that("mixed requests are normalised", Math.abs(ik.fl) <= 1.0001 && Math.abs(ik.fr) <= 1.0001);

        RobotConfig cfg = new RobotConfig();
        ShotTable table = cfg.shotTable();
        that("shot table has rows", !table.isEmpty());
        double mid = (table.minRange() + table.maxRange()) / 2;
        that("shot table interpolates a sane rpm", table.rpmFor(mid) > 500 && table.rpmFor(mid) < 8000);
        that("shot table clamps below its range", table.rpmFor(table.minRange() - 50) == table.rpmFor(table.minRange()));
        that("exit speed rises with rpm", cfg.exitSpeedFor(4000) > cfg.exitSpeedFor(2000));
        near("exit speed is k*omega*r", cfg.exitSpeedFor(3000),
                0.45 * 0.048 * (3000 * 2 * Math.PI / 60.0), 1e-9);

        // ---- the land-probability gate, which has to agree with the TypeScript that
        // ---- calibrated it. These are the same assertions as tests/landprob.test.ts.
        near("normalCdf(0)", LandProbability.normalCdf(0), 0.5, 1e-7);
        near("normalCdf(1)", LandProbability.normalCdf(1), 0.8413447, 1e-6);
        near("normalCdf(-1)", LandProbability.normalCdf(-1), 0.1586553, 1e-6);
        near("a two-sigma band is 95%", LandProbability.pThread(-2, 2, 0, 1), 0.9544997, 1e-5);
        that("no scatter is a hard test",
                LandProbability.pThread(1, 2, 1.5, 0) == 1 && LandProbability.pThread(1, 2, 3, 0) == 0);
        that("calibration never promises more than was measured",
                LandProbability.calibrate(1.0) <= ShotTableData.CAL_CEILING + 1e-9);
        that("calibration is monotone",
                LandProbability.calibrate(0.5) <= LandProbability.calibrate(0.9));

        double pOn = LandProbability.pLand(table, mid, cfg.exitSpeedFor(table.rpmFor(mid)),
                mid * 0.0254, 0, cfg.flywheelYawScatterDeg);
        double pOff = LandProbability.pLand(table, mid, cfg.exitSpeedFor(table.rpmFor(mid)),
                mid * 0.0254, 12, cfg.flywheelYawScatterDeg);
        that("P(land) is a probability", pOn >= 0 && pOn <= 1);
        that("pointing 12 degrees off is worse than pointing at it", pOff < pOn);

        System.out.println("teamcode self-check: " + checks + " assertions passed");
    }
}
