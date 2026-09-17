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

    /** ... and how fast, horizontally and vertically. The vertical is where the bug was. */
    private static double resultingHoriz(double azimuthDeg, double speed, double elevDeg,
                                         double vx, double vy, double headingDeg) {
        double horiz = speed * Math.cos(Math.toRadians(elevDeg));
        double a = Math.toRadians(headingDeg + azimuthDeg);
        return Math.hypot(horiz * Math.cos(a) + vx, horiz * Math.sin(a) + vy);
    }

    public static void main(String[] args) {
        final double elev = 45, speed = 10, heading = 30, bearing = 20;
        final double HOOD_LO = 0, HOOD_HI = 90;
        ShotLead lead = new ShotLead();

        lead.solve(bearing, speed, elev, 0, 0, heading, HOOD_LO, HOOD_HI);
        near("stationary needs no lead", lead.azimuthDeg, bearing, 1e-6);
        near("stationary keeps the table speed", lead.speed, speed, 1e-6);
        near("stationary keeps the table hood", lead.elevationDeg, elev, 1e-6);

        double fb = Math.toRadians(heading + bearing);
        double vx = -Math.sin(fb) * 2.0;
        double vy = Math.cos(fb) * 2.0;
        that("uncorrected aim actually misses",
                Math.abs(resultingBearing(bearing, speed, elev, vx, vy, heading) - (heading + bearing)) > 10);
        lead.solve(bearing, speed, elev, vx, vy, heading, HOOD_LO, HOOD_HI);
        near("lead puts the ball on the bearing",
                resultingBearing(lead.azimuthDeg, lead.speed, lead.elevationDeg, vx, vy, heading),
                heading + bearing, 1e-4);

        // THE WHOLE LAUNCH VECTOR, not just the ground track. Solving speed at a fixed hood
        // holds the horizontal and breaks the vertical, and a ball with the wrong hang time
        // does not reach the CELL mouth's height at all. Mirrors tests/shotlead.test.ts.
        double wantHoriz = speed * Math.cos(Math.toRadians(elev));
        double wantVert = speed * Math.sin(Math.toRadians(elev));
        double[][] motions = { {0, 0}, {2, 0}, {-2, 0}, {0, 2}, {1.5, -1.5} };
        for (int i = 0; i < motions.length; i++) {
            double along = motions[i][0], across = motions[i][1];
            double mx = Math.cos(fb) * along - Math.sin(fb) * across;
            double my = Math.sin(fb) * along + Math.cos(fb) * across;
            lead.solve(bearing, speed, elev, mx, my, heading, HOOD_LO, HOOD_HI);
            near("lead holds the bearing",
                    resultingBearing(lead.azimuthDeg, lead.speed, lead.elevationDeg, mx, my, heading),
                    heading + bearing, 1e-4);
            near("lead holds the horizontal",
                    resultingHoriz(lead.azimuthDeg, lead.speed, lead.elevationDeg, mx, my, heading),
                    wantHoriz, 1e-4);
            near("lead holds the VERTICAL",
                    lead.speed * Math.sin(Math.toRadians(lead.elevationDeg)), wantVert, 1e-4);
        }

        lead.solve(bearing, speed, elev, Math.cos(fb) * 2, Math.sin(fb) * 2, heading, HOOD_LO, HOOD_HI);
        that("closing needs less speed", lead.speed < speed);
        that("closing needs a steeper hood", lead.elevationDeg > elev);
        lead.solve(bearing, speed, elev, -Math.cos(fb) * 2, -Math.sin(fb) * 2, heading, HOOD_LO, HOOD_HI);
        that("retreating needs more speed", lead.speed > speed);
        that("retreating needs a flatter hood", lead.elevationDeg < elev);

        // Closing nearly as fast as the ball's own horizontal leaves a near-vertical
        // solution; a hood that cannot get there stops at its stop rather than asking for a
        // speed no wheel has.
        double vNear = speed * Math.cos(Math.toRadians(elev)) - 0.05;
        lead.solve(bearing, speed, elev, Math.cos(fb) * vNear, Math.sin(fb) * vNear, heading, 30, 85);
        that("the solved hood is clamped to its travel",
                lead.elevationDeg <= 85.0001 && lead.elevationDeg >= 29.9999);
        that("and the speed stays sane when it is", lead.speed < speed * 1.2);

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
