package sim.sdk;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import sim.bridge.Json;

import java.util.ArrayList;
import java.util.List;

/** A motor on the far side of the bridge. Reads are cached; writes go into the frame. */
public class SimDcMotorEx implements DcMotorEx {
    private final SimHub hub;
    private final String name;
    private final double ticksPerRev;

    private RunMode mode = RunMode.RUN_WITHOUT_ENCODER;
    private Direction direction = Direction.FORWARD;
    private ZeroPowerBehavior zeroPower = ZeroPowerBehavior.BRAKE;
    private double power = 0;
    private double velocity = Double.NaN;
    private int target = 0;
    private int tolerance = 10;
    private boolean enabled = true;
    private boolean resetPending = false;
    private PIDFCoefficients pidf = new PIDFCoefficients(10, 3, 0, 12);

    public SimDcMotorEx(SimHub hub, String name, double ticksPerRev) {
        this.hub = hub;
        this.name = name;
        this.ticksPerRev = ticksPerRev <= 0 ? 537.7 : ticksPerRev;
    }

    private double sign() { return direction == Direction.REVERSE ? -1 : 1; }

    private void emit() {
        java.util.Map<String, Object> m = new java.util.LinkedHashMap<String, Object>();
        m.put("mode", mode.name());
        // Sticky: TeamCode normally does setMode(STOP_AND_RESET_ENCODER) then immediately
        // setMode(RUN_WITHOUT_ENCODER), and only the last mode survives into the frame.
        if (resetPending) m.put("reset", Boolean.TRUE);
        if (!enabled) {
            m.put("power", Double.valueOf(0));
            hub.out.setMotor(name, m);
            return;
        }
        if (mode == RunMode.RUN_TO_POSITION) {
            m.put("target", Double.valueOf(target * sign()));
            m.put("power", Double.valueOf(Math.abs(power)));
        } else if (mode == RunMode.RUN_USING_ENCODER && !Double.isNaN(velocity)) {
            m.put("velocity", Double.valueOf(velocity * sign()));
        } else {
            m.put("power", Double.valueOf(power * sign()));
        }
        m.put("brake", Boolean.valueOf(zeroPower != ZeroPowerBehavior.FLOAT));
        List<Object> gains = new ArrayList<Object>();
        gains.add(Double.valueOf(pidf.p));
        gains.add(Double.valueOf(pidf.i));
        gains.add(Double.valueOf(pidf.d));
        gains.add(Double.valueOf(pidf.f));
        m.put("pidf", gains);
        hub.out.setMotor(name, m);
    }

    @Override public void setPower(double p) { power = p; velocity = Double.NaN; emit(); }
    @Override public double getPower() { return power; }
    @Override public void setDirection(Direction d) { direction = d; emit(); }
    @Override public Direction getDirection() { return direction; }

    @Override public void setMode(RunMode m) {
        mode = m;
        if (m == RunMode.STOP_AND_RESET_ENCODER) {
            power = 0;
            velocity = Double.NaN;
            resetPending = true;
        }
        emit();
    }

    /** The runner calls this once the frame has been sent. */
    public void frameSent() { resetPending = false; }
    @Override public RunMode getMode() { return mode; }

    @Override public void setZeroPowerBehavior(ZeroPowerBehavior b) { zeroPower = b; emit(); }
    @Override public ZeroPowerBehavior getZeroPowerBehavior() { return zeroPower; }

    @Override public int getCurrentPosition() {
        return (int) Math.round(Json.num(hub.read().motor(name), "pos", 0) * sign());
    }
    @Override public void setTargetPosition(int p) { target = p; emit(); }
    @Override public int getTargetPosition() { return target; }
    @Override public boolean isBusy() {
        return mode == RunMode.RUN_TO_POSITION && Math.abs(target - getCurrentPosition()) > tolerance;
    }
    @Override public void setTargetPositionTolerance(int t) { tolerance = t; }
    @Override public int getTargetPositionTolerance() { return tolerance; }

    @Override public void setVelocity(double ticksPerSecond) {
        velocity = ticksPerSecond;
        mode = RunMode.RUN_USING_ENCODER;
        emit();
    }
    @Override public void setVelocity(double angularRate, AngleUnit unit) {
        setVelocity(unit.toDegrees(angularRate) / 360.0 * ticksPerRev);
    }
    @Override public double getVelocity() {
        return Json.num(hub.read().motor(name), "vel", 0) * sign();
    }
    @Override public double getVelocity(AngleUnit unit) {
        double revsPerSec = getVelocity() / ticksPerRev;
        return unit.fromDegrees(revsPerSec * 360.0);
    }
    @Override public void setPIDFCoefficients(RunMode m, PIDFCoefficients c) { pidf = c; emit(); }
    @Override public PIDFCoefficients getPIDFCoefficients(RunMode m) { return pidf; }
    @Override public double getCurrent(CurrentUnit unit) {
        return unit.fromAmps(Json.num(hub.read().motor(name), "amps", 0));
    }
    @Override public void setMotorEnable() { enabled = true; emit(); }
    @Override public void setMotorDisable() { enabled = false; emit(); }
    @Override public boolean isMotorEnabled() { return enabled; }
}
