package sim.sdk;

import com.qualcomm.robotcore.hardware.AngularVelocity;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import sim.bridge.Json;

/** The world already applies the hub's IMU latency, so this just reads the frame. */
public class SimIMU implements IMU {
    private final SimHub hub;
    private double yawOffset = 0;

    public SimIMU(SimHub hub) { this.hub = hub; }

    @Override public boolean initialize(Parameters parameters) { return true; }
    @Override public void resetYaw() { yawOffset = rawYaw(); }

    private double rawYaw() { return Json.num(hub.read().imu(), "yaw", 0); }

    @Override public YawPitchRollAngles getRobotYawPitchRollAngles() {
        return new YawPitchRollAngles(
                AngleUnit.DEGREES,
                AngleUnit.DEGREES.normalize(rawYaw() - yawOffset),
                Json.num(hub.read().imu(), "pitch", 0),
                Json.num(hub.read().imu(), "roll", 0),
                System.nanoTime());
    }

    @Override public AngularVelocity getRobotAngularVelocity(AngleUnit unit) {
        double rateDeg = Json.num(hub.read().imu(), "yawRate", 0);
        return new AngularVelocity(unit, 0f, 0f, (float) unit.fromDegrees(rateDeg), System.nanoTime());
    }
}
