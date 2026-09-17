package sim.sdk;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/** The world ray-casts from the named mount in robot.json and reports inches. */
public class SimDistanceSensor implements DistanceSensor {
    private final SimHub hub;
    private final String name;

    public SimDistanceSensor(SimHub hub, String name) { this.hub = hub; this.name = name; }

    @Override public double getDistance(DistanceUnit unit) {
        return unit.fromInches(hub.read().distance(name));
    }
}
