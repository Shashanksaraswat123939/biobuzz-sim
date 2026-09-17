package sim.sdk;

import org.firstinspires.ftc.teamcode.control.Localizer;
import sim.bridge.Json;

/**
 * Ground truth from the world, handed to TeamCode through the interface TeamCode itself
 * defines. TeamCode never imports anything from here; it looks the device up by name and
 * gets null on the hub, where a Pinpoint or Limelight implementation takes over.
 */
public class SimLocalizer implements Localizer {
    private final SimHub hub;
    public SimLocalizer(SimHub hub) { this.hub = hub; }

    private double f(String key) { return Json.num(hub.read().localizer(), key, 0); }

    @Override public double getX() { return f("x"); }
    @Override public double getY() { return f("y"); }
    @Override public double getHeadingDeg() { return f("heading"); }
    @Override public double getVx() { return f("vx"); }
    @Override public double getVy() { return f("vy"); }
    @Override public double getOmegaDps() { return f("omega"); }
    @Override public void update() { }
}
