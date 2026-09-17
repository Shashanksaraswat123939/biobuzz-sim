package sim.sdk;

import com.qualcomm.robotcore.hardware.VoltageSensor;

public class SimVoltageSensor implements VoltageSensor {
    private final SimHub hub;
    public SimVoltageSensor(SimHub hub) { this.hub = hub; }
    @Override public double getVoltage() { return hub.battery(); }
}
