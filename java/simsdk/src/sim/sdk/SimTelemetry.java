package sim.sdk;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.List;

/** Telemetry lines ride back to the world on the ActuatorFrame and show up in the UI. */
public class SimTelemetry implements Telemetry {
    private final SimHub hub;
    private final List<String[]> pending = new ArrayList<String[]>();
    private boolean echo = false;

    public SimTelemetry(SimHub hub) { this.hub = hub; }

    public void setEcho(boolean e) { echo = e; }

    @Override public void addData(String caption, Object value) {
        pending.add(new String[] { caption, String.valueOf(value) });
    }
    @Override public void addData(String caption, String format, Object... args) {
        pending.add(new String[] { caption, String.format(format, args) });
    }
    @Override public void addLine(String line) { pending.add(new String[] { line, "" }); }

    @Override public boolean update() {
        List<Object> lines = new ArrayList<Object>();
        for (String[] kv : pending) {
            List<Object> pair = new ArrayList<Object>();
            pair.add(kv[0]);
            pair.add(kv[1]);
            lines.add(pair);
            if (echo) System.out.println("  " + kv[0] + ": " + kv[1]);
        }
        hub.out.setTelemetry(lines);
        pending.clear();
        return true;
    }

    @Override public void clearAll() { pending.clear(); }
    @Override public void setMsTransmissionInterval(int ms) { }
}
