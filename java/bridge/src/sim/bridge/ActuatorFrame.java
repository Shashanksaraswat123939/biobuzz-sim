package sim.bridge;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

/**
 * Brain -> world. Motor powers, servo positions, telemetry. Nothing else.
 *
 * Every mutation is atomic because two threads touch it: the OpMode thread writes through
 * the fake hardware while the runner serialises and clears it. Handing over a finished map
 * per device, under one lock, is cheaper than trying to make the whole handoff lock-free
 * and it removes the ConcurrentModificationException that killed long runs.
 */
public class ActuatorFrame {
    private final Object lock = new Object();
    private final Map<String, Object> motors = new LinkedHashMap<String, Object>();
    private final Map<String, Object> servos = new LinkedHashMap<String, Object>();
    private final List<Object> telemetry = new ArrayList<Object>();
    private final List<Object> log = new ArrayList<Object>();

    public void setMotor(String name, Map<String, Object> state) {
        synchronized (lock) { motors.put(name, state); }
    }

    public void setServo(String name, double position) {
        synchronized (lock) { servos.put(name, Double.valueOf(position)); }
    }

    public void setTelemetry(List<Object> lines) {
        synchronized (lock) {
            telemetry.clear();
            telemetry.addAll(lines);
        }
    }

    public void addLog(String line) {
        synchronized (lock) { log.add(line); }
    }

    public void clear() {
        synchronized (lock) {
            motors.clear();
            servos.clear();
            telemetry.clear();
            log.clear();
        }
    }

    public String toJson(long seq) {
        synchronized (lock) {
            Map<String, Object> m = new LinkedHashMap<String, Object>();
            m.put("type", "actuator");
            m.put("seq", Double.valueOf(seq));
            m.put("motors", new LinkedHashMap<String, Object>(motors));
            m.put("servos", new LinkedHashMap<String, Object>(servos));
            m.put("telemetry", new ArrayList<Object>(telemetry));
            m.put("log", new ArrayList<Object>(log));
            return Json.write(m);
        }
    }
}
