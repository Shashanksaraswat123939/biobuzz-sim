package sim.sdk;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Iterator;
import java.util.List;
import java.util.Map;

/**
 * Resolves by the names in robot.json -> hardware, and throws on an unknown name
 * exactly as the hub does, so a typo fails here rather than at an event.
 */
public class SimHardwareMap extends HardwareMap {
    private final SimHub hub;
    private final Map<String, Object> devices = new HashMap<String, Object>();
    private final Map<String, Double> ticksPerRev = new HashMap<String, Double>();
    private final SimLynxModule lynx;

    public SimHardwareMap(SimHub hub) {
        this.hub = hub;
        this.lynx = new SimLynxModule(hub);
        final List<VoltageSensor> volts = new ArrayList<VoltageSensor>();
        volts.add(new SimVoltageSensor(hub));
        this.voltageSensor = new DeviceMapping<VoltageSensor>() {
            @Override public Iterator<VoltageSensor> iterator() { return volts.iterator(); }
        };
        devices.put("Control Hub", volts.get(0));
    }

    /** Register any extra device (the runner adds the sim-only Localizer this way). */
    public void register(String name, Object device) { devices.put(name, device); }

    /** Make an existing device answer to a second name (robot.json -> hardware). */
    public void alias(String existing, String alsoKnownAs) {
        Object d = devices.get(existing);
        if (d != null) devices.put(alsoKnownAs, d);
    }

    /** Called by the runner from robot.json before the OpMode is constructed. */
    public void registerMotor(String name, double tpr) {
        ticksPerRev.put(name, Double.valueOf(tpr));
        devices.put(name, new SimDcMotorEx(hub, name, tpr));
    }
    public void registerServo(String name) { devices.put(name, new SimServo(hub, name)); }
    public void registerCRServo(String name) { devices.put(name, new SimCRServo(hub, name)); }
    public void registerImu(String name) { devices.put(name, new SimIMU(hub)); }
    public void registerDistance(String name) { devices.put(name, new SimDistanceSensor(hub, name)); }

    @Override
    @SuppressWarnings("unchecked")
    public <T> T get(Class<? extends T> cls, String deviceName) {
        Object d = devices.get(deviceName);
        if (d == null) {
            throw new IllegalArgumentException(
                    "Unable to find a hardware device with the name \"" + deviceName + "\"");
        }
        if (!cls.isInstance(d)) {
            throw new IllegalArgumentException(
                    "Device \"" + deviceName + "\" is a " + d.getClass().getSimpleName()
                            + ", not a " + cls.getSimpleName());
        }
        return (T) d;
    }

    @Override
    @SuppressWarnings("unchecked")
    public <T> T tryGet(Class<? extends T> cls, String deviceName) {
        Object d = devices.get(deviceName);
        return cls.isInstance(d) ? (T) d : null;
    }

    @Override
    @SuppressWarnings("unchecked")
    public <T> List<T> getAll(Class<? extends T> cls) {
        List<T> out = new ArrayList<T>();
        if (cls == LynxModule.class) {
            out.add((T) lynx);
            return out;
        }
        for (Object d : devices.values()) if (cls.isInstance(d)) out.add((T) d);
        return out;
    }

    /** Tell every motor its frame has shipped, so sticky flags clear exactly once. */
    public void frameSent() {
        for (Object d : devices.values()) if (d instanceof SimDcMotorEx) ((SimDcMotorEx) d).frameSent();
    }

    public boolean has(String name) { return devices.containsKey(name); }
    public double ticksPerRev(String name) {
        Double d = ticksPerRev.get(name);
        return d == null ? 537.7 : d.doubleValue();
    }
}
