package sim.sdk;

import sim.bridge.ActuatorFrame;
import sim.bridge.Json;
import sim.bridge.SensorFrame;

import com.qualcomm.hardware.lynx.LynxModule;

/**
 * The fake hub. Holds the frame every device reads from and the frame they all write to.
 *
 * Bulk caching is emulated for real (PLAN.md section 3.4): with AUTO every read inside one
 * loop sees the same snapshot, so TeamCode written against this behaves the same way when
 * a real hub charges it 2-3 ms per uncached read.
 */
public class SimHub {
    public final ActuatorFrame out = new ActuatorFrame();

    private volatile SensorFrame latest = SensorFrame.empty();
    private SensorFrame cached = SensorFrame.empty();
    private LynxModule.BulkCachingMode mode = LynxModule.BulkCachingMode.AUTO;
    private boolean cacheValid = false;

    /** The runner calls this when a new frame arrives from the world. */
    public void push(SensorFrame frame) { latest = frame; }

    /** The runner calls this at the top of every OpMode loop. */
    public void newLoop() {
        if (mode == LynxModule.BulkCachingMode.AUTO) cacheValid = false;
    }

    public SensorFrame read() {
        if (mode == LynxModule.BulkCachingMode.OFF) return latest;
        if (!cacheValid) {
            cached = latest;
            cacheValid = true;
        }
        return cached;
    }

    public void setBulkCachingMode(LynxModule.BulkCachingMode m) {
        mode = m;
        cacheValid = false;
    }

    public LynxModule.BulkCachingMode getBulkCachingMode() { return mode; }

    public void clearBulkCache() { cacheValid = false; }

    public double simTime() { return latest.t(); }
    public long seq() { return latest.seq(); }
    public SensorFrame latest() { return latest; }
    public double battery() { return Json.num(Json.obj(read().raw, "battery"), "volts", 12.0); }
}
