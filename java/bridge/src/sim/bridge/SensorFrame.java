package sim.bridge;

import java.util.LinkedHashMap;
import java.util.Map;

/** World -> brain, one per world frame. PLAN.md section 3.6. */
public class SensorFrame {
    public final Map<String, Object> raw;

    public SensorFrame(Map<String, Object> raw) { this.raw = raw == null ? new LinkedHashMap<String, Object>() : raw; }

    public static SensorFrame empty() { return new SensorFrame(new LinkedHashMap<String, Object>()); }

    public long seq() { return (long) Json.num(raw, "seq", 0); }
    public double t() { return Json.num(raw, "t", 0); }

    public Map<String, Object> match() { return Json.obj(raw, "match"); }
    public Map<String, Object> motor(String name) { return Json.obj(Json.obj(raw, "motors"), name); }
    public Map<String, Object> servo(String name) { return Json.obj(Json.obj(raw, "servos"), name); }
    public Map<String, Object> imu() { return Json.obj(raw, "imu"); }
    public Map<String, Object> game() { return Json.obj(raw, "game"); }
    public Map<String, Object> localizer() { return Json.obj(raw, "localizer"); }
    public Map<String, Object> gamepad(int which) { return Json.obj(raw, which == 2 ? "gamepad2" : "gamepad1"); }

    public double battery() { return Json.num(Json.obj(raw, "battery"), "volts", 12.0); }
    public double distance(String name) { return Json.num(Json.obj(raw, "distance"), name, 0); }

    public String period() { return Json.str(match(), "period", "STAGING"); }
    public boolean started() { return Json.bool(match(), "started", false); }
    public boolean stopped() { return Json.bool(match(), "stopped", false); }
}
