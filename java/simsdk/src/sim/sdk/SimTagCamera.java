package sim.sdk;

import org.firstinspires.ftc.teamcode.control.TagCamera;
import sim.bridge.Json;

import java.util.Map;

/**
 * The world's tag pipeline, read off the wire.
 *
 * This is deliberately DUMB: it copies out the detection and nothing else. Every gate that
 * decides whether there is a detection at all -- field of view, range, incidence, the rocker
 * swinging -- lives in the world (packages/core/src/physics/tagCamera.ts), because that is
 * where a camera's limits live on a real robot too. TeamCode never imports anything from
 * here; it looks the device up by name and gets null on the hub, where an AprilTagProcessor
 * implementation of the same interface takes over.
 */
public class SimTagCamera implements TagCamera {
    private final SimHub hub;

    public SimTagCamera(SimHub hub) { this.hub = hub; }

    /** Null on the wire means the camera has never decoded the tag, which is not an object. */
    private Map<String, Object> tag() {
        Object v = hub.read().raw.get("tag");
        return v instanceof Map ? Json.obj(hub.read().raw, "tag") : null;
    }

    @Override public boolean hasFix() { return tag() != null; }
    @Override public int getId() { return (int) Json.num(tag(), "id", 0); }
    @Override public double getBearingDeg() { return Json.num(tag(), "bearingDeg", 0); }
    @Override public double getRangeIn() { return Json.num(tag(), "rangeIn", 0); }
    @Override public double getOpenDeg() { return Json.num(tag(), "openDeg", 180); }
    /** The world stamps each detection with the time the photons left, not the time it was read. */
    @Override public double getSampleTimeS() { return Json.num(tag(), "sampleT", -1e9); }
    @Override public double getTimeS() { return hub.simTime(); }
}
