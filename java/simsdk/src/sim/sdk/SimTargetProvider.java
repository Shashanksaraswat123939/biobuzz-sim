package sim.sdk;

import org.firstinspires.ftc.teamcode.control.TargetProvider;
import sim.bridge.Json;

/**
 * The sim's answer to "where is the up CELL". It reads the world's `game` block, which
 * exists precisely so early OpModes can aim before there is a vision stack
 * (PLAN.md section 3.6). Swapping in a LimelightTargetProvider is a one-line change.
 */
public class SimTargetProvider implements TargetProvider {
    private final SimHub hub;
    public SimTargetProvider(SimHub hub) { this.hub = hub; }

    @Override public double getAzimuthDeg() { return Json.num(hub.read().game(), "upCellAzimuthDeg", 0); }
    @Override public double getRangeIn() { return Json.num(hub.read().game(), "upCellRangeIn", 0); }
    @Override public boolean isTipping() { return Json.bool(hub.read().game(), "hiveTipping", false); }
    @Override public boolean isValid() { return !hub.read().raw.isEmpty(); }
}
