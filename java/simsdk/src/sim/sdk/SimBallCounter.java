package sim.sdk;

import org.firstinspires.ftc.teamcode.control.BallCounter;
import sim.bridge.Json;

/** The world knows exactly how many balls the robot is carrying, so it just says. */
public class SimBallCounter implements BallCounter {
    private final SimHub hub;
    public SimBallCounter(SimHub hub) { this.hub = hub; }
    @Override public int getCount() { return (int) Math.round(Json.num(hub.read().game(), "hopper", 0)); }
}
