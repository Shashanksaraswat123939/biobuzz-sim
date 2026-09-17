package sim.sdk;

import com.qualcomm.robotcore.hardware.Servo;
import sim.bridge.Json;

public class SimServo implements Servo {
    private final SimHub hub;
    private final String name;
    private Direction direction = Direction.FORWARD;
    private double min = 0, max = 1;
    private double commanded = 0.5;

    public SimServo(SimHub hub, String name) { this.hub = hub; this.name = name; }

    @Override public void setPosition(double position) {
        commanded = position;
        double p = min + position * (max - min);
        if (direction == Direction.REVERSE) p = 1 - p;
        hub.out.setServo(name, p < 0 ? 0 : (p > 1 ? 1 : p));
    }
    @Override public double getPosition() {
        double actual = Json.num(hub.read().servo(name), "pos", commanded);
        if (direction == Direction.REVERSE) actual = 1 - actual;
        return max > min ? (actual - min) / (max - min) : actual;
    }
    @Override public void setDirection(Direction d) { direction = d; }
    @Override public Direction getDirection() { return direction; }
    @Override public void scaleRange(double lo, double hi) { min = lo; max = hi; }
}
