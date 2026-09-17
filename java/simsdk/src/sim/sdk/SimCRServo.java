package sim.sdk;

import com.qualcomm.robotcore.hardware.CRServo;

public class SimCRServo implements CRServo {
    private final SimHub hub;
    private final String name;
    private Direction direction = Direction.FORWARD;
    private double power = 0;

    public SimCRServo(SimHub hub, String name) { this.hub = hub; this.name = name; }

    @Override public void setDirection(Direction d) { direction = d; }
    @Override public Direction getDirection() { return direction; }
    @Override public void setPower(double p) {
        power = p;
        double v = direction == Direction.REVERSE ? -p : p;
        hub.out.setServo(name, (v + 1) / 2);
    }
    @Override public double getPower() { return power; }
}
