package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.config.RobotConfig;
import org.firstinspires.ftc.teamcode.control.BallCounter;

/**
 * How many balls the robot thinks it is holding.
 *
 * If a real counter is available it wins; otherwise this dead-reckons from feeds and intake
 * events. Dead reckoning drifts, which is exactly why the interface exists: fit a break-beam
 * and only this class changes.
 */
public class Hopper {
    private BallCounter counter;
    private int count = 0;
    private int capacity = 6;

    public void init(HardwareMap hw, RobotConfig cfg) {
        capacity = cfg.hopperCapacity;
        counter = hw.tryGet(BallCounter.class, "hopper");
    }

    /** True when the count is measured rather than guessed. */
    public boolean isMeasured() { return counter != null; }

    public void setCount(int n) { count = n < 0 ? 0 : (n > capacity ? capacity : n); }

    public int getCount() { return counter != null ? Math.min(counter.getCount(), capacity) : count; }

    public boolean isEmpty() { return getCount() <= 0; }
    public boolean isFull() { return getCount() >= capacity; }
    public void onIntake() { if (count < capacity) count++; }
    public void onFeed() { if (count > 0) count--; }

    public void telemetry(Telemetry t) {
        t.addData("hopper", "%d / %d%s", getCount(), capacity, counter == null ? " (dead reckoned)" : "");
    }
}
