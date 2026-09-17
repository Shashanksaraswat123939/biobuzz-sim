package org.firstinspires.ftc.teamcode.control;

/**
 * How many game elements the robot is really holding.
 *
 * Same shape as {@link Localizer} and {@link TargetProvider}: the sim supplies one, the hub
 * returns null from tryGet until there is a break-beam or a colour sensor to back it, and
 * {@link org.firstinspires.ftc.teamcode.subsystems.Hopper} falls back to dead reckoning.
 */
public interface BallCounter {
    int getCount();
}
