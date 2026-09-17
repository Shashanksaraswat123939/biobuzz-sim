package com.qualcomm.robotcore.hardware;

/** SDK shim: public fields exactly as the real Gamepad exposes them. */
public class Gamepad {
    public float left_stick_x, left_stick_y, right_stick_x, right_stick_y;
    public float left_trigger, right_trigger;
    public boolean a, b, x, y;
    public boolean dpad_up, dpad_down, dpad_left, dpad_right;
    public boolean left_bumper, right_bumper;
    public boolean start, back, guide;
    public boolean left_stick_button, right_stick_button;

    public void rumble(int milliseconds) { }
    public void rumble(double rumble1, double rumble2, int milliseconds) { }
    public void stopRumble() { }
}
