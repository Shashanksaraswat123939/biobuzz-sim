package sim.sdk;

import com.qualcomm.robotcore.hardware.Gamepad;
import sim.bridge.Json;

import java.util.Map;

/** Fills the SDK's Gamepad fields from the browser's gamepad, over the bridge. */
public final class SimGamepad {
    private SimGamepad() { }

    public static void fill(Gamepad pad, Map<String, Object> src) {
        pad.left_stick_x = (float) Json.num(src, "left_stick_x", 0);
        pad.left_stick_y = (float) Json.num(src, "left_stick_y", 0);
        pad.right_stick_x = (float) Json.num(src, "right_stick_x", 0);
        pad.right_stick_y = (float) Json.num(src, "right_stick_y", 0);
        pad.left_trigger = (float) Json.num(src, "left_trigger", 0);
        pad.right_trigger = (float) Json.num(src, "right_trigger", 0);
        pad.a = Json.bool(src, "a", false);
        pad.b = Json.bool(src, "b", false);
        pad.x = Json.bool(src, "x", false);
        pad.y = Json.bool(src, "y", false);
        pad.dpad_up = Json.bool(src, "dpad_up", false);
        pad.dpad_down = Json.bool(src, "dpad_down", false);
        pad.dpad_left = Json.bool(src, "dpad_left", false);
        pad.dpad_right = Json.bool(src, "dpad_right", false);
        pad.left_bumper = Json.bool(src, "left_bumper", false);
        pad.right_bumper = Json.bool(src, "right_bumper", false);
        pad.start = Json.bool(src, "start", false);
        pad.back = Json.bool(src, "back", false);
        pad.left_stick_button = Json.bool(src, "left_stick_button", false);
        pad.right_stick_button = Json.bool(src, "right_stick_button", false);
    }
}
