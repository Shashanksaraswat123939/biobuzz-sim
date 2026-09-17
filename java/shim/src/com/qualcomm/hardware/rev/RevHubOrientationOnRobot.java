package com.qualcomm.hardware.rev;

public class RevHubOrientationOnRobot {
    public enum LogoFacingDirection { UP, DOWN, LEFT, RIGHT, FORWARD, BACKWARD }
    public enum UsbFacingDirection { UP, DOWN, LEFT, RIGHT, FORWARD, BACKWARD }

    public final LogoFacingDirection logo;
    public final UsbFacingDirection usb;

    public RevHubOrientationOnRobot(LogoFacingDirection logo, UsbFacingDirection usb) {
        this.logo = logo; this.usb = usb;
    }
}
