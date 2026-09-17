package com.qualcomm.robotcore.util;

public class Range {
    public static double clip(double value, double min, double max) {
        return value < min ? min : (value > max ? max : value);
    }
    public static int clip(int value, int min, int max) {
        return value < min ? min : (value > max ? max : value);
    }
    public static float clip(float value, float min, float max) {
        return value < min ? min : (value > max ? max : value);
    }
    public static double scale(double n, double x1, double x2, double y1, double y2) {
        return ((n - x1) / (x2 - x1)) * (y2 - y1) + y1;
    }
}
