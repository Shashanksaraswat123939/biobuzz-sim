package com.qualcomm.robotcore.hardware;

import java.util.List;

/** SDK shim: the lookup surface TeamCode is allowed to use (PLAN.md Appendix E). */
public abstract class HardwareMap {
    public interface DeviceMapping<T> extends Iterable<T> { }

    public DeviceMapping<VoltageSensor> voltageSensor;

    public abstract <T> T get(Class<? extends T> classOrInterface, String deviceName);
    public abstract <T> T tryGet(Class<? extends T> classOrInterface, String deviceName);
    public abstract <T> List<T> getAll(Class<? extends T> classOrInterface);
}
