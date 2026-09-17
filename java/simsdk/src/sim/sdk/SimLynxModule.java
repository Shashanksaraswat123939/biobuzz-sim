package sim.sdk;

import com.qualcomm.hardware.lynx.LynxModule;

public class SimLynxModule implements LynxModule {
    private final SimHub hub;
    public SimLynxModule(SimHub hub) { this.hub = hub; }
    @Override public void setBulkCachingMode(BulkCachingMode mode) { hub.setBulkCachingMode(mode); }
    @Override public BulkCachingMode getBulkCachingMode() { return hub.getBulkCachingMode(); }
    @Override public void clearBulkCache() { hub.clearBulkCache(); }
}
