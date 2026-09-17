package com.qualcomm.hardware.lynx;

/**
 * SDK shim. Bulk caching is emulated for real: with AUTO every sensor read inside one
 * loop returns the same frame, so TeamCode written here survives the hub's real I/O cost.
 */
public interface LynxModule {
    enum BulkCachingMode { OFF, AUTO, MANUAL }
    void setBulkCachingMode(BulkCachingMode mode);
    BulkCachingMode getBulkCachingMode();
    void clearBulkCache();
}
