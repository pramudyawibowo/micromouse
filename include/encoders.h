#pragma once

#include <Arduino.h>

class Encoders {
public:
    static void init();
    static void reset();
    
    static int32_t getLeftTicks();
    static int32_t getRightTicks();
    
    static float getLeftDistanceMm();
    static float getRightDistanceMm();
    static float getAverageDistanceMm();

    // Encoder availability (Dual-mode support for N20 motors with/without encoders)
    static bool isAvailable();
    static void setAvailable(bool avail);
    static void notifyMovement(bool hasPulses);

    // ISR Callbacks
    static void handleLeftInterrupt();
    static void handleRightInterrupt();

private:
    static volatile int32_t leftCount;
    static volatile int32_t rightCount;
    static bool available;
    static bool checked;
};
