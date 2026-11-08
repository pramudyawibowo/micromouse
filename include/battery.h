#pragma once

#include <Arduino.h>

class Battery {
public:
    static void init();
    static float getVoltage();
    static uint8_t getPercentage();
    static bool isLow();
    static bool isCritical();

private:
    static float lastVoltage;
};
