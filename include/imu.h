#pragma once

#include <Arduino.h>
#include <Wire.h>

class IMU {
public:
    static bool init();
    static void calibrate();
    static void update();
    static void resetYaw();
    
    static float getYaw();       // Heading in degrees (-180.0 to +180.0)
    static float getGyroZ();     // Angular rate in deg/sec
    static bool isAvailable();

private:
    static bool available;
    static uint8_t activeAddr;
    static float yaw;
    static float gyroZ;
    static float gyroZOffset;
    static uint32_t lastUpdateUs;

    static void writeRegister(uint8_t reg, uint8_t val);
    static uint8_t readRegister(uint8_t reg);
};
