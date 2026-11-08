#pragma once

#include <Arduino.h>

struct IRSensorData {
    uint16_t left;
    uint16_t frontLeft;
    uint16_t frontCenter;
    uint16_t frontRight;
    uint16_t right;
};

class Sensors {
public:
    static void init();
    static void update();
    
    static IRSensorData getRaw();
    static IRSensorData getFiltered();

    // Wall detection booleans
    static bool hasWallFront();
    static bool hasWallLeft();
    static bool hasWallRight();

    // Alignment and error for PID wall-following
    static float getCenteringError(); // Error between left and right walls for centering
    static float getAngleError();     // Trigonometric optical tilt angle error (independent of lateral shift)

    // Calibration & Adaptive Ambient Tracking
    static void calibrateThresholds();
    static void calibrateAmbient(uint16_t sampleCount = 60);
    static void setThresholds(uint16_t frontThresh, uint16_t sideThresh);
    static uint16_t getSideThreshold();
    static uint16_t getFrontThreshold();
    static uint16_t getLeftNominal();
    static uint16_t getRightNominal();

private:
    static IRSensorData rawData;
    static IRSensorData filteredData;
    static uint16_t frontThreshold;
    static uint16_t sideThreshold;
    static uint16_t leftNominal;
    static uint16_t rightNominal;

    static uint16_t readAnalogSmooth(uint8_t pin, uint8_t samples = 4);
};
