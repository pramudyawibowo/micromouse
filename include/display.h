#pragma once

#include <Arduino.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "config.h"
#include "sensors.h"
#include "maze.h"

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1

class Display {
public:
    static bool init();
    static bool isConnected();
    static bool checkConnection(); // Hot-plug auto-detect when plugged in later

    static void showSplash(float batteryVolt);
    static void showMenu(uint8_t mode, bool isRunning, float batteryVolt, uint8_t batteryPct, uint8_t speedTier = SPEED_TIER_AGGRESSIVE);
    static void showDiagnostics(const IRSensorData& ir, bool wallL, bool wallF, bool wallR, float yaw, float distMm, float batV);
    static void showMazeStatus(uint8_t x, uint8_t y, Direction dir, Action nextAction, bool isGoal);
    static void showFastRunStep(uint8_t step, uint8_t totalSteps, PathCommand cmd, uint8_t count);
    static void showCalibration(const char* stepName, bool isDone);
    static void drawMiniMap(uint8_t robotX, uint8_t robotY, Direction heading, uint8_t startX = 64, uint8_t startY = 0);
    static void showLapResult(uint32_t lapTimeMs, uint8_t speedTier, bool isGoal);
    static void clear();

private:
    static Adafruit_SSD1306 oled;
    static bool connected;
    static uint8_t i2cAddress;
};
