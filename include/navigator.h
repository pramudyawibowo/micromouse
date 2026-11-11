#pragma once

#include <Arduino.h>
#include "config.h"

class Navigator {
public:
    static void init();
    
    // Core discrete movement primitives (Scan Mode / Eksplorasi Labirin Presisi)
    static bool moveForwardCell(float targetSpeed = SCAN_SPEED_FORWARD, bool brakeAtEnd = true);
    static bool turn90(bool turnLeft, float turnSpeed = SCAN_SPEED_TURN);
    static bool turn180(float turnSpeed = SCAN_SPEED_TURN);
    static void alignFrontWall();
    static bool ensureFrontWallClearance();

    // High-Speed Fast Run Primitives (Jalur Tercepat & Gerakan Mengular Slalom 90°)
    static bool smoothCurveTurn(bool turnLeft, float baseSpeed = FAST_RUN_CURVE_SPEED);
    static bool sprintDistance(float distanceMm, float cruiseSpeed = FAST_RUN_SPRINT_SPEED, bool brakeAtEnd = true);

    // Diagonal Fast Run Primitives (Gerakan Diagonal 45° & Mengular Fluid)
    static bool smoothCurveTurn45(bool turnLeft, float baseSpeed = FAST_RUN_DIAG_CURVE_SPEED);
    static bool sprintDiagonal(float numDiagonalSteps, float cruiseSpeed = FAST_RUN_DIAG_SPRINT_SPEED);

    // Auto-Alignment (Meluruskan Badan Sejajar Dinding Tanpa IMU via Trigonometri Sensor IR)
    static bool autoAlignWithWalls(bool doBackTouch = true);

    // Low-level motion control
    static void driveStraight(float distanceMm, float targetSpeed = SCAN_SPEED_FORWARD);
    static void emergencyStop();
};
