#pragma once

#include <Arduino.h>

#define MAZE_WIDTH  16
#define MAZE_HEIGHT 16

enum Direction : uint8_t {
    DIR_NORTH = 0,
    DIR_EAST  = 1,
    DIR_SOUTH = 2,
    DIR_WEST  = 3
};

enum MazeTarget : uint8_t {
    TARGET_GOAL  = 0, // Menuju kotak finish di pusat labirin (7,7)-(8,8)
    TARGET_START = 1  // Menuju kembali ke kotak start (0,0)
};

enum Action : uint8_t {
    ACTION_FORWARD = 0,
    ACTION_TURN_LEFT,
    ACTION_TURN_RIGHT,
    ACTION_TURN_AROUND,
    ACTION_STOP_GOAL
};

enum PathCommand : uint8_t {
    CMD_SPRINT = 0,          // Straight sprint for N cells (smooth acceleration/cruise)
    CMD_CURVE_LEFT = 1,      // Smooth slalom curve 90° left (gerakan mengular 90° tanpa stop)
    CMD_CURVE_RIGHT = 2,     // Smooth slalom curve 90° right (gerakan mengular 90° tanpa stop)
    CMD_PIVOT_180 = 3,       // Pivot 180° (dead end)
    CMD_DIAG_ENTER_LEFT = 4, // 45° smooth curve left into diagonal line (mengular masuk diagonal)
    CMD_DIAG_ENTER_RIGHT = 5,// 45° smooth curve right into diagonal line (mengular masuk diagonal)
    CMD_DIAG_SPRINT = 6,     // Diagonal sprint for N diagonal steps (N * 254.56mm)
    CMD_DIAG_EXIT_LEFT = 7,  // 45° smooth curve left out of diagonal back to orthogonal
    CMD_DIAG_EXIT_RIGHT = 8, // 45° smooth curve right out of diagonal back to orthogonal
    CMD_FINISH = 9           // Goal reached
};

struct PathStep {
    PathCommand cmd;
    uint8_t cellCount;       // For CMD_SPRINT: 180mm cells; For CMD_DIAG_SPRINT: 254.56mm diagonal steps
};

class Maze {
public:
    static void init();
    static void resetWalls();
    static void resetPosition();
    
    // Position & Orientation
    static uint8_t getX();
    static uint8_t getY();
    static Direction getHeading();
    static void setPosition(uint8_t x, uint8_t y, Direction heading);
    
    // Target Management (Goal vs Return-to-Start)
    static void setTarget(MazeTarget target);
    static MazeTarget getTarget();
    static bool isAtTarget();

    // Goal / Start check
    static bool isAtGoal();
    static bool isAtStart();
    static bool isMapped();

    // Wall sensing and updating
    static void updateWalls(bool frontWall, bool leftWall, bool rightWall);
    
    // Floodfill solver
    static void updateDistances();
    static Action getNextAction();
    static void applyAction(Action action);

    // Fast Run path extraction (Jalur Tercepat dengan Gerakan Mengular)
    static uint8_t generateFastestPath(PathStep* pathOut, uint8_t maxSteps = 64);

    // Flash / EEPROM Persistence
    static bool saveToFlash();
    static bool loadFromFlash();

    static void printMaze();
    static uint8_t getWallByte(uint8_t x, uint8_t y);
    static bool hasWallAt(uint8_t x, uint8_t y, Direction dir);

private:
    static uint8_t walls[MAZE_WIDTH][MAZE_HEIGHT];
    static uint16_t dist[MAZE_WIDTH][MAZE_HEIGHT];
    static uint8_t posX;
    static uint8_t posY;
    static Direction currentHeading;
    static MazeTarget currentTarget;
    static bool mazeMapped;

    static void setWall(uint8_t x, uint8_t y, Direction dir);
    static bool hasWall(uint8_t x, uint8_t y, Direction dir);
};
