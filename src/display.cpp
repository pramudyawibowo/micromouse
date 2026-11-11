#include "display.h"
#include "config.h"
#include <Wire.h>

Adafruit_SSD1306 Display::oled(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
bool Display::connected = false;
uint8_t Display::i2cAddress = 0x3C;

bool Display::init() {
    // Check if device responds at 0x3C or 0x3D
    Wire.beginTransmission(0x3C);
    if (Wire.endTransmission() == 0) {
        i2cAddress = 0x3C;
    } else {
        Wire.beginTransmission(0x3D);
        if (Wire.endTransmission() == 0) {
            i2cAddress = 0x3D;
        } else {
            connected = false;
            return false; // OLED is not connected
        }
    }

    if (!oled.begin(SSD1306_SWITCHCAPVCC, i2cAddress)) {
        connected = false;
        return false;
    }

    connected = true;
    oled.clearDisplay();
    oled.setTextColor(SSD1306_WHITE);
    oled.display();
    return true;
}

bool Display::isConnected() {
    return connected;
}

bool Display::checkConnection() {
    // Probe primary I2C address 0x3C
    Wire.beginTransmission(0x3C);
    if (Wire.endTransmission() == 0) {
        if (!connected || i2cAddress != 0x3C) {
            i2cAddress = 0x3C;
            return init();
        }
        return true;
    }

    // Probe secondary I2C address 0x3D
    Wire.beginTransmission(0x3D);
    if (Wire.endTransmission() == 0) {
        if (!connected || i2cAddress != 0x3D) {
            i2cAddress = 0x3D;
            return init();
        }
        return true;
    }

    // Device not responding
    connected = false;
    return false;
}

void Display::clear() {
    if (!connected) return;
    oled.clearDisplay();
}

void Display::showSplash(float batteryVolt) {
    if (!connected) return;
    oled.clearDisplay();
    oled.setTextSize(1);
    oled.setCursor(25, 8);
    oled.println(F("STM32F411CEU6"));
    oled.setTextSize(2);
    oled.setCursor(8, 22);
    oled.println(F("MICROMOUSE"));
    oled.setTextSize(1);
    oled.setCursor(40, 42);
    oled.println(F("v0.1 F411"));
    oled.drawFastHLine(0, 54, 128, SSD1306_WHITE);
    oled.setCursor(24, 56);
    oled.print(F("BAT: "));
    oled.print(batteryVolt, 2);
    oled.println(F("V"));
    oled.display();
}

void Display::showMenu(uint8_t mode, bool isRunning, float batteryVolt, uint8_t batteryPct, uint8_t speedTier) {
    if (!connected) return;
    oled.clearDisplay();

    // Top status bar
    oled.setTextSize(1);
    oled.setCursor(0, 0);
    oled.print(F("MODE: "));
    oled.print(mode);
    oled.print(F(" ["));
    oled.print(batteryVolt, 1);
    oled.print(F("V "));
    oled.print(batteryPct);
    oled.println(F("%]"));
    oled.drawFastHLine(0, 9, 128, SSD1306_WHITE);

    // Mode title
    oled.setCursor(0, 14);
    switch (mode) {
        case 1:
            oled.println(F(">> DIAGNOSTICS <<"));
            oled.setCursor(0, 26);
            oled.println(F("Live Sensor & Motor"));
            oled.setCursor(0, 36);
            oled.println(F("Telemetry stream"));
            break;
        case 2:
            oled.println(F(">> WALL FOLLOWER <<"));
            oled.setCursor(0, 26);
            oled.println(F("Reactive left/right"));
            oled.setCursor(0, 36);
            oled.println(F("corridor navigation"));
            break;
        case 3:
            oled.println(F(">> SCAN & SAVE MAZE <<"));
            oled.setCursor(0, 26);
            oled.println(F("Floodfill mapping"));
            oled.setCursor(0, 36);
            oled.println(F("Explore & Save Flash"));
            break;
        case 4:
            oled.println(F(">> FAST RUN (MENGULAR) <<"));
            oled.setCursor(0, 26);
            if (speedTier == SPEED_TIER_SAFE) {
                oled.println(F("Speed: [TIER 1: SAFE]"));
                oled.setCursor(0, 36);
                oled.println(F("180 PWM sprint (0% slip)"));
            } else if (speedTier == SPEED_TIER_INSANE) {
                oled.println(F("Speed: [TIER 3: INSANE]"));
                oled.setCursor(0, 36);
                oled.println(F("250 PWM max attack"));
            } else {
                oled.println(F("Speed: [TIER 2: AGGR]"));
                oled.setCursor(0, 36);
                oled.println(F("220 PWM sprint (Fast)"));
            }
            break;
        case 5:
            oled.println(F(">> CALIBRATION <<"));
            oled.setCursor(0, 26);
            oled.println(F("Measure IR ambient"));
            oled.setCursor(0, 36);
            oled.println(F("& Gyro Z zero-bias"));
            break;
        default:
            oled.println(F(">> IDLE <<"));
            break;
    }

    // Bottom action bar
    oled.drawFastHLine(0, 48, 128, SSD1306_WHITE);
    oled.setCursor(0, 52);
    if (isRunning) {
        oled.print(F("* RUNNING * [SW3:EMG]"));
    } else {
        if (mode == 4) {
            oled.print(F("SW4:GO | SW2:TIER"));
        } else {
            oled.print(F("READY [SW4 to START]"));
        }
    }
    oled.display();
}

void Display::showDiagnostics(const IRSensorData& ir, bool wallL, bool wallF, bool wallR, float yaw, float distMm, float batV) {
    if (!connected) return;
    oled.clearDisplay();

    // Header
    oled.setTextSize(1);
    oled.setCursor(0, 0);
    oled.print(F("DIAG: "));
    oled.print(batV, 2);
    oled.print(F("V | "));
    oled.print(yaw, 0);
    oled.println(F("deg"));

    // Wall status
    oled.setCursor(0, 10);
    oled.print(F("WALLS: ["));
    oled.print(wallL ? "L " : ". ");
    oled.print(wallF ? "F " : ". ");
    oled.print(wallR ? "R]" : ".]");
    oled.print(F(" D:"));
    oled.print((int)distMm);
    oled.println(F("mm"));

    // IR Sensor Bar Graphs
    // Sensors: L, FL, FC, FR, R (each max 4095, mapped to height 0..28)
    const uint16_t vals[5] = {ir.left, ir.frontLeft, ir.frontCenter, ir.frontRight, ir.right};
    const uint8_t xPos[5] = {10, 34, 58, 82, 106};

    oled.drawFastHLine(0, 22, 128, SSD1306_WHITE);
    for (int i = 0; i < 5; i++) {
        uint8_t barH = map(constrain(vals[i], 0, 4000), 0, 4000, 0, 28);
        oled.fillRect(xPos[i], 56 - barH, 12, barH, SSD1306_WHITE);
    }

    // Labels under bars
    oled.setCursor(12, 57); oled.print(F("L"));
    oled.setCursor(34, 57); oled.print(F("FL"));
    oled.setCursor(58, 57); oled.print(F("FC"));
    oled.setCursor(82, 57); oled.print(F("FR"));
    oled.setCursor(108, 57); oled.print(F("R"));

    oled.display();
}

void Display::showMazeStatus(uint8_t x, uint8_t y, Direction dir, Action nextAction, bool isGoal) {
    if (!connected) return;
    oled.clearDisplay();

    oled.setTextSize(1);
    oled.setCursor(0, 0);
    if (Maze::getTarget() == TARGET_START) {
        oled.print(F("RETURN: "));
        if (isGoal) {
            oled.println(F("AT START!"));
        } else {
            oled.println(F("TO START"));
        }
    } else {
        oled.print(F("SCAN: "));
        if (isGoal) {
            oled.println(F("GOAL REACH!"));
        } else {
            oled.println(F("TO GOAL"));
        }
    }
    oled.drawFastHLine(0, 9, 128, SSD1306_WHITE);

    // Coordinates and direction
    oled.setTextSize(2);
    oled.setCursor(0, 16);
    oled.print(F("("));
    oled.print(x);
    oled.print(F(","));
    oled.print(y);
    oled.print(F(") "));

    const char* dirStr[4] = {"N", "E", "S", "W"};
    oled.println(dirStr[dir]);

    // Next action
    oled.setTextSize(1);
    oled.setCursor(0, 40);
    oled.print(F("Action: "));
    switch (nextAction) {
        case ACTION_FORWARD: oled.println(F("MOVE FORWARD")); break;
        case ACTION_TURN_LEFT: oled.println(F("TURN LEFT")); break;
        case ACTION_TURN_RIGHT: oled.println(F("TURN RIGHT")); break;
        case ACTION_TURN_AROUND: oled.println(F("TURN 180 (BACK)")); break;
        case ACTION_STOP_GOAL: oled.println(F("SOLVED / STOP")); break;
    }

    oled.drawFastHLine(0, 52, 128, SSD1306_WHITE);
    oled.setCursor(0, 55);
    oled.print(F("SW3: Emergency Brake"));
    oled.display();
}

void Display::showFastRunStep(uint8_t step, uint8_t totalSteps, PathCommand cmd, uint8_t count) {
    if (!connected) return;
    oled.clearDisplay();
    oled.setTextSize(1);
    oled.setCursor(0, 0);
    oled.print(F("FAST RUN ["));
    oled.print(step + 1);
    oled.print(F("/"));
    oled.print(totalSteps);
    oled.println(F("]"));
    oled.drawFastHLine(0, 9, 128, SSD1306_WHITE);

    oled.setTextSize(2);
    oled.setCursor(0, 18);
    switch (cmd) {
        case CMD_SPRINT:
            oled.print(F("SPRINT "));
            oled.print(count);
            oled.println(F("x"));
            oled.setTextSize(1);
            oled.setCursor(0, 38);
            oled.print(F("Distance: "));
            oled.print(count * 180);
            oled.println(F(" mm"));
            break;
        case CMD_CURVE_LEFT:
            oled.println(F("MENGULAR <"));
            oled.setTextSize(1);
            oled.setCursor(0, 38);
            oled.println(F("Slalom Curve Left 90"));
            break;
        case CMD_CURVE_RIGHT:
            oled.println(F("MENGULAR >"));
            oled.setTextSize(1);
            oled.setCursor(0, 38);
            oled.println(F("Slalom Curve Right 90"));
            break;
        case CMD_DIAG_ENTER_LEFT:
            oled.println(F("DIAG ENT <"));
            oled.setTextSize(1);
            oled.setCursor(0, 38);
            oled.println(F("Curve 45 Left -> Diag"));
            break;
        case CMD_DIAG_ENTER_RIGHT:
            oled.println(F("DIAG ENT >"));
            oled.setTextSize(1);
            oled.setCursor(0, 38);
            oled.println(F("Curve 45 Right -> Diag"));
            break;
        case CMD_DIAG_SPRINT:
            oled.print(F("DIAG "));
            oled.print(count);
            oled.println(F("x"));
            oled.setTextSize(1);
            oled.setCursor(0, 38);
            oled.print(F("Sprint: "));
            oled.print((int)(count * CELL_DIAGONAL_MM));
            oled.println(F(" mm"));
            break;
        case CMD_DIAG_EXIT_LEFT:
            oled.println(F("DIAG EXT <"));
            oled.setTextSize(1);
            oled.setCursor(0, 38);
            oled.println(F("Diag Exit -> 45 Left"));
            break;
        case CMD_DIAG_EXIT_RIGHT:
            oled.println(F("DIAG EXT >"));
            oled.setTextSize(1);
            oled.setCursor(0, 38);
            oled.println(F("Diag Exit -> 45 Right"));
            break;
        case CMD_PIVOT_180:
            oled.println(F("PIVOT 180"));
            oled.setTextSize(1);
            oled.setCursor(0, 38);
            oled.println(F("Turn Around"));
            break;
        case CMD_FINISH:
            oled.println(F("GOAL REACHED"));
            oled.setTextSize(1);
            oled.setCursor(0, 38);
            oled.println(F("Maximum Speed Finish!"));
            break;
    }

    oled.drawFastHLine(0, 52, 128, SSD1306_WHITE);
    oled.setCursor(0, 55);
    oled.print(F("SW3: Emergency Brake"));
    oled.display();
}

void Display::showCalibration(const char* stepName, bool isDone) {
    if (!connected) return;
    oled.clearDisplay();
    oled.setTextSize(1);
    oled.setCursor(12, 8);
    oled.println(F(">> CALIBRATION <<"));
    oled.drawFastHLine(0, 18, 128, SSD1306_WHITE);

    oled.setCursor(0, 26);
    oled.println(stepName);

    oled.setCursor(0, 44);
    if (isDone) {
        oled.println(F("STATUS: SUCCESS!"));
    } else {
        oled.println(F("Please keep still..."));
    }
    oled.display();
}

void Display::drawMiniMap(uint8_t robotX, uint8_t robotY, Direction heading, uint8_t startX, uint8_t startY) {
    if (!connected) return;

    // Hitung ukuran sel piksel dinamis agar pas memenuhi area layar 64x64
    const uint8_t maxDim = (MAZE_WIDTH > MAZE_HEIGHT) ? MAZE_WIDTH : MAZE_HEIGHT;
    const uint8_t cellPx = (maxDim > 0) ? (64 / maxDim) : 4;
    const uint8_t mazePixW = MAZE_WIDTH * cellPx;
    const uint8_t mazePixH = MAZE_HEIGHT * cellPx;

    // Garis batas luar labirin
    oled.drawRect(startX, startY, mazePixW, mazePixH, SSD1306_WHITE);

    // Kotak finish / goal di mini-map
    int16_t gx = startX + (MAZE_GOAL_MIN_X * cellPx);
    int16_t gy = startY + (MAZE_HEIGHT - 1 - MAZE_GOAL_MAX_Y) * cellPx;
    int16_t gw = (MAZE_GOAL_MAX_X - MAZE_GOAL_MIN_X + 1) * cellPx;
    int16_t gh = (MAZE_GOAL_MAX_Y - MAZE_GOAL_MIN_Y + 1) * cellPx;
    oled.drawRect(gx, gy, gw, gh, SSD1306_WHITE);

    // Gambar seluruh dinding yang terpetakan
    for (uint8_t x = 0; x < MAZE_WIDTH; x++) {
        for (uint8_t y = 0; y < MAZE_HEIGHT; y++) {
            uint8_t w = Maze::getWallByte(x, y);
            if (w == 0) continue;
            int16_t px = startX + (x * cellPx);
            int16_t py = startY + (MAZE_HEIGHT - 1 - y) * cellPx;

            if (w & (1 << DIR_NORTH)) {
                oled.drawFastHLine(px, py, cellPx, SSD1306_WHITE);
            }
            if (w & (1 << DIR_EAST)) {
                oled.drawFastVLine(px + cellPx - 1, py, cellPx, SSD1306_WHITE);
            }
            if (w & (1 << DIR_SOUTH)) {
                oled.drawFastHLine(px, py + cellPx - 1, cellPx, SSD1306_WHITE);
            }
            if (w & (1 << DIR_WEST)) {
                oled.drawFastVLine(px, py, cellPx, SSD1306_WHITE);
            }
        }
    }

    // Gambar posisi robot (ukuran titik menyesuaikan skala)
    uint8_t robotDot = (cellPx >= 8) ? (cellPx / 2) : 2;
    int16_t rx = startX + (robotX * cellPx) + (cellPx - robotDot) / 2;
    int16_t ry = startY + (MAZE_HEIGHT - 1 - robotY) * cellPx + (cellPx - robotDot) / 2;
    oled.fillRect(rx, ry, robotDot, robotDot, SSD1306_WHITE);
}

void Display::showLapResult(uint32_t lapTimeMs, uint8_t speedTier, bool isGoal) {
    if (!connected) return;
    oled.clearDisplay();

    // Sisi Kiri: Waktu Tempuh & Info Run
    oled.setTextSize(1);
    oled.setCursor(0, 2);
    if (isGoal) {
        oled.println(F("FINISH!"));
    } else {
        oled.println(F("START (0,0)"));
    }

    oled.drawFastHLine(0, 11, 62, SSD1306_WHITE);

    oled.setCursor(0, 15);
    oled.println(F("TIME:"));
    oled.setCursor(0, 26);
    float sec = (float)lapTimeMs / 1000.0f;
    oled.print(sec, 3);
    oled.println(F("s"));

    oled.setCursor(0, 40);
    oled.print(F("T:"));
    if (speedTier == SPEED_TIER_SAFE) oled.println(F("SAFE"));
    else if (speedTier == SPEED_TIER_INSANE) oled.println(F("INSANE"));
    else oled.println(F("AGGR"));

    oled.setCursor(0, 53);
    oled.println(F("SW4:MENU"));

    // Sisi Kanan: Mini-Map Labirin 16x16
    drawMiniMap(Maze::getX(), Maze::getY(), Maze::getHeading(), 64, 0);

    oled.display();
}


