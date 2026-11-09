#include <Arduino.h>
#include "config.h"
#include "buzzer.h"
#include "motors.h"
#include "encoders.h"
#include "sensors.h"
#include "imu.h"
#include "battery.h"
#include "ui.h"
#include "navigator.h"
#include "maze.h"
#include "display.h"

enum OperatingMode : uint8_t {
    MODE_IDLE = 0,
    MODE_TEST_DIAGNOSTICS = 1,
    MODE_WALL_FOLLOWER = 2,
    MODE_SCAN_MAZE = 3,
    MODE_FAST_RUN = 4,
    MODE_CALIBRATE = 5
};

static OperatingMode currentMode = MODE_IDLE;
static uint8_t selectedMenuMode = 1;
static uint8_t fastRunSpeedTier = SPEED_TIER_AGGRESSIVE; // Default: Tier 2 (Agresif Cepat)
static uint32_t lapStartTime = 0;
static bool running = false;
static uint32_t lastDiagPrint = 0;
static uint32_t lastDisplayUpdate = 0;

// Forward declarations
void printMenu();
void runDiagnosticsStep();
void runWallFollowerStep();
void runScanMazeStep();
void runFastRun();
void returnToStartCalmly();
void runCalibration();
void startSelectedMode();

void printMenu() {
    Serial.println(F("\n=============================================="));
    Serial.println(F("         MICROMOUSE v0.1 (STM32F411)          "));
    Serial.println(F("=============================================="));
    Serial.print(F("Battery Voltage: "));
    Serial.print(Battery::getVoltage(), 2);
    Serial.print(F("V ("));
    Serial.print(Battery::getPercentage());
    Serial.println(F("%)"));
    Serial.print(F("Learned Maze Saved in Memory: "));
    Serial.println(Maze::isMapped() ? F("YES [READY FOR FAST RUN]") : F("NO [SCAN REQUIRED]"));
    Serial.println(F("Controls:"));
    Serial.println(F("  [SW1] - Cycle Operating Mode (1-5)"));
    Serial.println(F("  [SW2] - Speed Tier (Mode 4) / Sensor Calibration"));
    Serial.println(F("  [SW3] - EMERGENCY BRAKE / Rem Darurat"));
    Serial.println(F("  [SW4] - START / STOP Selected Mode"));
    Serial.println(F("Serial Terminal Commands:"));
    Serial.println(F("  '1' atau 'diag' : Buka Diagnostic sensor & battery"));
    Serial.println(F("  '2' atau 'wall' : Mode Wall Follower"));
    Serial.println(F("  '3' atau 'scan' : Mode Scan Maze"));
    Serial.println(F("  '4' atau 'fast' : Mode Fast Run"));
    Serial.println(F("  '5' atau 'cal'  : Mode Kalibrasi"));
    Serial.println(F("  'motor' / 'test': Tes putar kedua motor maju 1.5 detik"));
    Serial.println(F("  's' atau 'stop' : Hentikan robot"));
    Serial.println(F("  'map'           : Tampilkan peta labirin ASCII"));
    Serial.println(F("  'sim'           : Tes simulasi solver Floodfill"));
    Serial.println(F("  'buzzer on/off' : Toggle suara buzzer"));
    Serial.println(F("  'help' / 'menu' : Tampilkan menu ini"));
    Serial.println(F("\nAvailable Modes:"));
    Serial.println(F("  1: Hardware & Sensor Diagnostics (Live Stream)"));
    Serial.println(F("  2: Basic Wall Follower"));
    Serial.println(F("  3: Scan Maze (Floodfill Mapping -> Save to Flash)"));
    Serial.println(F("  4: FAST RUN (Jalur Tercepat & Gerakan Mengular)"));
    Serial.println(F("  5: In-Situ Sensor & Gyro Calibration"));
    Serial.println(F("----------------------------------------------"));
    Serial.print(F("Selected: Mode "));
    Serial.print(selectedMenuMode);
    if (selectedMenuMode == MODE_FAST_RUN) {
        Serial.print(F(" [TIER "));
        Serial.print(fastRunSpeedTier);
        Serial.print(F("]"));
    }
    Serial.println();

    if (Display::isConnected()) {
        Display::showMenu(selectedMenuMode, running, Battery::getVoltage(), Battery::getPercentage(), fastRunSpeedTier);
    }
}

void setup() {
#if ENABLE_SERIAL_DEBUG
    Serial.begin(115200);

    // Tunggu koneksi Serial Monitor aktif (maksimal 2.5 detik agar tidak hang jika offline)
    uint32_t serialWaitStart = millis();
    while (!Serial && (millis() - serialWaitStart < 2500)) {
        delay(10);
    }
#else
    // Matikan USB CDC peripheral agar pin PA11 dan PA12 terlepas menjadi GPIO digital normal
    Serial.end();
#endif

    // Kunci ADC STM32 ke native 12-bit (0 - 4095) agar pembacaan Battery dan Sensor IR akurat
    analogReadResolution(12);

    // Initialize hardware modules
    UI::init();
    Buzzer::init();
    Battery::init();
    Motors::init();
    Encoders::init();
    Sensors::init();
    IMU::init();
    Display::init();
    Maze::init();

    // Otomatis pilih mode default cerdas:
    // Jika peta labirin sudah tersimpan di Flash -> Langsung MODE 4 (Fast Run)
    // Jika peta labirin belum ada -> Langsung MODE 3 (Scan Maze)
    if (Maze::isMapped()) {
        selectedMenuMode = MODE_FAST_RUN;
    } else {
        selectedMenuMode = MODE_SCAN_MAZE;
    }

    // Welcome sound & visual feedback
    if (Display::isConnected()) {
        Display::showSplash(Battery::getVoltage());
    }
    UI::blinkAll(2, 80);
    Buzzer::playStartup();

#if ENABLE_LEDS
    if (selectedMenuMode == MODE_SCAN_MAZE) {
        UI::setLed1(true);
        UI::setLed2(true);
    } else if (selectedMenuMode == MODE_FAST_RUN) {
        UI::setLed1(false);
        UI::setLed2(false);
    }
#endif

    if (Display::isConnected()) {
        delay(800);
        Display::showMenu(selectedMenuMode, running, Battery::getVoltage(), Battery::getPercentage(), fastRunSpeedTier);
    }

    printMenu();
}

void runDiagnostics() {
    Sensors::update();
    IMU::update();

    if (millis() - lastDiagPrint > 100) {
        lastDiagPrint = millis();
        IRSensorData ir = Sensors::getFiltered();

        Serial.print(F("BAT: "));
        Serial.print(Battery::getVoltage(), 2);
        Serial.print(F("V | IR [L:"));
        Serial.print(ir.left);
        Serial.print(F(" FL:"));
        Serial.print(ir.frontLeft);
        Serial.print(F(" FC:"));
        Serial.print(ir.frontCenter);
        Serial.print(F(" FR:"));
        Serial.print(ir.frontRight);
        Serial.print(F(" R:"));
        Serial.print(ir.right);
        Serial.print(F("] | WALLS: ["));
        Serial.print(Sensors::hasWallLeft() ? "L " : ". ");
        Serial.print(Sensors::hasWallFront() ? "F " : ". ");
        Serial.print(Sensors::hasWallRight() ? "R" : ".");
        Serial.print(F("] | ENC: "));
        if (Encoders::isAvailable()) {
            Serial.print(F("[L:"));
            Serial.print(Encoders::getLeftTicks());
            Serial.print(F(" R:"));
            Serial.print(Encoders::getRightTicks());
            Serial.print(F(" ("));
            Serial.print(Encoders::getAverageDistanceMm(), 1);
            Serial.print(F("mm)]"));
        } else {
            Serial.print(F("[N20 NO-ENCODER]"));
        }
        Serial.print(F(" | YAW: "));
        Serial.print(IMU::getYaw(), 1);
        Serial.println(F(" deg"));

#if ENABLE_LEDS
        UI::setLed1(Sensors::hasWallLeft());
        UI::setLed2(Sensors::hasWallRight());
#endif
        UI::setOnboardLed(Sensors::hasWallFront());

        if (Display::isConnected()) {
            Display::showDiagnostics(
                ir,
                Sensors::hasWallLeft(),
                Sensors::hasWallFront(),
                Sensors::hasWallRight(),
                IMU::getYaw(),
                Encoders::getAverageDistanceMm(),
                Battery::getVoltage()
            );
        }
    }
}

void runWallFollower() {
    Sensors::update();
    IMU::update();

    // Cek emergency stop di setiap langkah wall follower
    if (UI::checkEmergencyStop()) {
        Navigator::emergencyStop();
        running = false;
        return;
    }

    if (Sensors::hasWallFront()) {
        Navigator::emergencyStop();
        Buzzer::beep(1500, 40);
        delay(80);

        if (!Sensors::hasWallLeft()) {
            Navigator::turn90(true, SCAN_SPEED_TURN);
        } else if (!Sensors::hasWallRight()) {
            Navigator::turn90(false, SCAN_SPEED_TURN);
        } else {
            Navigator::turn180(SCAN_SPEED_TURN);
        }
        delay(50);
    } else {
        Navigator::moveForwardCell(SCAN_SPEED_FORWARD);
        delay(40);
    }
}

// ==============================================================================
// MODE 3: SCAN MAZE (EKSPLORASI FLOODFILL HINGGA FINISH & SIMPAN DI MEMORY FLASH)
// Mode ini fokus pada PRESISI TINGGI & PEMETAAN DINDING TANPA GERAKAN MENGULAR.
// Kecepatan tenang/sedang (SCAN_SPEED_FORWARD) agar pembacaan dinding akurat 100%.
// ==============================================================================
void runScanMazeStep() {
    Sensors::update();
    IMU::update();

    // 1. FASE 1: Jika robot sedang menuju GOAL dan berhasil sampai di Finish (7,7)-(8,8)
    if (Maze::getTarget() == TARGET_GOAL && Maze::isAtGoal()) {
        Navigator::emergencyStop();
        UI::blinkAll(4, 70);
        Buzzer::beep(2000, 100);
        delay(120);
        Buzzer::beep(2400, 200);

        uint32_t lapDuration = millis() - lapStartTime;
        Serial.println(F("\n=============================================="));
        Serial.println(F(">>> FINISH (GOAL) TERCAPAI! PETA DISIMPAN <<<"));
        Serial.print(F("WAKTU SCAN MENUJU GOAL: "));
        Serial.print((float)lapDuration / 1000.0f, 3);
        Serial.println(F(" detik"));
        Serial.println(F("Memulai perjalanan kembali ke START (0,0)..."));
        Serial.println(F("Kecepatan: Sedang (~130 mm/s) agar presisi & aman"));
        Serial.println(F("=============================================="));

        Maze::saveToFlash();
        Maze::printMaze();

        if (Display::isConnected()) {
            Display::showLapResult(lapDuration, fastRunSpeedTier, true);
            delay(1500);
        }

        // Beralih target Floodfill ke Start (0,0)
        Maze::setTarget(TARGET_START);
        delay(800);
        return;
    }

    // 2. FASE 2: Jika robot sedang kembali ke START dan telah sampai di sel (0,0)
    if (Maze::getTarget() == TARGET_START && Maze::isAtStart()) {
        Navigator::emergencyStop();

        // Putar robot menghadap UTARA (DIR_NORTH) agar siap di kotak start
        Direction h = Maze::getHeading();
        if (h == DIR_SOUTH) {
            Navigator::turn180(RETURN_SPEED_TURN);
            Maze::setPosition(0, 0, DIR_NORTH);
        } else if (h == DIR_WEST) {
            Navigator::turn90(false, RETURN_SPEED_TURN); // Belok kanan menghadap Utara
            Maze::setPosition(0, 0, DIR_NORTH);
        } else if (h == DIR_EAST) {
            Navigator::turn90(true, RETURN_SPEED_TURN);  // Belok kiri menghadap Utara
            Maze::setPosition(0, 0, DIR_NORTH);
        }

        uint32_t lapDuration = millis() - lapStartTime;
        UI::blinkAll(6, 60);
        Buzzer::playVictory();

        Serial.println(F("\n========================================================"));
        Serial.println(F(">>> BERHASIL KEMBALI DI START (0,0) DENGAN AMAN! <<<"));
        Serial.print(F("TOTAL WAKTU PULANG KE START: "));
        Serial.print((float)lapDuration / 1000.0f, 3);
        Serial.println(F(" detik"));
        Serial.println(F("Orientasi telah disetel menghadap UTARA."));
        Serial.println(F("Peta labirin lengkap telah disimpan ke Flash Memory."));
        Serial.println(F("Sekarang robot SIAP MENJALANKAN MODE 4 (FAST RUN)!"));
        Serial.println(F("========================================================"));

        Maze::saveToFlash();
        Maze::printMaze();

        if (Display::isConnected()) {
            Display::showLapResult(lapDuration, fastRunSpeedTier, false);
            delay(2000);
        }

        // Reset target kembali ke GOAL untuk siklus berikutnya
        Maze::setTarget(TARGET_GOAL);
        selectedMenuMode = MODE_FAST_RUN; // Otomatis siap MODE 4 (Fast Run) untuk start berikutnya!
        running = false;
        return;
    }

    // 3. Baca sensor IR dan simpan dinding yang terdeteksi (memperkaya peta di sepanjang perjalanan)
    Sensors::update();
    IMU::update();
    bool front = Sensors::hasWallFront();
    bool left  = Sensors::hasWallLeft();
    bool right = Sensors::hasWallRight();
    Maze::updateWalls(front, left, right);

    // 4. Tentukan langkah optimal dari Floodfill
    Action next = Maze::getNextAction();

    // Kecepatan tenang: Saat pulang ke start gunakan RETURN_SPEED (80 PWM), bukan kecepatan penuh!
    float fwdSpeed  = (Maze::getTarget() == TARGET_START) ? RETURN_SPEED_FORWARD : SCAN_SPEED_FORWARD;
    float turnSpeed = (Maze::getTarget() == TARGET_START) ? RETURN_SPEED_TURN : SCAN_SPEED_TURN;

    // Kurangi frekuensi display update agar tidak membebani I2C bus (tiap ~500ms saja)
    static uint32_t lastScanDisplayUpdate = 0;
    if (Display::isConnected() && (millis() - lastScanDisplayUpdate > 500)) {
        lastScanDisplayUpdate = millis();
        Display::showMazeStatus(Maze::getX(), Maze::getY(), Maze::getHeading(), next, false);
    }

    Serial.print(F("Pos ("));
    Serial.print(Maze::getX());
    Serial.print(F(","));
    Serial.print(Maze::getY());
    Serial.print(F(") Dir: "));
    Serial.print(Maze::getHeading());
    Serial.print(Maze::getTarget() == TARGET_START ? F(" [RETURN->START] Action: ") : F(" [SCAN->GOAL] Action: "));

    switch (next) {
        case ACTION_FORWARD:
            Serial.println(F("FORWARD"));
            if (!Navigator::moveForwardCell(fwdSpeed, true)) {
                running = false;
                return;
            }
            Maze::applyAction(ACTION_FORWARD);
            break;

        case ACTION_TURN_LEFT:
            Serial.println(F("TURN LEFT 90 -> FORWARD"));
            if (!Navigator::turn90(true, turnSpeed)) {
                running = false;
                return;
            }
            Maze::applyAction(ACTION_TURN_LEFT);
            if (!Navigator::moveForwardCell(fwdSpeed, true)) {
                running = false;
                return;
            }
            Maze::applyAction(ACTION_FORWARD);
            break;

        case ACTION_TURN_RIGHT:
            Serial.println(F("TURN RIGHT 90 -> FORWARD"));
            if (!Navigator::turn90(false, turnSpeed)) {
                running = false;
                return;
            }
            Maze::applyAction(ACTION_TURN_RIGHT);
            if (!Navigator::moveForwardCell(fwdSpeed, true)) {
                running = false;
                return;
            }
            Maze::applyAction(ACTION_FORWARD);
            break;

        case ACTION_TURN_AROUND:
            Serial.println(F("TURN 180 -> FORWARD"));
            if (!Navigator::turn180(turnSpeed)) {
                running = false;
                return;
            }
            Maze::applyAction(ACTION_TURN_AROUND);
            if (!Navigator::moveForwardCell(fwdSpeed, true)) {
                running = false;
                return;
            }
            Maze::applyAction(ACTION_FORWARD);
            break;

        case ACTION_STOP_GOAL:
            Navigator::emergencyStop();
            running = false;
            break;
    }

    if (UI::checkEmergencyStop()) {
        Navigator::emergencyStop();
        running = false;
        return;
    }
}

// ==============================================================================
// MODE 4: FAST RUN (JALUR TERCEPAT DENGAN GERAKAN MENGULAR / SLALOM CURVE)
// Mode ini fokus pada KECEPATAN MAKSIMAL. Membaca peta yang sudah tersimpan di
// Flash Memory, lalu menjalankan sprint multi-sel dan belokan mengular tanpa henti.
// ==============================================================================
void runFastRun() {
    Serial.println(F("\n=============================================="));
    Serial.println(F(">>> STARTING FAST RUN (JALUR TERCEPAT) <<<"));
    Serial.println(F("Menggunakan kompresi sprint lurus & belokan mengular (slalom)"));
    Serial.print(F("Speed Tier Aktif: "));
    Serial.print(fastRunSpeedTier);
    if (fastRunSpeedTier == SPEED_TIER_SAFE) Serial.println(F(" [TIER 1: SAFE]"));
    else if (fastRunSpeedTier == SPEED_TIER_AGGRESSIVE) Serial.println(F(" [TIER 2: AGGR]"));
    else if (fastRunSpeedTier == SPEED_TIER_INSANE) Serial.println(F(" [TIER 3: INSANE]"));
    Serial.println(F("=============================================="));

    if (!Maze::isMapped()) {
        Serial.println(F("PERINGATAN: Peta labirin belum tersimpan di memori!"));
        Serial.println(F("Silakan jalankan MODE 3 (Scan Maze) terlebih dahulu."));
        Buzzer::playAlert();
        running = false;
        return;
    }

    // Set kecepatan dinamis sesuai Tier terpilih
    float sprintSpd     = TIER2_SPRINT_SPEED;
    float curveSpd      = TIER2_CURVE_SPEED;
    float diagSprintSpd = TIER2_DIAG_SPRINT_SPEED;
    float diagCurveSpd  = TIER2_DIAG_CURVE_SPEED;

    if (fastRunSpeedTier == SPEED_TIER_SAFE) {
        sprintSpd     = TIER1_SPRINT_SPEED;
        curveSpd      = TIER1_CURVE_SPEED;
        diagSprintSpd = TIER1_DIAG_SPRINT_SPEED;
        diagCurveSpd  = TIER1_DIAG_CURVE_SPEED;
    } else if (fastRunSpeedTier == SPEED_TIER_INSANE) {
        sprintSpd     = TIER3_SPRINT_SPEED;
        curveSpd      = TIER3_CURVE_SPEED;
        diagSprintSpd = TIER3_DIAG_SPRINT_SPEED;
        diagCurveSpd  = TIER3_DIAG_CURVE_SPEED;
    }

    PathStep path[64];
    uint8_t totalSteps = Maze::generateFastestPath(path, 64);

    Serial.print(F("Total Langkah Jalur Tercepat: "));
    Serial.println(totalSteps);

    for (uint8_t i = 0; i < totalSteps; i++) {
        if (!running) break;

        if (Display::isConnected()) {
            Display::showFastRunStep(i, totalSteps, path[i].cmd, path[i].cellCount);
        }

        switch (path[i].cmd) {
            case CMD_SPRINT:
                Serial.print(F("Step "));
                Serial.print(i + 1);
                Serial.print(F(": SPRINT "));
                Serial.print(path[i].cellCount);
                Serial.println(F(" cells"));
                // Kecepatan tinggi untuk sprint lurus panjang sesuai tier
                if (!Navigator::sprintDistance(path[i].cellCount * CELL_SIZE_MM, sprintSpd)) {
                    running = false;
                    return;
                }
                break;

            case CMD_CURVE_LEFT:
                Serial.print(F("Step "));
                Serial.print(i + 1);
                Serial.println(F(": MENGULAR KIRI (Slalom Curve 90 Left)"));
                // Gerakan mengular kurva tanpa berhenti sesuai tier
                if (!Navigator::smoothCurveTurn(true, curveSpd)) {
                    running = false;
                    return;
                }
                break;

            case CMD_CURVE_RIGHT:
                Serial.print(F("Step "));
                Serial.print(i + 1);
                Serial.println(F(": MENGULAR KANAN (Slalom Curve 90 Right)"));
                // Gerakan mengular kurva tanpa berhenti sesuai tier
                if (!Navigator::smoothCurveTurn(false, curveSpd)) {
                    running = false;
                    return;
                }
                break;

            case CMD_DIAG_ENTER_LEFT:
                Serial.print(F("Step "));
                Serial.print(i + 1);
                Serial.println(F(": DIAG ENTER LEFT (Smooth 45 Curve Left)"));
                if (!Navigator::smoothCurveTurn45(true, diagCurveSpd)) {
                    running = false;
                    return;
                }
                break;

            case CMD_DIAG_ENTER_RIGHT:
                Serial.print(F("Step "));
                Serial.print(i + 1);
                Serial.println(F(": DIAG ENTER RIGHT (Smooth 45 Curve Right)"));
                if (!Navigator::smoothCurveTurn45(false, diagCurveSpd)) {
                    running = false;
                    return;
                }
                break;

            case CMD_DIAG_SPRINT:
                Serial.print(F("Step "));
                Serial.print(i + 1);
                Serial.print(F(": DIAG SPRINT "));
                Serial.print(path[i].cellCount);
                Serial.println(F(" diagonal steps (Post Balancing)"));
                if (!Navigator::sprintDiagonal(path[i].cellCount, diagSprintSpd)) {
                    running = false;
                    return;
                }
                break;

            case CMD_DIAG_EXIT_LEFT:
                Serial.print(F("Step "));
                Serial.print(i + 1);
                Serial.println(F(": DIAG EXIT LEFT (Smooth 45 Curve Left)"));
                if (!Navigator::smoothCurveTurn45(true, diagCurveSpd)) {
                    running = false;
                    return;
                }
                break;

            case CMD_DIAG_EXIT_RIGHT:
                Serial.print(F("Step "));
                Serial.print(i + 1);
                Serial.println(F(": DIAG EXIT RIGHT (Smooth 45 Curve Right)"));
                if (!Navigator::smoothCurveTurn45(false, diagCurveSpd)) {
                    running = false;
                    return;
                }
                break;

            case CMD_PIVOT_180:
                Serial.print(F("Step "));
                Serial.print(i + 1);
                Serial.println(F(": PIVOT 180"));
                if (!Navigator::turn180(SCAN_SPEED_TURN)) {
                    running = false;
                    return;
                }
                break;

            case CMD_FINISH: {
                uint32_t lapDuration = millis() - lapStartTime;
                Serial.println(F("\n>>> FINISH! GOAL TERCAPAI DENGAN WAKTU TERCEPAT! <<<"));
                Serial.print(F("STOPWATCH LAP TIME: "));
                Serial.print((float)lapDuration / 1000.0f, 3);
                Serial.println(F(" detik"));

                Navigator::emergencyStop();
                Buzzer::playVictory();
                UI::blinkAll(6, 60);

                if (Display::isConnected()) {
                    Display::showLapResult(lapDuration, fastRunSpeedTier, true);
                    delay(2500);
                }

                // Jeda 2 detik di garis finish, lalu otomatis kembali ke Start (0,0) dengan kecepatan sedang/aman
                Serial.println(F("Jeda 2 detik... Robot akan kembali ke Start (0,0) dengan kecepatan sedang (80 PWM)."));
                delay(2000);

                returnToStartCalmly();
                running = false;
                return;
            }
        }
    }

    Navigator::emergencyStop();
    running = false;
}

// ==============================================================================
// RETURN TO START CALMLY (KEMBALI KE KOTAK (0,0) DENGAN KECEPATAN SEDANG / AMAN)
// Menggunakan Floodfill terbalik ke Start (0,0), kecepatan 80 PWM, dan putar ke Utara
// ==============================================================================
void returnToStartCalmly() {
    Serial.println(F("\n=============================================="));
    Serial.println(F(">>> MEMULAI PERJALANAN KEMBALI KE START (0,0) <<<"));
    Serial.println(F("Kecepatan: Sedang (80 PWM), bukan kecepatan penuh."));
    Serial.println(F("=============================================="));

    Maze::setTarget(TARGET_START);

    while (running && !Maze::isAtStart()) {
        Sensors::update();
        IMU::update();

        // Update walls jika ada informasi baru
        bool front = Sensors::hasWallFront();
        bool left  = Sensors::hasWallLeft();
        bool right = Sensors::hasWallRight();
        Maze::updateWalls(front, left, right);

        Action next = Maze::getNextAction();
        if (next == ACTION_STOP_GOAL) break;

        if (Display::isConnected()) {
            Display::showMazeStatus(Maze::getX(), Maze::getY(), Maze::getHeading(), next, false);
        }

        switch (next) {
            case ACTION_FORWARD:
                Navigator::moveForwardCell(RETURN_SPEED_FORWARD);
                Maze::applyAction(ACTION_FORWARD);
                break;
            case ACTION_TURN_LEFT:
                Navigator::turn90(true, RETURN_SPEED_TURN);
                Maze::applyAction(ACTION_TURN_LEFT);
                delay(40);
                Navigator::moveForwardCell(RETURN_SPEED_FORWARD);
                Maze::applyAction(ACTION_FORWARD);
                break;
            case ACTION_TURN_RIGHT:
                Navigator::turn90(false, RETURN_SPEED_TURN);
                Maze::applyAction(ACTION_TURN_RIGHT);
                delay(40);
                Navigator::moveForwardCell(RETURN_SPEED_FORWARD);
                Maze::applyAction(ACTION_FORWARD);
                break;
            case ACTION_TURN_AROUND:
                Navigator::turn180(RETURN_SPEED_TURN);
                Maze::applyAction(ACTION_TURN_AROUND);
                delay(40);
                Navigator::moveForwardCell(RETURN_SPEED_FORWARD);
                Maze::applyAction(ACTION_FORWARD);
                break;
            default:
                break;
        }
        delay(40);
    }

    // Orientasikan robot menghadap UTARA saat tiba di kotak (0,0)
    Direction h = Maze::getHeading();
    if (h == DIR_SOUTH) {
        Navigator::turn180(RETURN_SPEED_TURN);
        Maze::setPosition(0, 0, DIR_NORTH);
    } else if (h == DIR_WEST) {
        Navigator::turn90(false, RETURN_SPEED_TURN);
        Maze::setPosition(0, 0, DIR_NORTH);
    } else if (h == DIR_EAST) {
        Navigator::turn90(true, RETURN_SPEED_TURN);
        Maze::setPosition(0, 0, DIR_NORTH);
    }

    Navigator::emergencyStop();
    Buzzer::beep(2400, 250);
    Maze::saveToFlash();
    Maze::setTarget(TARGET_GOAL);

    if (Display::isConnected()) {
        Display::showMazeStatus(0, 0, DIR_NORTH, ACTION_STOP_GOAL, true);
    }

    Serial.println(F("\n========================================================"));
    Serial.println(F(">>> ROBOT BERHASIL KEMBALI DI START (0,0) DENGAN AMAN! <<<"));
    Serial.println(F("Orientasi: Menghadap UTARA. Siap untuk Start berikutnya!"));
    Serial.println(F("========================================================"));
}

void runCalibration() {
    Serial.println(F("\n>>> STARTING CALIBRATION <<<"));
    Serial.println(F("Keep the micromouse stationary inside a standard maze cell..."));
    if (Display::isConnected()) {
        Display::showCalibration("Calibrating Gyro...", false);
    }
    Buzzer::beep(1000, 100);
    delay(1000);

    Serial.println(F("Calibrating Gyro bias..."));
    IMU::calibrate();

    if (Display::isConnected()) {
        Display::showCalibration("Calibrating IR...", false);
    }
    Serial.println(F("Calibrating IR Sensor thresholds..."));
    Sensors::calibrateThresholds();

    if (Display::isConnected()) {
        Display::showCalibration("CALIBRATION OK", true);
        delay(1200);
        Display::showMenu(selectedMenuMode, false, Battery::getVoltage(), Battery::getPercentage(), fastRunSpeedTier);
    }

    Buzzer::playConfirm();
    Serial.println(F(">>> CALIBRATION COMPLETE! <<<"));
    running = false;
}

// ==============================================================================
// START SELECTED MODE (START DENGAN HITUNG MUNDUR & KALIBRASI GYRO OTOMATIS)
// Pengguna memilih mode SEBELUM robot ditaruh. Saat ditaruh di kotak Start,
// pengguna tinggal tekan SW2 (START). Sistem memberikan jeda 2 detik hitung mundur
// (agar tangan sempat dilepas) lalu mengunci nol-bias Gyro MPU6050 di lantai sel.
// ==============================================================================
void startSelectedMode() {
    // 1. LiPo Low-Voltage Cutoff Protection (Kunci motor jika tegangan kritis < 3.15V)
    if (Battery::isCritical()) {
        Serial.println(F("\n[CRITICAL ERROR] TEGANGAN BATERAI DI BAWAH 3.15V!"));
        Serial.println(F("Motor dikunci untuk melindungi sel LiPo dari over-discharge!"));
        if (Display::isConnected()) {
            Display::showCalibration("LOW BATT CRITICAL!", true);
        }
        for (int i = 0; i < 4; i++) {
            Buzzer::beep(800, 150);
            delay(100);
        }
        running = false;
        return;
    }

    if (Battery::getVoltage() < 0.8f) {
        Serial.println(F("[INFO] Mode berjalan tanpa baterai (daya USB / bench test)."));
    }

    if (selectedMenuMode == MODE_FAST_RUN && !Maze::isMapped()) {
        Serial.println(F("\nPERINGATAN: Peta labirin belum tersimpan di memori!"));
        Serial.println(F("Silakan jalankan MODE 3 (Scan Maze) terlebih dahulu."));
        Buzzer::playAlert();
        running = false;
        return;
    }

    running = true;
    UI::resetEmergencyStop();
    Buzzer::playConfirm();
    Serial.print(F("\n>>> STARTING MODE "));
    Serial.println(selectedMenuMode);

    if (selectedMenuMode == MODE_SCAN_MAZE || selectedMenuMode == MODE_FAST_RUN) {
        Maze::resetPosition();
        if (selectedMenuMode == MODE_SCAN_MAZE) {
            Maze::resetWalls(); // Reset peta dinding agar scan baru dimulai dari peta bersih 16x16
        }

        Serial.println(F("Robot ditaruh di sel Start (0,0). Lepaskan tangan Anda..."));
        if (Display::isConnected()) {
            Display::showCalibration("HAND CLEARANCE...", false);
        }

        // Detik 1: Beep peringatan tangan dilepaskan dari bodi robot + Auto Ambient Light Calibration
        Buzzer::beep(1700, 60);
        if (Display::isConnected()) {
            Display::showCalibration("CALIB AMBIENT...", false);
        }
        Sensors::calibrateAmbient(60);
        delay(200);

        // Detik 2: Penguncian Nol-Bias Gyro MPU6050 di Lantai Sel Start (Diam Stabil)
        if (Display::isConnected()) {
            Display::showCalibration("LOCKING GYRO...", false);
        }
        Buzzer::beep(2000, 60);
        delay(200);
        IMU::calibrate();
        IMU::resetYaw();
        delay(150);

        // Pastikan Yaw MPU6050 terkunci di 0.00° dan reset status rem darurat sebelum berangkat
        IMU::resetYaw();
        UI::resetEmergencyStop();

        // GO! Catat lapStartTime untuk Stopwatch
        lapStartTime = millis();
        Buzzer::beep(2500, 180);
        UI::blinkAll(2, 40);
        Serial.println(F(">>> GO! ROBOT BERANGKAT! <<<"));
    }
}

void handleSerialCommands() {
    if (!Serial.available()) return;
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    cmd.toLowerCase();

    if (cmd == "help" || cmd == "menu") {
        printMenu();
    } else if (cmd == "tier" || cmd == "speed") {
        fastRunSpeedTier++;
        if (fastRunSpeedTier > SPEED_TIER_INSANE) fastRunSpeedTier = SPEED_TIER_SAFE;
        printMenu();
    } else if (cmd == "tier1" || cmd == "safe") {
        fastRunSpeedTier = SPEED_TIER_SAFE;
        printMenu();
    } else if (cmd == "tier2" || cmd == "aggr") {
        fastRunSpeedTier = SPEED_TIER_AGGRESSIVE;
        printMenu();
    } else if (cmd == "tier3" || cmd == "insane") {
        fastRunSpeedTier = SPEED_TIER_INSANE;
        printMenu();
    } else if (cmd == "diag" || cmd == "1") {
        selectedMenuMode = MODE_TEST_DIAGNOSTICS;
        startSelectedMode();
        if (Display::isConnected()) Display::showMenu(selectedMenuMode, running, Battery::getVoltage(), Battery::getPercentage(), fastRunSpeedTier);
    } else if (cmd == "wall" || cmd == "2") {
        selectedMenuMode = MODE_WALL_FOLLOWER;
        startSelectedMode();
        if (Display::isConnected()) Display::showMenu(selectedMenuMode, running, Battery::getVoltage(), Battery::getPercentage(), fastRunSpeedTier);
    } else if (cmd == "scan" || cmd == "3") {
        selectedMenuMode = MODE_SCAN_MAZE;
        startSelectedMode();
        if (Display::isConnected()) Display::showMenu(selectedMenuMode, running, Battery::getVoltage(), Battery::getPercentage(), fastRunSpeedTier);
    } else if (cmd == "fast" || cmd == "4") {
        selectedMenuMode = MODE_FAST_RUN;
        startSelectedMode();
        if (Display::isConnected()) Display::showMenu(selectedMenuMode, running, Battery::getVoltage(), Battery::getPercentage(), fastRunSpeedTier);
    } else if (cmd == "cal" || cmd == "5") {
        selectedMenuMode = MODE_CALIBRATE;
        startSelectedMode();
        if (Display::isConnected()) Display::showMenu(selectedMenuMode, running, Battery::getVoltage(), Battery::getPercentage(), fastRunSpeedTier);
    } else if (cmd == "stop" || cmd == "s") {
        running = false;
        Navigator::emergencyStop();
        Buzzer::playAlert();
        if (Display::isConnected()) Display::showMenu(selectedMenuMode, running, Battery::getVoltage(), Battery::getPercentage(), fastRunSpeedTier);
    } else if (cmd == "map") {
        Maze::printMaze();
    } else if (cmd == "buzzer on" || cmd == "buzzer 1") {
        Buzzer::setEnabled(true);
        Serial.println(F("[BUZZER] DI-AKTIFKAN (ON)"));
    } else if (cmd == "buzzer off" || cmd == "buzzer 0") {
        Buzzer::setEnabled(false);
        Serial.println(F("[BUZZER] DI-NONAKTIFKAN (OFF)"));
    } else if (cmd == "sim") {
        Serial.println(F("\n--- SIMULASI FLOODFILL SOLVER ---"));
        Maze::resetPosition();
        Maze::setTarget(TARGET_GOAL);
        Maze::updateDistances();
        Serial.print(F("Posisi Awal: ("));
        Serial.print(Maze::getX());
        Serial.print(F(","));
        Serial.print(Maze::getY());
        Serial.println(F(")"));
        Action nextAct = Maze::getNextAction();
        Serial.print(F("Rekomendasi Action Pertama: "));
        switch (nextAct) {
            case ACTION_FORWARD: Serial.println(F("MAJU (ACTION_FORWARD)")); break;
            case ACTION_TURN_LEFT: Serial.println(F("BELOK KIRI (ACTION_TURN_LEFT)")); break;
            case ACTION_TURN_RIGHT: Serial.println(F("BELOK KANAN (ACTION_TURN_RIGHT)")); break;
            case ACTION_TURN_AROUND: Serial.println(F("PUTAR BALIK (ACTION_TURN_AROUND)")); break;
            case ACTION_STOP_GOAL: Serial.println(F("STOP DI GOAL (ACTION_STOP_GOAL)")); break;
        }
        Maze::printMaze();
    } else if (cmd == "motor" || cmd == "test") {
        Serial.println(F("\n--- TES MOTOR DRV8833 (MAJU 1.5 DETIK) ---"));
        Motors::enable();
        Motors::setSpeeds(120, 120);
        delay(1500);
        Motors::brake();
        delay(50);
        Motors::disable();
        Serial.println(F(">>> Tes Motor Selesai! <<<"));
    }
}

void loop() {
    UI::update();
    handleSerialCommands();

    // Check low battery (Non-blocking: tidak membekukan loop() dengan delay(500))
    static uint32_t lastLowBatWarn = 0;
    if (Battery::isLow()) {
#if ENABLE_LEDS
        UI::setLed2(true);
#endif
        if (millis() - lastLowBatWarn > 4000) {
            lastLowBatWarn = millis();
            Buzzer::beep(600, 80);
        }
    }

    // --- BUTTON CONTROLS (Sesuai Skematik EasyEDA, README, & OLED Display) ---
    // SW1: Cycle Menu Mode (1-5) - Dilakukan sebelum robot ditaruh
    if (UI::wasButtonClicked(BTN_SW1)) {
        selectedMenuMode++;
        if (selectedMenuMode > 5) selectedMenuMode = 1;
        Serial.print(F("Selected Mode: "));
        Serial.println(selectedMenuMode);

#if ENABLE_LEDS
        // Jika ENABLE_LEDS true: Setel juga pola visual LED sesuai mode
        switch (selectedMenuMode) {
            case MODE_TEST_DIAGNOSTICS: // Mode 1: LED1 ON, LED2 OFF
                UI::setLed1(true);
                UI::setLed2(false);
                break;
            case MODE_WALL_FOLLOWER:    // Mode 2: LED1 OFF, LED2 ON
                UI::setLed1(false);
                UI::setLed2(true);
                break;
            case MODE_SCAN_MAZE:        // Mode 3: LED1 ON, LED2 ON (Keduanya Nyala)
                UI::setLed1(true);
                UI::setLed2(true);
                break;
            case MODE_FAST_RUN:         // Mode 4: LED1 OFF, LED2 OFF (Keduanya Mati)
                UI::setLed1(false);
                UI::setLed2(false);
                break;
            case MODE_CALIBRATE:        // Mode 5: Kedip 3x lalu OFF
                for (int k = 0; k < 3; k++) {
                    UI::setLed1(true);
                    UI::setLed2(true);
                    delay(80);
                    UI::setLed1(false);
                    UI::setLed2(false);
                    delay(80);
                }
                break;
        }
#endif

        // Audio feedback: Buzzer SELALU berbunyi bip N kali sesuai nomor mode terpilih!
        // Mode 1: 1 bip, Mode 2: 2 bip, Mode 3: 3 bip, Mode 4: 4 bip, Mode 5: 5 bip
        for (uint8_t b = 0; b < selectedMenuMode; b++) {
            Buzzer::beep(2000, 80);
            delay(50);
        }

        if (Display::isConnected()) {
            Display::showMenu(selectedMenuMode, running, Battery::getVoltage(), Battery::getPercentage(), fastRunSpeedTier);
        }
    }

    // SW4: START / STOP (Tombol Utama Start / Stop Jalan - PC14 - Sesuai Display "SW4 to START")
    if (UI::wasButtonClicked(BTN_SW4)) {
        if (!running) {
            startSelectedMode();
        } else {
            running = false;
            Navigator::emergencyStop();
            Buzzer::playAlert();
            Serial.println(F(">>> STOPPED BY SW4 <<<"));
        }

        if (Display::isConnected()) {
            Display::showMenu(selectedMenuMode, running, Battery::getVoltage(), Battery::getPercentage(), fastRunSpeedTier);
        }
    }

    // SW3: EMERGENCY BRAKE (Rem Darurat Utama - PC15)
    if (UI::wasButtonClicked(BTN_SW3)) {
        running = false;
        Navigator::emergencyStop();
        Buzzer::playAlert();
        Serial.println(F(">>> EMERGENCY BRAKE ACTIVATED (SW3) <<<"));

        if (Display::isConnected()) {
            Display::showMenu(selectedMenuMode, running, Battery::getVoltage(), Battery::getPercentage(), fastRunSpeedTier);
        }
    }

    // SW2: Cycle Speed Tier (jika di Menu Mode 4) ATAU In-Situ Calibration shortcut (PB12)
    if (UI::wasButtonClicked(BTN_SW2)) {
        if (!running && selectedMenuMode == MODE_FAST_RUN) {
            fastRunSpeedTier++;
            if (fastRunSpeedTier > SPEED_TIER_INSANE) fastRunSpeedTier = SPEED_TIER_SAFE;
            Serial.print(F("Fast Run Speed Tier diubah ke: TIER "));
            Serial.println(fastRunSpeedTier);
            // Beep indicator sesuai tier (1 beep = Safe, 2 beeps = Aggr, 3 beeps = Insane)
            for (uint8_t t = 0; t < fastRunSpeedTier; t++) {
                Buzzer::beep(2400 + (t * 200), 50);
                delay(60);
            }
            if (Display::isConnected()) {
                Display::showMenu(selectedMenuMode, running, Battery::getVoltage(), Battery::getPercentage(), fastRunSpeedTier);
            }
        } else if (!running) {
            runCalibration();
        }
    }

    // --- EXECUTE ACTIVE MODE ---
    if (running) {
        switch (selectedMenuMode) {
            case MODE_TEST_DIAGNOSTICS:
                runDiagnostics();
                break;
            case MODE_WALL_FOLLOWER:
                runWallFollower();
                break;
            case MODE_SCAN_MAZE:
                runScanMazeStep();
                break;
            case MODE_FAST_RUN:
                runFastRun();
                break;
            case MODE_CALIBRATE:
                runCalibration();
                break;
        }
    } else {
        // Idle heartbeat, hot-plug check & display refresh
        static uint32_t lastHeartbeat = 0;
        static uint32_t lastOledProbe = 0;

        // Auto-detect jika OLED baru saja dicolokkan saat robot sudah menyala (setiap 2.5 detik)
        if (millis() - lastOledProbe > 2500) {
            lastOledProbe = millis();
            if (!Display::isConnected()) {
                if (Display::checkConnection()) {
                    Display::showMenu(selectedMenuMode, false, Battery::getVoltage(), Battery::getPercentage(), fastRunSpeedTier);
                }
            }
        }

        if (millis() - lastHeartbeat > 500) {
            lastHeartbeat = millis();
            UI::toggleOnboardLed();
            if (Display::isConnected() && (millis() - lastDisplayUpdate > 1000)) {
                lastDisplayUpdate = millis();
                Display::showMenu(selectedMenuMode, false, Battery::getVoltage(), Battery::getPercentage(), fastRunSpeedTier);
            }
        }
    }
}