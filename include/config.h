#pragma once

#include <Arduino.h>

// ==============================================================================
// MICROMOUSE HARDWARE CONFIGURATION & PIN MAPPING (EASYEDA PRO SCHEMATIC)
// Microcontroller: STM32F411CEU6 (WeAct Black Pill)
// ==============================================================================

// --- MOTOR DRIVER (DRV8833 - U3) ---
#define PIN_MOTOR_SLEEP    PA15   // nSLEEP (HIGH = Enabled, LOW = Sleep/Disabled)
#define PIN_MOTOR_L_IN1    PA10   // Left Motor IN1 (Dibalik agar arah Forward maju)
#define PIN_MOTOR_L_IN2    PA9    // Left Motor IN2 (Dibalik agar arah Reverse mundur)
#define PIN_MOTOR_R_IN1    PB8    // Right Motor IN3 (PWM / Forward)
#define PIN_MOTOR_R_IN2    PB9    // Right Motor IN4 (PWM / Reverse)

// --- QUADRATURE WHEEL ENCODERS (N20 Motors with Magnetic Encoders) ---
#define PIN_ENC_L_A        PB4    // Left Encoder Channel A
#define PIN_ENC_L_B        PB5    // Left Encoder Channel B
#define PIN_ENC_R_A        PA0    // Right Encoder Channel A
#define PIN_ENC_R_B        PA1    // Right Encoder Channel B

// --- IR DISTANCE / WALL SENSORS (5x TCRT5000) ---
// Connected to Analog ADC1 Pins
#define PIN_IR_LEFT        PB1    // Far Left IR Sensor (ADC1_IN9)
#define PIN_IR_FRONT_L     PB0    // Front-Left 45° IR Sensor (ADC1_IN8)
#define PIN_IR_FRONT_C     PA7    // Front-Center IR Sensor (ADC1_IN7)
#define PIN_IR_FRONT_R     PA3    // Front-Right 45° IR Sensor (ADC1_IN3)
#define PIN_IR_RIGHT       PA2    // Far Right IR Sensor (ADC1_IN2)

// --- BATTERY VOLTAGE MONITOR ---
// Voltage divider: 10k (R11 to BAT+) + 10k (R12 to GND) -> V_pin = V_bat / 2
#define PIN_BAT_SENSE      PA4    // Battery ADC input (ADC1_IN4)
// Jika baterai Anda LiPo 2S (7.4V - 8.4V) ganti divider atau voltase di sini.
// Default divider factor untuk R11=10k, R12=10k adalah 2.0f
#define BATTERY_DIVIDER_RATIO       2.0f
// Kalibrasi tegangan referensi STM32 (biasanya 3.30V)
#define BATTERY_ADC_VREF            3.30f

// --- IMU (MPU6050 - U4) ---
#define PIN_I2C_SCL        PB6    // I2C1 SCL
#define PIN_I2C_SDA        PB7    // I2C1 SDA
#define MPU6050_I2C_ADDR   0x68   // ADD pin tied to GND (0x68) atau VCC (0x69)
#define MPU6050_I2C_ADDR_ALT 0x69 // Alamat alternatif jika pin AD0 ditarik HIGH

// --- USER INTERFACE (Buttons, LEDs, Buzzer) ---
// SAKLAR KONTROL SERIAL MONITOR & LED1 (PA11):
// - Set true  : Aktifkan Serial Monitor lewat kabel USB untuk debugging (COM port aktif normal)
// - Set false : Matikan Serial Monitor; USB dilepas penuh sehingga PA11 berfungsi sebagai LED1
#define ENABLE_SERIAL_DEBUG true

// SAKLAR KONTROL STATUS LED (LED1 PA11 & LED2 PB3):
// - Set true  : Aktifkan LED kanan & kiri untuk indikator visual mode dan deteksi dinding
// - Set false : Matikan LED total; indikator mode menggunakan Buzzer (Mode 1 = 1 bip, Mode 2 = 2 bip, dst)
#define ENABLE_LEDS        false

#define PIN_LED1           PA11   // Status LED 1 (Active HIGH)
#define PIN_LED2           PB3    // Status LED 2 (Active HIGH)
#define PIN_LED_BOARD      PC13   // Black Pill Onboard LED (Active LOW)
#define PIN_BUZZER         PB10   // Buzzer via Q2 NPN Transistor (Active HIGH)

#define PIN_SW1            PB13   // Push Button 1 (Active LOW, internal pullup)
#define PIN_SW2            PB12   // Push Button 2 (Active LOW, internal pullup)
#define PIN_SW3            PC15   // Push Button 3 (Active LOW, internal pullup)
#define PIN_SW4            PC14   // Push Button 4 (Active LOW, internal pullup)

// --- BUZZER CONFIGURATION ---
// Set false untuk mematikan buzzer sepenuhnya jika dirasa bising/mengganggu saat testing
#define BUZZER_ENABLED     true

// ==============================================================================
// ROBOT PHYSICAL SPECIFICATIONS & MAZE DIMENSIONS (DERIVED FROM EASYEDA PCB SCAN)
// ==============================================================================
#define PCB_OUTLINE_LENGTH_MM  119.38f // PCB board outline length
#define PCB_OUTLINE_WIDTH_MM   114.10f // PCB board outline width
#define ROBOT_TOTAL_LENGTH_MM  122.00f // Panjang keseluruhan robot (ujung sensor TCRT5000 depan ke pantat belakang)
#define ROBOT_TOTAL_WIDTH_MM   104.00f // Lebar keseluruhan robot termasuk ban kiri & kanan
#define AXLE_TO_FRONT_MM        78.60f // Jarak poros roda belakang (EXT-1/EXT-2 y=-2274) ke sensor depan (CO1 y=+821)
#define WHEEL_DIAMETER_MM       44.00f // Diameter luar roda + ban micromouse (44mm)
#define WHEEL_BASE_MM           82.00f // Jarak antar pusat tapak roda kiri dan kanan (~82mm)
#define ENCODER_TICKS_PER_REV  360.00f // Resolusi encoder kuadratur motor N20 per putaran roda (dapat disesuaikan)
#define CELL_SIZE_MM           180.00f // Standar ukuran sel labirin IEEE Micromouse (180mm x 180mm)
#define CELL_DIAGONAL_MM       254.56f // Jarak diagonal antar pusat sel (180mm * sqrt(2) = 254.56mm)
#define CURVE_TURN_RADIUS_MM    90.00f // Radius belokan mengular (slalom curve) di pusat sel (90mm)

// ==============================================================================
// UKURAN GRID LABIRIN & TARGET GOAL (CUSTOMIZABLE UNTUK UJI COBA ARENA KECIL)
// ==============================================================================
// Anda dapat memperkecil ukuran maze untuk uji coba di arena kecil (misal: 4x4, 8x8).
// Standar kompetisi resmi Micromouse adalah 16x16.
//
// CONTOH PRESET:
// 1. Arena Uji Coba Mini (4x4):
//    #define MAZE_WIDTH       4
//    #define MAZE_HEIGHT      4
//    #define MAZE_GOAL_MIN_X  1
//    #define MAZE_GOAL_MAX_X  2
//    #define MAZE_GOAL_MIN_Y  1
//    #define MAZE_GOAL_MAX_Y  2
//
// 2. Arena Uji Coba Sedang (6x6) - Pusat:
//    #define MAZE_WIDTH       6
//    #define MAZE_HEIGHT      6
//    #define MAZE_GOAL_MIN_X  2
//    #define MAZE_GOAL_MAX_X  3
//    #define MAZE_GOAL_MIN_Y  2
//    #define MAZE_GOAL_MAX_Y  3
//
// 3. Arena Kompetisi Resmi (16x16):
//    #define MAZE_WIDTH       16
//    #define MAZE_HEIGHT      16
//    #define MAZE_GOAL_MIN_X  7
//    #define MAZE_GOAL_MAX_X  8
//    #define MAZE_GOAL_MIN_Y  7
//    #define MAZE_GOAL_MAX_Y  8

#define MAZE_WIDTH              6     // Lebar kolom labirin (X: 0 sampai MAZE_WIDTH - 1)
#define MAZE_HEIGHT             6     // Tinggi baris labirin (Y: 0 sampai MAZE_HEIGHT - 1)

// Koordinat Area Kotak Finish / Goal (Inklusif: MIN <= Posisi <= MAX)
// Diatur tepat di tengah labirin 6x6 (sel X: 2..3, Y: 2..3)
#define MAZE_GOAL_MIN_X         2     // Batas minimum kolom X finish (pusat 6x6: sel 2..3)
#define MAZE_GOAL_MAX_X         3     // Batas maksimum kolom X finish
#define MAZE_GOAL_MIN_Y         2     // Batas minimum baris Y finish (pusat 6x6: sel 2..3)
#define MAZE_GOAL_MAX_Y         3     // Batas maksimum baris Y finish

// ==============================================================================
// ENCODER & MOTOR HARDWARE CONFIGURATION (DUAL-MODE N20 SUPPORT)
// ==============================================================================
// 0 = AUTO_DETECT (Otomatis deteksi ada/tidaknya pulsa encoder saat motor berputar)
// 1 = FORCE_ENCODER (Hanya pakai encoder magnetik)
// 2 = NO_ENCODER (Mode motor N20 biasa tanpa encoder - navigasi timing terkompensasi voltase + IR)
#define ENCODER_MODE_AUTO        0
#define ENCODER_MODE_FORCE       1
#define ENCODER_MODE_NONE        2
#define ENCODER_HARDWARE_CONFIG  ENCODER_MODE_AUTO

// Estimasi Waktu Tempuh 1 Sel (180mm) untuk Motor N20 Tanpa Encoder (milidetik)
// Disesuaikan agar pergerakan sel gesit dan lincah (~200mm/s @ 185 PWM)
#define TIME_PER_CELL_NOMINAL_MS 900

// Flash Memory Persistence (Sector 5 on F401 256KB @ 0x08020000; Sector 7 on F411 512KB @ 0x08060000)
#if defined(STM32F401xC)
#define FLASH_MAZE_SECTOR      FLASH_SECTOR_5
#define FLASH_MAZE_ADDR        0x08020000UL
#else
#define FLASH_MAZE_SECTOR      FLASH_SECTOR_7
#define FLASH_MAZE_ADDR        0x08060000UL
#endif
#define FLASH_MAZE_MAGIC       (0xA55A0000UL | ((uint32_t)(MAZE_WIDTH) << 8) | (uint32_t)(MAZE_HEIGHT))

// Wall Detection Thresholds (12-bit ADC: 0 - 4095)
// Ambang batas kebal cahaya ruangan (Ambient Light Rejection):
// Cahaya lampu ruangan normal (150-450 ADC) diabaikan 100%, hanya mendeteksi pantulan dinding fisik nyata (>600-800 ADC)
#define DEFAULT_WALL_FRONT_THRESHOLD   750   // Ambang dinding depan sel saat ini (kebal gangguan cahaya/bayangan)
#define DEFAULT_WALL_SIDE_THRESHOLD    600   // Threshold dinding samping (kebal cahaya lampu samping)

// Ambang Presisi Jarak Dinding Depan Sebelum Berbelok (Ideal: Pas untuk radius putar tanpa mundur)
#define WALL_FRONT_STOP_ADC            950    // Target henti sensor depan (~42-45mm dari dinding, aman anti-tabrak & pas radius putar)
#define WALL_FRONT_TOO_CLOSE_ADC       1200   // Batas jika terlalu mepet (pengaman darurat rem)
#define WALL_FRONT_TOO_FAR_ADC         650    // Batas jika berhenti terlalu jauh dari dinding depan

// --- MOTION SPEEDS & TUNING CONSTANTS (STRICT FLOOR: TIDAK PERNAH < 150 PWM) ---
// Mode 3: Scan / Exploration Mode
#define SCAN_SPEED_FORWARD     185.0f  // PWM kecepatan maju saat scan (gesit, lincah & bertenaga)
#define SCAN_SPEED_TURN        170.0f  // PWM belok pivot 90/180 derajat di tempat (cepat, bertenaga & anti-stall)

// Mode 4: Fast Run Mode (Fokus: Jalur Tercepat, High Speed Sprint & Slalom Mengular)
#define FAST_RUN_SPRINT_SPEED       240.0f  // PWM sprint kecepatan tinggi pada lintasan lurus multi-sel
#define FAST_RUN_CURVE_SPEED        185.0f  // PWM belokan kurva halus mengular (slalom) 90 derajat tanpa henti
#define FAST_RUN_DIAG_SPRINT_SPEED  245.0f  // PWM sprint kecepatan tinggi pada lintasan diagonal 45 derajat
#define FAST_RUN_DIAG_CURVE_SPEED   175.0f  // PWM belokan kurva halus 45 derajat (masuk/keluar diagonal)

// Return-to-Start Speed (Kecepatan saat kembali dari Finish ke Start (0,0))
#define RETURN_SPEED_FORWARD        185.0f  // PWM kecepatan maju saat kembali ke Start
#define RETURN_SPEED_TURN           170.0f  // PWM kecepatan belok saat kembali ke Start

// ==============================================================================
// CASCADED PID / PD CONTROLLER GAINS (ANTI-WOBBLE & ROTATIONAL DAMPING)
// ==============================================================================
// 1. Steering & Wall Centering PD Gains (Meredam goyangan zig-zag di lorong labirin)
#define PID_WALL_KP                 0.020f  // Penguatan proporsional error tengah dinding (ADC) - sangat halus
#define PID_WALL_KD                 0.008f  // Penguatan diferensial laju perubahan error dinding (de/dt)
#define PID_WALL_DAMP_GYRO          0.150f  // Peredam aktif laju angular fisik Gyro Z (deg/s)
#define PID_WALL_DEADBAND           40.0f   // Deadband ADC (abaikan getaran sensor minor agar laju lurus mulus)

// 2. Heading Lock PD Gains (Lorong terbuka tanpa dinding & jalur lurus multi-sel)
#define PID_GYRO_KP                 2.000f  // Penguatan proporsional error Yaw angle (deg)
#define PID_GYRO_KD                 0.100f  // Penguatan diferensial laju angular fisik Gyro Z (deg/s)

// 3. Discrete Pivot Turn PID Gains (Belok 90° & 180° di tempat tanpa bablas / zero overshoot)
#define PID_TURN_KP                 2.800f  // Penguatan proporsional sudut putar (responsif & cepat)
#define PID_TURN_KI                 0.025f  // Penguatan integral untuk mengatasi stiction gearbox saat mendekati nol
#define PID_TURN_KD                 0.160f  // Penguatan peredam diferensial Gyro Z saat deselerasi mendekati target
#define PID_TURN_I_LIMIT            30.0f   // Anti-windup batas maksimal akumulasi integral term

// 4. Slalom Curve & Diagonal PD Gains (Pelacak kelengkungan busur kurva dinamis)
#define PID_CURVE_KP                0.450f  // Penguatan proporsional trim roda dalam kurva slalom
#define PID_CURVE_KD                0.060f  // Penguatan diferensial trim kurva slalom

// ==============================================================================
// ADVANCED COMPETITION TUNING CONSTANTS (7 FITUR UNGGULAN MICROMOUSE)
// ==============================================================================
// 1. Fast Run Speed Tiers (Tingkat Kecepatan Fast Run, dipilih via SW3)
#define SPEED_TIER_SAFE             1       // Tier 1: Aman & Stabil (180 PWM sprint, 160 PWM slalom)
#define SPEED_TIER_AGGRESSIVE       2       // Tier 2: Agresif Cepat (220 PWM sprint, 170 PWM slalom) - Default
#define SPEED_TIER_INSANE           3       // Tier 3: Maksimum Serang (250 PWM sprint, 195 PWM slalom)

// Kecepatan Tier 1 (Safe - Semua Kecepatan >= 150 PWM)
#define TIER1_SPRINT_SPEED          180.0f
#define TIER1_CURVE_SPEED           160.0f
#define TIER1_DIAG_SPRINT_SPEED     185.0f
#define TIER1_DIAG_CURVE_SPEED      155.0f

// Kecepatan Tier 2 (Aggressive - Default)
#define TIER2_SPRINT_SPEED          220.0f
#define TIER2_CURVE_SPEED           170.0f
#define TIER2_DIAG_SPRINT_SPEED     225.0f
#define TIER2_DIAG_CURVE_SPEED      160.0f

// Kecepatan Tier 3 (Insane Attack)
#define TIER3_SPRINT_SPEED          250.0f
#define TIER3_CURVE_SPEED           195.0f
#define TIER3_DIAG_SPRINT_SPEED     250.0f
#define TIER3_DIAG_CURVE_SPEED      180.0f

// 2. Wall Edge / Post Peaking Longitudinal Odometry Reset
#define POST_PEAKING_DROP_THRESHOLD 600.0f  // Penurunan tajam ADC sensor samping saat melewati ujung tiang/dinding
#define POST_PEAKING_MIN_INTERVAL   110.0f  // Jarak tempuh minimal (mm) antar deteksi tiang pasak berturut-turut

// 3. Deteksi Tabrakan & Anti-Stall Failsafe
#define STALL_TIMEOUT_MS            1200    // Waktu toleransi longgar agar motor tidak tersendat saat start
#define STALL_MIN_PROGRESS_MM       2.0f    // Pergerakan minimal roda dalam periode STALL_TIMEOUT_MS

// 4. Proteksi Baterai LiPo Kritis
#define BATTERY_CRITICAL_VOLTAGE    3.5f   // Batas tegangan kritis LiPo 1S: Tolak start motor jika di bawah nilai ini
#define BATTERY_LOW_VOLTAGE         3.6f   // Batas tegangan rendah peringatan

// 5. Turn-Penalty Floodfill Cost
#define TURN_PENALTY_COST           15      // Penalti bobot belokan (setara 1.5 sel) agar solver memilih jalur minim belok

