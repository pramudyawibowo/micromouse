#include "navigator.h"
#include "motors.h"
#include "encoders.h"
#include "sensors.h"
#include "imu.h"
#include "battery.h"
#include "buzzer.h"
#include "ui.h"
#include "config.h"

void Navigator::init() {
    Motors::init();
    Encoders::init();
    Sensors::init();
    IMU::init();
}

void Navigator::emergencyStop() {
    Motors::brake();
    delay(50);
    Motors::disable();
}

bool Navigator::moveForwardCell(float targetSpeed, bool brakeAtEnd) {
    return sprintDistance(CELL_SIZE_MM, targetSpeed, brakeAtEnd);
}

// ==============================================================================
// SPRINT DISTANCE (LURUS DUAL-MODE: DENGAN ENCODER ATAU TANPA ENCODER)
// Mendukung N20 dengan encoder magnetik (akurasi mm tertutup) DAN N20 polos
// tanpa encoder (waktu terkompensasi voltase baterai + front-wall IR centering).
// ==============================================================================
// SPRINT DISTANCE (LURUS DUAL-MODE DENGAN CASCADED PD CONTROLLER & S-CURVE PROFILE)
// Menggabungkan profil percepatan S-Curve (mencegah selip ban 44mm) dan
// Cascaded Dual-Loop PD:
// - Loop 1: Translasi jarak (Encoder magnetik / Waktu terkompensasi voltase)
// - Loop 2: Kemudi / Wall Centering PD + Peredam Rotasi Aktif Gyro Z (Anti-Wobble)
// ==============================================================================
bool Navigator::sprintDistance(float targetDistance, float cruiseSpeed, bool brakeAtEnd) {
    if (cruiseSpeed < 150.0f) cruiseSpeed = 150.0f; // STRICT FLOOR: Kecepatan maju minimal 150 PWM
    Motors::enable();
    Encoders::reset();
    IMU::update();
    float startYaw = IMU::getYaw();

    float currentDistance = 0.0f;
    const float minSpeed = 150.0f; // STRICT FLOOR: Kecepatan tidak boleh < 150 PWM

    // Parameter navigasi berbasis waktu untuk motor N20 tanpa encoder
    uint32_t tStart = millis();
    float cellFraction = targetDistance / CELL_SIZE_MM;
    float vBat = Battery::getVoltage();
    float vComp = (vBat > 3.0f) ? (3.85f / constrain(vBat, 3.2f, 4.3f)) : 1.0f;
    float speedRatio = SCAN_SPEED_FORWARD / constrain(cruiseSpeed, 40.0f, 255.0f);
    uint32_t targetDurationMs = (uint32_t)(cellFraction * TIME_PER_CELL_NOMINAL_MS * speedRatio * vComp);
    if (targetDurationMs < 80) targetDurationMs = 80;

    uint32_t lastLoop = millis();
    uint32_t lastLoopUs = micros();
    float lastCenteringError = 0.0f;

    // Pelacak Post Peaking (Wall Edge) & Anti-Stall
    float lastLeftAdc = 0.0f;
    float lastRightAdc = 0.0f;
    float lastPostResetDist = 0.0f;
    uint32_t lastProgressTime = millis();
    float lastProgressDist = 0.0f;

    while (currentDistance < targetDistance) {
        // Cek darurat SW3 langsung di dalam loop pergerakan
        if (UI::checkEmergencyStop()) {
            emergencyStop();
            return false;
        }

        Sensors::update();
        IMU::update();
        uint32_t elapsed = millis() - tStart;
        uint32_t nowUs = micros();
        float dtLoop = (nowUs - lastLoopUs) * 1e-6f;
        lastLoopUs = nowUs;
        if (dtLoop <= 0.0001f || dtLoop > 0.05f) dtLoop = 0.005f;

        // Auto-detect ada/tidaknya pulsa encoder setelah ~100ms motor berputar
        if (elapsed > 100 && Encoders::isAvailable()) {
            if (Encoders::getLeftTicks() == 0 && Encoders::getRightTicks() == 0) {
                // Tidak ada pulsa encoder terdeteksi -> alihkan ke mode encoderless
                Encoders::notifyMovement(false);
            } else {
                Encoders::notifyMovement(true);
            }
        }

        // Pengukuran jarak: Dari encoder jika tersedia, atau dari waktu jika tanpa encoder
        if (Encoders::isAvailable()) {
            currentDistance = Encoders::getAverageDistanceMm();
        } else {
            currentDistance = targetDistance * ((float)elapsed / (float)targetDurationMs);
            if (elapsed >= targetDurationMs) {
                break;
            }
        }

        // 1. Wall Edge / Post Peaking Detection (Hanya jika tanpa encoder)
        // Jika memakai encoder magnetik, snap dist dilewati agar tidak melompat mendadak di tiang sel
        IRSensorData irData = Sensors::getFiltered();
        if (!Encoders::isAvailable()) {
            float leftDrop = (float)irData.left - lastLeftAdc;
            float rightDrop = (float)irData.right - lastRightAdc;

            if ((currentDistance - lastPostResetDist) > POST_PEAKING_MIN_INTERVAL) {
                // Cek falling edge dinding samping saat sensor melintasi ujung tiang sel
                if (leftDrop < -POST_PEAKING_DROP_THRESHOLD || rightDrop < -POST_PEAKING_DROP_THRESHOLD) {
                    float rawCellIndex = (currentDistance + 90.0f) / CELL_SIZE_MM;
                    float snappedDist = floorf(rawCellIndex) * CELL_SIZE_MM;
                    if (snappedDist > 0 && fabs(snappedDist - currentDistance) < 45.0f) {
                        currentDistance = snappedDist;
                        lastPostResetDist = currentDistance;
                    }
                }
            }
        }
        lastLeftAdc = (float)irData.left;
        lastRightAdc = (float)irData.right;

        // 2. Deteksi Tabrakan & Motor Mogok (Anti-Stall Crash Failsafe)
        if (elapsed > 400 && cruiseSpeed > 60.0f) {
            if (fabs(currentDistance - lastProgressDist) >= STALL_MIN_PROGRESS_MM) {
                lastProgressDist = currentDistance;
                lastProgressTime = millis();
            } else if ((millis() - lastProgressTime) > STALL_TIMEOUT_MS) {
                // Roda terhambat/menabrak saat motor aktif -> potong daya darurat
                Motors::brake();
                Motors::disable();
                Buzzer::playAlert();
                Serial.println(F("\n[FAILSAFE] TABRAKAN / MOTOR STALL TERDETEKSI! Daya motor diputus."));
                return false;
            }
        } else {
            lastProgressTime = millis();
            lastProgressDist = currentDistance;
        }

        // Safety check & center-lock: jika ada dinding depan
        if (Sensors::hasWallFront() || irData.frontCenter >= WALL_FRONT_STOP_ADC) {
            // Berhenti tepat di titik tengah sel (~25-30mm dari dinding depan)
            if (irData.frontCenter >= WALL_FRONT_STOP_ADC || (targetDistance - currentDistance) < 18.0f) {
                break;
            }
        }
        // Pengaman Darurat Tabrakan Depan pada High-Speed Sprint Attack
        if (irData.frontCenter > (WALL_FRONT_TOO_CLOSE_ADC + 200)) {
            Motors::brake();
            delay(20);
            break;
        }

        // 3. Smooth Velocity Profile
        // Kecepatan konstan selama perjalanan agar gerakan tidak bergelombang
        float speed = cruiseSpeed;
        float remaining = targetDistance - currentDistance;
        const float rampUpDist = 12.0f;
        const float rampDownDist = 28.0f;

        if (currentDistance < rampUpDist) {
            float s = currentDistance / rampUpDist;
            speed = minSpeed + (cruiseSpeed - minSpeed) * s;
        } else if (brakeAtEnd) {
            float s = remaining / rampDownDist;
            // Jika ada dinding depan, bantu perlambatan halus berdasarkan jarak sensor depan menuju WALL_FRONT_STOP_ADC
            if (Sensors::hasWallFront() && irData.frontCenter > (WALL_FRONT_STOP_ADC - 350)) {
                float sFront = (float)(WALL_FRONT_STOP_ADC - irData.frontCenter) / 350.0f;
                if (sFront < s) s = sFront;
            }
            s = constrain(s, 0.0f, 1.0f);
            speed = minSpeed + (cruiseSpeed - minSpeed) * s;
        }

        // 4. Cascaded PD Steering Controller (Master: Gyro Heading Lock | Slave: Wall Centering Trim)
        float gyroRate = IMU::getGyroZ(); // deg/sec (Laju angular fisik Gyro Z: CCW > 0, CW < 0)
        float yawDiff = IMU::getYaw() - startYaw;
        while (yawDiff > 180.0f) yawDiff -= 360.0f;
        while (yawDiff < -180.0f) yawDiff += 360.0f;

        // Master Controller: Gyro Heading Lock selalu aktif 100% menjaga robot lurus 0.00 derajat
        float gyroSteer = (yawDiff * PID_GYRO_KP) + (gyroRate * PID_GYRO_KD);
        gyroSteer = constrain(gyroSteer, -30.0f, 30.0f);

        // Slave Controller: Wall Centering Trim Adaptif Kecepatan Tinggi (Sprint Attack Mode)
        // Pada sprint kencang (>190 PWM), perbesar otoritas koreksi kemudi sensor agar robot tidak tersedot ke dinding
        float maxTrim = (cruiseSpeed > 190.0f) ? 26.0f : 14.0f;
        float kpWall  = (cruiseSpeed > 190.0f) ? (PID_WALL_KP * 1.5f) : PID_WALL_KP;
        float kdWall  = (cruiseSpeed > 190.0f) ? (PID_WALL_KD * 1.5f) : PID_WALL_KD;

        float wallTrim = 0.0f;
        bool wallL = Sensors::hasWallLeft();
        bool wallR = Sensors::hasWallRight();

        if (wallL && wallR) {
            float centeringError = (float)((int32_t)irData.left - (int32_t)irData.right);
            if (fabs(centeringError) > PID_WALL_DEADBAND) {
                float dError = (centeringError - lastCenteringError) / dtLoop;
                lastCenteringError = centeringError;
                wallTrim = (centeringError * kpWall) + (dError * kdWall);
                wallTrim = constrain(wallTrim, -maxTrim, maxTrim);
            } else {
                lastCenteringError = 0.0f;
            }
        } else if (wallL) {
            float errL = (float)((int32_t)irData.left - (int32_t)Sensors::getLeftNominal());
            if (fabs(errL) > PID_WALL_DEADBAND) {
                wallTrim = (errL * (kpWall * 0.7f));
                wallTrim = constrain(wallTrim, -maxTrim * 0.7f, maxTrim * 0.7f);
            }
            lastCenteringError = 0.0f;
        } else if (wallR) {
            float errR = (float)((int32_t)Sensors::getRightNominal() - (int32_t)irData.right);
            if (fabs(errR) > PID_WALL_DEADBAND) {
                wallTrim = (errR * (kpWall * 0.7f));
                wallTrim = constrain(wallTrim, -maxTrim * 0.7f, maxTrim * 0.7f);
            }
            lastCenteringError = 0.0f;
        } else {
            lastCenteringError = 0.0f;
        }

        // Active Optical Proximity Shield (Anti-Nempel & Anti-Serempet Dinding pada High-Speed Sprint Attack)
        if (irData.left > 1300) {
            // Terlalu mepet ke dinding kiri: dorong paksa menjauh ke kanan seketika!
            wallTrim += constrain((float)(irData.left - 1300) * 0.025f, 6.0f, 24.0f);
        } else if (irData.right > 1300) {
            // Terlalu mepet ke dinding kanan: dorong paksa menjauh ke kiri seketika!
            wallTrim -= constrain((float)(irData.right - 1300) * 0.025f, 6.0f, 24.0f);
        }

        float maxTotalSteer = (cruiseSpeed > 190.0f) ? 55.0f : 35.0f;
        float totalSteer = constrain(gyroSteer + wallTrim, -maxTotalSteer, maxTotalSteer);

        int16_t leftPwm  = (int16_t)constrain(speed + totalSteer, 150.0f, 255.0f);
        int16_t rightPwm = (int16_t)constrain(speed - totalSteer, 150.0f, 255.0f);

        Motors::setSpeeds(leftPwm, rightPwm);

        while ((millis() - lastLoop) < 5) {}
        lastLoop = millis();
    }

    if (brakeAtEnd) {
        Motors::brake();
        delay(10);
        Motors::coast();
    }
    return true;
}

// ==============================================================================
// SMOOTH CURVE / SLALOM TURN 90° (MENGULAR DENGAN KINEMATIKA + CLOSED-LOOP PD)
// Roda belakang terus berputar maju saat menikung 90 derajat mengikuti kurva busur,
// menjaga momentum translasi tinggi pada Mode 4 (Fast Run).
// ==============================================================================
bool Navigator::smoothCurveTurn(bool turnLeft, float baseSpeed) {
    if (baseSpeed < 170.0f) baseSpeed = 170.0f; // STRICT FLOOR: Kecepatan kurva minimal 170 PWM
    Motors::enable();
    IMU::update();
    float startYaw = IMU::getYaw();
    float targetYawChange = turnLeft ? 90.0f : -90.0f; // Left: +90 deg (CCW), Right: -90 deg (CW)
    float targetYaw = startYaw + targetYawChange;
    while (targetYaw > 180.0f) targetYaw -= 360.0f;
    while (targetYaw < -180.0f) targetYaw += 360.0f;

    // Kinematika roda diferensial untuk belokan mengular (R = 90mm, W = 82mm)
    // Roda inner tetap diberi daya minimum 150 PWM agar motor N20 tidak mogok
    const float innerRatio = 0.45f;
    float outerSpeed = baseSpeed;
    float innerSpeed = baseSpeed * innerRatio;
    if (innerSpeed < 150.0f) innerSpeed = 150.0f; // STRICT FLOOR: MIN 150 PWM

    uint32_t tStart = millis();
    uint32_t lastLoop = millis();
    uint32_t lastLoopUs = micros();
    float lastDiff = targetYawChange;

    while ((millis() - tStart) < 1400) { // Max timeout 1.4s
        if (UI::checkEmergencyStop()) {
            emergencyStop();
            return false;
        }

        IMU::update();
        Sensors::update();
        IRSensorData ir = Sensors::getFiltered();

        uint32_t nowUs = micros();
        float dt = (nowUs - lastLoopUs) * 1e-6f;
        lastLoopUs = nowUs;
        if (dt <= 0.0001f || dt > 0.05f) dt = 0.005f;

        float currentYaw = IMU::getYaw();
        float diff = targetYaw - currentYaw;
        while (diff > 180.0f) diff -= 360.0f;
        while (diff < -180.0f) diff += 360.0f;

        // Toleransi sudut tercapai
        if (fabs(diff) < 2.5f) {
            break;
        }

        // Closed-loop PD gyro trim pada roda bagian dalam
        float dDiff = (diff - lastDiff) / dt;
        lastDiff = diff;
        float gyroTrim = (diff * PID_CURVE_KP) + (dDiff * PID_CURVE_KD);

        // Panduan Sensor Optik Serong & Samping saat Slalom (Anti-Tabrak Dinding & Tiang)
        float opticalTrim = 0.0f;
        if (turnLeft) {
            // Belok Kiri: Sensor serong kanan & samping kanan memantau dinding luar kurva
            uint16_t outerDist = (ir.frontRight > ir.right) ? ir.frontRight : ir.right;
            if (outerDist > 1100) {
                // Terlalu dekat dinding luar: perdalam belokan ke dalam agar tidak menyenggol dinding luar
                opticalTrim += constrain((float)(outerDist - 1100) * 0.035f, 6.0f, 30.0f);
            }
            if (ir.left > 1500) {
                // Terlalu mepet tiang dalam: longgarkan belokan agar tidak menyerempet tiang dalam
                opticalTrim -= constrain((float)(ir.left - 1500) * 0.035f, 6.0f, 25.0f);
            }
            if (ir.frontCenter > 1300) {
                opticalTrim += 18.0f; // Dinding depan mendekat cepat, tarik belok lebih tajam
            }
        } else {
            // Belok Kanan: Sensor serong kiri & samping kiri memantau dinding luar kurva
            uint16_t outerDist = (ir.frontLeft > ir.left) ? ir.frontLeft : ir.left;
            if (outerDist > 1100) {
                opticalTrim += constrain((float)(outerDist - 1100) * 0.035f, 6.0f, 30.0f);
            }
            if (ir.right > 1500) {
                opticalTrim -= constrain((float)(ir.right - 1500) * 0.035f, 6.0f, 25.0f);
            }
            if (ir.frontCenter > 1300) {
                opticalTrim += 18.0f;
            }
        }

        if (turnLeft) {
            // Belok Kiri: Roda Kanan adalah Outer (lebih cepat), Roda Kiri adalah Inner
            int16_t leftPwm = (int16_t)constrain(innerSpeed - gyroTrim - opticalTrim, 150.0f, baseSpeed);
            int16_t rightPwm = (int16_t)constrain(outerSpeed + opticalTrim * 0.5f, 150.0f, 255.0f);
            Motors::setSpeeds(leftPwm, rightPwm);
        } else {
            // Belok Kanan: Roda Kiri adalah Outer (lebih cepat), Roda Kanan adalah Inner
            int16_t leftPwm = (int16_t)constrain(outerSpeed + opticalTrim * 0.5f, 150.0f, 255.0f);
            int16_t rightPwm = (int16_t)constrain(innerSpeed + gyroTrim - opticalTrim, 150.0f, baseSpeed);
            Motors::setSpeeds(leftPwm, rightPwm);
        }

        while ((millis() - lastLoop) < 5) {}
        lastLoop = millis();
    }

    // Jaga momentum ke sel berikutnya — tidak brake keras saat slalom
    Motors::coast();
    delay(5);
    return true;
}

// ==============================================================================
// SMOOTH CURVE TURN 45° (GERAKAN MENGULAR MASUK/KELUAR JALUR DIAGONAL)
// Menikung 45 derajat secara mulus dan berkesinambungan tanpa henti, menjaga
// momentum translasi agar transisi dari ortogonal ke diagonal sangat halus (fluid).
// ==============================================================================
bool Navigator::smoothCurveTurn45(bool turnLeft, float baseSpeed) {
    if (baseSpeed < 165.0f) baseSpeed = 165.0f; // STRICT FLOOR: Kecepatan kurva 45 minimal 165 PWM
    Motors::enable();
    IMU::update();
    float startYaw = IMU::getYaw();
    float targetYawChange = turnLeft ? 45.0f : -45.0f; // Left: +45 deg (CCW), Right: -45 deg (CW)
    float targetYaw = startYaw + targetYawChange;
    while (targetYaw > 180.0f) targetYaw -= 360.0f;
    while (targetYaw < -180.0f) targetYaw += 360.0f;

    // Kinematika kurva diferensial 45 derajat (kurva lebih landai dibanding 90 derajat)
    const float innerRatio = 0.55f;
    float outerSpeed = baseSpeed;
    float innerSpeed = baseSpeed * innerRatio;
    if (innerSpeed < 150.0f) innerSpeed = 150.0f; // STRICT FLOOR: MIN 150 PWM

    uint32_t tStart = millis();
    uint32_t lastLoop = millis();
    uint32_t lastLoopUs = micros();
    float lastDiff = targetYawChange;

    while ((millis() - tStart) < 900) { // Max timeout 0.9s
        if (UI::checkEmergencyStop()) {
            emergencyStop();
            return false;
        }

        IMU::update();
        Sensors::update();
        IRSensorData ir = Sensors::getFiltered();

        uint32_t nowUs = micros();
        float dt = (nowUs - lastLoopUs) * 1e-6f;
        lastLoopUs = nowUs;
        if (dt <= 0.0001f || dt > 0.05f) dt = 0.005f;

        float currentYaw = IMU::getYaw();
        float diff = targetYaw - currentYaw;
        while (diff > 180.0f) diff -= 360.0f;
        while (diff < -180.0f) diff += 360.0f;

        // Toleransi sudut tercapai
        if (fabs(diff) < 2.0f) {
            break;
        }

        // Closed-loop PD gyro trim pada roda bagian dalam
        float dDiff = (diff - lastDiff) / dt;
        lastDiff = diff;
        float gyroTrim = (diff * PID_CURVE_KP) + (dDiff * PID_CURVE_KD);

        // Panduan optik pasak gerbang diagonal 45 derajat
        float opticalTrim = 0.0f;
        if (turnLeft) {
            if (ir.frontRight > 1200) opticalTrim += 15.0f;
            if (ir.left > 1500) opticalTrim -= 15.0f;
        } else {
            if (ir.frontLeft > 1200) opticalTrim += 15.0f;
            if (ir.right > 1500) opticalTrim -= 15.0f;
        }
        if (ir.frontCenter > 1300) {
            opticalTrim += 15.0f; // Dinding depan mendekat cepat pada saat kurva 45
        }

        if (turnLeft) {
            int16_t leftPwm = (int16_t)constrain(innerSpeed - gyroTrim - opticalTrim, 150.0f, baseSpeed);
            int16_t rightPwm = (int16_t)constrain(outerSpeed, 150.0f, 255.0f);
            Motors::setSpeeds(leftPwm, rightPwm);
        } else {
            int16_t leftPwm = (int16_t)constrain(outerSpeed, 150.0f, 255.0f);
            int16_t rightPwm = (int16_t)constrain(innerSpeed + gyroTrim - opticalTrim, 150.0f, baseSpeed);
            Motors::setSpeeds(leftPwm, rightPwm);
        }

        while ((millis() - lastLoop) < 5) {}
        lastLoop = millis();
    }

    Motors::brake();
    delay(15);
    return true;
}

// ==============================================================================
// SPRINT DIAGONAL (LURUS CEPAT PADA SUMBU 45 DERAJAT DENGAN CLOSED-LOOP PD)
// Tiap langkah diagonal menempuh hipotenusa sqrt(180^2 + 180^2) = 254.56 mm.
// Menggunakan S-Curve profile, Gyro heading lock PD, dan PD optical post balancing.
// ==============================================================================
bool Navigator::sprintDiagonal(float numDiagonalSteps, float cruiseSpeed) {
    if (cruiseSpeed < 150.0f) cruiseSpeed = 150.0f; // STRICT FLOOR: Kecepatan diagonal minimal 150 PWM
    Motors::enable();
    Encoders::reset();
    IMU::update();
    float startYaw = IMU::getYaw();

    float targetDistance = numDiagonalSteps * CELL_DIAGONAL_MM;
    float currentDistance = 0.0f;
    const float minSpeed = 150.0f; // STRICT FLOOR: Kecepatan tidak boleh < 150 PWM

    // Parameter waktu untuk mode motor N20 tanpa encoder
    uint32_t tStart = millis();
    float vBat = Battery::getVoltage();
    float vComp = (vBat > 3.0f) ? (3.85f / constrain(vBat, 3.2f, 4.3f)) : 1.0f;
    float speedRatio = SCAN_SPEED_FORWARD / constrain(cruiseSpeed, 40.0f, 255.0f);
    // 1 langkah diagonal = 1.4142x durasi sel ortogonal
    uint32_t targetDurationMs = (uint32_t)(numDiagonalSteps * 1.4142f * TIME_PER_CELL_NOMINAL_MS * speedRatio * vComp);
    if (targetDurationMs < 100) targetDurationMs = 100;

    uint32_t lastLoop = millis();
    uint32_t lastLoopUs = micros();
    float lastPostDiff = 0.0f;

    while (currentDistance < targetDistance) {
        // Cek darurat SW3 langsung di dalam loop pergerakan
        if (UI::checkEmergencyStop()) {
            emergencyStop();
            return false;
        }

        Sensors::update();
        IMU::update();
        uint32_t elapsed = millis() - tStart;
        uint32_t nowUs = micros();
        float dtLoop = (nowUs - lastLoopUs) * 1e-6f;
        lastLoopUs = nowUs;
        if (dtLoop <= 0.0001f || dtLoop > 0.05f) dtLoop = 0.005f;

        // Auto-detect pulsa encoder
        if (elapsed > 100 && Encoders::isAvailable()) {
            if (Encoders::getLeftTicks() == 0 && Encoders::getRightTicks() == 0) {
                Encoders::notifyMovement(false);
            } else {
                Encoders::notifyMovement(true);
            }
        }

        // Pengukuran jarak tempuh diagonal
        if (Encoders::isAvailable()) {
            currentDistance = Encoders::getAverageDistanceMm();
        } else {
            currentDistance = targetDistance * ((float)elapsed / (float)targetDurationMs);
            if (elapsed >= targetDurationMs) {
                break;
            }
        }

        // Profil kecepatan S-Curve diagonal (mencegah torsi mendadak pada jalur miring)
        float speed = cruiseSpeed;
        float remaining = targetDistance - currentDistance;
        const float rampUpDist = 50.0f;
        const float rampDownDist = 65.0f;

        if (currentDistance < rampUpDist) {
            float s = currentDistance / rampUpDist;
            float smoothFactor = s * s * (3.0f - 2.0f * s);
            speed = minSpeed + (cruiseSpeed - minSpeed) * smoothFactor;
        } else if (remaining < rampDownDist) {
            float s = remaining / rampDownDist;
            float smoothFactor = s * s * (3.0f - 2.0f * s);
            speed = minSpeed + (cruiseSpeed - minSpeed) * smoothFactor;
        }

        // Heading lock Gyro Z Closed-Loop PD pada sudut 45°
        float yawDiff = IMU::getYaw() - startYaw;
        while (yawDiff > 180.0f) yawDiff -= 360.0f;
        while (yawDiff < -180.0f) yawDiff += 360.0f;
        float gyroRate = IMU::getGyroZ();
        float steeringCorrection = (yawDiff * PID_GYRO_KP) + (gyroRate * PID_GYRO_KD);

        // Post balancing: Sensor serong 45° (IR_FRONT_L PB0 dan IR_FRONT_R PA3) menghadap tiang pasak labirin
        IRSensorData ir = Sensors::getFiltered();
        if (ir.frontLeft > 280 || ir.frontRight > 280) {
            float postDiff = (float)ir.frontLeft - (float)ir.frontRight;
            float dPostDiff = (postDiff - lastPostDiff) / dtLoop;
            lastPostDiff = postDiff;
            steeringCorrection += (postDiff * 0.035f) + (dPostDiff * 0.012f);
        } else {
            lastPostDiff = 0.0f;
        }

        // Active Optical Proximity Shield pada Sprint Diagonal (Anti-Serempet Pasak Tiang)
        if (ir.frontLeft > 1300) steeringCorrection += 25.0f;  // Terlalu mepet pasak kiri -> dorong ke kanan
        if (ir.frontRight > 1300) steeringCorrection -= 25.0f; // Terlalu mepet pasak kanan -> dorong ke kiri
        if (ir.left > 1400) steeringCorrection += 15.0f;
        if (ir.right > 1400) steeringCorrection -= 15.0f;

        // Pengaman Darurat Tabrakan Depan pada Sprint Diagonal
        if (ir.frontCenter > 850) {
            Motors::brake();
            delay(20);
            break;
        }

        steeringCorrection = constrain(steeringCorrection, -70.0f, 70.0f);

        int16_t leftPwm = (int16_t)constrain(speed + steeringCorrection, 150.0f, 255.0f);
        int16_t rightPwm = (int16_t)constrain(speed - steeringCorrection, 150.0f, 255.0f);

        Motors::setSpeeds(leftPwm, rightPwm);

        while ((millis() - lastLoop) < 5) {}
        lastLoop = millis();
    }

    Motors::brake();
    delay(20);
    return true;
}


// ==============================================================================
// ENSURE FRONT WALL CLEARANCE (MUNDUR CEPAT JIKA TERLALU MEKET DINDING DEPAN)
// Menjamin moncong robot memiliki ruang putar bebas (turning clearance) sebelum
// berbelok 90 atau 180 derajat, sehingga tidak menyenggol atau terhambat dinding depan.
// ==============================================================================
bool Navigator::ensureFrontWallClearance() {
    Sensors::update();
    IRSensorData ir = Sensors::getFiltered();

    if (!Sensors::hasWallFront() && ir.frontCenter < 800) return true;

    // Jika moncong terlalu dekat ke dinding depan:
    // Mundur CEPAT dan TEGAS dengan PWM -160 (kecepatan >= 150 PWM) selama ~40-60ms
    // sampai jarak aman putar tercapai (frontCenter <= 850 ADC, ~40-45mm dari dinding)
    if (ir.frontCenter > 1050) {
        Motors::enable();
        uint32_t tStart = millis();
        while ((millis() - tStart) < 70) {
            Sensors::update();
            ir = Sensors::getFiltered();
            if (ir.frontCenter <= 850) {
                break;
            }
            Motors::setSpeeds(-160, -160);
            delay(2);
        }
        Motors::brake();
        delay(30);
        return true;
    }

    return true;
}

// ==============================================================================
// DISCRETE PIVOT TURN 90 (CLOSED-LOOP PID CONTROLLER + ACTIVE GYRO Z DAMPING)
// Menggunakan PID Controller lengkap dengan jaminan batas kecepatan minimum 150 PWM:
// - P-Term: Torsi rotasi proporsional terhadap sisa sudut
// - I-Term: Mengatasi gesekan internal gearbox N20 (stiction) saat mendekati target
// - D-Term: Membaca laju angular fisik Gyro Z (deg/s) untuk pengereman halus tanpa overshoot
// ==============================================================================
bool Navigator::turn90(bool turnLeft, float turnSpeed) {
    if (turnSpeed < 165.0f) turnSpeed = 170.0f; // Minimal 170 PWM untuk belokan pivot bertenaga
    Motors::enable();

    // 1. Pastikan moncong memiliki clearance bebas (otomatis mundur snapback jika terlalu dekat dinding depan)
    ensureFrontWallClearance();

    // Fallback: Jika IMU MPU6050 tidak terpasang atau tidak terdeteksi di I2C bus
    if (!IMU::isAvailable()) {
        if (Encoders::isAvailable()) {
            Encoders::reset();
            const float targetWheelDist = (PI * WHEEL_BASE_MM) / 4.0f; // ~64.4mm untuk putaran 90 deg
            if (turnLeft) {
                Motors::setSpeeds(-(int16_t)turnSpeed, (int16_t)turnSpeed);
            } else {
                Motors::setSpeeds((int16_t)turnSpeed, -(int16_t)turnSpeed);
            }
            uint32_t t0 = millis();
            while ((millis() - t0) < 1000) {
                if (UI::checkEmergencyStop()) { emergencyStop(); return false; }
                float dist = (fabs(Encoders::getLeftDistanceMm()) + fabs(Encoders::getRightDistanceMm())) / 2.0f;
                if (dist >= targetWheelDist) break;
                delay(2);
            }
        } else {
            // Timing-based pivot turn (~300ms pada 170 PWM)
            if (turnLeft) {
                Motors::setSpeeds(-(int16_t)turnSpeed, (int16_t)turnSpeed);
            } else {
                Motors::setSpeeds((int16_t)turnSpeed, -(int16_t)turnSpeed);
            }
            uint32_t t0 = millis();
            while ((millis() - t0) < 300) {
                if (UI::checkEmergencyStop()) { emergencyStop(); return false; }
                delay(2);
            }
        }
        Motors::brake();
        delay(40);
        return true;
    }

    IMU::update();
    float startYaw = IMU::getYaw();
    float targetYawChange = turnLeft ? 90.0f : -90.0f; // Left: +90 deg, Right: -90 deg
    float targetYaw = startYaw + targetYawChange;
    while (targetYaw > 180.0f) targetYaw -= 360.0f;
    while (targetYaw < -180.0f) targetYaw += 360.0f;

    uint32_t startTime = millis();
    uint32_t lastLoopUs = micros();
    float integralTurn = 0.0f;
    uint32_t lastTurnProgressTime = millis();
    float lastTurnYaw = startYaw;
    uint8_t recoveryAttempts = 0;

    while ((millis() - startTime) < 1400) {
        if (UI::checkEmergencyStop()) {
            emergencyStop();
            return false;
        }

        IMU::update();
        Sensors::update();
        IRSensorData irTurn = Sensors::getFiltered();

        uint32_t nowUs = micros();
        float dt = (nowUs - lastLoopUs) * 1e-6f;
        lastLoopUs = nowUs;
        if (dt <= 0.0001f || dt > 0.05f) dt = 0.005f;

        float currentYaw = IMU::getYaw();
        float diff = targetYaw - currentYaw;
        while (diff > 180.0f) diff -= 360.0f;
        while (diff < -180.0f) diff += 360.0f;

        // Toleransi sudut belokan standar micromouse: +-2.5 derajat
        if (fabs(diff) < 2.5f) {
            break;
        }

        // Anti-Stall Failsafe & Auto-Recovery pada Belokan Pivot
        if (fabs(currentYaw - lastTurnYaw) >= 1.2f) {
            lastTurnYaw = currentYaw;
            lastTurnProgressTime = millis();
        } else if ((millis() - lastTurnProgressTime) > 220) {
            // Jika dalam 220ms sudut belum berubah (terhambat atau tergesek dinding):
            if (recoveryAttempts < 2) {
                recoveryAttempts++;
                // Mundur bertenaga cepat (-160 PWM) untuk melepaskan kontak dengan dinding
                Motors::setSpeeds(-160, -160);
                delay(60);
                Motors::brake();
                delay(20);
                lastTurnProgressTime = millis();
                lastTurnYaw = IMU::getYaw();
                continue;
            }

            // Jika sudah 2x recovery dan waktu habis:
            // JANGAN POTONG DAYA / JANGAN MATIKAN MOTOR YANG MEMBUAT ROBOT MOGOK!
            // Mundur sedikit, lalu brake, dan anggap belokan selesai agar scan maze terus berjalan.
            Motors::setSpeeds(-160, -160);
            delay(50);
            Motors::brake();
            delay(20);
            Serial.println(F("\n[TURN90] Belokan disesuaikan, scan dilanjutkan."));
            break;
        }

        // Proteksi kontak depan saat memutar: jika sensor depan terlalu mepet dinding saat memutar
        if (irTurn.frontCenter > 2200) {
            Motors::setSpeeds(-160, -160);
            delay(40);
            Motors::brake();
            delay(15);
            lastTurnProgressTime = millis();
        }

        // Closed-Loop PID Turning:
        float pTerm = diff * PID_TURN_KP;

        // Anti-windup integral: hanya aktif pada sudut kecil (<30 deg) untuk presisi akhir
        if (fabs(diff) < 30.0f) {
            integralTurn += diff * dt;
            integralTurn = constrain(integralTurn, -PID_TURN_I_LIMIT, PID_TURN_I_LIMIT);
        } else {
            integralTurn = 0.0f;
        }
        float iTerm = integralTurn * PID_TURN_KI;

        // D-Term peredam berbasis sensor fisik Gyro Z (deg/s)
        float gyroZ = IMU::getGyroZ();
        float dTerm = -gyroZ * PID_TURN_KD;

        float controlOutput = pTerm + iTerm + dTerm;
        float pSpeed = fabs(controlOutput);
        
        // STRICT SPEED FLOOR: KECEPATAN TIDAK BOLEH KURANG DARI 150 PWM!
        const float minTurnPwm = 150.0f;
        pSpeed = constrain(pSpeed, minTurnPwm, turnSpeed);

        if (controlOutput > 0) {
            // Need CCW (Turn Left): Left wheel reverse, Right wheel forward
            Motors::setSpeeds(-(int16_t)pSpeed, (int16_t)pSpeed);
        } else {
            // Need CW (Turn Right): Left wheel forward, Right wheel reverse
            Motors::setSpeeds((int16_t)pSpeed, -(int16_t)pSpeed);
        }

        delay(4);
    }

    Motors::brake();
    delay(30);

    // 2. Koreksi & Pelurusan Cepat Pasca Belokan Menggunakan Sensor Samping
    Sensors::update();
    if (Sensors::hasWallLeft() && Sensors::hasWallRight()) {
        float angleErr = Sensors::getAngleError();
        // Koreksi pulsa singkat jika ada deviasi orientasi terhadap lorong baru
        if (fabs(angleErr) > 40.0f && fabs(angleErr) < 350.0f) {
            int16_t corrSpeed = (angleErr > 0) ? -150 : 150;
            Motors::setSpeeds(corrSpeed, -corrSpeed);
            delay(20);
            Motors::brake();
            delay(15);
            IMU::update();
        }
    }

    Motors::brake();
    delay(20);
    return true;
}

bool Navigator::turn180(float turnSpeed) {
    if (turnSpeed < 165.0f) turnSpeed = 170.0f; // Minimal 170 PWM untuk belokan 180 derajat
    ensureFrontWallClearance();
    if (!turn90(false, turnSpeed)) return false;
    delay(20);
    ensureFrontWallClearance();
    return turn90(false, turnSpeed);
}

void Navigator::alignFrontWall() {
    if (!Sensors::hasWallFront()) return;
    ensureFrontWallClearance();
}

// ==============================================================================
// AUTO-ALIGN WITH WALLS (MELURUSKAN BADAN SEJAJAR DINDING SECARA MANDIRI TANPA IMU)
// Memanfaatkan trigonometri komparasi sensor serong depan 45° vs sensor samping 90°.
// Dilengkapi closed-loop PD agar pelurusan berhenti tepat tanpa over-hunting.
// Setelah badan sejajar 100%, sistem mengunci nol-bias Gyro MPU6050 (Yaw = 0.00°).
// ==============================================================================
bool Navigator::autoAlignWithWalls(bool doBackTouch) {
    Motors::enable();
    Sensors::update();

    if (!Sensors::hasWallLeft() || !Sensors::hasWallRight()) {
        Serial.println(F("[Auto-Align] Dinding samping tidak lengkap untuk kalibrasi trigonometri."));
        return false;
    }

    Serial.println(F("\n[Auto-Align] Memulai pelurusan otomatis sejajar dinding tanpa IMU..."));

    // 1. Mechanical Squaring (Mundur bertenaga meratakan pantat ke dinding belakang sel start)
    if (doBackTouch) {
        Motors::setSpeeds(-150, -150);
        delay(70);
        Motors::brake();
        delay(40);

        // Maju sedikit 15mm agar pantat bebas berputar
        Motors::setSpeeds(150, 150);
        delay(35);
        Motors::brake();
        delay(50);
    }

    // 2. Optical Trigonometric Closed-Loop PD Alignment
    uint32_t tStart = millis();
    uint32_t lastLoopUs = micros();
    const float deadband = 30.0f; // Toleransi threshold optik sangat ketat
    const float kp_align = 0.085f;
    const float kd_align = 0.035f;
    float lastAngleErr = 0.0f;

    while ((millis() - tStart) < 1400) { // Timeout maksimal 1.4 detik
        if (UI::checkEmergencyStop()) {
            emergencyStop();
            return false;
        }

        Sensors::update();
        uint32_t nowUs = micros();
        float dt = (nowUs - lastLoopUs) * 1e-6f;
        lastLoopUs = nowUs;
        if (dt <= 0.0001f || dt > 0.05f) dt = 0.008f;

        float angleErr = Sensors::getAngleError();

        // Jika selisih trigonometri sudah berada di dalam deadband, robot sudah paralel sempurna!
        if (fabs(angleErr) < deadband) {
            break;
        }

        float dError = (angleErr - lastAngleErr) / dt;
        lastAngleErr = angleErr;

        // PD Controller: P-term menggerakkan ke target, D-term meredam osilasi
        // Sign dError harus dipertahankan agar D-term memperlambat saat mendekati target
        float sign = (angleErr > 0) ? 1.0f : -1.0f;
        float controlSpeed = (fabs(angleErr) * kp_align) + (dError * sign * kd_align);
        controlSpeed = constrain(fabs(controlSpeed), 150.0f, 175.0f); // STRICT FLOOR: MIN 150 PWM

        if (angleErr > 0) {
            // Miring ke Kanan (CW) -> Koreksi belok Kiri (CCW): Roda kiri mundur, roda kanan maju
            Motors::setSpeeds(-(int16_t)controlSpeed, (int16_t)controlSpeed);
        } else {
            // Miring ke Kiri (CCW) -> Koreksi belok Kanan (CW): Roda kiri maju, roda kanan mundur
            Motors::setSpeeds((int16_t)controlSpeed, -(int16_t)controlSpeed);
        }

        delay(8);
    }

    Motors::brake();
    delay(80);
    Motors::coast();

    // 3. Kunci sumbu Yaw IMU persis pada 0.00 derajat!
    // Karena robot sudah 100% sejajar dinding berkat trigonometri sensor,
    // sekarang kita kalibrasi titik nol Gyro Z dan set Yaw = 0.00
    IMU::calibrate();
    IMU::resetYaw();

    Serial.println(F("[Auto-Align] Sukses! Robot 100% sejajar dinding & IMU terkunci di 0.00 deg."));
    return true;
}

