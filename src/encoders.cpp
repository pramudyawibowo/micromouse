#include "encoders.h"
#include "config.h"

volatile int32_t Encoders::leftCount = 0;
volatile int32_t Encoders::rightCount = 0;
bool Encoders::available = (ENCODER_HARDWARE_CONFIG != ENCODER_MODE_NONE);
bool Encoders::checked = (ENCODER_HARDWARE_CONFIG != ENCODER_MODE_AUTO);

void Encoders::init() {
#if ENCODER_HARDWARE_CONFIG == ENCODER_MODE_NONE
    available = false;
    checked = true;
    return;
#endif

    pinMode(PIN_ENC_L_A, INPUT_PULLUP);
    pinMode(PIN_ENC_L_B, INPUT_PULLUP);
    pinMode(PIN_ENC_R_A, INPUT_PULLUP);
    pinMode(PIN_ENC_R_B, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(PIN_ENC_L_A), handleLeftInterrupt, CHANGE);
    attachInterrupt(digitalPinToInterrupt(PIN_ENC_R_A), handleRightInterrupt, CHANGE);

    reset();

#if ENCODER_HARDWARE_CONFIG == ENCODER_MODE_FORCE
    available = true;
    checked = true;
#else
    available = true;
    checked = false;
#endif
}

bool Encoders::isAvailable() {
    return available;
}

void Encoders::setAvailable(bool avail) {
    available = avail;
    checked = true;
}

void Encoders::notifyMovement(bool hasPulses) {
    if (!checked) {
        checked = true;
        available = hasPulses;
    }
}

void Encoders::reset() {
    noInterrupts();
    leftCount = 0;
    rightCount = 0;
    interrupts();
}

int32_t Encoders::getLeftTicks() {
    noInterrupts();
    int32_t c = leftCount;
    interrupts();
    return c;
}

int32_t Encoders::getRightTicks() {
    noInterrupts();
    int32_t c = rightCount;
    interrupts();
    return c;
}

float Encoders::getLeftDistanceMm() {
    const float mmPerTick = (PI * WHEEL_DIAMETER_MM) / ENCODER_TICKS_PER_REV;
    return (float)getLeftTicks() * mmPerTick;
}

float Encoders::getRightDistanceMm() {
    const float mmPerTick = (PI * WHEEL_DIAMETER_MM) / ENCODER_TICKS_PER_REV;
    return (float)getRightTicks() * mmPerTick;
}

float Encoders::getAverageDistanceMm() {
    return (getLeftDistanceMm() + getRightDistanceMm()) / 2.0f;
}

void Encoders::handleLeftInterrupt() {
    // Read current state of Channel A and B
    bool a = digitalRead(PIN_ENC_L_A);
    bool b = digitalRead(PIN_ENC_L_B);
    // Disesuaikan agar saat roda kiri berputar maju, nilai hitungan ticks bertambah POSITIF (+)
    if (a == b) {
        leftCount++;
    } else {
        leftCount--;
    }
}

void Encoders::handleRightInterrupt() {
    bool a = digitalRead(PIN_ENC_R_A);
    bool b = digitalRead(PIN_ENC_R_B);
    // Right wheel is mirrored on the opposite side of chassis
    if (a != b) {
        rightCount++;
    } else {
        rightCount--;
    }
}
