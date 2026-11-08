#include "battery.h"
#include "config.h"

float Battery::lastVoltage = 4.0f;

void Battery::init() {
    pinMode(PIN_BAT_SENSE, INPUT_ANALOG);
}

float Battery::getVoltage() {
    uint32_t sum = 0;
    for (int i = 0; i < 32; i++) {
        sum += analogRead(PIN_BAT_SENSE);
        delayMicroseconds(20);
    }
    float rawAdc = (float)sum / 32.0f;
    // 12-bit ADC (4095), Vref 3.3V, resistor divider factor
    float vPin = (rawAdc / 4095.0f) * BATTERY_ADC_VREF;
    lastVoltage = vPin * BATTERY_DIVIDER_RATIO;
    return lastVoltage;
}

uint8_t Battery::getPercentage() {
    // Gunakan cached voltage dari panggilan getVoltage() terakhir
    float v = lastVoltage;
    if (v >= 4.2f) return 100;
    if (v <= 3.3f) return 0;
    return (uint8_t)(((v - 3.3f) / (4.2f - 3.3f)) * 100.0f);
}

bool Battery::isLow() {
    float v = getVoltage();
    if (v < 0.8f) return false; // Abaikan jika tidak ada baterai (misal daya murni USB / test mode)
    return (v < BATTERY_LOW_VOLTAGE);
}

bool Battery::isCritical() {
    float v = getVoltage();
    if (v < 0.8f) return false; // Abaikan jika tidak ada baterai (misal daya murni USB / test mode)
    return (v < BATTERY_CRITICAL_VOLTAGE);
}
