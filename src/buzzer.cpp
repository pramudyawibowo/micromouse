#include "buzzer.h"
#include "config.h"

#ifndef BUZZER_ENABLED
#define BUZZER_ENABLED true
#endif

bool Buzzer::_enabled = BUZZER_ENABLED;

void Buzzer::init() {
    pinMode(PIN_BUZZER, OUTPUT);
    digitalWrite(PIN_BUZZER, LOW);
}

void Buzzer::setEnabled(bool enabled) {
    _enabled = enabled;
    if (!_enabled) {
        stop();
    }
}

bool Buzzer::isEnabled() {
    return _enabled;
}

void Buzzer::tonePlay(uint16_t freq, uint16_t durationMs) {
    if (!_enabled || freq == 0) {
        return;
    }
    // High-resolution software or timer tone for reliable sound across boards
    uint32_t periodUs = 1000000UL / freq;
    uint32_t halfPeriod = periodUs / 2;
    uint32_t cycles = (uint32_t)freq * durationMs / 1000UL;
    for (uint32_t i = 0; i < cycles; i++) {
        digitalWrite(PIN_BUZZER, HIGH);
        delayMicroseconds(halfPeriod);
        digitalWrite(PIN_BUZZER, LOW);
        delayMicroseconds(halfPeriod);
    }
}

void Buzzer::beep(uint16_t freq, uint16_t durationMs) {
    if (!_enabled) return;
    tonePlay(freq, durationMs);
    delay(20);
}

void Buzzer::stop() {
    digitalWrite(PIN_BUZZER, LOW);
}

void Buzzer::playStartup() {
    if (!_enabled) return;
    tonePlay(1046, 70); // C6
    delay(30);
    tonePlay(1318, 70); // E6
    delay(30);
    tonePlay(1568, 70); // G6
    delay(30);
    tonePlay(2093, 140); // C7
}

void Buzzer::playClick() {
    if (!_enabled) return;
    tonePlay(2400, 25);
}

void Buzzer::playConfirm() {
    if (!_enabled) return;
    tonePlay(1760, 60);
    delay(30);
    tonePlay(2637, 100);
}

void Buzzer::playAlert() {
    if (!_enabled) return;
    for (int i = 0; i < 3; i++) {
        tonePlay(800, 100);
        delay(50);
    }
}

void Buzzer::playVictory() {
    if (!_enabled) return;
    uint16_t notes[] = {1318, 1318, 1318, 1046, 1318, 1568, 784};
    uint16_t durations[] = {100, 100, 100, 80, 120, 180, 220};
    for (size_t i = 0; i < sizeof(notes) / sizeof(notes[0]); i++) {
        tonePlay(notes[i], durations[i]);
        delay(40);
    }
}
