#pragma once

#include <Arduino.h>

class Buzzer {
public:
    static void init();
    static void setEnabled(bool enabled);
    static bool isEnabled();
    static void beep(uint16_t freq = 2000, uint16_t durationMs = 80);
    static void tonePlay(uint16_t freq, uint16_t durationMs);
    static void stop();
    
    // Preset sound effects
    static void playStartup();
    static void playClick();
    static void playConfirm();
    static void playAlert();
    static void playVictory();

private:
    static bool _enabled;
};
