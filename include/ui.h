#pragma once

#include <Arduino.h>

enum ButtonID {
    BTN_SW1 = 0,
    BTN_SW2,
    BTN_SW3,
    BTN_SW4
};

class UI {
public:
    static void init();
    static void update(); // Call in loop()

    // Button states
    static bool isButtonPressed(ButtonID btn);
    static bool wasButtonClicked(ButtonID btn);
    static bool checkEmergencyStop(); // Deteksi SW3 rem darurat dengan filter debounce 30ms
    static void resetEmergencyStop();

    // LEDs
    static void setLed1(bool on);
    static void setLed2(bool on);
    static void setOnboardLed(bool on);
    static void toggleLed1();
    static void toggleLed2();
    static void toggleOnboardLed();
    static void blinkAll(uint8_t times = 3, uint16_t delayMs = 100);

private:
    static bool btnState[4];
    static bool lastBtnState[4];
    static bool btnClicked[4];
    static uint32_t lastDebounceTime[4];
};
