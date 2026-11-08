#include "ui.h"
#include "config.h"

bool UI::btnState[4] = {false, false, false, false};
bool UI::lastBtnState[4] = {false, false, false, false};
bool UI::btnClicked[4] = {false, false, false, false};
uint32_t UI::lastDebounceTime[4] = {0, 0, 0, 0};

static const uint8_t BUTTON_PINS[4] = {PIN_SW1, PIN_SW2, PIN_SW3, PIN_SW4};

void UI::init() {
    // Pastikan backup domain unlocked dan LSE mati agar PC14 (SW4) dan PC15 (SW3)
    // dapat digunakan sebagai pin GPIO digital murni
    __HAL_RCC_PWR_CLK_ENABLE();
    HAL_PWR_EnableBkUpAccess();
    __HAL_RCC_LSE_CONFIG(RCC_LSE_OFF);
    __HAL_RCC_GPIOC_CLK_ENABLE();

    GPIO_InitTypeDef gpio_sw = {0};
    gpio_sw.Pin = GPIO_PIN_14 | GPIO_PIN_15;
    gpio_sw.Mode = GPIO_MODE_INPUT;
    gpio_sw.Pull = GPIO_PULLUP;
    gpio_sw.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &gpio_sw);

    // Pastikan peripheral clock GPIOB aktif dan setel PB12 (SW2) & PB13 (SW1) sebagai input pullup hardware
    __HAL_RCC_GPIOB_CLK_ENABLE();
    GPIO_InitTypeDef gpio_pb_sw = {0};
    gpio_pb_sw.Pin = GPIO_PIN_12 | GPIO_PIN_13;
    gpio_pb_sw.Mode = GPIO_MODE_INPUT;
    gpio_pb_sw.Pull = GPIO_PULLUP;
    gpio_pb_sw.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &gpio_pb_sw);

    for (int i = 0; i < 4; i++) {
        pinMode(BUTTON_PINS[i], INPUT_PULLUP);
    }

    delay(20);

    // Inisialisasi status awal tombol agar tidak memicu false click saat booting
    for (int i = 0; i < 4; i++) {
        bool initial = (digitalRead(BUTTON_PINS[i]) == LOW);
        btnState[i] = initial;
        lastBtnState[i] = initial;
        btnClicked[i] = false;
        lastDebounceTime[i] = millis();
    }

#if !ENABLE_SERIAL_DEBUG
    // Putus total hardware USB OTG FS dari clock dan reset agar PA11 & PA12 lepas bebas
    __HAL_RCC_USB_OTG_FS_FORCE_RESET();
    __HAL_RCC_USB_OTG_FS_RELEASE_RESET();
    __HAL_RCC_USB_OTG_FS_CLK_DISABLE();

    // Pastikan clock GPIOA aktif
    __HAL_RCC_GPIOA_CLK_ENABLE();

    // Konfigurasi level hardware STM32 HAL: Paksa PA11 murni Push-Pull Output (melepas AF10 USB)
    GPIO_InitTypeDef gpio_pa11 = {0};
    gpio_pa11.Pin = GPIO_PIN_11;
    gpio_pa11.Mode = GPIO_MODE_OUTPUT_PP;
    gpio_pa11.Pull = GPIO_NOPULL;
    gpio_pa11.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &gpio_pa11);
#endif

    // Pastikan peripheral clock GPIOB aktif dan setel PB3 sebagai Push-Pull Output murni
    __HAL_RCC_GPIOB_CLK_ENABLE();
    pinMode(PIN_LED2, OUTPUT);
    pinMode(PIN_LED_BOARD, OUTPUT);

    setLed1(false);
    setLed2(false);
    setOnboardLed(false);
}

void UI::update() {
    uint32_t now = millis();
    for (int i = 0; i < 4; i++) {
        btnClicked[i] = false;
        // Buttons are active LOW
        bool rawPressed = (digitalRead(BUTTON_PINS[i]) == LOW);

        if (rawPressed != lastBtnState[i]) {
            lastDebounceTime[i] = now;
            lastBtnState[i] = rawPressed;
        }

        if ((now - lastDebounceTime[i]) >= 15) { // 15ms debounce window cepat & responsif
            if (rawPressed != btnState[i]) {
                btnState[i] = rawPressed;
                if (btnState[i]) {
                    btnClicked[i] = true; // Trigger on press down
                }
            }
        }
    }
}

bool UI::isButtonPressed(ButtonID btn) {
    return btnState[btn];
}

bool UI::wasButtonClicked(ButtonID btn) {
    return btnClicked[btn];
}

static uint32_t emgPressStart = 0;

void UI::resetEmergencyStop() {
    emgPressStart = 0;
}

bool UI::checkEmergencyStop() {
    // Tombol SW3 (PC15) adalah tombol REM DARURAT (Emergency Brake)
    // Berfungsi sebagai pemutus daya darurat motor seketika
    bool sw3_pressed = (digitalRead(PIN_SW3) == LOW);

    if (sw3_pressed) {
        uint32_t now = millis();
        if (emgPressStart == 0) {
            emgPressStart = now;
        } else if ((now - emgPressStart) >= 30) {
            emgPressStart = 0;
            return true;
        }
    } else {
        emgPressStart = 0;
    }
    return false;
}

void UI::setLed1(bool on) {
#if ENABLE_LEDS
#if !ENABLE_SERIAL_DEBUG
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
#endif
#else
    (void)on;
#if !ENABLE_SERIAL_DEBUG
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);
#endif
#endif
}

void UI::setLed2(bool on) {
#if ENABLE_LEDS
    digitalWrite(PIN_LED2, on ? HIGH : LOW);
#else
    (void)on;
    digitalWrite(PIN_LED2, LOW);
#endif
}

void UI::setOnboardLed(bool on) {
    // Black Pill PC13 LED is active LOW
    digitalWrite(PIN_LED_BOARD, on ? LOW : HIGH);
}

void UI::toggleLed1() {
#if ENABLE_LEDS && !ENABLE_SERIAL_DEBUG
    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_11);
#endif
}

void UI::toggleLed2() {
#if ENABLE_LEDS
    digitalWrite(PIN_LED2, !digitalRead(PIN_LED2));
#else
    digitalWrite(PIN_LED2, LOW);
#endif
}

void UI::toggleOnboardLed() {
    digitalWrite(PIN_LED_BOARD, !digitalRead(PIN_LED_BOARD));
}

void UI::blinkAll(uint8_t times, uint16_t delayMs) {
    for (uint8_t i = 0; i < times; i++) {
#if ENABLE_LEDS
        setLed1(true);
        setLed2(true);
#endif
        setOnboardLed(true);
        delay(delayMs);
#if ENABLE_LEDS
        setLed1(false);
        setLed2(false);
#endif
        setOnboardLed(false);
        delay(delayMs);
    }
}
