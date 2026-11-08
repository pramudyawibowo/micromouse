#include "motors.h"
#include "config.h"

void Motors::init() {
    __HAL_RCC_GPIOA_CLK_ENABLE();
    GPIO_InitTypeDef gpio_sleep = {0};
    gpio_sleep.Pin = GPIO_PIN_15;
    gpio_sleep.Mode = GPIO_MODE_OUTPUT_PP;
    gpio_sleep.Pull = GPIO_PULLDOWN;
    gpio_sleep.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &gpio_sleep);

    pinMode(PIN_MOTOR_SLEEP, OUTPUT);
    digitalWrite(PIN_MOTOR_SLEEP, LOW); // Start disabled

    pinMode(PIN_MOTOR_L_IN1, OUTPUT);
    pinMode(PIN_MOTOR_L_IN2, OUTPUT);
    pinMode(PIN_MOTOR_R_IN1, OUTPUT);
    pinMode(PIN_MOTOR_R_IN2, OUTPUT);

    coast();
}

void Motors::enable() {
    digitalWrite(PIN_MOTOR_SLEEP, HIGH);
}

void Motors::disable() {
    coast();
    digitalWrite(PIN_MOTOR_SLEEP, LOW);
}

void Motors::setLeftSpeed(int16_t speed) {
    speed = constrain(speed, -255, 255);
    if (speed > 0) {
        analogWrite(PIN_MOTOR_L_IN1, speed);
        analogWrite(PIN_MOTOR_L_IN2, 0);
    } else if (speed < 0) {
        analogWrite(PIN_MOTOR_L_IN1, 0);
        analogWrite(PIN_MOTOR_L_IN2, -speed);
    } else {
        analogWrite(PIN_MOTOR_L_IN1, 0);
        analogWrite(PIN_MOTOR_L_IN2, 0);
    }
}

void Motors::setRightSpeed(int16_t speed) {
    speed = constrain(speed, -255, 255);
    if (speed > 0) {
        analogWrite(PIN_MOTOR_R_IN1, speed);
        analogWrite(PIN_MOTOR_R_IN2, 0);
    } else if (speed < 0) {
        analogWrite(PIN_MOTOR_R_IN1, 0);
        analogWrite(PIN_MOTOR_R_IN2, -speed);
    } else {
        analogWrite(PIN_MOTOR_R_IN1, 0);
        analogWrite(PIN_MOTOR_R_IN2, 0);
    }
}

void Motors::setSpeeds(int16_t leftSpeed, int16_t rightSpeed) {
    setLeftSpeed(leftSpeed);
    setRightSpeed(rightSpeed);
}

void Motors::brake() {
    analogWrite(PIN_MOTOR_L_IN1, 255);
    analogWrite(PIN_MOTOR_L_IN2, 255);
    analogWrite(PIN_MOTOR_R_IN1, 255);
    analogWrite(PIN_MOTOR_R_IN2, 255);
}

void Motors::coast() {
    analogWrite(PIN_MOTOR_L_IN1, 0);
    analogWrite(PIN_MOTOR_L_IN2, 0);
    analogWrite(PIN_MOTOR_R_IN1, 0);
    analogWrite(PIN_MOTOR_R_IN2, 0);
}
