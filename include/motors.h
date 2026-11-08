#pragma once

#include <Arduino.h>

class Motors {
public:
    static void init();
    static void enable();
    static void disable(); // Enter sleep mode (nSLEEP = LOW)
    
    // Set individual motor speed (-255 to +255)
    static void setLeftSpeed(int16_t speed);
    static void setRightSpeed(int16_t speed);
    
    // Set both motor speeds simultaneously
    static void setSpeeds(int16_t leftSpeed, int16_t rightSpeed);
    
    static void brake();
    static void coast();
};
