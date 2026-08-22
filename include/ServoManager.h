#pragma once
#include <ESP32Servo.h>
#include <Config.h>

class ServoManager {
    private:
        Servo servos[5];
        int currentAngles[5];
        int maxAngles[5]; // Finger maximum angles Calculated from Calibration
    
    public:
        ServoManager();
        void begin();

        // Setting of each motor angle
        void setAngle(uint8_t index, int angle);
        int getAngle(uint8_t index);

        // 100ms Delay for Training (velocity level)
        void moveToAnglesStaggered(const int targerAngle[5], uint16_t speedDelayMs);
        
        // Safety Control: Returning All Motor to home position
        void returnToInitialPosition(uint16_t stepDelayMs = 15);

        // Update target Angles correspondig to Level
        void setMaxAngleForFinger(uint8_t index, int angle);
};