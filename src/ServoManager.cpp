#include "ServoManager.h"

ServoManager::ServoManager() {
    for (int i = 0; i < 5; i++) {
        currentAngles[i] = 0;
        maxAngles[i] = 180; // Basic
    }
}

void ServoManager::begin() {
    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);

    for (int i = 0; i < 5; i++) {
        servos[i].setPeriodHertz(50);
        servos[i].attach(SERVO_PINS[i], 500, 2400);
        servos[i].write(0); // Initial position
        currentAngles[i] = 0;
    }
}

void ServoManager::setAngle(uint8_t index, int angle) {
    if (index < 5 && angle >= 0 && angle <= 180) {
        servos[index].write(angle);
        currentAngles[index] = angle;
    }
}

int ServoManager::getAngle(uint8_t index) {
    if (index < 5) return currentAngles[index];
    return 0;
}

void ServoManager::setMaxAngleForFinger(uint8_t index, int angle) {
    if (index < 5) {
        maxAngles[index] = angle;
    }
}

// Activate the finger motors one by one with a 100ms interval.
void ServoManager::moveToAnglesStaggered(const int targerAngles[5], uint16_t speedDelayMs) {
    for (int i = 0; i < 5; i++) {
        setAngle(i, targerAngles[i]);
        delay(MOTOR_STAGGER_DELAY_MS); // Apply a 100ms time lag
    }
}

// Safe return mechanism (slowly returns all motors to 0 degrees)
void ServoManager::returnToInitialPosition(uint16_t stepDelayMs) {
    bool allAtZero = false;

    while (!allAtZero) {
        allAtZero = true;
        for (int i = 0; i < 5; i++) {
            if (currentAngles[i] > 0) {
                currentAngles[i]--;
                servos[i].write(currentAngles[i]);
                allAtZero = false;
            }
        }
        delay(stepDelayMs); //Control the motor to move slowly without jerking.
    }
}