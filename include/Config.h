#pragma once
#include <Arduino.h>

// =====================================
// 1. Pin Definitions
// =====================================
// Servo motor 5pins (0: thumb, 1: index, 2: middle, 3: ring, 4: pinky)
const uint8_t SERVO_PINS[5] = {18, 19, 21, 22, 23};

// EMG sensor pins
const uint8_t EMG_EXTENSOR_PIN = 34; // extensor
const uint8_t EMG_FLEXOR_PIN = 35;   // flexor

// =====================================
// 2. BLE settings
// =====================================
#define BLE_DEVICE_NAME        "RehabGlove"
#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

// =====================================
// 3. Function of Rehabilitation motion and Control
// =====================================
const uint16_t MOTOR_STAGGER_DELAY_MS = 100; // Delay between each motor movement
const float STALL_CURRENT_THRES_MA = 800.0; // Threshold for detecting stall current
const uint32_t CALIBRATION_DURATION_MS = 5000; // Duration for EMG calibration

// State Machine
enum class SystemState {
    IDLE,                // Waiting for EMG signal
    CALIB_REST,          // Calibrating EMG baseline
    CALIB_EXTENSOR_MAX,  // Calibrating extensor max
    CALIB_FLEXOR_MAX,    // Calibrating flexor max
    CALIB_PASSIVE_RANGE, // Calibrating passive range of motion
    TRAINING_READY,       // Ready for training
    TRAINING_ACTIVE,     // Active training in progress
    TRAINING_RETURNING,  // EMERGENCY STOP/RETURNING to home position
};

