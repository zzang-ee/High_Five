#pragma once

#include <Arduino.h>

// Hardware layout -----------------------------------------------------------
constexpr uint8_t FINGER_COUNT = 5;
constexpr uint8_t ALL_FINGERS_MASK = (1U << FINGER_COUNT) - 1U;

// 0: thumb, 1: index, 2: middle, 3: ring, 4: pinky
constexpr uint8_t SERVO_PINS[FINGER_COUNT] = {16, 17, 18, 19, 21};
// Logical angles are the values exposed to the app: home is always 0 and
// extension always increases. Thumb/middle map logical 0..179 to physical
// 1..180; index/ring/pinky map them to physical 180..1.
constexpr bool SERVO_REVERSED[FINGER_COUNT] = {false, true, false, true, true};
constexpr int SERVO_LOGICAL_HOME_ANGLE = 0;
constexpr int SERVO_LOGICAL_MAX_ANGLE = 179;
constexpr int SERVO_PHYSICAL_MIN_ANGLE = 1;
constexpr int SERVO_PHYSICAL_MAX_ANGLE = 180;
constexpr int SERVO_MIN_PULSE_US = 500;
constexpr int SERVO_MAX_PULSE_US = 2400;
constexpr uint32_t SERVO_COMMAND_INTERVAL_MS = 10;
constexpr uint8_t SERVO_MAX_COMMAND_STEP_DEG = 2;
constexpr uint32_t SERVO_HOME_SETTLE_MS = 300;

constexpr uint8_t EMG_EXTENSOR_PIN = 34;
constexpr uint8_t EMG_FLEXOR_PIN = 35;

// 2S LiPo voltage monitor. Connect battery+ through 47 kohm to GPIO32 and
// GPIO32 through 22 kohm to battery-/ESP32 GND. Place 100 nF in parallel with
// the 22 kohm resistor. The ADC must never be connected to the battery
// directly. At the 8.4 V pack maximum the ADC pin sees about 2.68 V.
constexpr uint8_t BATTERY_ADC_PIN = 32;
constexpr float BATTERY_DIVIDER_TOP_OHMS = 47000.0f;
constexpr float BATTERY_DIVIDER_BOTTOM_OHMS = 22000.0f;
constexpr uint32_t BATTERY_SUBSAMPLE_INTERVAL_US = 100000;
constexpr uint8_t BATTERY_AVERAGE_SAMPLE_COUNT = 10;
constexpr float BATTERY_FILTER_ALPHA = 0.25f;
// Values outside this broad electrical plausibility window are reported as
// JSON null. Percentage itself is clamped to the 2S usable range in firmware.
constexpr float BATTERY_VALID_MIN_VOLTAGE = 5.5f;
constexpr float BATTERY_VALID_MAX_VOLTAGE = 9.0f;

// Wire.begin() takes SDA first and SCL second.
constexpr uint8_t I2C_SCL_PIN = 22;
constexpr uint8_t I2C_SDA_PIN = 23;
constexpr uint32_t I2C_FREQUENCY_HZ = 100000;

// BLE ----------------------------------------------------------------------
#define BLE_DEVICE_NAME        "RehabGlove"
#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

constexpr uint8_t BLE_COMMAND_QUEUE_LENGTH = 8;
constexpr size_t BLE_MAX_COMMAND_LENGTH = 64;
// JSON telemetry uses the database field names directly. A 247-byte ATT MTU
// leaves a 244-byte notification payload; the final log is split into two
// complete JSON objects so no application-level string fragmentation is used.
constexpr uint16_t BLE_LOCAL_MTU = 247;
constexpr uint8_t BLE_PROTOCOL_VERSION = 8;
constexpr uint32_t BLE_SESSION_SUMMARY_RETRY_MS = 2000;

// Development passkey. Replace this value before a deployed device is paired.
constexpr uint32_t BLE_STATIC_PASSKEY = 654321;

// Raw sEMG acquisition and feature extraction ------------------------------
// The sensor output is a 1.5 V-centred raw waveform with useful energy up to
// 500 Hz, so acquisition runs independently from the 10 ms control loop.
constexpr uint32_t EMG_RAW_SAMPLE_RATE_HZ = 2000;
constexpr uint32_t EMG_RAW_SAMPLE_INTERVAL_US =
    1000000UL / EMG_RAW_SAMPLE_RATE_HZ;
constexpr uint32_t EMG_RAW_GAP_LIMIT_US = 1500;
constexpr uint32_t EMG_FEATURE_INTERVAL_MS = 10;
constexpr uint32_t EMG_EVIDENCE_MAX_GAP_MS = 30;
constexpr uint32_t EMG_FEATURE_HEALTH_TIMEOUT_MS = 40;
// Preserve the original 250 ms envelope while acquiring the raw waveform at
// a rate that can actually represent the sensor's 20-500 Hz signal.
constexpr uint16_t EMG_RMS_WINDOW_MS = 250;
constexpr uint16_t EMG_RMS_SAMPLE_COUNT =
    static_cast<uint16_t>((EMG_RAW_SAMPLE_RATE_HZ * EMG_RMS_WINDOW_MS) /
                          1000UL);
// Match the sensor vendor's 20 Hz motion-artifact cutoff. Coefficients are
// calculated for this project's 2 kHz sampling rate instead of copying the
// vendor library's fixed 500/1000 Hz tables.
constexpr float EMG_HIGH_PASS_CUTOFF_HZ = 20.0f;
constexpr float EMG_LOW_PASS_CUTOFF_HZ = 450.0f;
constexpr float EMG_NOTCH_FREQUENCY_HZ = 60.0f;
constexpr float EMG_FILTER_Q = 0.70710678f;
constexpr float EMG_LOW_PASS_Q_1 = 0.54119610f;
constexpr float EMG_LOW_PASS_Q_2 = 1.30656296f;
constexpr float EMG_NOTCH_Q = 30.0f;
constexpr uint16_t EMG_ADC_RAIL_MARGIN_COUNTS = 32;
constexpr float EMG_MAX_SATURATION_FRACTION = 0.01f;
constexpr uint8_t EMG_SAMPLING_TASK_PRIORITY = 4;
constexpr uint16_t EMG_SAMPLING_TASK_STACK = 4096;
constexpr uint8_t EMG_SAMPLING_TASK_CORE = 1;
constexpr float EMG_REST_ROBUST_SIGMA_MULTIPLIER = 3.0f;
constexpr float EMG_MAD_TO_SIGMA_SCALE = 1.4826f;
constexpr float EMG_REST_NOISE_PERCENTILE = 0.95f;
constexpr float EMG_MVC_TOP_FRACTION = 0.10f;
// CH1 remains strict because it drives the training trigger/participation
// decision. Recent repeated CH2 logs showed a stable 13-15 ADC rest-to-flexion
// span, so CH2 uses a lower absolute floor while retaining the same relative
// separation requirement. This still rejects the 8.86 ADC weak-signal case.
constexpr float EMG_EXTENSOR_MIN_CALIBRATION_SPAN = 20.0f;
constexpr float EMG_FLEXOR_MIN_CALIBRATION_SPAN = 10.0f;
constexpr float EMG_MIN_CALIBRATION_SPAN_RATIO = 0.25f;

constexpr uint32_t CURRENT_SAMPLE_INTERVAL_MS = 10;
constexpr float CURRENT_FILTER_ALPHA = 0.25f;
// Absolute soft current is retained as a fallback electrical/mechanical load
// ceiling. ROM uses a per-finger relative-current endpoint, while training
// uses only the selected group's scaled safety ceiling.
constexpr float STALL_CURRENT_THRES_MA = 800.0f;
// Hard overcurrent can also mean a jam, short, or severe impact. It always
// detaches the servos and is never accepted as a calibration endpoint.
constexpr float HARD_CURRENT_THRES_MA = 1600.0f;
constexpr uint32_t STALL_CONFIRM_TIME_MS = 150;
constexpr uint32_t RETURN_STALL_CONFIRM_TIME_MS = 250;

// ROM per-finger current-rise allowances. These 150 mA values are only
// initial bench-test values; they are not a validated human-force limit.
constexpr float ROM_FINGER_CURRENT_RISE_LIMIT_MA[FINGER_COUNT] = {
    150.0f, 150.0f, 150.0f, 150.0f, 150.0f};

// Calibration ---------------------------------------------------------------
constexpr uint32_t CALIBRATION_PREPARE_MS = 2000;
constexpr uint32_t CALIBRATION_DURATION_MS = 5000;
constexpr uint32_t CALIBRATION_COLLECTION_TIMEOUT_MS =
    CALIBRATION_DURATION_MS + 250;
constexpr uint16_t EMG_CALIBRATION_FEATURE_COUNT =
    CALIBRATION_DURATION_MS / EMG_FEATURE_INTERVAL_MS;
constexpr uint32_t ROM_PREPARE_MS = 1000;
constexpr uint32_t ROM_STEP_INTERVAL_MS = 80;
constexpr uint8_t ROM_SAFETY_MARGIN_DEG = 3;
constexpr uint8_t ROM_MIN_TARGET_ANGLE = 5;
// Ignore attach inrush, then form a robust per-finger home-current baseline
// from fresh INA219 samples. A relative load must remain high for a few new
// samples; no further extension step is issued while it is being checked.
constexpr uint32_t ROM_CURRENT_BASELINE_SETTLE_MS = 300;
constexpr uint16_t ROM_CURRENT_BASELINE_MAX_SAMPLES = 64;
constexpr uint16_t ROM_CURRENT_BASELINE_MIN_SAMPLES = 30;
constexpr float ROM_CURRENT_NOISE_SIGMA_MULTIPLIER = 4.0f;
constexpr uint8_t ROM_LOAD_RISE_CONFIRM_SAMPLES = 3;
// CSV-like Serial output is intentionally enabled while the ROM/training rise
// limits above are being measured. Disable it after bench values are fixed.
constexpr bool CURRENT_DIAGNOSTICS_ENABLED = true;
constexpr float ASSISTANCE_TARGET_ACTIVATION = 0.60f;
constexpr float ASSISTANCE_RELATIVE_SPREAD_FULL = 0.20f;

// Training -----------------------------------------------------------------
constexpr uint32_t TRAIN_TRIGGER_HOLD_MS = 300;
constexpr float TRAIN_TRIGGER_ACTIVATION = 0.15f;
constexpr uint32_t TRAIN_RELEASE_HOLD_MS = 300;
constexpr float TRAIN_RELEASE_ACTIVATION = 0.05f;
constexpr float COCONTRACTION_EXTENSOR_LEVEL = 0.20f;
constexpr float COCONTRACTION_FLEXOR_LEVEL = 0.90f;
constexpr uint32_t COCONTRACTION_HOLD_MS = 1000;

// Every assistance level uses the same trajectory duration. Levels change
// only the CH1 participation requirement below; they never make the motor
// faster or more aggressive.
constexpr uint32_t TRAIN_MOVE_DURATION_MS = 5000;
constexpr float TRAIN_PARTICIPATION_LEVEL_1_RATIO = 0.70f;
constexpr float TRAIN_PARTICIPATION_LEVEL_10_RATIO = 0.35f;
constexpr float TRAIN_PARTICIPATION_MIN_ACTIVATION = 0.05f;
constexpr float TRAIN_PARTICIPATION_RECOVERY_MARGIN = 0.03f;
constexpr uint32_t TRAIN_PARTICIPATION_WARNING_MS = 500;
constexpr uint32_t TRAIN_PARTICIPATION_FAILURE_MS = 1500;
constexpr uint32_t TRAIN_PARTICIPATION_RECOVERY_HOLD_MS = 300;
// PWM attachment is staggered only to reduce startup inrush. Once armed, all
// selected motors use the same trajectory clock and are updated together. The
// shared INA219 monitors aggregate group current and never attributes a
// training failure to an individual motor.
constexpr uint32_t TRAIN_MOTOR_ATTACH_STAGGER_MS = 30;
constexpr uint32_t TRAIN_ATTACH_INRUSH_IGNORE_MS = 100;
constexpr uint8_t TRAIN_STALL_PERSIST_SAMPLES =
    static_cast<uint8_t>((STALL_CONFIRM_TIME_MS +
                          CURRENT_SAMPLE_INTERVAL_MS - 1U) /
                         CURRENT_SAMPLE_INTERVAL_MS);
constexpr uint8_t TRAIN_STALL_CLEAR_SAMPLES = 5;
constexpr uint32_t TRAIN_STALL_CONFIRM_TIMEOUT_MS = 500;
constexpr uint8_t TRAIN_MAX_TRANSIENT_RESUMES = 3;
constexpr uint32_t TRAIN_RESUME_MIN_DURATION_MS = 500;
// The soft group limit rises with the number of simultaneously enabled
// fingers. HARD_CURRENT_THRES_MA remains an absolute electrical ceiling and
// is deliberately not multiplied by finger count.
constexpr float TRAIN_ADDITIONAL_FINGER_CURRENT_ALLOWANCE_MA = 150.0f;
constexpr float TRAIN_HARD_CURRENT_MARGIN_MA = 100.0f;
constexpr uint8_t TRAIN_GROUP_MAX_COMMAND_STEP_DEG = 10;
constexpr uint32_t TRAIN_ATTACH_SETTLE_MS = 300;
constexpr uint32_t TRAIN_PREPARE_TIMEOUT_MS = 3000;
constexpr uint32_t TRAIN_HOLD_TIME_MS = 3000;
constexpr uint32_t SERVO_RETURN_DURATION_MS = 2500;
constexpr uint32_t SERVO_RETURN_TIMEOUT_MS = 4500;
constexpr uint8_t FAILURES_BEFORE_LEVEL_UP = 3;
constexpr uint8_t TRAIN_SUCCESS_GOAL = 5;
constexpr uint32_t TELEMETRY_INTERVAL_MS = 1000;
// A longer loop pause makes EMG and current evidence unreliable. Fail closed
// instead of continuing an actuator trajectory on stale measurements.
constexpr uint32_t CONTROL_LOOP_MAX_GAP_MS = 35;

static_assert(SERVO_PHYSICAL_MIN_ANGLE > 0 &&
                  SERVO_PHYSICAL_MIN_ANGLE < SERVO_PHYSICAL_MAX_ANGLE &&
                  SERVO_PHYSICAL_MAX_ANGLE <= 180,
              "Physical servo angles must stay inside 1..180 degrees");
static_assert(SERVO_LOGICAL_HOME_ANGLE == 0 &&
                  SERVO_LOGICAL_MAX_ANGLE - SERVO_LOGICAL_HOME_ANGLE ==
                      SERVO_PHYSICAL_MAX_ANGLE - SERVO_PHYSICAL_MIN_ANGLE,
              "Logical and physical servo spans must match");
static_assert(!SERVO_REVERSED[0] && SERVO_REVERSED[1] &&
                  !SERVO_REVERSED[2] && SERVO_REVERSED[3] &&
                  SERVO_REVERSED[4],
              "Servo direction mapping does not match the glove mechanics");
static_assert(SERVO_COMMAND_INTERVAL_MS > 0 &&
                  SERVO_MAX_COMMAND_STEP_DEG > 0,
              "Servo slew limiter must make progress");
static_assert(HARD_CURRENT_THRES_MA > STALL_CURRENT_THRES_MA,
              "Hard current limit must exceed the soft limit");
static_assert(ROM_FINGER_CURRENT_RISE_LIMIT_MA[0] > 0.0f &&
                  ROM_FINGER_CURRENT_RISE_LIMIT_MA[1] > 0.0f &&
                  ROM_FINGER_CURRENT_RISE_LIMIT_MA[2] > 0.0f &&
                  ROM_FINGER_CURRENT_RISE_LIMIT_MA[3] > 0.0f &&
                  ROM_FINGER_CURRENT_RISE_LIMIT_MA[4] > 0.0f,
              "Every ROM current-rise allowance must be positive");
static_assert(EMG_RAW_SAMPLE_RATE_HZ >= 1000 &&
                  1000000UL % EMG_RAW_SAMPLE_RATE_HZ == 0,
              "EMG sample rate must be at least 1 kHz and divide 1 MHz");
static_assert(BATTERY_ADC_PIN != EMG_EXTENSOR_PIN &&
                  BATTERY_ADC_PIN != EMG_FLEXOR_PIN &&
                  BATTERY_DIVIDER_TOP_OHMS > 0.0f &&
                  BATTERY_DIVIDER_BOTTOM_OHMS > 0.0f &&
                  BATTERY_SUBSAMPLE_INTERVAL_US >=
                      EMG_RAW_SAMPLE_INTERVAL_US &&
                  BATTERY_AVERAGE_SAMPLE_COUNT > 0 &&
                  BATTERY_FILTER_ALPHA > 0.0f &&
                  BATTERY_FILTER_ALPHA <= 1.0f &&
                  BATTERY_VALID_MIN_VOLTAGE <
                      BATTERY_VALID_MAX_VOLTAGE,
              "Battery monitor configuration is invalid");
static_assert(EMG_LOW_PASS_CUTOFF_HZ <
                  static_cast<float>(EMG_RAW_SAMPLE_RATE_HZ) * 0.5f,
              "EMG low-pass cutoff must be below Nyquist");
static_assert(EMG_HIGH_PASS_CUTOFF_HZ > 0.0f &&
                  EMG_HIGH_PASS_CUTOFF_HZ < EMG_LOW_PASS_CUTOFF_HZ,
              "EMG passband is invalid");
static_assert(EMG_NOTCH_FREQUENCY_HZ > EMG_HIGH_PASS_CUTOFF_HZ &&
                  EMG_NOTCH_FREQUENCY_HZ < EMG_LOW_PASS_CUTOFF_HZ,
              "EMG notch frequency must lie in the passband");
static_assert(EMG_RMS_SAMPLE_COUNT > 0 &&
                  EMG_RMS_SAMPLE_COUNT <= UINT16_MAX,
              "EMG RMS window is invalid");
static_assert(EMG_FEATURE_INTERVAL_MS > 0 &&
                  EMG_EVIDENCE_MAX_GAP_MS >= EMG_FEATURE_INTERVAL_MS,
              "EMG evidence gap must allow one normal sample interval");
static_assert((EMG_RAW_SAMPLE_RATE_HZ * EMG_FEATURE_INTERVAL_MS) % 1000UL ==
                  0,
              "EMG feature interval must contain a whole sample count");
static_assert(EMG_ADC_RAIL_MARGIN_COUNTS < 2048 &&
                  EMG_MAX_SATURATION_FRACTION >= 0.0f &&
                  EMG_MAX_SATURATION_FRACTION < 0.5f,
              "EMG saturation limits are invalid");
static_assert(EMG_FEATURE_HEALTH_TIMEOUT_MS >= EMG_EVIDENCE_MAX_GAP_MS,
              "EMG health timeout must allow the evidence gap");
static_assert(EMG_REST_NOISE_PERCENTILE > 0.5f &&
                  EMG_REST_NOISE_PERCENTILE < 1.0f &&
                  EMG_MVC_TOP_FRACTION > 0.0f &&
                  EMG_MVC_TOP_FRACTION <= 0.5f,
              "EMG robust calibration fractions are invalid");
static_assert(EMG_EXTENSOR_MIN_CALIBRATION_SPAN >=
                      EMG_FLEXOR_MIN_CALIBRATION_SPAN &&
                  EMG_FLEXOR_MIN_CALIBRATION_SPAN > 0.0f &&
                  EMG_MIN_CALIBRATION_SPAN_RATIO > 0.0f,
              "Channel-specific EMG calibration spans are invalid");
static_assert(COCONTRACTION_EXTENSOR_LEVEL > 0.0f &&
                  COCONTRACTION_EXTENSOR_LEVEL <= 1.0f &&
                  COCONTRACTION_FLEXOR_LEVEL > 0.0f &&
                  COCONTRACTION_FLEXOR_LEVEL <= 1.0f &&
                  COCONTRACTION_HOLD_MS > 0,
              "Cocontraction confirmation settings are invalid");
static_assert(EMG_CALIBRATION_FEATURE_COUNT > 0,
              "EMG calibration feature count is invalid");
static_assert(CALIBRATION_COLLECTION_TIMEOUT_MS > CALIBRATION_DURATION_MS,
              "EMG calibration timeout needs acquisition margin");
static_assert(ROM_CURRENT_BASELINE_SETTLE_MS < ROM_PREPARE_MS &&
                  ROM_CURRENT_BASELINE_MIN_SAMPLES > 0 &&
                  ROM_CURRENT_BASELINE_MIN_SAMPLES <=
                      ROM_CURRENT_BASELINE_MAX_SAMPLES &&
                  ROM_CURRENT_NOISE_SIGMA_MULTIPLIER > 0.0f &&
                  ROM_LOAD_RISE_CONFIRM_SAMPLES > 0,
              "ROM relative-current baseline configuration is invalid");
static_assert(TRAIN_MOVE_DURATION_MS > 0,
              "Training duration must be non-zero");
static_assert(TRAIN_PARTICIPATION_LEVEL_1_RATIO >=
                      TRAIN_PARTICIPATION_LEVEL_10_RATIO &&
                  TRAIN_PARTICIPATION_LEVEL_10_RATIO > 0.0f &&
                  TRAIN_PARTICIPATION_LEVEL_1_RATIO <= 1.0f &&
                  TRAIN_PARTICIPATION_MIN_ACTIVATION >= 0.0f &&
                  TRAIN_PARTICIPATION_MIN_ACTIVATION < 1.0f &&
                  TRAIN_PARTICIPATION_RECOVERY_MARGIN >= 0.0f &&
                  TRAIN_PARTICIPATION_WARNING_MS > 0 &&
                  TRAIN_PARTICIPATION_FAILURE_MS >
                      TRAIN_PARTICIPATION_WARNING_MS &&
                  TRAIN_PARTICIPATION_RECOVERY_HOLD_MS > 0,
              "Training participation configuration is invalid");
static_assert(TRAIN_MOTOR_ATTACH_STAGGER_MS > 0 &&
                  TRAIN_STALL_PERSIST_SAMPLES > 0 &&
                  TRAIN_STALL_CLEAR_SAMPLES > 0 &&
                  TRAIN_MAX_TRANSIENT_RESUMES > 0 &&
                  TRAIN_STALL_CONFIRM_TIMEOUT_MS > STALL_CONFIRM_TIME_MS,
              "Training attach/current safety timing is invalid");
static_assert(TRAIN_ADDITIONAL_FINGER_CURRENT_ALLOWANCE_MA >= 0.0f &&
                  TRAIN_HARD_CURRENT_MARGIN_MA > 0.0f &&
                  TRAIN_HARD_CURRENT_MARGIN_MA < HARD_CURRENT_THRES_MA,
              "Training group-current limits are invalid");
static_assert(TRAIN_PREPARE_TIMEOUT_MS >
                  TRAIN_MOTOR_ATTACH_STAGGER_MS * FINGER_COUNT +
                      TRAIN_ATTACH_INRUSH_IGNORE_MS + TRAIN_ATTACH_SETTLE_MS,
              "Training prepare timeout is too short");
static_assert(
    static_cast<uint32_t>(TRAIN_STALL_PERSIST_SAMPLES - 1U) *
            CONTROL_LOOP_MAX_GAP_MS <
        TRAIN_STALL_CONFIRM_TIMEOUT_MS,
    "Stall confirmation evidence cannot fit inside its timeout");
static_assert(SERVO_RETURN_TIMEOUT_MS >
                  SERVO_RETURN_DURATION_MS + SERVO_HOME_SETTLE_MS,
              "Servo return timeout has no settling margin");
static_assert(TRAIN_SUCCESS_GOAL > 0,
              "Training success goal must be non-zero");
static_assert(BLE_LOCAL_MTU >= 23 && BLE_LOCAL_MTU <= 517,
              "BLE MTU is outside the ATT range");

enum class SystemState : uint8_t {
  IDLE,
  CALIB_REST,
  CALIB_EXTENSOR_MAX,
  CALIB_FLEXOR_MAX,
  CALIB_PASSIVE_RANGE,
  TRAINING_READY,
  TRAINING_ACTIVE,
  SAFETY_RETURNING,
  FAULT
};

