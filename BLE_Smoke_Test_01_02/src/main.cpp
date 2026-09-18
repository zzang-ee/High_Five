#include <Arduino.h>
#include <esp_system.h>
#include <math.h>
#include <stdlib.h>

#include "BleManager.h"
#include "EmgSensor.h"
#include "ServoManager.h"

namespace {
constexpr int TEST_TARGET_ANGLE = 15;
constexpr uint32_t TEST_MOVE_DURATION_MS = 2000;
constexpr uint32_t TEST_HOLD_DURATION_MS = 1000;
constexpr uint32_t SIM_CALIBRATION_EVENT_INTERVAL_MS = 180;

enum class TestPhase : uint8_t {
  IDLE,
  MOVING,
  HOLDING,
  RETURNING
};

enum class ReturnReason : uint8_t {
  CYCLE,
  STOP,
  EMERGENCY,
  DISCONNECT
};

enum class SimCalibrationStep : uint8_t {
  IDLE,
  REST_PREPARE,
  REST_START,
  EXTENSOR_PREPARE,
  EXTENSOR_START,
  FLEXOR_PREPARE,
  FLEXOR_START,
  ROM_PREPARE,
  ROM_START,
  ROM_DONE,
  CALIBRATION_DONE
};

BleManager ble;
EmgSensor emg;
ServoManager servo;

bool emgReady = false;
bool servoReady = false;
uint8_t selectedMask = 0;
TestPhase phase = TestPhase::IDLE;
ReturnReason returnReason = ReturnReason::CYCLE;
uint32_t phaseStartTimeMs = 0;
uint32_t lastTelemetryTimeMs = 0;
uint32_t telemetrySequence = 0;
uint32_t bootId = 1;
SimCalibrationStep calibrationStep = SimCalibrationStep::IDLE;
uint8_t calibrationFinger = 0;
uint32_t nextCalibrationEventMs = 0;
bool simulatedCalibrationDone = false;

bool calibrationBusy() {
  return calibrationStep != SimCalibrationStep::IDLE;
}

void sendTelemetry(uint32_t nowMs, bool force = false);

const char* phaseName() {
  switch (phase) {
    case TestPhase::MOVING:
      return "moving";
    case TestPhase::HOLDING:
      return "holding";
    case TestPhase::RETURNING:
      return "returning";
    case TestPhase::IDLE:
    default:
      return "idle";
  }
}

void sendError(const char* code) {
  char message[64];
  snprintf(message, sizeof(message), "{\"error\":\"%s\"}", code);
  ble.sendData(message);
}

void sendSimulatedControl() {
  char message[244];
  snprintf(message, sizeof(message),
           "{\"q\":%lu,\"control\":{\"mode\":\"STOP\"," 
           "\"target_angle\":%d,\"hold_time_sec\":1,\"motor_level\":5,"
           "\"threshold_extensor\":120,\"threshold_flexor\":100,"
           "\"mvc_extensor\":700,\"mvc_flexor\":600,"
           "\"participation_required\":0.10}}",
           static_cast<unsigned long>(telemetrySequence), TEST_TARGET_ANGLE);
  ble.sendData(message);
}

bool isDailyTestCommand(const String& command) {
  // DAILY_TEST_START is the preferred spelling. The aliases keep this small
  // bench firmware usable while the separate app is still being integrated.
  return command.startsWith("DAILY") || command == "SELF_TEST" ||
         command == "SELF_TEST_START" || command == "TEST_START";
}

void startSimulatedCalibration(uint32_t nowMs) {
  calibrationFinger = 0;
  simulatedCalibrationDone = false;
  calibrationStep = SimCalibrationStep::REST_PREPARE;
  nextCalibrationEventMs = nowMs;
}

void cancelSimulatedCalibration() {
  calibrationStep = SimCalibrationStep::IDLE;
  calibrationFinger = 0;
  nextCalibrationEventMs = 0;
}

void processSimulatedCalibration(uint32_t nowMs) {
  if (!calibrationBusy() ||
      static_cast<int32_t>(nowMs - nextCalibrationEventMs) < 0) {
    return;
  }

  switch (calibrationStep) {
    case SimCalibrationStep::REST_PREPARE:
      ble.sendData(
          "{\"event\":\"calibration\",\"stage\":\"rest\","
          "\"state\":\"prepare\"}");
      calibrationStep = SimCalibrationStep::REST_START;
      break;
    case SimCalibrationStep::REST_START:
      ble.sendData(
          "{\"event\":\"calibration\",\"stage\":\"rest\","
          "\"state\":\"start\"}");
      calibrationStep = SimCalibrationStep::EXTENSOR_PREPARE;
      break;
    case SimCalibrationStep::EXTENSOR_PREPARE:
      ble.sendData(
          "{\"event\":\"calibration\",\"stage\":\"extensor\","
          "\"state\":\"prepare\"}");
      calibrationStep = SimCalibrationStep::EXTENSOR_START;
      break;
    case SimCalibrationStep::EXTENSOR_START:
      ble.sendData(
          "{\"event\":\"calibration\",\"stage\":\"extensor\","
          "\"state\":\"start\"}");
      calibrationStep = SimCalibrationStep::FLEXOR_PREPARE;
      break;
    case SimCalibrationStep::FLEXOR_PREPARE:
      ble.sendData(
          "{\"event\":\"calibration\",\"stage\":\"flexor\","
          "\"state\":\"prepare\"}");
      calibrationStep = SimCalibrationStep::FLEXOR_START;
      break;
    case SimCalibrationStep::FLEXOR_START:
      ble.sendData(
          "{\"event\":\"calibration\",\"stage\":\"flexor\","
          "\"state\":\"start\"}");
      calibrationStep = SimCalibrationStep::ROM_PREPARE;
      break;
    case SimCalibrationStep::ROM_PREPARE: {
      char message[72];
      snprintf(message, sizeof(message),
               "{\"event\":\"rom\",\"state\":\"prepare\",\"finger\":%u}",
               calibrationFinger);
      ble.sendData(message);
      calibrationStep = SimCalibrationStep::ROM_START;
      break;
    }
    case SimCalibrationStep::ROM_START: {
      char message[72];
      snprintf(message, sizeof(message),
               "{\"event\":\"rom\",\"state\":\"start\",\"finger\":%u}",
               calibrationFinger);
      ble.sendData(message);
      calibrationStep = SimCalibrationStep::ROM_DONE;
      break;
    }
    case SimCalibrationStep::ROM_DONE: {
      static const char* const names[FINGER_COUNT] = {
          "thumb", "index", "middle", "ring", "pinky"};
      char message[220];
      snprintf(message, sizeof(message),
               "{\"event\":\"rom_done\",\"finger\":\"%s\","
               "\"finger_index\":%u,\"detected_angle\":%d,"
               "\"target_angle\":%d,\"endpoint\":\"travel_end\","
               "\"detector\":\"smoke_test\"}",
               names[calibrationFinger], calibrationFinger,
               TEST_TARGET_ANGLE, TEST_TARGET_ANGLE);
      ble.sendData(message);
      ++calibrationFinger;
      calibrationStep = calibrationFinger < FINGER_COUNT
                            ? SimCalibrationStep::ROM_PREPARE
                            : SimCalibrationStep::CALIBRATION_DONE;
      break;
    }
    case SimCalibrationStep::CALIBRATION_DONE: {
      char message[220];
      snprintf(message, sizeof(message),
               "{\"event\":\"calibration_done\",\"mask\":%u,"
               "\"target_angles\":{\"thumb\":%d,\"index\":%d,"
               "\"middle\":%d,\"ring\":%d,\"pinky\":%d},"
               "\"smoke_test\":true}",
               ALL_FINGERS_MASK, TEST_TARGET_ANGLE, TEST_TARGET_ANGLE,
               TEST_TARGET_ANGLE, TEST_TARGET_ANGLE, TEST_TARGET_ANGLE);
      ble.sendData(message);
      simulatedCalibrationDone = true;
      cancelSimulatedCalibration();
      sendTelemetry(nowMs, true);
      sendSimulatedControl();
      return;
    }
    case SimCalibrationStep::IDLE:
    default:
      return;
  }

  nextCalibrationEventMs = nowMs + SIM_CALIBRATION_EVENT_INTERVAL_MS;
}

bool parseFingerMask(const String& value, uint8_t& result) {
  String text = value;
  text.trim();
  if (text.length() == 0 || text.length() > 2) {
    return false;
  }
  for (uint16_t i = 0; i < text.length(); ++i) {
    if (!isHexadecimalDigit(text[i])) {
      return false;
    }
  }

  char* end = nullptr;
  const unsigned long parsed = strtoul(text.c_str(), &end, 16);
  if (end == text.c_str() || *end != '\0' || parsed == 0 ||
      (parsed & ~static_cast<unsigned long>(ALL_FINGERS_MASK)) != 0) {
    return false;
  }
  result = static_cast<uint8_t>(parsed);
  return true;
}

void sendTelemetry(uint32_t nowMs, bool force) {
  if (!ble.isConnected() || !ble.hasRequiredMtu()) {
    return;
  }
  if (!force && lastTelemetryTimeMs != 0 &&
      static_cast<uint32_t>(nowMs - lastTelemetryTimeMs) <
          TELEMETRY_INTERVAL_MS) {
    return;
  }
  lastTelemetryTimeMs = nowMs;
  ++telemetrySequence;
  if (telemetrySequence == 0) {
    telemetrySequence = 1;
  }

  char batteryText[5] = "null";
  if (emgReady && emg.hasValidBatteryReading()) {
    snprintf(batteryText, sizeof(batteryText), "%u",
             static_cast<unsigned>(emg.getBatteryPercent()));
  }

  char status[245];
  snprintf(status, sizeof(status),
           "{\"v\":%u,\"boot\":%lu,\"q\":%lu,\"device_ms\":%lu,"
           "\"status\":{\"mode\":\"%s\",\"is_triggered\":%s,"
           "\"is_cocontraction\":false,"
           "\"is_participation_low\":false,\"battery\":%s,"
           "\"stall_detected\":false}}",
           BLE_PROTOCOL_VERSION, static_cast<unsigned long>(bootId),
           static_cast<unsigned long>(telemetrySequence),
           static_cast<unsigned long>(nowMs),
           phase == TestPhase::IDLE ? "STOP" : "START",
           (phase == TestPhase::MOVING || phase == TestPhase::HOLDING)
               ? "true"
               : "false",
           batteryText);
  ble.sendData(status);

  int targets[FINGER_COUNT] = {};
  for (uint8_t i = 0; i < FINGER_COUNT; ++i) {
    if (simulatedCalibrationDone || (selectedMask & (1U << i)) != 0) {
      targets[i] = TEST_TARGET_ANGLE;
    }
  }

  char angles[245];
  snprintf(angles, sizeof(angles),
           "{\"q\":%lu,\"angles\":{\"thumb\":%d,\"index\":%d,"
           "\"middle\":%d,\"ring\":%d,\"pinky\":%d},"
           "\"target_angles\":{\"thumb\":%d,\"index\":%d,"
           "\"middle\":%d,\"ring\":%d,\"pinky\":%d}}",
           static_cast<unsigned long>(telemetrySequence), servo.getAngle(0),
           servo.getAngle(1), servo.getAngle(2), servo.getAngle(3),
           servo.getAngle(4), targets[0], targets[1], targets[2], targets[3],
           targets[4]);
  ble.sendData(angles);

  char emgMessage[96];
  if (emgReady && emg.isWindowReady() && emg.isSignalQualityGood()) {
    snprintf(emgMessage, sizeof(emgMessage),
             "{\"q\":%lu,\"emg\":{\"ch1\":%lu,\"ch2\":%lu}}",
             static_cast<unsigned long>(telemetrySequence),
             static_cast<unsigned long>(lroundf(emg.getExtensorRMS())),
             static_cast<unsigned long>(lroundf(emg.getFlexorRMS())));
  } else {
    snprintf(emgMessage, sizeof(emgMessage),
             "{\"q\":%lu,\"emg\":{\"ch1\":null,\"ch2\":null}}",
             static_cast<unsigned long>(telemetrySequence));
  }
  ble.sendData(emgMessage);

  char smokeState[128];
  snprintf(smokeState, sizeof(smokeState),
           "{\"q\":%lu,\"smoke_test\":{\"phase\":\"%s\","
           "\"selected_mask\":%u,\"calibrated\":%s,"
           "\"current_sensor\":false}}",
           static_cast<unsigned long>(telemetrySequence), phaseName(),
           selectedMask, simulatedCalibrationDone ? "true" : "false");
  ble.sendData(smokeState);
}

bool startReturn(uint32_t nowMs, ReturnReason reason) {
  if (!servoReady) {
    return false;
  }
  servo.stopMotion();
  returnReason = reason;
  if (!servo.startReturn(selectedMask, TEST_MOVE_DURATION_MS, nowMs)) {
    return false;
  }
  phase = TestPhase::RETURNING;
  phaseStartTimeMs = nowMs;
  if (ble.isConnected()) {
    const char* reasonText = "cycle";
    if (reason == ReturnReason::STOP) {
      reasonText = "user_stop";
    } else if (reason == ReturnReason::EMERGENCY) {
      reasonText = "EMERG";
    } else if (reason == ReturnReason::DISCONNECT) {
      reasonText = "LINK";
    }
    char message[112];
    snprintf(message, sizeof(message),
             "{\"event\":\"safety_return\",\"reason\":\"%s\"}",
             reasonText);
    ble.sendData(message);
  }
  return true;
}

void handleSafety(BleSafetyCommand command, uint32_t nowMs) {
  if (command == BleSafetyCommand::NONE) {
    return;
  }
  if (calibrationBusy()) {
    cancelSimulatedCalibration();
    ble.sendData(
        "{\"event\":\"calibration\",\"state\":\"cancelled\","
        "\"smoke_test\":true}");
    return;
  }
  if (command == BleSafetyCommand::EMERGENCY) {
    if (!startReturn(nowMs, ReturnReason::EMERGENCY)) {
      servo.detachAll();
      phase = TestPhase::IDLE;
    }
    return;
  }
  if (command == BleSafetyCommand::DISCONNECT) {
    if (!startReturn(nowMs, ReturnReason::DISCONNECT)) {
      servo.detachAll();
      phase = TestPhase::IDLE;
    }
    return;
  }
  if (command == BleSafetyCommand::STOP) {
    if (!startReturn(nowMs, ReturnReason::STOP)) {
      sendError("RETURN_START");
    }
  }
}

void handleCommand(String command, uint32_t nowMs) {
  command.trim();
  if (command == "MTU?") {
    char response[20];
    snprintf(response, sizeof(response), "{\"mtu\":%u}",
             static_cast<unsigned>(ble.getNegotiatedMtu()));
    ble.sendData(response);
    return;
  }
  if (!ble.hasRequiredMtu()) {
    sendError("MTU");
    return;
  }
  if (command == "STATUS?") {
    sendTelemetry(nowMs, true);
    return;
  }
  if (isDailyTestCommand(command)) {
    ble.sendData(
        "{\"event\":\"daily_test\",\"state\":\"done\","
        "\"success\":true,\"passed\":true,\"emg_ch1_ok\":true,"
        "\"emg_ch2_ok\":true,\"servo_ok\":true,"
        "\"smoke_test\":true}");
    sendTelemetry(nowMs, true);
    return;
  }
  if (command.startsWith("FINGERS:")) {
    if (!servoReady || calibrationBusy() || phase != TestPhase::IDLE ||
        !servo.allAtHome(ALL_FINGERS_MASK)) {
      sendError("BUSY");
      return;
    }
    uint8_t mask = 0;
    if (!parseFingerMask(command.substring(8), mask)) {
      sendError("FINGER_MASK");
      return;
    }
    if (!servo.setEnabledMask(mask)) {
      sendError("SERVO_ENABLE");
      return;
    }
    selectedMask = mask;
    char response[64];
    snprintf(response, sizeof(response),
             "{\"event\":\"fingers_selected\",\"mask\":%u}",
             selectedMask);
    ble.sendData(response);
    sendTelemetry(nowMs, true);
    return;
  }
  if (command == "CALIB_START") {
    if (!servoReady || phase != TestPhase::IDLE || calibrationBusy()) {
      sendError("BUSY");
      return;
    }
    if (!servo.allAtHome(ALL_FINGERS_MASK)) {
      sendError("NOT_HOME");
      return;
    }
    startSimulatedCalibration(nowMs);
    processSimulatedCalibration(nowMs);
    return;
  }
  if (command == "TRAIN_START") {
    if (!servoReady) {
      sendError("SERVO");
      return;
    }
    if (selectedMask == 0) {
      sendError("NO_FINGERS");
      return;
    }
    if (calibrationBusy() || !simulatedCalibrationDone) {
      sendError("NOT_READY");
      return;
    }
    if (phase != TestPhase::IDLE || !servo.allAtHome(selectedMask)) {
      sendError("BUSY");
      return;
    }
    if (!servo.setEnabledMask(selectedMask)) {
      sendError("SERVO_ENABLE");
      return;
    }

    int targets[FINGER_COUNT] = {};
    uint32_t durations[FINGER_COUNT] = {};
    for (uint8_t i = 0; i < FINGER_COUNT; ++i) {
      targets[i] = (selectedMask & (1U << i)) != 0
                       ? TEST_TARGET_ANGLE
                       : servo.getAngle(i);
      durations[i] = TEST_MOVE_DURATION_MS;
    }
    if (!servo.startMoveFingers(selectedMask, targets, durations, nowMs)) {
      sendError("MOVE_START");
      return;
    }
    phase = TestPhase::MOVING;
    phaseStartTimeMs = nowMs;
    char response[88];
    snprintf(response, sizeof(response),
             "{\"event\":\"training\",\"state\":\"moving\","
             "\"mask\":%u,\"smoke_test\":true}",
             selectedMask);
    ble.sendData(response);
    return;
  }

  // Session logging deliberately remains in the production firmware.
  if (command == "SES_GET" || command.startsWith("SES_ACK:")) {
    sendError("NOT_IN_SMOKE_TEST");
    return;
  }
  sendError("UNKNOWN_CMD");
}

void processTestState(uint32_t nowMs) {
  if (phase == TestPhase::MOVING && !servo.isBusy()) {
    phase = TestPhase::HOLDING;
    phaseStartTimeMs = nowMs;
    ble.sendData(
        "{\"event\":\"training\",\"state\":\"holding\","
        "\"hold_time_sec\":1}");
    return;
  }
  if (phase == TestPhase::HOLDING &&
      static_cast<uint32_t>(nowMs - phaseStartTimeMs) >=
          TEST_HOLD_DURATION_MS) {
    if (!startReturn(nowMs, ReturnReason::CYCLE)) {
      sendError("RETURN_START");
      servo.detachAll();
      phase = TestPhase::IDLE;
    }
    return;
  }
  if (phase != TestPhase::RETURNING || servo.isBusy() ||
      !servo.allAtHome(selectedMask)) {
    return;
  }

  const ReturnReason completedReason = returnReason;
  phase = TestPhase::IDLE;
  phaseStartTimeMs = nowMs;
  if (completedReason == ReturnReason::EMERGENCY ||
      completedReason == ReturnReason::DISCONNECT) {
    servo.setEnabledMask(0);
  }
  if (!ble.isConnected()) {
    return;
  }
  if (completedReason == ReturnReason::CYCLE) {
    ble.sendData(
        "{\"event\":\"training\",\"state\":\"cycle_done\","
        "\"success\":true,\"smoke_test\":true}");
  } else if (completedReason == ReturnReason::STOP) {
    ble.sendData(
        "{\"event\":\"training\",\"state\":\"ready\","
        "\"stopped\":true,\"session_continues\":true}");
  } else if (completedReason == ReturnReason::EMERGENCY) {
    ble.sendData("{\"event\":\"emergency\",\"state\":\"home\"}");
  }
  sendTelemetry(nowMs, true);
}
}  // namespace

void setup() {
  Serial.begin(115200);
  bootId = esp_random();
  if (bootId == 0) {
    bootId = 1;
  }

  servoReady = servo.begin();
  emgReady = emg.begin();
  ble.begin();

  Serial.println("BLE_SMOKE_TEST_READY");
  Serial.println("WARNING: no current sensor; bench test only, do not wear");
}

void loop() {
  const uint32_t nowMs = millis();
  if (emgReady) {
    emg.update(nowMs);
  }

  handleSafety(ble.consumeSafetyCommand(), nowMs);
  for (uint8_t i = 0; i < BLE_COMMAND_QUEUE_LENGTH && ble.available(); ++i) {
    handleCommand(ble.readData(), nowMs);
  }

  servo.update(nowMs);
  processSimulatedCalibration(nowMs);
  processTestState(nowMs);
  sendTelemetry(nowMs);
}
