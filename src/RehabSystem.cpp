#include "RehabSystem.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <esp_system.h>

namespace {
const char* const FINGER_NAMES[FINGER_COUNT] = {
    "thumb", "index", "middle", "ring", "pinky"};
}

RehabSystem::RehabSystem()
    : currentState(SystemState::IDLE),
      safetyReturnDestination(SystemState::IDLE),
      hardwareReady(false),
      previousBleConnected(false),
      selectedFingerMask(0),
      calibratedFingerMask(0),
      trainingMask(0),
      stateStartTime(0),
      calibrationCollecting(false),
      calibrationExtensorSamples{},
      calibrationFlexorSamples{},
      sampleCount(0),
      romPhase(RomPhase::PREPARING),
      calibFingerIndex(0),
      lastStepTime(0),
      romActivationSum(0.0f),
      romActivationSamples(0),
      romRetryCurrentFinger(false),
      romCurrentBaselineSamples{},
      romCurrentBaselineSampleCount(0),
      romLoadRiseHighSamples(0),
      romStepPeakCurrentMa(0.0f),
      trainingPhase(TrainingPhase::WAITING_RELEASE),
      trainingReturnCause(TrainingReturnCause::ATTEMPT),
      trainingTriggered(false),
      trainingSessionActive(false),
      trainingGoalReached(false),
      trainingCurrentSafetyIncidentLatched(false),
      lastAttemptSuccess(false),
      phaseStartTime(0),
      trainingAttachMask(0),
      lastTrainingAttachTime(0),
      trainingAttachCompleteTime(0),
      stallConfirmationHighSamples(0),
      stallConfirmationClearSamples(0),
      trainingResumeCount(0),
      trainingTriggerActivationReference(0.0f),
      trainingParticipationRequired(0.0f),
      participationLowPending(false),
      participationWarningActive(false),
      participationLowStartTime(0),
      participationRecoveryStartTime(0),
      stallPending(false),
      stallStartTime(0),
      triggerPending(false),
      triggerStartTime(0),
      releasePending(false),
      releaseStartTime(0),
      cocontractionPending(false),
      cocontractionStartTime(0),
      lastEvidenceSampleTime(0),
      homeSettlePending(false),
      homeSettleStartTime(0),
      sessionTotalAttempts(0),
      sessionSuccessCount(0),
      sessionParticipationFailureCount(0),
      sessionCocontractionCnt(0),
      sessionCurrentSafetyCount(0),
      sessionAngleSum(0.0),
      sessionAngleSampleCount(0),
      sessionMaxExtensorRms(0.0f),
      sessionMaxFlexorRms(0.0f),
      bootId(0),
      trainingSessionId(0),
      trainingStartDeviceMs(0),
      completedTrainingEndDeviceMs(0),
      completedTrainingMask(0),
      sessionMetricsSequence(0),
      pendingSummaryMetricsSequence(0),
      lastSummaryTransmitTime(0),
      telemetrySequence(0),
      sessionSummaryPending(false),
      completedTrainingEndReason(nullptr),
      lastTelemetryTime(0),
      telemetryFingerIndex(0),
      lastControlLoopTime(0),
      stableBatteryPercent(0),
      stableBatteryReadingValid(false) {
  resetFingerProfiles();
}

void RehabSystem::begin() {
  Serial.begin(115200);
  if (CURRENT_DIAGNOSTICS_ENABLED) {
    Serial.println(
        "ROM_CURRENT_HEADER,finger,angle,baseline_ma,raw_ma,filtered_ma,"
        "step_peak_ma,trip_ma");
    Serial.println(
        "TRAIN_CURRENT_HEADER,event,mask,raw_ma,filtered_ma,group_limit_ma");
  }
  bootId = esp_random();
  if (bootId == 0) {
    bootId = 1;
  }

  const bool emgStarted = emgSensor.begin();

  const bool currentStarted = currentSensor.begin();
  bool initialCurrentReady = false;
  if (currentStarted) {
    currentSensor.update(millis());
    initialCurrentReady = currentSensor.isReady() &&
                          !currentSensor.isHardOverCurrent();
  }

  bool servoStarted = false;
  if (emgStarted && initialCurrentReady) {
    servoStarted = servo.begin();
  }

  // BLE remains available even in FAULT so the app can read telemetry.
  ble.begin();

  hardwareReady = emgStarted && initialCurrentReady && servoStarted;
  if (!hardwareReady) {
    servo.detachAll();
    currentState = SystemState::FAULT;
    Serial.println("[SYSTEM] Hardware initialization failed; actuation locked");
  } else {
    currentState = SystemState::IDLE;
    Serial.println("[SYSTEM] Rehab glove initialized");
  }
}

void RehabSystem::update() {
  const uint32_t nowMs = millis();
  const bool controlLoopGap =
      lastControlLoopTime != 0 &&
      static_cast<uint32_t>(nowMs - lastControlLoopTime) >
          CONTROL_LOOP_MAX_GAP_MS;
  lastControlLoopTime = nowMs;

  const BleSafetyCommand safetyCommand = ble.consumeSafetyCommand();
  if (safetyCommand != BleSafetyCommand::NONE) {
    handleSafetyCommand(safetyCommand, nowMs);
  }

  const bool connected = ble.isConnected();
  if (connected && ble.hasRequiredMtu()) {
    sendPendingTrainingSessionSummary(nowMs);
  }
  if (previousBleConnected && !connected) {
    selectedFingerMask = 0;
    if (isCalibrationState()) {
      resetFingerProfiles();
    }
    if (safetyCommand == BleSafetyCommand::NONE &&
        (isCalibrationState() || isActuationState())) {
      const SystemState destination =
          calibratedFingerMask != 0 ? SystemState::TRAINING_READY
                                    : SystemState::IDLE;
      triggerSafetyReturn("LINK", destination, true, nowMs);
    }
  }
  previousBleConnected = connected;

  for (uint8_t i = 0; i < BLE_COMMAND_QUEUE_LENGTH && ble.available(); ++i) {
    const String queuedCommand = ble.readData();
    if (connected) {
      handleBleCommand(queuedCommand, nowMs);
    }
  }

  const bool newEmgSample = emgSensor.update(nowMs);
  const bool rawSamplingGap = emgSensor.consumeSamplingGap();
  const bool featureGap = emgSensor.consumeFeatureGap();
  const bool emgQualityFault =
      newEmgSample && emgSensor.isWindowReady() &&
      !emgSensor.isSignalQualityGood();
  const bool emgDataFault = rawSamplingGap || featureGap ||
                            !emgSensor.isSamplingHealthy(nowMs) ||
                            emgQualityFault;
  const bool newCurrentSample = currentSensor.update(nowMs);

  // Cache only a home/rest reading. Servo load can momentarily pull pack
  // voltage down, which would otherwise make the app's percentage jump while
  // a training trajectory is running.
  if (!isActuationState() && servo.allAtHome()) {
    stableBatteryReadingValid = emgSensor.hasValidBatteryReading();
    if (stableBatteryReadingValid) {
      stableBatteryPercent = emgSensor.getBatteryPercent();
    }
  }

  if (trainingSessionActive && newEmgSample && emgSensor.isWindowReady() &&
      emgSensor.isSignalQualityGood()) {
    sessionMaxExtensorRms =
        fmaxf(sessionMaxExtensorRms, emgSensor.getExtensorRMS());
    sessionMaxFlexorRms =
        fmaxf(sessionMaxFlexorRms, emgSensor.getFlexorRMS());
  }

  if (hardwareReady && currentState != SystemState::FAULT) {
    if (!currentSensor.isReady()) {
      enterFault("CURRENT_SENSOR");
    } else if (currentSensor.isHardOverCurrent()) {
      if (trainingSessionActive) {
        recordTrainingCurrentSafety("hard_current");
      }
      enterFault("HARD_CURRENT");
    }
  }

  if (hardwareReady && currentState != SystemState::FAULT && controlLoopGap &&
      isActuationState() && currentState != SystemState::SAFETY_RETURNING) {
    if (currentState == SystemState::CALIB_PASSIVE_RANGE) {
      // Loss of timing makes the current angle attribution unreliable, but it
      // is recoverable: release, return, and repeat only this ROM finger.
      retryFingerCalibration("loop_gap", nowMs);
    } else {
      triggerSafetyReturn("LOOP_GAP", SystemState::TRAINING_READY, false,
                          nowMs);
    }
  }

  switch (currentState) {
    case SystemState::CALIB_REST:
    case SystemState::CALIB_EXTENSOR_MAX:
    case SystemState::CALIB_FLEXOR_MAX:
      processEmgCalibration(nowMs, newEmgSample, emgDataFault);
      break;

    case SystemState::CALIB_PASSIVE_RANGE:
      processFingerCalibration(nowMs, newEmgSample, emgDataFault,
                               newCurrentSample);
      break;

    case SystemState::TRAINING_ACTIVE:
      processTraining(nowMs, newEmgSample, emgDataFault, newCurrentSample);
      break;

    case SystemState::SAFETY_RETURNING:
      processSafetyReturn(nowMs);
      break;

    case SystemState::IDLE:
    case SystemState::TRAINING_READY:
    case SystemState::FAULT:
    default:
      break;
  }

  // Servo updates happen after the safety/state decision so a newly detected
  // overcurrent can freeze a trajectory before another angle is commanded.
  servo.update(nowMs);
  sendRealtimeTelemetry(nowMs);
}

void RehabSystem::sendJsonError(const char* code) {
  if (code == nullptr) {
    return;
  }
  char message[72];
  snprintf(message, sizeof(message), "{\"error\":\"%s\"}", code);
  ble.sendData(message);
}

void RehabSystem::sendJsonState(const char* state) {
  if (state == nullptr) {
    return;
  }
  char message[64];
  snprintf(message, sizeof(message), "{\"state\":\"%s\"}", state);
  ble.sendData(message);
}

void RehabSystem::sendCalibrationJson(const char* stage,
                                      const char* state) {
  if (stage == nullptr || state == nullptr) {
    return;
  }
  char message[96];
  snprintf(message, sizeof(message),
           "{\"event\":\"calibration\",\"stage\":\"%s\","
           "\"state\":\"%s\"}",
           stage, state);
  ble.sendData(message);
}

void RehabSystem::handleBleCommand(String command, uint32_t nowMs) {
  command.trim();
  if (command.length() == 0) {
    return;
  }

  if (command == "MTU?") {
    // Keep this response within the default 20-byte ATT payload so the app
    // can query it before negotiating the protocol's required MTU.
    char response[20];
    snprintf(response, sizeof(response), "{\"mtu\":%u}",
             static_cast<unsigned>(ble.getNegotiatedMtu()));
    ble.sendData(response);
    return;
  }

  if (!ble.hasRequiredMtu()) {
    sendJsonError("MTU");
    return;
  }

  if (command.startsWith("SES_ACK:")) {
    unsigned long ackBootId = 0;
    unsigned long ackSessionId = 0;
    unsigned long ackSequence = 0;
    int consumed = 0;
    const int fields =
        sscanf(command.c_str(), "SES_ACK:%lu,%lu,%lu%n", &ackBootId,
               &ackSessionId, &ackSequence, &consumed);
    const bool matchingSummary =
        pendingSummaryMetricsSequence != 0 && ackBootId == bootId &&
        ackSessionId == trainingSessionId &&
        ackSequence == pendingSummaryMetricsSequence;
    if (fields != 3 || consumed != static_cast<int>(command.length()) ||
        !matchingSummary) {
      sendJsonError("SES_ACK");
      return;
    }

    if (sessionSummaryPending) {
      sessionSummaryPending = false;
      completedTrainingEndReason = nullptr;
      lastSummaryTransmitTime = 0;
    }
    // Retain the acknowledged sequence until the next session starts so a
    // retried ACK remains idempotent if the JSON ACK response was lost.
    ble.sendData("{\"ack\":\"session\",\"ok\":true}");
    return;
  }

  if (command == "SES_GET") {
    if (!sessionSummaryPending) {
      ble.sendData("{\"event\":\"session_pending\",\"pending\":false}");
      return;
    }
    lastSummaryTransmitTime = 0;
    sendPendingTrainingSessionSummary(nowMs);
    return;
  }

  if (command == "STATUS?") {
    lastTelemetryTime = nowMs - TELEMETRY_INTERVAL_MS;
    sendRealtimeTelemetry(nowMs);
    return;
  }

  if (!hardwareReady || currentState == SystemState::FAULT) {
    sendJsonError("FAULT");
    return;
  }

  if (command.startsWith("FINGERS:")) {
    if (currentState != SystemState::IDLE &&
        currentState != SystemState::TRAINING_READY) {
      sendJsonError("BUSY");
      return;
    }

    uint8_t mask = 0;
    if (!parseFingerMask(command.substring(8), mask)) {
      sendJsonError("FINGER_MASK");
      return;
    }

    selectedFingerMask = mask;
    char response[56];
    snprintf(response, sizeof(response),
             "{\"event\":\"fingers_selected\",\"mask\":%u}",
             selectedFingerMask);
    ble.sendData(response);
    return;
  }

  if (command == "CALIB_START") {
    if (currentState != SystemState::IDLE &&
        currentState != SystemState::TRAINING_READY) {
      sendJsonError("BUSY");
      return;
    }
    if (!servo.allAtHome()) {
      sendJsonError("NOT_HOME");
      return;
    }

    emgSensor.clearCalibration();
    resetFingerProfiles();
    startEmgCalibrationStage(SystemState::CALIB_REST, nowMs);
    return;
  }

  if (command == "TRAIN_START") {
    if (sessionSummaryPending) {
      sendJsonError("SES_PENDING");
      return;
    }
    if (currentState != SystemState::TRAINING_READY) {
      sendJsonError("NOT_READY");
      return;
    }
    if (selectedFingerMask == 0) {
      sendJsonError("NO_FINGERS");
      return;
    }
    if (!allSelectedFingersCalibrated(selectedFingerMask)) {
      sendJsonError("UNCALIBRATED");
      return;
    }
    startTrainingSession(nowMs);
    return;
  }

  sendJsonError("UNKNOWN_CMD");
}

void RehabSystem::handleSafetyCommand(BleSafetyCommand command,
                                      uint32_t nowMs) {
  if (currentState == SystemState::FAULT) {
    return;
  }

  if (command == BleSafetyCommand::EMERGENCY) {
    // Preserve calibration, learned levels, failure stacks, and selection,
    // but end an active training session and command every enabled finger
    // back to home.
    const SystemState destination =
        calibratedFingerMask != 0 ? SystemState::TRAINING_READY
                                  : SystemState::IDLE;
    triggerSafetyReturn("EMERG", destination, false, nowMs);
    return;
  }

  if (command == BleSafetyCommand::STOP && trainingSessionActive &&
      currentState == SystemState::TRAINING_ACTIVE) {
    // STOP cancels only the in-progress cycle. A return already in progress
    // must not be restarted, because repeated app commands would otherwise
    // keep the quintic trajectory near zero velocity indefinitely.
    if (trainingPhase == TrainingPhase::RETURNING) {
      ble.sendData(
          "{\"event\":\"training\",\"state\":\"returning\","
          "\"reason\":\"user_stop\",\"session_continues\":true}");
      return;
    }
    beginTrainingReturn(nowMs, false, TrainingReturnCause::USER_STOP);
    return;
  }

  if (isCalibrationState()) {
    resetFingerProfiles();
  }

  if (command == BleSafetyCommand::DISCONNECT) {
    const SystemState destination =
        calibratedFingerMask != 0 ? SystemState::TRAINING_READY
                                  : SystemState::IDLE;
    triggerSafetyReturn("LINK", destination, true, nowMs);
    return;
  }

  const SystemState destination =
      calibratedFingerMask != 0 ? SystemState::TRAINING_READY
                                : SystemState::IDLE;
  const char* reason =
      trainingSessionActive && trainingGoalReached ? "GOAL" : "USER";
  triggerSafetyReturn(reason, destination, false, nowMs);
}

void RehabSystem::startEmgCalibrationStage(SystemState stage,
                                            uint32_t nowMs) {
  currentState = stage;
  stateStartTime = nowMs;
  calibrationCollecting = false;
  sampleCount = 0;
  emgSensor.resetSignalWindow();
  resetActivationEvidence();

  switch (stage) {
    case SystemState::CALIB_REST:
      sendCalibrationJson("rest", "prepare");
      break;
    case SystemState::CALIB_EXTENSOR_MAX:
      sendCalibrationJson("extensor", "prepare");
      break;
    case SystemState::CALIB_FLEXOR_MAX:
      sendCalibrationJson("flexor", "prepare");
      break;
    default:
      break;
  }
}

void RehabSystem::retryEmgCalibrationStage(const char* reason,
                                            uint32_t nowMs) {
  const char* stage = "unknown";
  switch (currentState) {
    case SystemState::CALIB_REST:
      stage = "rest";
      break;
    case SystemState::CALIB_EXTENSOR_MAX:
      stage = "extensor";
      break;
    case SystemState::CALIB_FLEXOR_MAX:
      stage = "flexor";
      break;
    default:
      return;
  }

  calibrationCollecting = false;
  sampleCount = 0;
  stateStartTime = nowMs;
  emgSensor.resetSignalWindow();
  resetActivationEvidence();

  char message[128];
  snprintf(message, sizeof(message),
           "{\"event\":\"calibration\",\"stage\":\"%s\","
           "\"state\":\"retry\",\"reason\":\"%s\"}",
           stage, reason != nullptr ? reason : "retry");
  ble.sendData(message);
}

void RehabSystem::processEmgCalibration(uint32_t nowMs,
                                         bool newEmgSample,
                                         bool emgDataFault) {
  if (emgDataFault) {
    // A continuously disconnected sensor must not flood BLE with retry frames.
    // Retry immediately while collecting, otherwise report at most once per
    // normal preparation interval while waiting for the signal to recover.
    if (calibrationCollecting ||
        static_cast<uint32_t>(nowMs - stateStartTime) >=
            CALIBRATION_PREPARE_MS) {
      retryEmgCalibrationStage("emg_signal", nowMs);
    }
    return;
  }

  if (!calibrationCollecting) {
    if (static_cast<uint32_t>(nowMs - stateStartTime) <
        CALIBRATION_PREPARE_MS) {
      return;
    }
    if (!newEmgSample || !emgSensor.isWindowReady() ||
        !emgSensor.isSignalQualityGood()) {
      return;
    }

    calibrationCollecting = true;
    stateStartTime = nowMs;
    sampleCount = 0;

    switch (currentState) {
      case SystemState::CALIB_REST:
        sendCalibrationJson("rest", "start");
        break;
      case SystemState::CALIB_EXTENSOR_MAX:
        sendCalibrationJson("extensor", "start");
        break;
      case SystemState::CALIB_FLEXOR_MAX:
        sendCalibrationJson("flexor", "start");
        break;
      default:
        break;
    }
  }

  if (newEmgSample && emgSensor.isWindowReady() &&
      emgSensor.isSignalQualityGood() &&
      sampleCount < EMG_CALIBRATION_FEATURE_COUNT) {
    calibrationExtensorSamples[sampleCount] = emgSensor.getExtensorRMS();
    calibrationFlexorSamples[sampleCount] = emgSensor.getFlexorRMS();
    ++sampleCount;
  }

  if (sampleCount < EMG_CALIBRATION_FEATURE_COUNT) {
    if (static_cast<uint32_t>(nowMs - stateStartTime) >=
        CALIBRATION_COLLECTION_TIMEOUT_MS) {
      retryEmgCalibrationStage("sample_timeout", nowMs);
    }
    return;
  }

  if (currentState == SystemState::CALIB_REST) {
    const float extensorThreshold = calculateRestThreshold(
        calibrationExtensorSamples, sampleCount);
    const float flexorThreshold = calculateRestThreshold(
        calibrationFlexorSamples, sampleCount);
    if (!emgSensor.setRestThresholds(extensorThreshold, flexorThreshold)) {
      retryEmgCalibrationStage("rest_invalid", nowMs);
      return;
    }
    startEmgCalibrationStage(SystemState::CALIB_EXTENSOR_MAX, nowMs);
    return;
  }

  if (currentState == SystemState::CALIB_EXTENSOR_MAX) {
    emgSensor.setExtensorMax(
        calculateRobustMvc(calibrationExtensorSamples, sampleCount));
    if (emgSensor.getExtensorMVC() - emgSensor.getExtensorThreshold() <
        fmaxf(EMG_MIN_CALIBRATION_SPAN,
              emgSensor.getExtensorThreshold() *
                  EMG_MIN_CALIBRATION_SPAN_RATIO)) {
      retryEmgCalibrationStage("extensor_mvc_invalid", nowMs);
      return;
    }
    startEmgCalibrationStage(SystemState::CALIB_FLEXOR_MAX, nowMs);
    return;
  }

  emgSensor.setFlexorMax(
      calculateRobustMvc(calibrationFlexorSamples, sampleCount));
  if (!emgSensor.hasValidCalibration()) {
    retryEmgCalibrationStage("flexor_mvc_invalid", nowMs);
    return;
  }

  startFingerCalibration(0, nowMs);
}

void RehabSystem::startFingerCalibration(uint8_t finger, uint32_t nowMs) {
  if (finger >= FINGER_COUNT) {
    finalizeAssistanceProfiles();
    return;
  }

  if (!servo.setEnabledMask(static_cast<uint8_t>(1U << finger))) {
    enterFault("SERVO_ATTACH");
    return;
  }

  currentState = SystemState::CALIB_PASSIVE_RANGE;
  calibFingerIndex = finger;
  romPhase = RomPhase::PREPARING;
  phaseStartTime = nowMs;
  lastStepTime = nowMs;
  romActivationSum = 0.0f;
  romActivationSamples = 0;
  romRetryCurrentFinger = false;
  romCurrentBaselineSampleCount = 0;
  romLoadRiseHighSamples = 0;
  romStepPeakCurrentMa = currentSensor.getCurrentmA();
  resetStallEvidence();
  resetActivationEvidence();
  emgSensor.resetSignalWindow();
  if (!servo.setMaxAngleForFinger(
          finger, servo.getSoftwareEndAngleForFinger(finger))) {
    enterFault("SERVO_RANGE");
    return;
  }

  char message[72];
  snprintf(message, sizeof(message),
           "{\"event\":\"rom\",\"state\":\"prepare\",\"finger\":%u}",
           finger);
  ble.sendData(message);
}

void RehabSystem::processFingerCalibration(uint32_t nowMs,
                                            bool newEmgSample,
                                            bool emgDataFault,
                                            bool newCurrentSample) {
  const bool emgGap =
      emgDataFault || refreshEmgEvidence(nowMs, newEmgSample);
  if (emgGap && romPhase != RomPhase::RETURNING) {
    retryFingerCalibration("emg_signal", nowMs);
    return;
  }

  if (romPhase == RomPhase::PREPARING) {
    const StallStatus homeStall =
        updateStallStatus(nowMs, RETURN_STALL_CONFIRM_TIME_MS);
    if (homeStall == StallStatus::CONFIRMED) {
      enterFault("HOME_STALL");
      return;
    }
    if (homeStall == StallStatus::PENDING) {
      return;
    }

    const uint32_t preparationElapsed =
        static_cast<uint32_t>(nowMs - phaseStartTime);
    if (newCurrentSample &&
        preparationElapsed >= ROM_CURRENT_BASELINE_SETTLE_MS &&
        romCurrentBaselineSampleCount < ROM_CURRENT_BASELINE_MAX_SAMPLES) {
      romCurrentBaselineSamples[romCurrentBaselineSampleCount++] =
          currentSensor.getCurrentmA();
    }

    if (preparationElapsed < ROM_PREPARE_MS ||
        romCurrentBaselineSampleCount < ROM_CURRENT_BASELINE_MIN_SAMPLES) {
      return;
    }
    if (!emgSensor.isWindowReady() ||
        !emgSensor.isSignalQualityGood()) {
      return;
    }

    if (!finalizeFingerCurrentBaseline(calibFingerIndex)) {
      retryFingerCalibration("current_baseline", nowMs);
      return;
    }

    romPhase = RomPhase::MOVING;
    lastStepTime = nowMs;
    romActivationSum = 0.0f;
    romActivationSamples = 0;
    romLoadRiseHighSamples = 0;
    romStepPeakCurrentMa = currentSensor.getCurrentmA();
    resetStallEvidence();
    resetActivationEvidence();

    char message[72];
    snprintf(message, sizeof(message),
             "{\"event\":\"rom\",\"state\":\"start\",\"finger\":%u}",
             calibFingerIndex);
    ble.sendData(message);
    return;
  }

  if (romPhase == RomPhase::MOVING) {
    FingerProfile& profile = fingerProfiles[calibFingerIndex];
    if (!profile.currentBaselineValid) {
      retryFingerCalibration("current_baseline", nowMs);
      return;
    }

    if (newCurrentSample) {
      const float filteredCurrentMa = currentSensor.getCurrentmA();
      romStepPeakCurrentMa = fmaxf(romStepPeakCurrentMa, filteredCurrentMa);
      if (filteredCurrentMa >= profile.currentTripThresholdMa) {
        if (romLoadRiseHighSamples < UINT8_MAX) {
          ++romLoadRiseHighSamples;
        }
      } else {
        romLoadRiseHighSamples = 0;
      }
    }

    // Freeze extension as soon as the first high sample appears. Confirmation
    // counts only fresh INA219 samples, so one stale or PWM-spike sample cannot
    // advance the finger while the relative load is being checked.
    if (romLoadRiseHighSamples > 0 &&
        romLoadRiseHighSamples < ROM_LOAD_RISE_CONFIRM_SAMPLES) {
      return;
    }

    if (romLoadRiseHighSamples >= ROM_LOAD_RISE_CONFIRM_SAMPLES) {
      if (CURRENT_DIAGNOSTICS_ENABLED) {
        Serial.printf(
            "ROM_CURRENT_TRIP,%u,%d,%.1f,%.1f,%.1f,%.1f,%.1f\n",
            calibFingerIndex, servo.getAngle(calibFingerIndex),
            profile.homeCurrentBaselineMa,
            currentSensor.getRawCurrentmA(), currentSensor.getCurrentmA(),
            romStepPeakCurrentMa, profile.currentTripThresholdMa);
      }
      if (emgSensor.isWindowReady() &&
          emgSensor.getFlexorActivation() >=
              COCONTRACTION_FLEXOR_LEVEL) {
        // Active flexor resistance is not a passive range endpoint. Release a
        // few degrees and invalidate this calibration instead of saving a
        // falsely small target angle.
        retryFingerCalibration("active_resistance", nowMs);
        return;
      }
      finishFingerExtension(true, nowMs);
      return;
    }

    if (newEmgSample && emgSensor.isWindowReady() &&
        emgSensor.isSignalQualityGood()) {
      romActivationSum += emgSensor.getExtensorActivation();
      ++romActivationSamples;
    }

    if (isCocontractionConfirmed(nowMs, newEmgSample)) {
      if (sessionCocontractionCnt < UINT16_MAX) {
        ++sessionCocontractionCnt;
      }
      retryFingerCalibration("cocontraction", nowMs);
      return;
    }

    if (static_cast<uint32_t>(nowMs - lastStepTime) < ROM_STEP_INTERVAL_MS) {
      return;
    }
    lastStepTime = nowMs;

    if (CURRENT_DIAGNOSTICS_ENABLED) {
      const FingerProfile& currentProfile = fingerProfiles[calibFingerIndex];
      Serial.printf("ROM_CURRENT,%u,%d,%.1f,%.1f,%.1f,%.1f,%.1f\n",
                    calibFingerIndex, servo.getAngle(calibFingerIndex),
                    currentProfile.homeCurrentBaselineMa,
                    currentSensor.getRawCurrentmA(),
                    currentSensor.getCurrentmA(), romStepPeakCurrentMa,
                    currentProfile.currentTripThresholdMa);
    }
    romStepPeakCurrentMa = currentSensor.getCurrentmA();

    const int softwareEnd =
        servo.getSoftwareEndAngleForFinger(calibFingerIndex);
    const int nextAngle = servo.getAngle(calibFingerIndex) + 1;
    if (nextAngle >= softwareEnd) {
      if (!servo.setAngle(calibFingerIndex, softwareEnd)) {
        enterFault("ROM_STEP");
        return;
      }
      finishFingerExtension(false, nowMs);
      return;
    }
    if (!servo.setAngle(calibFingerIndex, nextAngle)) {
      enterFault("ROM_STEP");
    }
    return;
  }

  // ROM_RETURNING. Keep moving in the relieving direction while soft current
  // is being qualified. Stop and detach only if it remains high long enough.
  if (static_cast<uint32_t>(nowMs - phaseStartTime) >=
      SERVO_RETURN_TIMEOUT_MS) {
    enterFault("RET_TIMEOUT");
    return;
  }
  const StallStatus stall =
      updateStallStatus(
          nowMs, RETURN_STALL_CONFIRM_TIME_MS,
          calculateGroupSoftCurrentLimit(servo.getEnabledMask()));
  if (stall == StallStatus::CONFIRMED) {
    enterFault("RETURN_STALL");
    return;
  }

  if (!isReturnSettled(nowMs, stall)) {
    return;
  }
  if (!servo.allAtHome(static_cast<uint8_t>(1U << calibFingerIndex))) {
    enterFault("RET_POS");
    return;
  }

  const uint8_t nextFinger =
      romRetryCurrentFinger
          ? calibFingerIndex
          : static_cast<uint8_t>(calibFingerIndex + 1U);
  romRetryCurrentFinger = false;
  if (nextFinger < FINGER_COUNT) {
    startFingerCalibration(nextFinger, nowMs);
  } else {
    finalizeAssistanceProfiles();
  }
}

void RehabSystem::retryFingerCalibration(const char* reason,
                                          uint32_t nowMs) {
  servo.stopMotion();
  const int homeAngle = servo.getHomeAngleForFinger(calibFingerIndex);
  const int currentAngle = servo.getAngle(calibFingerIndex);
  const int reliefAngle =
      max(homeAngle, currentAngle - static_cast<int>(ROM_SAFETY_MARGIN_DEG));
  if (!servo.setAngle(calibFingerIndex, reliefAngle)) {
    enterFault("ROM_RELEASE");
    return;
  }

  FingerProfile& profile = fingerProfiles[calibFingerIndex];
  profile.targetAngle = homeAngle;
  profile.meanExtensorActivation = 0.0f;
  profile.homeCurrentBaselineMa = 0.0f;
  profile.currentNoiseMa = 0.0f;
  profile.currentTripThresholdMa = STALL_CURRENT_THRES_MA;
  profile.calibrated = false;
  profile.physicalLimitDetected = false;
  profile.currentBaselineValid = false;
  calibratedFingerMask &=
      static_cast<uint8_t>(~(1U << calibFingerIndex));
  romRetryCurrentFinger = true;

  char message[144];
  snprintf(message, sizeof(message),
           "{\"event\":\"rom\",\"state\":\"error\",\"finger\":%u,"
           "\"reason\":\"%s\",\"retry\":true}",
           calibFingerIndex, reason != nullptr ? reason : "retry");
  ble.sendData(message);

  resetStallEvidence();
  resetActivationEvidence();
  if (!servo.startReturn(static_cast<uint8_t>(1U << calibFingerIndex),
                         SERVO_RETURN_DURATION_MS, nowMs)) {
    enterFault("RETURN_START");
    return;
  }
  romPhase = RomPhase::RETURNING;
  phaseStartTime = nowMs;
}

void RehabSystem::finishFingerExtension(bool physicalLimitDetected,
                                         uint32_t nowMs) {
  const int observedAngle = servo.getAngle(calibFingerIndex);
  const int homeAngle = servo.getHomeAngleForFinger(calibFingerIndex);
  const int targetAngle =
      max(homeAngle,
          observedAngle - static_cast<int>(ROM_SAFETY_MARGIN_DEG));
  FingerProfile& profile = fingerProfiles[calibFingerIndex];

  const bool enoughSamples = romActivationSamples >= 25U;
  const bool validTarget =
      targetAngle - homeAngle >= ROM_MIN_TARGET_ANGLE;
  const bool reachedSoftwareEnd =
      observedAngle >= servo.getSoftwareEndAngleForFinger(calibFingerIndex);
  const bool acceptedEndpoint = physicalLimitDetected || reachedSoftwareEnd;
  // setMaxAngleForFinger() immediately clamps the currently commanded angle
  // down to targetAngle when a load endpoint was found. This provides the
  // 3-degree relief before the slower return trajectory begins.
  if (acceptedEndpoint && enoughSamples && validTarget &&
      servo.setMaxAngleForFinger(calibFingerIndex, targetAngle)) {
    profile.targetAngle = targetAngle;
    profile.meanExtensorActivation =
        romActivationSum / static_cast<float>(romActivationSamples);
    profile.calibrated = true;
    profile.physicalLimitDetected = physicalLimitDetected;
    calibratedFingerMask |= static_cast<uint8_t>(1U << calibFingerIndex);

    char message[224];
    snprintf(message, sizeof(message),
             "{\"event\":\"rom_done\",\"finger\":\"%s\","
             "\"finger_index\":%u,\"detected_angle\":%d,"
             "\"target_angle\":%d,\"endpoint\":\"%s\","
             "\"detector\":\"%s\"}",
             FINGER_NAMES[calibFingerIndex], calibFingerIndex,
             observedAngle, targetAngle,
             physicalLimitDetected ? "stall" : "travel_end",
             physicalLimitDetected ? "relative_current" : "software_end");
    ble.sendData(message);
  } else {
    retryFingerCalibration(!enoughSamples ? "insufficient_data"
                                          : "target_too_small",
                           nowMs);
    return;
  }

  resetStallEvidence();
  resetActivationEvidence();
  servo.stopMotion();
  if (!servo.startReturn(static_cast<uint8_t>(1U << calibFingerIndex),
                         SERVO_RETURN_DURATION_MS, nowMs)) {
    enterFault("RETURN_START");
    return;
  }
  romPhase = RomPhase::RETURNING;
  phaseStartTime = nowMs;
}

bool RehabSystem::finalizeFingerCurrentBaseline(uint8_t finger) {
  if (finger >= FINGER_COUNT ||
      romCurrentBaselineSampleCount < ROM_CURRENT_BASELINE_MIN_SAMPLES) {
    return false;
  }

  sortFloatSamples(romCurrentBaselineSamples,
                   romCurrentBaselineSampleCount);
  const float baseline = medianOfSorted(
      romCurrentBaselineSamples, romCurrentBaselineSampleCount);
  if (!isfinite(baseline) || baseline < 0.0f) {
    return false;
  }

  for (uint16_t i = 0; i < romCurrentBaselineSampleCount; ++i) {
    romCurrentBaselineSamples[i] =
        fabsf(romCurrentBaselineSamples[i] - baseline);
  }
  sortFloatSamples(romCurrentBaselineSamples,
                   romCurrentBaselineSampleCount);
  const float mad = medianOfSorted(
      romCurrentBaselineSamples, romCurrentBaselineSampleCount);
  const float robustNoise = EMG_MAD_TO_SIGMA_SCALE * mad;
  const float configuredRise = getRomFingerCurrentRiseLimit(finger);
  const float tripThreshold =
      fminf(STALL_CURRENT_THRES_MA, baseline + configuredRise);
  if (!isfinite(robustNoise) || !isfinite(configuredRise) ||
      !isfinite(tripThreshold) || configuredRise <= 0.0f ||
      tripThreshold <= baseline || tripThreshold >= HARD_CURRENT_THRES_MA) {
    return false;
  }

  FingerProfile& profile = fingerProfiles[finger];
  profile.homeCurrentBaselineMa = baseline;
  profile.currentNoiseMa = robustNoise;
  profile.currentTripThresholdMa = tripThreshold;

  const bool noiseWarning =
      ROM_CURRENT_NOISE_SIGMA_MULTIPLIER * robustNoise >= configuredRise;
  profile.currentBaselineValid = !noiseWarning;
  char message[224];
  snprintf(message, sizeof(message),
           "{\"event\":\"rom_current_baseline\",\"finger\":\"%s\","
           "\"finger_index\":%u,\"baseline_ma\":%.1f,"
           "\"noise_ma\":%.1f,\"rise_limit_ma\":%.1f,"
           "\"trip_ma\":%.1f,\"noise_warning\":%s}",
           FINGER_NAMES[finger], finger, baseline, robustNoise,
           configuredRise, tripThreshold,
           noiseWarning ? "true" : "false");
  ble.sendData(message);
  Serial.println(message);
  return !noiseWarning;
}

float RehabSystem::getRomFingerCurrentRiseLimit(uint8_t finger) const {
  return finger < FINGER_COUNT
             ? ROM_FINGER_CURRENT_RISE_LIMIT_MA[finger]
             : STALL_CURRENT_THRES_MA;
}

void RehabSystem::finalizeAssistanceProfiles() {
  if (calibratedFingerMask != ALL_FINGERS_MASK) {
    currentState = SystemState::IDLE;
    sendJsonError("ROM_CAL");
    return;
  }

  float bestActivation = 0.0f;
  float lowestActivation = 1.0f;
  for (uint8_t i = 0; i < FINGER_COUNT; ++i) {
    if (fingerProfiles[i].calibrated) {
      const float activation =
          clampUnit(fingerProfiles[i].meanExtensorActivation);
      if (activation > bestActivation) {
        bestActivation = activation;
      }
      if (activation < lowestActivation) {
        lowestActivation = activation;
      }
    }
  }

  const float activationSpread = bestActivation - lowestActivation;
  const float relativeBlend =
      ASSISTANCE_RELATIVE_SPREAD_FULL > 0.0f
          ? 0.30f * clampUnit(activationSpread /
                              ASSISTANCE_RELATIVE_SPREAD_FULL)
          : 0.0f;

  for (uint8_t i = 0; i < FINGER_COUNT; ++i) {
    FingerProfile& profile = fingerProfiles[i];
    if (!profile.calibrated) {
      continue;
    }

    const float activation = clampUnit(profile.meanExtensorActivation);
    const float absoluteWeakness = clampUnit(
        (ASSISTANCE_TARGET_ACTIVATION - activation) /
        ASSISTANCE_TARGET_ACTIVATION);
    const float relativeWeakness =
        bestActivation > 0.0f
            ? clampUnit((bestActivation - activation) / bestActivation)
            : 0.0f;
    const float weakness =
        clampUnit(((1.0f - relativeBlend) * absoluteWeakness) +
                  (relativeBlend * relativeWeakness));
    const float weightedWeakness = smoothStep(weakness);

    profile.assistLevel = static_cast<uint8_t>(
        1U + static_cast<uint8_t>(lroundf(9.0f * weightedWeakness)));
    profile.failuresAtLevel = 0;
  }

  selectedFingerMask &= calibratedFingerMask;
  currentState = SystemState::TRAINING_READY;
  if (!servo.setEnabledMask(0)) {
    enterFault("SERVO_DISABLE");
    return;
  }
  emgSensor.resetSignalWindow();
  resetActivationEvidence();

  char message[224];
  snprintf(message, sizeof(message),
           "{\"event\":\"calibration_done\",\"mask\":%u,"
           "\"target_angles\":{\"thumb\":%d,\"index\":%d,"
           "\"middle\":%d,\"ring\":%d,\"pinky\":%d}}",
           calibratedFingerMask,
           fingerProfiles[0].targetAngle, fingerProfiles[1].targetAngle,
           fingerProfiles[2].targetAngle, fingerProfiles[3].targetAngle,
           fingerProfiles[4].targetAngle);
  ble.sendData(message);
}

void RehabSystem::startTrainingSession(uint32_t nowMs) {
  trainingMask = selectedFingerMask & calibratedFingerMask;
  if (trainingMask == 0 || !servo.allAtHome(trainingMask) ||
      !servo.setEnabledMask(0)) {
    enterFault("TRAIN_INIT");
    return;
  }

  // Attach selected servos once, with a small stagger, before the first
  // trigger. They remain attached at home between cycles so the cable keeps
  // the requested mechanical constraint.
  trainingPhase = TrainingPhase::PREPARING;
  ++trainingSessionId;
  if (trainingSessionId == 0) {
    trainingSessionId = 1;
  }
  trainingStartDeviceMs = nowMs;
  trainingReturnCause = TrainingReturnCause::ATTEMPT;
  trainingTriggered = false;
  trainingSessionActive = true;
  trainingGoalReached = false;
  trainingCurrentSafetyIncidentLatched = false;
  phaseStartTime = nowMs;
  trainingAttachMask = 0;
  lastTrainingAttachTime = nowMs - TRAIN_MOTOR_ATTACH_STAGGER_MS;
  trainingAttachCompleteTime = 0;
  stallConfirmationHighSamples = 0;
  stallConfirmationClearSamples = 0;
  trainingResumeCount = 0;
  trainingTriggerActivationReference = 0.0f;
  trainingParticipationRequired = 0.0f;
  sessionTotalAttempts = 0;
  sessionSuccessCount = 0;
  sessionParticipationFailureCount = 0;
  sessionCocontractionCnt = 0;
  sessionCurrentSafetyCount = 0;
  sessionAngleSum = 0.0;
  sessionAngleSampleCount = 0;
  sessionMaxExtensorRms = 0.0f;
  sessionMaxFlexorRms = 0.0f;
  lastAttemptSuccess = false;
  sessionMetricsSequence = 0;
  pendingSummaryMetricsSequence = 0;
  lastSummaryTransmitTime = 0;
  resetStallEvidence();
  resetActivationEvidence();
  resetParticipationEvidence();
  emgSensor.resetSignalWindow();
  currentState = SystemState::TRAINING_ACTIVE;
  char sessionMessage[128];
  snprintf(sessionMessage, sizeof(sessionMessage),
           "{\"event\":\"session_start\",\"v\":%u,\"boot\":%lu,"
           "\"sid\":%lu,\"start_ms\":%lu,\"mask\":%u}",
           BLE_PROTOCOL_VERSION,
           static_cast<unsigned long>(bootId),
           static_cast<unsigned long>(trainingSessionId),
           static_cast<unsigned long>(trainingStartDeviceMs), trainingMask);
  ble.sendData(sessionMessage);
  ble.sendData("{\"event\":\"training\",\"state\":\"arming\"}");
}

void RehabSystem::processTraining(uint32_t nowMs,
                                  bool newEmgSample,
                                  bool emgDataFault,
                                  bool newCurrentSample) {
  const bool emgGap =
      emgDataFault || refreshEmgEvidence(nowMs, newEmgSample);
  if (emgGap && trainingPhase != TrainingPhase::RETURNING) {
    const float groupLimit = calculateGroupSoftCurrentLimit(trainingMask);
    if ((trainingPhase == TrainingPhase::MOVING ||
         trainingPhase == TrainingPhase::STALL_CONFIRMING ||
         trainingPhase == TrainingPhase::HOLDING) &&
        currentSensor.getCurrentmA() >= groupLimit &&
        !relieveTrainingGroup()) {
      enterFault("STALL_RELEASE");
      return;
    }
    triggerSafetyReturn("EMG_BAD", SystemState::TRAINING_READY, false,
                        nowMs);
    return;
  }

  if (trainingPhase == TrainingPhase::WAITING_RELEASE) {
    trainingTriggered = false;
    const StallStatus waitingLoad = updateStallStatus(
        nowMs, STALL_CONFIRM_TIME_MS,
        calculateGroupSoftCurrentLimit(trainingMask));
    if (waitingLoad == StallStatus::CONFIRMED) {
      recordTrainingCurrentSafety("waiting_release");
      beginTrainingReturn(nowMs, false,
                          TrainingReturnCause::CURRENT_SAFETY);
      return;
    }
    if (waitingLoad == StallStatus::PENDING) {
      return;
    }
    if (!newEmgSample || !emgSensor.isWindowReady()) {
      return;
    }

    const bool relaxed =
        emgSensor.getExtensorActivation() <= TRAIN_RELEASE_ACTIVATION &&
        emgSensor.getFlexorActivation() <= TRAIN_RELEASE_ACTIVATION;
    if (!relaxed) {
      releasePending = false;
      return;
    }
    if (!releasePending) {
      releasePending = true;
      releaseStartTime = nowMs;
      return;
    }
    if (static_cast<uint32_t>(nowMs - releaseStartTime) <
        TRAIN_RELEASE_HOLD_MS) {
      return;
    }

    resetActivationEvidence();
    resetStallEvidence();
    trainingPhase = TrainingPhase::WAITING_TRIGGER;
    ble.sendData("{\"event\":\"training\",\"state\":\"ready\"}");
    return;
  }

  if (trainingPhase == TrainingPhase::WAITING_TRIGGER) {
    const StallStatus waitingLoad = updateStallStatus(
        nowMs, STALL_CONFIRM_TIME_MS,
        calculateGroupSoftCurrentLimit(trainingMask));
    if (waitingLoad == StallStatus::CONFIRMED) {
      recordTrainingCurrentSafety("waiting_trigger");
      beginTrainingReturn(nowMs, false,
                          TrainingReturnCause::CURRENT_SAFETY);
      return;
    }
    if (waitingLoad == StallStatus::PENDING) {
      return;
    }
    if (!newEmgSample || !emgSensor.isWindowReady()) {
      return;
    }
    if (isCocontractionConfirmed(nowMs, true)) {
      triggerPending = false;
      return;
    }

    const bool triggerCondition =
        emgSensor.getExtensorActivation() >= TRAIN_TRIGGER_ACTIVATION &&
        emgSensor.getFlexorActivation() < COCONTRACTION_FLEXOR_LEVEL;
    if (!triggerCondition) {
      triggerPending = false;
      return;
    }
    if (!triggerPending) {
      triggerPending = true;
      triggerStartTime = nowMs;
      return;
    }
    if (static_cast<uint32_t>(nowMs - triggerStartTime) <
        TRAIN_TRIGGER_HOLD_MS) {
      return;
    }

    triggerPending = false;
    trainingTriggerActivationReference =
        fmaxf(emgSensor.getExtensorActivation(), TRAIN_TRIGGER_ACTIVATION);
    trainingParticipationRequired = calculateParticipationRequirement(
        trainingMask, trainingTriggerActivationReference);
    resetParticipationEvidence();
    if (!startTrainingGroup(nowMs)) {
      enterFault("TRAIN_START");
    }
    return;
  }

  if (trainingPhase == TrainingPhase::PREPARING) {
    if (static_cast<uint32_t>(nowMs - phaseStartTime) >=
        TRAIN_PREPARE_TIMEOUT_MS) {
      triggerSafetyReturn("PREP_TO", SystemState::TRAINING_READY, false,
                          nowMs);
      return;
    }

    if (trainingAttachMask != trainingMask) {
      if (static_cast<uint32_t>(nowMs - lastTrainingAttachTime) <
          TRAIN_MOTOR_ATTACH_STAGGER_MS) {
        return;
      }

      uint8_t nextFinger = FINGER_COUNT;
      for (uint8_t i = 0; i < FINGER_COUNT; ++i) {
        if ((trainingMask & (1U << i)) != 0 &&
            (trainingAttachMask & (1U << i)) == 0) {
          nextFinger = i;
          break;
        }
      }
      if (nextFinger >= FINGER_COUNT) {
        enterFault("ATTACH_MASK");
        return;
      }

      const uint8_t nextMask =
          trainingAttachMask | static_cast<uint8_t>(1U << nextFinger);
      if (!servo.setEnabledMask(nextMask)) {
        enterFault("SERVO_ATTACH");
        return;
      }
      trainingAttachMask = nextMask;
      lastTrainingAttachTime = nowMs;
      resetStallEvidence();
      if (trainingAttachMask == trainingMask) {
        trainingAttachCompleteTime = nowMs;
      }
      return;
    }

    if (static_cast<uint32_t>(nowMs - trainingAttachCompleteTime) <
        TRAIN_ATTACH_INRUSH_IGNORE_MS) {
      resetStallEvidence();
      return;
    }

    const float groupLimit =
        calculateGroupSoftCurrentLimit(trainingAttachMask);
    const StallStatus attachLoad =
        updateStallStatus(nowMs, STALL_CONFIRM_TIME_MS, groupLimit);
    if (attachLoad == StallStatus::CONFIRMED) {
      recordTrainingCurrentSafety("arming");
      beginTrainingReturn(nowMs, false,
                          TrainingReturnCause::CURRENT_SAFETY);
      return;
    }
    if (attachLoad == StallStatus::PENDING) {
      homeSettlePending = false;
      homeSettleStartTime = 0;
      return;
    }

    if (!homeSettlePending) {
      homeSettlePending = true;
      homeSettleStartTime = nowMs;
      return;
    }
    if (static_cast<uint32_t>(nowMs - homeSettleStartTime) <
        TRAIN_ATTACH_SETTLE_MS) {
      return;
    }

    trainingPhase = TrainingPhase::WAITING_RELEASE;
    phaseStartTime = nowMs;
    trainingTriggered = false;
    resetStallEvidence();
    resetActivationEvidence();
    resetParticipationEvidence();
    ble.sendData("{\"event\":\"training\",\"state\":\"armed\"}");
    return;
  }

  if (trainingPhase == TrainingPhase::STALL_CONFIRMING) {
    if (isCocontractionConfirmed(nowMs, newEmgSample) ||
        (emgSensor.isWindowReady() &&
         emgSensor.getFlexorActivation() >=
             COCONTRACTION_FLEXOR_LEVEL)) {
      if (sessionCocontractionCnt < UINT16_MAX) {
        ++sessionCocontractionCnt;
      }
      triggerSafetyReturn("RESIST", SystemState::TRAINING_READY, false,
                          nowMs);
      return;
    }

    if (static_cast<uint32_t>(nowMs - phaseStartTime) >=
        TRAIN_STALL_CONFIRM_TIMEOUT_MS) {
      recordTrainingCurrentSafety("moving_timeout");
      beginTrainingReturn(nowMs, true,
                          TrainingReturnCause::CURRENT_SAFETY);
      return;
    }
    if (!newCurrentSample) {
      return;
    }

    const bool currentStillHigh =
        currentSensor.getCurrentmA() >=
        calculateGroupSoftCurrentLimit(trainingMask);

    if (!currentStillHigh) {
      if (stallConfirmationClearSamples < UINT8_MAX) {
        ++stallConfirmationClearSamples;
      }
      if (stallConfirmationClearSamples < TRAIN_STALL_CLEAR_SAMPLES) {
        return;
      }

      if (trainingResumeCount >= TRAIN_MAX_TRANSIENT_RESUMES) {
        recordTrainingCurrentSafety("moving_unstable");
        beginTrainingReturn(nowMs, true,
                            TrainingReturnCause::CURRENT_SAFETY);
        return;
      }
      ++trainingResumeCount;
      if (!startTrainingTrajectory(nowMs, true)) {
        enterFault("MOVE_RESUME");
      }
      return;
    }

    stallConfirmationClearSamples = 0;
    if (stallConfirmationHighSamples < UINT8_MAX) {
      ++stallConfirmationHighSamples;
    }
    if (stallConfirmationHighSamples < TRAIN_STALL_PERSIST_SAMPLES) {
      return;
    }
    recordTrainingCurrentSafety("moving");
    beginTrainingReturn(nowMs, true,
                        TrainingReturnCause::CURRENT_SAFETY);
    return;
  }

  if (trainingPhase == TrainingPhase::MOVING) {
    if (isCocontractionConfirmed(nowMs, newEmgSample)) {
      if (sessionCocontractionCnt < UINT16_MAX) {
        ++sessionCocontractionCnt;
      }
      triggerSafetyReturn("COCON", SystemState::TRAINING_READY, false, nowMs);
      return;
    }

    const float groupLimit =
        calculateGroupSoftCurrentLimit(trainingMask);
    const StallStatus aggregateStall =
        updateStallStatus(nowMs, STALL_CONFIRM_TIME_MS, groupLimit);

    if (aggregateStall != StallStatus::CLEAR) {
      beginTrainingStallConfirmation(nowMs);
      return;
    }

    if (processTrainingParticipation(nowMs, newEmgSample)) {
      recordTrainingAttempt(trainingMask, false);
      beginTrainingReturn(nowMs, false);
      return;
    }

    if (!servo.isBusy()) {
      const uint8_t incompleteMask = getIncompleteTrainingMask();
      if (incompleteMask != 0) {
        enterFault("MOVE_POS");
        return;
      }

      trainingPhase = TrainingPhase::HOLDING;
      phaseStartTime = nowMs;
      stallConfirmationHighSamples = 0;
      resetParticipationEvidence();
      resetStallEvidence();
      char message[104];
      snprintf(message, sizeof(message),
               "{\"event\":\"training\",\"state\":\"holding\","
               "\"mask\":%u,\"hold_time_sec\":%lu}",
               trainingMask,
               static_cast<unsigned long>(TRAIN_HOLD_TIME_MS / 1000UL));
      ble.sendData(message);
    }
    return;
  }

  if (trainingPhase == TrainingPhase::HOLDING) {
    if (isCocontractionConfirmed(nowMs, newEmgSample)) {
      if (sessionCocontractionCnt < UINT16_MAX) {
        ++sessionCocontractionCnt;
      }
      triggerSafetyReturn("COCON", SystemState::TRAINING_READY, false, nowMs);
      return;
    }

    const StallStatus holdStall =
        updateStallStatus(nowMs, STALL_CONFIRM_TIME_MS,
                          calculateGroupSoftCurrentLimit(trainingMask));
    if (holdStall == StallStatus::CONFIRMED) {
      recordTrainingCurrentSafety("holding");
      beginTrainingReturn(nowMs, true,
                          TrainingReturnCause::CURRENT_SAFETY);
      return;
    }

    if (holdStall == StallStatus::PENDING ||
        cocontractionPending || !newEmgSample) {
      return;
    }

    if (static_cast<uint32_t>(nowMs - phaseStartTime) >=
        TRAIN_HOLD_TIME_MS) {
      recordTrainingAttempt(trainingMask, true);
      beginTrainingReturn(nowMs, false);
    }
    return;
  }

  // RETURNING: every selected finger follows the same return trajectory.
  if (static_cast<uint32_t>(nowMs - phaseStartTime) >=
      SERVO_RETURN_TIMEOUT_MS) {
    enterFault("RET_TIMEOUT");
    return;
  }
  const StallStatus returnStall =
      updateStallStatus(nowMs, RETURN_STALL_CONFIRM_TIME_MS,
                        calculateGroupSoftCurrentLimit(trainingMask));
  if (returnStall == StallStatus::CONFIRMED) {
    recordTrainingCurrentSafety("returning");
    enterFault("RETURN_STALL");
    return;
  }

  if (!isReturnSettled(nowMs, returnStall)) {
    return;
  }
  if (!servo.allAtHome(trainingMask)) {
    enterFault("RET_POS");
    return;
  }
  trainingAttachMask = servo.getEnabledMask() & trainingMask;
  stallConfirmationHighSamples = 0;
  stallConfirmationClearSamples = 0;
  trainingTriggered = false;
  resetStallEvidence();
  resetActivationEvidence();
  resetParticipationEvidence();
  emgSensor.resetSignalWindow();

  if (trainingReturnCause != TrainingReturnCause::ATTEMPT) {
    const char* resumeAfter =
        trainingReturnCause == TrainingReturnCause::CURRENT_SAFETY
            ? "current_safety"
            : "user_stop";
    trainingReturnCause = TrainingReturnCause::ATTEMPT;
    trainingCurrentSafetyIncidentLatched = false;
    trainingResumeCount = 0;
    const bool needsArming = trainingAttachMask != trainingMask;
    trainingPhase = needsArming ? TrainingPhase::PREPARING
                                : TrainingPhase::WAITING_RELEASE;
    phaseStartTime = nowMs;
    if (needsArming) {
      lastTrainingAttachTime = nowMs - TRAIN_MOTOR_ATTACH_STAGGER_MS;
      trainingAttachCompleteTime = 0;
    }

    char resumeMessage[192];
    snprintf(resumeMessage, sizeof(resumeMessage),
             "{\"event\":\"training\",\"state\":\"%s\","
             "\"sid\":%lu,\"resume_after\":\"%s\","
             "\"session_continues\":true,\"success_count\":%u}",
             needsArming ? "arming" : "waiting_release",
             static_cast<unsigned long>(trainingSessionId), resumeAfter,
             sessionSuccessCount);
    ble.sendData(resumeMessage);
    return;
  }

  trainingPhase = TrainingPhase::WAITING_RELEASE;
  phaseStartTime = nowMs;
  char cycleMessage[176];
  snprintf(cycleMessage, sizeof(cycleMessage),
           "{\"event\":\"cycle_done\",\"sid\":%lu,\"success\":%s,"
           "\"success_count\":%u,\"success_goal\":%u,"
           "\"goal_reached\":%s}",
           static_cast<unsigned long>(trainingSessionId),
           lastAttemptSuccess ? "true" : "false", sessionSuccessCount,
           TRAIN_SUCCESS_GOAL,
           trainingGoalReached ? "true" : "false");
  ble.sendData(cycleMessage);

  if (trainingGoalReached) {
    if (!servo.setEnabledMask(0)) {
      enterFault("SERVO_DISABLE");
      return;
    }
    trainingAttachMask = 0;
    // The fifth successful cycle is home and can now release PWM safely.
    // End locally so a lost BLE command cannot leave the session open.
    finishTrainingSession("GOAL");
    trainingMask = 0;
    trainingGoalReached = false;
    trainingReturnCause = TrainingReturnCause::ATTEMPT;
    currentState = SystemState::TRAINING_READY;
    sendJsonState("READY");
  }
}
bool RehabSystem::startTrainingGroup(uint32_t nowMs) {
  if (trainingMask == 0 ||
      !allSelectedFingersCalibrated(trainingMask) ||
      !servo.allAtHome(trainingMask) || servo.isBusy() ||
      (servo.getEnabledMask() & trainingMask) != trainingMask) {
    return false;
  }

  trainingCurrentSafetyIncidentLatched = false;
  trainingResumeCount = 0;
  stallConfirmationHighSamples = 0;
  stallConfirmationClearSamples = 0;
  trainingTriggered = true;
  resetStallEvidence();
  resetActivationEvidence();
  return startTrainingTrajectory(nowMs, false);
}

bool RehabSystem::startTrainingTrajectory(uint32_t nowMs, bool resumed) {
  if (trainingMask == 0 ||
      (servo.getEnabledMask() & trainingMask) != trainingMask) {
    return false;
  }

  // A transient current check pauses the motor. Do not count that pause as
  // time in which the patient failed to participate.
  if (resumed) {
    const uint32_t pausedMs = static_cast<uint32_t>(nowMs - phaseStartTime);
    if (participationLowPending) {
      participationLowStartTime += pausedMs;
    }
    if (participationRecoveryStartTime != 0) {
      participationRecoveryStartTime += pausedMs;
    }
  }

  int targetAngles[FINGER_COUNT];
  uint32_t durations[FINGER_COUNT];
  for (uint8_t i = 0; i < FINGER_COUNT; ++i) {
    targetAngles[i] = servo.getAngle(i);
    durations[i] = TRAIN_MOVE_DURATION_MS;
    if ((trainingMask & (1U << i)) != 0) {
      targetAngles[i] = fingerProfiles[i].targetAngle;
      const uint32_t fullDuration = TRAIN_MOVE_DURATION_MS;
      const uint32_t remainingAngle = static_cast<uint32_t>(
          abs(targetAngles[i] - servo.getAngle(i)));
      const uint32_t fullAngle = static_cast<uint32_t>(
          max(1, abs(targetAngles[i] -
                     servo.getHomeAngleForFinger(i))));
      uint32_t requestedDuration = fullDuration;
      if (resumed) {
        requestedDuration = static_cast<uint32_t>(
            (static_cast<uint64_t>(fullDuration) * remainingAngle +
             fullAngle - 1U) /
            fullAngle);
        requestedDuration =
            max(requestedDuration, TRAIN_RESUME_MIN_DURATION_MS);
      }
      durations[i] = requestedDuration;
    }
  }

  resetStallEvidence();
  resetActivationEvidence();
  stallConfirmationHighSamples = 0;
  stallConfirmationClearSamples = 0;
  if (!servo.startMoveFingers(trainingMask, targetAngles, durations, nowMs,
                              TRAIN_GROUP_MAX_COMMAND_STEP_DEG)) {
    return false;
  }

  trainingPhase = TrainingPhase::MOVING;
  char message[104];
  snprintf(message, sizeof(message),
           "{\"event\":\"training\",\"state\":\"moving\",\"mask\":%u,"
           "\"resumed\":%s}",
           trainingMask, resumed ? "true" : "false");
  ble.sendData(message);
  return true;
}

void RehabSystem::beginTrainingStallConfirmation(uint32_t nowMs) {
  servo.stopMotion();
  trainingPhase = TrainingPhase::STALL_CONFIRMING;
  phaseStartTime = nowMs;
  // The sample that caused this state transition is the first high-evidence
  // sample. Further high samples may be interleaved with brief PWM lows.
  stallConfirmationHighSamples = 1;
  stallConfirmationClearSamples = 0;
  stallPending = false;
  stallStartTime = 0;
  homeSettlePending = false;
  homeSettleStartTime = 0;

  char message[128];
  snprintf(message, sizeof(message),
           "{\"event\":\"current_safety\",\"state\":\"checking\","
           "\"sid\":%lu,\"mask\":%u,\"limit_ma\":%.1f}",
           static_cast<unsigned long>(trainingSessionId), trainingMask,
           calculateGroupSoftCurrentLimit(trainingMask));
  ble.sendData(message);
}

void RehabSystem::beginTrainingReturn(uint32_t nowMs,
                                      bool applyImmediateRelief,
                                      TrainingReturnCause cause) {
  servo.stopMotion();
  if (applyImmediateRelief && !relieveTrainingGroup()) {
    enterFault("STALL_RELEASE");
    return;
  }
  if (!servo.startReturn(trainingMask, SERVO_RETURN_DURATION_MS, nowMs)) {
    enterFault("RETURN_START");
    return;
  }

  trainingReturnCause = cause;
  trainingPhase = TrainingPhase::RETURNING;
  trainingTriggered = false;
  phaseStartTime = nowMs;
  stallConfirmationHighSamples = 0;
  stallConfirmationClearSamples = 0;
  resetStallEvidence();
  resetActivationEvidence();
  resetParticipationEvidence();

  if (cause != TrainingReturnCause::ATTEMPT) {
    const char* reason =
        cause == TrainingReturnCause::CURRENT_SAFETY ? "current_safety"
                                                     : "user_stop";
    char interruption[176];
    snprintf(interruption, sizeof(interruption),
             "{\"event\":\"cycle_interrupted\",\"sid\":%lu,"
             "\"reason\":\"%s\",\"counted_attempt\":false,"
             "\"session_continues\":true}",
             static_cast<unsigned long>(trainingSessionId), reason);
    ble.sendData(interruption);
  }

  char message[80];
  snprintf(message, sizeof(message),
           "{\"event\":\"training\",\"state\":\"returning\","
           "\"mask\":%u}",
           trainingMask);
  ble.sendData(message);
}

void RehabSystem::recordTrainingAttempt(uint8_t fingerMask, bool success) {
  if (trainingMask == 0) {
    return;
  }

  if (sessionTotalAttempts < UINT16_MAX) {
    ++sessionTotalAttempts;
  }
  lastAttemptSuccess = success;
  for (uint8_t i = 0; i < FINGER_COUNT; ++i) {
    if ((trainingMask & (1U << i)) == 0) {
      continue;
    }
    sessionAngleSum += static_cast<double>(servo.getAngle(i));
    if (sessionAngleSampleCount < UINT32_MAX) {
      ++sessionAngleSampleCount;
    }
  }

  if (success) {
    if (sessionSuccessCount < UINT16_MAX) {
      ++sessionSuccessCount;
    }
    if (sessionSuccessCount >= TRAIN_SUCCESS_GOAL) {
      trainingGoalReached = true;
    }
    char attemptMessage[112];
    snprintf(attemptMessage, sizeof(attemptMessage),
             "{\"event\":\"attempt\",\"sid\":%lu,\"success\":true,"
             "\"reason\":\"target\",\"mask\":%u}",
             static_cast<unsigned long>(trainingSessionId), trainingMask);
    ble.sendData(attemptMessage);
    sendTrainingSessionMetrics();
    return;
  }

  if (sessionParticipationFailureCount < UINT16_MAX) {
    ++sessionParticipationFailureCount;
  }

  uint8_t attributedMask = fingerMask & trainingMask;
  if (attributedMask == 0) {
    attributedMask = trainingMask;
  }
  for (uint8_t i = 0; i < FINGER_COUNT; ++i) {
    const uint8_t bit = static_cast<uint8_t>(1U << i);
    if ((attributedMask & bit) == 0) {
      continue;
    }

    FingerProfile& profile = fingerProfiles[i];
    if (profile.failuresAtLevel < FAILURES_BEFORE_LEVEL_UP) {
      ++profile.failuresAtLevel;
    }
    if (profile.failuresAtLevel < FAILURES_BEFORE_LEVEL_UP) {
      continue;
    }

    if (profile.assistLevel < 10) {
      profile.failuresAtLevel = 0;
      ++profile.assistLevel;
      char message[112];
      snprintf(message, sizeof(message),
               "{\"event\":\"assist_level\",\"finger\":%u,"
               "\"level\":%u,\"reason\":\"participation\"}",
               i, profile.assistLevel);
      ble.sendData(message);
    }
  }

  char attemptMessage[128];
  snprintf(attemptMessage, sizeof(attemptMessage),
           "{\"event\":\"attempt\",\"sid\":%lu,\"success\":false,"
           "\"reason\":\"participation\",\"mask\":%u}",
           static_cast<unsigned long>(trainingSessionId), attributedMask);
  ble.sendData(attemptMessage);
  sendTrainingSessionMetrics();
}

bool RehabSystem::processTrainingParticipation(uint32_t nowMs,
                                                bool newEmgSample) {
  if (!newEmgSample || !emgSensor.isWindowReady() ||
      !emgSensor.isSignalQualityGood()) {
    return false;
  }

  const float activation = emgSensor.getExtensorActivation();
  if (activation < trainingParticipationRequired) {
    participationRecoveryStartTime = 0;
    if (!participationLowPending) {
      participationLowPending = true;
      participationLowStartTime = nowMs;
      return false;
    }

    const uint32_t lowDuration =
        static_cast<uint32_t>(nowMs - participationLowStartTime);
    if (!participationWarningActive &&
        lowDuration >= TRAIN_PARTICIPATION_WARNING_MS) {
      participationWarningActive = true;
      char warning[176];
      snprintf(warning, sizeof(warning),
               "{\"event\":\"participation\",\"state\":\"warning\","
               "\"sid\":%lu,\"mask\":%u,\"level\":%u,"
               "\"actual_pct\":%u,\"required_pct\":%u,"
               "\"grace_ms\":%lu}",
               static_cast<unsigned long>(trainingSessionId), trainingMask,
               getMaximumAssistLevel(trainingMask),
               static_cast<unsigned>(lroundf(100.0f * activation)),
               static_cast<unsigned>(
                   lroundf(100.0f * trainingParticipationRequired)),
               static_cast<unsigned long>(
                   TRAIN_PARTICIPATION_FAILURE_MS -
                   TRAIN_PARTICIPATION_WARNING_MS));
      ble.sendData(warning);
    }

    if (lowDuration < TRAIN_PARTICIPATION_FAILURE_MS) {
      return false;
    }

    char failed[128];
    snprintf(failed, sizeof(failed),
             "{\"event\":\"participation\",\"state\":\"failed\","
             "\"sid\":%lu,\"mask\":%u,\"level\":%u}",
             static_cast<unsigned long>(trainingSessionId), trainingMask,
             getMaximumAssistLevel(trainingMask));
    ble.sendData(failed);
    return true;
  }

  participationLowPending = false;
  participationLowStartTime = 0;
  if (!participationWarningActive) {
    participationRecoveryStartTime = 0;
    return false;
  }

  if (activation < trainingParticipationRequired +
                       TRAIN_PARTICIPATION_RECOVERY_MARGIN) {
    participationRecoveryStartTime = 0;
    return false;
  }
  if (participationRecoveryStartTime == 0) {
    participationRecoveryStartTime = nowMs;
    return false;
  }
  if (static_cast<uint32_t>(nowMs - participationRecoveryStartTime) <
      TRAIN_PARTICIPATION_RECOVERY_HOLD_MS) {
    return false;
  }

  char recovered[112];
  snprintf(recovered, sizeof(recovered),
           "{\"event\":\"participation\",\"state\":\"recovered\","
           "\"sid\":%lu,\"mask\":%u}",
           static_cast<unsigned long>(trainingSessionId), trainingMask);
  ble.sendData(recovered);
  resetParticipationEvidence();
  return false;
}

void RehabSystem::resetParticipationEvidence() {
  participationLowPending = false;
  participationWarningActive = false;
  participationLowStartTime = 0;
  participationRecoveryStartTime = 0;
}

float RehabSystem::calculateParticipationRequirement(
    uint8_t mask, float triggerReference) const {
  uint8_t level = getMaximumAssistLevel(mask);
  if (level < 1) {
    level = 1;
  }
  const float levelPosition = static_cast<float>(level - 1U) / 9.0f;
  const float ratio = TRAIN_PARTICIPATION_LEVEL_1_RATIO +
                      (TRAIN_PARTICIPATION_LEVEL_10_RATIO -
                       TRAIN_PARTICIPATION_LEVEL_1_RATIO) *
                          levelPosition;
  return constrain(fmaxf(TRAIN_PARTICIPATION_MIN_ACTIVATION,
                         triggerReference * ratio),
                   0.0f, 1.0f);
}

void RehabSystem::recordTrainingCurrentSafety(const char* phase) {
  if (trainingCurrentSafetyIncidentLatched) {
    return;
  }
  trainingCurrentSafetyIncidentLatched = true;
  if (sessionCurrentSafetyCount < UINT16_MAX) {
    ++sessionCurrentSafetyCount;
  }

  const float measured = constrain(currentSensor.getCurrentmA(),
                                   0.0f, 99999.9f);
  const float limit = constrain(calculateGroupSoftCurrentLimit(trainingMask),
                                0.0f, 99999.9f);
  if (CURRENT_DIAGNOSTICS_ENABLED) {
    Serial.printf("TRAIN_CURRENT,TRIGGERED,%u,%.1f,%.1f,%.1f\n",
                  trainingMask, currentSensor.getRawCurrentmA(), measured,
                  limit);
  }
  char message[208];
  snprintf(message, sizeof(message),
           "{\"event\":\"current_safety\",\"state\":\"triggered\","
           "\"sid\":%lu,\"mask\":%u,\"finger\":null,"
           "\"current_ma\":%.1f,\"limit_ma\":%.1f,"
           "\"scope\":\"group\",\"phase\":\"%s\"}",
           static_cast<unsigned long>(trainingSessionId), trainingMask,
           measured, limit, phase != nullptr ? phase : "unknown");
  ble.sendData(message);
}

void RehabSystem::finishTrainingSession(const char* reason) {
  if (!trainingSessionActive) {
    return;
  }

  trainingSessionActive = false;
  trainingTriggered = false;
  completedTrainingEndReason = reason != nullptr ? reason : "END";
  completedTrainingEndDeviceMs = millis();
  completedTrainingMask = trainingMask;
  sessionSummaryPending = true;
  pendingSummaryMetricsSequence = 0;
  lastSummaryTransmitTime = 0;
  if (ble.isConnected() && ble.hasRequiredMtu()) {
    sendPendingTrainingSessionSummary(completedTrainingEndDeviceMs);
  }
}

void RehabSystem::sendPendingTrainingSessionSummary(uint32_t nowMs) {
  if (!sessionSummaryPending || !ble.isConnected() ||
      !ble.hasRequiredMtu()) {
    return;
  }

  if (lastSummaryTransmitTime != 0 &&
      static_cast<uint32_t>(nowMs - lastSummaryTransmitTime) <
          BLE_SESSION_SUMMARY_RETRY_MS) {
    return;
  }

  if (pendingSummaryMetricsSequence == 0) {
    ++sessionMetricsSequence;
    if (sessionMetricsSequence == 0) {
      sessionMetricsSequence = 1;
    }
    pendingSummaryMetricsSequence = sessionMetricsSequence;
  }

  char reasonMessage[176];
  snprintf(reasonMessage, sizeof(reasonMessage),
           "{\"event\":\"session_end\",\"v\":%u,\"boot\":%lu,"
           "\"sid\":%lu,\"q\":%lu,\"start_ms\":%lu,"
           "\"end_ms\":%lu,\"mask\":%u,"
           "\"reason\":\"%s\"}",
           BLE_PROTOCOL_VERSION,
           static_cast<unsigned long>(bootId),
           static_cast<unsigned long>(trainingSessionId),
           static_cast<unsigned long>(pendingSummaryMetricsSequence),
           static_cast<unsigned long>(trainingStartDeviceMs),
           static_cast<unsigned long>(completedTrainingEndDeviceMs),
           completedTrainingMask,
           completedTrainingEndReason != nullptr
               ? completedTrainingEndReason
               : "END");
  ble.sendData(reasonMessage);

  sendTrainingSessionMetrics(pendingSummaryMetricsSequence);
  lastSummaryTransmitTime = nowMs;
}

void RehabSystem::sendTrainingSessionMetrics(uint32_t fixedSequence) {
  uint32_t metricsSequence = fixedSequence;
  if (metricsSequence == 0) {
    ++sessionMetricsSequence;
    if (sessionMetricsSequence == 0) {
      sessionMetricsSequence = 1;
    }
    metricsSequence = sessionMetricsSequence;
  }

  const float averageAngle =
      sessionAngleSampleCount > 0
          ? static_cast<float>(
                sessionAngleSum /
                static_cast<double>(sessionAngleSampleCount))
          : 0.0f;
  const float successRate =
      sessionTotalAttempts > 0
          ? (100.0f * static_cast<float>(sessionSuccessCount) /
             static_cast<float>(sessionTotalAttempts))
          : 0.0f;

  const unsigned long maxExtensorRms = static_cast<unsigned long>(
      lroundf(fmaxf(0.0f, sessionMaxExtensorRms)));
  const unsigned long maxFlexorRms = static_cast<unsigned long>(
      lroundf(fmaxf(0.0f, sessionMaxFlexorRms)));

  // Keep each JSON notification below the 244-byte ATT payload. The app
  // merges both parts by sid/q and acknowledges only after both are stored.
  char outcome[245];
  snprintf(outcome, sizeof(outcome),
           "{\"sid\":%lu,\"q\":%lu,\"part\":\"outcome\","
           "\"training_log\":{" 
           "\"total_attempts\":%u,\"success_count\":%u,"
           "\"participation_failure_count\":%u,"
           "\"average_angle\":%.1f,\"success_rate\":%.1f}}",
           static_cast<unsigned long>(trainingSessionId),
           static_cast<unsigned long>(metricsSequence),
           sessionTotalAttempts, sessionSuccessCount,
           sessionParticipationFailureCount, fmaxf(0.0f, averageAngle),
           fmaxf(0.0f, successRate));
  ble.sendData(outcome);

  char safetyEmg[245];
  snprintf(safetyEmg, sizeof(safetyEmg),
           "{\"sid\":%lu,\"q\":%lu,\"part\":\"safety_emg\","
           "\"training_log\":{" 
           "\"cocontraction_cnt\":%u,\"current_safety_count\":%u,"
           "\"max_extensor_rms\":%lu,\"max_flexor_rms\":%lu}}",
           static_cast<unsigned long>(trainingSessionId),
           static_cast<unsigned long>(metricsSequence),
           sessionCocontractionCnt, sessionCurrentSafetyCount,
           maxExtensorRms, maxFlexorRms);
  ble.sendData(safetyEmg);
}

bool RehabSystem::relieveTrainingGroup() {
  servo.stopMotion();
  for (uint8_t i = 0; i < FINGER_COUNT; ++i) {
    if ((trainingMask & (1U << i)) == 0) {
      continue;
    }
    const int currentAngle = servo.getAngle(i);
    const int homeAngle = servo.getHomeAngleForFinger(i);
    const int reliefAngle =
        max(homeAngle,
            currentAngle - static_cast<int>(ROM_SAFETY_MARGIN_DEG));
    if (!servo.setAngle(i, reliefAngle)) {
      return false;
    }
  }
  return true;
}

uint8_t RehabSystem::getIncompleteTrainingMask() const {
  uint8_t incompleteMask = 0;
  for (uint8_t i = 0; i < FINGER_COUNT; ++i) {
    const uint8_t bit = static_cast<uint8_t>(1U << i);
    if ((trainingMask & bit) != 0 &&
        servo.getAngle(i) < fingerProfiles[i].targetAngle) {
      incompleteMask |= bit;
    }
  }
  return incompleteMask;
}

float RehabSystem::calculateGroupSoftCurrentLimit(uint8_t mask) const {
  mask &= ALL_FINGERS_MASK;
  uint8_t fingerCount = 0;
  for (uint8_t i = 0; i < FINGER_COUNT; ++i) {
    if ((mask & (1U << i)) != 0) {
      ++fingerCount;
    }
  }

  if (fingerCount == 0) {
    return STALL_CURRENT_THRES_MA;
  }
  const float scaledLimit =
      STALL_CURRENT_THRES_MA +
      static_cast<float>(fingerCount - 1U) *
          TRAIN_ADDITIONAL_FINGER_CURRENT_ALLOWANCE_MA;
  return fminf(scaledLimit,
               HARD_CURRENT_THRES_MA - TRAIN_HARD_CURRENT_MARGIN_MA);
}

int RehabSystem::calculateAverageTargetAngle(uint8_t mask) const {
  mask &= calibratedFingerMask;
  int targetSum = 0;
  uint8_t fingerCount = 0;
  for (uint8_t i = 0; i < FINGER_COUNT; ++i) {
    if ((mask & (1U << i)) != 0) {
      targetSum += fingerProfiles[i].targetAngle;
      ++fingerCount;
    }
  }
  return fingerCount > 0 ? targetSum / fingerCount
                         : SERVO_LOGICAL_HOME_ANGLE;
}

uint8_t RehabSystem::getMaximumAssistLevel(uint8_t mask) const {
  mask &= calibratedFingerMask;
  uint8_t maximumLevel = 0;
  for (uint8_t i = 0; i < FINGER_COUNT; ++i) {
    if ((mask & (1U << i)) != 0 &&
        fingerProfiles[i].assistLevel > maximumLevel) {
      maximumLevel = fingerProfiles[i].assistLevel;
    }
  }
  return maximumLevel;
}

void RehabSystem::triggerSafetyReturn(const char* reason,
                                      SystemState destination,
                                      bool clearSelection,
                                      uint32_t nowMs) {
  if (currentState == SystemState::FAULT) {
    return;
  }

  // Safety commands are commonly retried by an app. Do not restart the
  // quintic return trajectory on each retry; doing so could hold it near zero
  // velocity indefinitely. Only upgrade the destination to the safer IDLE
  // state and apply any stronger selection clearing request.
  if (currentState == SystemState::SAFETY_RETURNING) {
    if (destination == SystemState::IDLE) {
      safetyReturnDestination = SystemState::IDLE;
    }
    if (clearSelection) {
      selectedFingerMask = 0;
    }

    char response[96];
    snprintf(response, sizeof(response),
             "{\"event\":\"safety_return\",\"reason\":\"%s\"}",
             reason != nullptr ? reason : "STOP");
    ble.sendData(response);
    return;
  }

  finishTrainingSession(reason);
  trainingTriggered = false;
  servo.stopMotion();
  if (!servo.startReturn(ALL_FINGERS_MASK, SERVO_RETURN_DURATION_MS, nowMs)) {
    enterFault("RETURN_START");
    return;
  }
  safetyReturnDestination = destination;
  currentState = SystemState::SAFETY_RETURNING;
  phaseStartTime = nowMs;
  trainingMask = 0;
  if (clearSelection) {
    selectedFingerMask = 0;
  }
  resetStallEvidence();
  resetActivationEvidence();

  char response[96];
  snprintf(response, sizeof(response),
           "{\"event\":\"safety_return\",\"reason\":\"%s\"}",
           reason != nullptr ? reason : "STOP");
  ble.sendData(response);
}

void RehabSystem::processSafetyReturn(uint32_t nowMs) {
  if (static_cast<uint32_t>(nowMs - phaseStartTime) >=
      SERVO_RETURN_TIMEOUT_MS) {
    enterFault("RET_TIMEOUT");
    return;
  }
  const StallStatus stall =
      updateStallStatus(
          nowMs, RETURN_STALL_CONFIRM_TIME_MS,
          calculateGroupSoftCurrentLimit(servo.getEnabledMask()));
  if (stall == StallStatus::CONFIRMED) {
    enterFault("RETURN_STALL");
    return;
  }

  if (!isReturnSettled(nowMs, stall)) {
    return;
  }

  if (!servo.allAtHome(ALL_FINGERS_MASK)) {
    enterFault("RET_POS");
    return;
  }
  if (!servo.setEnabledMask(0)) {
    enterFault("SERVO_DISABLE");
    return;
  }

  currentState = safetyReturnDestination;
  trainingPhase = TrainingPhase::WAITING_RELEASE;
  trainingReturnCause = TrainingReturnCause::ATTEMPT;
  trainingGoalReached = false;
  trainingTriggered = false;
  trainingAttachMask = 0;
  stallConfirmationHighSamples = 0;
  stallConfirmationClearSamples = 0;
  resetStallEvidence();
  resetActivationEvidence();
  resetParticipationEvidence();
  emgSensor.resetSignalWindow();
  if (currentState == SystemState::TRAINING_READY) {
    sendJsonState("READY");
  } else {
    sendJsonState("IDLE");
  }
}

void RehabSystem::enterFault(const char* reason) {
  if (currentState == SystemState::FAULT) {
    return;
  }

  finishTrainingSession(reason);
  servo.detachAll();
  hardwareReady = false;
  selectedFingerMask = 0;
  trainingMask = 0;
  trainingTriggered = false;
  trainingGoalReached = false;
  trainingAttachMask = 0;
  stallConfirmationHighSamples = 0;
  stallConfirmationClearSamples = 0;
  trainingReturnCause = TrainingReturnCause::ATTEMPT;
  currentState = SystemState::FAULT;
  resetStallEvidence();
  resetActivationEvidence();
  resetParticipationEvidence();

  char response[96];
  snprintf(response, sizeof(response),
           "{\"event\":\"fault\",\"reason\":\"%s\"}",
           reason != nullptr ? reason : "UNKNOWN");
  ble.sendData(response);
  Serial.println(response);
}

RehabSystem::StallStatus RehabSystem::updateStallStatus(
    uint32_t nowMs, uint32_t confirmTimeMs, float thresholdMa) {
  if (!currentSensor.isReady() || !isfinite(thresholdMa) ||
      thresholdMa <= 0.0f || thresholdMa >= HARD_CURRENT_THRES_MA) {
    return StallStatus::CONFIRMED;
  }

  if (currentSensor.getCurrentmA() < thresholdMa) {
    stallPending = false;
    return StallStatus::CLEAR;
  }

  if (!stallPending) {
    stallPending = true;
    stallStartTime = nowMs;
    return StallStatus::PENDING;
  }

  if (static_cast<uint32_t>(nowMs - stallStartTime) >= confirmTimeMs) {
    return StallStatus::CONFIRMED;
  }
  return StallStatus::PENDING;
}

void RehabSystem::resetStallEvidence() {
  stallPending = false;
  stallStartTime = 0;
  homeSettlePending = false;
  homeSettleStartTime = 0;
}

bool RehabSystem::isReturnSettled(uint32_t nowMs, StallStatus stall) {
  if (servo.isBusy() || stall != StallStatus::CLEAR) {
    homeSettlePending = false;
    homeSettleStartTime = 0;
    return false;
  }

  if (!homeSettlePending) {
    homeSettlePending = true;
    homeSettleStartTime = nowMs;
    return false;
  }

  return static_cast<uint32_t>(nowMs - homeSettleStartTime) >=
         SERVO_HOME_SETTLE_MS;
}

bool RehabSystem::isCocontractionConfirmed(uint32_t nowMs,
                                            bool newEmgSample) {
  if (!newEmgSample) {
    return false;
  }

  if (!emgSensor.isWindowReady()) {
    cocontractionPending = false;
    return false;
  }

  const bool simultaneousActivation =
      emgSensor.getExtensorActivation() >= COCONTRACTION_EXTENSOR_LEVEL &&
      emgSensor.getFlexorActivation() >= COCONTRACTION_FLEXOR_LEVEL;
  if (!simultaneousActivation) {
    cocontractionPending = false;
    return false;
  }

  if (!cocontractionPending) {
    cocontractionPending = true;
    cocontractionStartTime = nowMs;
    return false;
  }

  return static_cast<uint32_t>(nowMs - cocontractionStartTime) >=
         COCONTRACTION_HOLD_MS;
}

void RehabSystem::resetActivationEvidence() {
  triggerPending = false;
  triggerStartTime = 0;
  releasePending = false;
  releaseStartTime = 0;
  cocontractionPending = false;
  cocontractionStartTime = 0;
  lastEvidenceSampleTime = 0;
}

bool RehabSystem::refreshEmgEvidence(uint32_t nowMs, bool newEmgSample) {
  if (!emgSensor.isSamplingHealthy(nowMs)) {
    resetActivationEvidence();
    return true;
  }

  if (!newEmgSample) {
    return false;
  }

  const bool gapDetected =
      lastEvidenceSampleTime != 0 &&
      static_cast<uint32_t>(nowMs - lastEvidenceSampleTime) >
          EMG_EVIDENCE_MAX_GAP_MS;
  if (gapDetected) {
    resetActivationEvidence();
    return true;
  }
  lastEvidenceSampleTime = nowMs;
  return false;
}

void RehabSystem::resetFingerProfiles() {
  calibratedFingerMask = 0;
  for (uint8_t i = 0; i < FINGER_COUNT; ++i) {
    fingerProfiles[i].targetAngle = servo.getHomeAngleForFinger(i);
    fingerProfiles[i].meanExtensorActivation = 0.0f;
    fingerProfiles[i].homeCurrentBaselineMa = 0.0f;
    fingerProfiles[i].currentNoiseMa = 0.0f;
    fingerProfiles[i].currentTripThresholdMa = STALL_CURRENT_THRES_MA;
    fingerProfiles[i].assistLevel = 1;
    fingerProfiles[i].failuresAtLevel = 0;
    fingerProfiles[i].calibrated = false;
    fingerProfiles[i].physicalLimitDetected = false;
    fingerProfiles[i].currentBaselineValid = false;
    servo.setMaxAngleForFinger(i,
                               servo.getSoftwareEndAngleForFinger(i));
  }
}

bool RehabSystem::allSelectedFingersCalibrated(uint8_t mask) const {
  mask &= ALL_FINGERS_MASK;
  return mask != 0 && (mask & static_cast<uint8_t>(~calibratedFingerMask)) == 0;
}

bool RehabSystem::isActuationState() const {
  return currentState == SystemState::CALIB_PASSIVE_RANGE ||
         currentState == SystemState::TRAINING_ACTIVE ||
         currentState == SystemState::SAFETY_RETURNING;
}

bool RehabSystem::isCalibrationState() const {
  return currentState == SystemState::CALIB_REST ||
         currentState == SystemState::CALIB_EXTENSOR_MAX ||
         currentState == SystemState::CALIB_FLEXOR_MAX ||
         currentState == SystemState::CALIB_PASSIVE_RANGE;
}

bool RehabSystem::parseFingerMask(const String& value, uint8_t& mask) const {
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

  mask = static_cast<uint8_t>(parsed);
  return true;
}

void RehabSystem::sendRealtimeTelemetry(uint32_t nowMs) {
  if (!ble.isConnected() || !ble.hasRequiredMtu() ||
      (lastTelemetryTime != 0 &&
       static_cast<uint32_t>(nowMs - lastTelemetryTime) <
           TELEMETRY_INTERVAL_MS)) {
    return;
  }
  lastTelemetryTime = nowMs;
  ++telemetrySequence;
  if (telemetrySequence == 0) {
    telemetrySequence = 1;
  }
  const unsigned long sequence =
      static_cast<unsigned long>(telemetrySequence);

  const uint8_t controlMask =
      trainingSessionActive ? trainingMask : selectedFingerMask;
  const uint8_t enabledMask = servo.getEnabledMask();
  const bool simultaneousActivation =
      emgSensor.isWindowReady() &&
      emgSensor.getExtensorActivation() >=
          COCONTRACTION_EXTENSOR_LEVEL &&
      emgSensor.getFlexorActivation() >=
          COCONTRACTION_FLEXOR_LEVEL;
  const bool stallDetected =
      (currentState == SystemState::TRAINING_ACTIVE &&
       trainingPhase == TrainingPhase::STALL_CONFIRMING) ||
      (currentSensor.isReady() && enabledMask != 0 &&
       currentSensor.getCurrentmA() >=
           calculateGroupSoftCurrentLimit(enabledMask));

  char batteryText[5] = "null";
  if (stableBatteryReadingValid) {
    snprintf(batteryText, sizeof(batteryText), "%u",
             static_cast<unsigned>(stableBatteryPercent));
  }

  char statusMessage[245];
  snprintf(statusMessage, sizeof(statusMessage),
           "{\"v\":%u,\"boot\":%lu,\"q\":%lu,\"device_ms\":%lu,"
           "\"status\":{\"mode\":\"%s\",\"is_triggered\":%s,"
           "\"is_cocontraction\":%s,\"is_participation_low\":%s,"
           "\"battery\":%s,"
           "\"stall_detected\":%s}}",
           BLE_PROTOCOL_VERSION, static_cast<unsigned long>(bootId), sequence,
           static_cast<unsigned long>(nowMs),
           trainingSessionActive ? "START" : "STOP",
           trainingTriggered ? "true" : "false",
           simultaneousActivation ? "true" : "false",
           participationWarningActive ? "true" : "false",
           batteryText,
           stallDetected ? "true" : "false");
  ble.sendData(statusMessage);

  char angleMessage[245];
  snprintf(angleMessage, sizeof(angleMessage),
           "{\"q\":%lu,\"angles\":{\"thumb\":%d,\"index\":%d,"
           "\"middle\":%d,\"ring\":%d,\"pinky\":%d},"
           "\"target_angles\":{\"thumb\":%d,\"index\":%d,"
           "\"middle\":%d,\"ring\":%d,\"pinky\":%d}}",
           sequence,
           servo.getAngle(0), servo.getAngle(1), servo.getAngle(2),
           servo.getAngle(3), servo.getAngle(4),
           fingerProfiles[0].targetAngle, fingerProfiles[1].targetAngle,
           fingerProfiles[2].targetAngle, fingerProfiles[3].targetAngle,
           fingerProfiles[4].targetAngle);
  ble.sendData(angleMessage);

  const unsigned long extensorRms = static_cast<unsigned long>(
      lroundf(fmaxf(0.0f, emgSensor.getExtensorRMS())));
  const unsigned long flexorRms = static_cast<unsigned long>(
      lroundf(fmaxf(0.0f, emgSensor.getFlexorRMS())));
  char emgMessage[88];
  snprintf(emgMessage, sizeof(emgMessage),
           "{\"q\":%lu,\"emg\":{\"ch1\":%lu,\"ch2\":%lu}}",
           sequence, extensorRms, flexorRms);
  ble.sendData(emgMessage);

  const unsigned long extensorThreshold = static_cast<unsigned long>(
      lroundf(fmaxf(0.0f, emgSensor.getExtensorThreshold())));
  const unsigned long flexorThreshold = static_cast<unsigned long>(
      lroundf(fmaxf(0.0f, emgSensor.getFlexorThreshold())));
  const unsigned long extensorMvc = static_cast<unsigned long>(
      lroundf(fmaxf(0.0f, emgSensor.getExtensorMVC())));
  const unsigned long flexorMvc = static_cast<unsigned long>(
      lroundf(fmaxf(0.0f, emgSensor.getFlexorMVC())));
  char controlMessage[245];
  snprintf(controlMessage, sizeof(controlMessage),
           "{\"q\":%lu,\"control\":{\"mode\":\"%s\","
           "\"target_angle\":%d,\"hold_time_sec\":%lu,"
           "\"motor_level\":%u,\"threshold_extensor\":%lu,"
           "\"threshold_flexor\":%lu,\"mvc_extensor\":%lu,"
           "\"mvc_flexor\":%lu,\"participation_required\":%.3f}}",
           sequence,
           trainingSessionActive ? "START" : "STOP",
           calculateAverageTargetAngle(controlMask),
           static_cast<unsigned long>(TRAIN_HOLD_TIME_MS / 1000UL),
           getMaximumAssistLevel(controlMask),
           extensorThreshold, flexorThreshold, extensorMvc, flexorMvc,
           trainingSessionActive ? trainingParticipationRequired : 0.0f);
  ble.sendData(controlMessage);

  const uint8_t finger = telemetryFingerIndex;
  telemetryFingerIndex =
      static_cast<uint8_t>((telemetryFingerIndex + 1U) % FINGER_COUNT);
  const FingerProfile& profile = fingerProfiles[finger];
  char profileMessage[224];
  snprintf(profileMessage, sizeof(profileMessage),
           "{\"q\":%lu,\"profile\":{\"finger\":%u,"
           "\"target_angle\":%d,\"motor_level\":%u,"
           "\"failure_stack\":%u,\"calibrated\":%s,"
           "\"current_baseline_ma\":%.1f,"
           "\"current_trip_ma\":%.1f}}",
           sequence, finger, profile.targetAngle,
           profile.assistLevel, profile.failuresAtLevel,
           profile.calibrated ? "true" : "false",
           profile.homeCurrentBaselineMa,
           profile.currentTripThresholdMa);
  ble.sendData(profileMessage);

  // The app caches this live log. The final session_end plus both log parts
  // are retried until SES_ACK; this live cache remains the power-loss fallback.
  if (trainingSessionActive) {
    sendTrainingSessionMetrics();
  }
}

void RehabSystem::sortFloatSamples(float* values, uint16_t count) {
  if (values == nullptr || count < 2) {
    return;
  }

  // Calibration runs with all motors stationary. Insertion sort avoids heap
  // allocation and is fast enough for the fixed 500-feature buffer.
  for (uint16_t i = 1; i < count; ++i) {
    const float value = values[i];
    uint16_t position = i;
    while (position > 0 && values[position - 1] > value) {
      values[position] = values[position - 1];
      --position;
    }
    values[position] = value;
  }
}

float RehabSystem::medianOfSorted(const float* values, uint16_t count) {
  if (values == nullptr || count == 0) {
    return NAN;
  }
  const uint16_t middle = count / 2U;
  if ((count & 1U) != 0U) {
    return values[middle];
  }
  return 0.5f * (values[middle - 1U] + values[middle]);
}

float RehabSystem::percentileOfSorted(const float* values, uint16_t count,
                                      float percentile) {
  if (values == nullptr || count == 0 || !isfinite(percentile)) {
    return NAN;
  }
  const float bounded = constrain(percentile, 0.0f, 1.0f);
  const float position = bounded * static_cast<float>(count - 1U);
  const uint16_t lower = static_cast<uint16_t>(floorf(position));
  const uint16_t upper =
      lower + 1U < count ? static_cast<uint16_t>(lower + 1U) : lower;
  const float fraction = position - static_cast<float>(lower);
  return values[lower] + fraction * (values[upper] - values[lower]);
}

float RehabSystem::calculateRestThreshold(float* values, uint16_t count) {
  if (values == nullptr || count == 0) {
    return NAN;
  }

  sortFloatSamples(values, count);
  const float median = medianOfSorted(values, count);
  const float noiseCeiling =
      percentileOfSorted(values, count, EMG_REST_NOISE_PERCENTILE);
  for (uint16_t i = 0; i < count; ++i) {
    values[i] = fabsf(values[i] - median);
  }
  sortFloatSamples(values, count);
  const float mad = medianOfSorted(values, count);
  const float robustLimit =
      median + EMG_REST_ROBUST_SIGMA_MULTIPLIER *
                   EMG_MAD_TO_SIGMA_SCALE * mad;
  return fmaxf(noiseCeiling, robustLimit);
}

float RehabSystem::calculateRobustMvc(float* values, uint16_t count) {
  if (values == nullptr || count == 0) {
    return NAN;
  }

  sortFloatSamples(values, count);
  uint16_t topCount = static_cast<uint16_t>(
      ceilf(static_cast<float>(count) * EMG_MVC_TOP_FRACTION));
  if (topCount == 0) {
    topCount = 1;
  }
  const uint16_t firstTop = static_cast<uint16_t>(count - topCount);
  return medianOfSorted(values + firstTop, topCount);
}

float RehabSystem::clampUnit(float value) {
  if (value <= 0.0f) {
    return 0.0f;
  }
  if (value >= 1.0f) {
    return 1.0f;
  }
  return value;
}

float RehabSystem::smoothStep(float value) {
  const float bounded = clampUnit(value);
  return bounded * bounded * (3.0f - (2.0f * bounded));
}
