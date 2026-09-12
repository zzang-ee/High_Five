#pragma once

#include "BleManager.h"
#include "Config.h"
#include "CurrentSensor.h"
#include "EmgSensor.h"
#include "ServoManager.h"

class RehabSystem {
public:
  RehabSystem();

  void begin();
  void update();

private:
  struct FingerProfile {
    int targetAngle;
    float meanExtensorActivation;
    float homeCurrentBaselineMa;
    float currentNoiseMa;
    float currentTripThresholdMa;
    uint8_t assistLevel;
    uint8_t failuresAtLevel;
    bool calibrated;
    bool physicalLimitDetected;
    bool currentBaselineValid;
  };

  enum class RomPhase : uint8_t {
    PREPARING,
    MOVING,
    RETURNING
  };

  enum class TrainingPhase : uint8_t {
    WAITING_RELEASE,
    WAITING_TRIGGER,
    PREPARING,
    MOVING,
    STALL_CONFIRMING,
    HOLDING,
    RETURNING
  };

  enum class StallStatus : uint8_t {
    CLEAR,
    PENDING,
    CONFIRMED
  };

  enum class TrainingReturnCause : uint8_t {
    ATTEMPT,
    CURRENT_SAFETY,
    USER_STOP
  };

  ServoManager servo;
  CurrentSensor currentSensor;
  EmgSensor emgSensor;
  BleManager ble;

  SystemState currentState;
  SystemState safetyReturnDestination;
  bool hardwareReady;
  bool previousBleConnected;

  FingerProfile fingerProfiles[FINGER_COUNT];
  uint8_t selectedFingerMask;
  uint8_t calibratedFingerMask;
  uint8_t trainingMask;

  // EMG baseline/MVC calibration state.
  uint32_t stateStartTime;
  bool calibrationCollecting;
  float calibrationExtensorSamples[EMG_CALIBRATION_FEATURE_COUNT];
  float calibrationFlexorSamples[EMG_CALIBRATION_FEATURE_COUNT];
  uint16_t sampleCount;

  // Per-finger ROM calibration state.
  RomPhase romPhase;
  uint8_t calibFingerIndex;
  uint32_t lastStepTime;
  float romActivationSum;
  uint32_t romActivationSamples;
  bool romRetryCurrentFinger;
  float romCurrentBaselineSamples[ROM_CURRENT_BASELINE_MAX_SAMPLES];
  uint16_t romCurrentBaselineSampleCount;
  uint8_t romLoadRiseHighSamples;
  float romStepPeakCurrentMa;

  // Training sub-state.
  TrainingPhase trainingPhase;
  TrainingReturnCause trainingReturnCause;
  bool trainingTriggered;
  bool trainingSessionActive;
  bool trainingGoalReached;
  bool trainingCurrentSafetyIncidentLatched;
  bool lastAttemptSuccess;
  uint32_t phaseStartTime;
  uint8_t trainingAttachMask;
  uint32_t lastTrainingAttachTime;
  uint32_t trainingAttachCompleteTime;
  uint8_t stallConfirmationHighSamples;
  uint8_t stallConfirmationClearSamples;
  uint8_t trainingResumeCount;
  float trainingTriggerActivationReference;
  float trainingParticipationRequired;
  bool participationLowPending;
  bool participationWarningActive;
  uint32_t participationLowStartTime;
  uint32_t participationRecoveryStartTime;

  // Time-qualified evidence and current protection.
  bool stallPending;
  uint32_t stallStartTime;
  bool triggerPending;
  uint32_t triggerStartTime;
  bool releasePending;
  uint32_t releaseStartTime;
  bool cocontractionPending;
  uint32_t cocontractionStartTime;
  uint32_t lastEvidenceSampleTime;
  bool homeSettlePending;
  uint32_t homeSettleStartTime;

  // Session counters and rotating telemetry.
  uint16_t sessionTotalAttempts;
  uint16_t sessionSuccessCount;
  uint16_t sessionParticipationFailureCount;
  uint16_t sessionCocontractionCnt;
  uint16_t sessionCurrentSafetyCount;
  double sessionAngleSum;
  uint32_t sessionAngleSampleCount;
  float sessionMaxExtensorRms;
  float sessionMaxFlexorRms;
  uint32_t bootId;
  uint32_t trainingSessionId;
  uint32_t trainingStartDeviceMs;
  uint32_t completedTrainingEndDeviceMs;
  uint8_t completedTrainingMask;
  uint32_t sessionMetricsSequence;
  uint32_t pendingSummaryMetricsSequence;
  uint32_t lastSummaryTransmitTime;
  uint32_t telemetrySequence;
  bool sessionSummaryPending;
  const char* completedTrainingEndReason;
  uint32_t lastTelemetryTime;
  uint8_t telemetryFingerIndex;
  uint32_t lastControlLoopTime;
  uint8_t stableBatteryPercent;
  bool stableBatteryReadingValid;

  void handleBleCommand(String command, uint32_t nowMs);
  void handleSafetyCommand(BleSafetyCommand command, uint32_t nowMs);

  void startEmgCalibrationStage(SystemState stage, uint32_t nowMs);
  void retryEmgCalibrationStage(const char* reason, uint32_t nowMs);
  void processEmgCalibration(uint32_t nowMs, bool newEmgSample,
                             bool emgDataFault);
  void startFingerCalibration(uint8_t finger, uint32_t nowMs);
  void processFingerCalibration(uint32_t nowMs, bool newEmgSample,
                                bool emgDataFault, bool newCurrentSample);
  void retryFingerCalibration(const char* reason, uint32_t nowMs);
  void finishFingerExtension(bool physicalLimitDetected, uint32_t nowMs);
  bool finalizeFingerCurrentBaseline(uint8_t finger);
  float getRomFingerCurrentRiseLimit(uint8_t finger) const;
  void finalizeAssistanceProfiles();

  void startTrainingSession(uint32_t nowMs);
  void processTraining(uint32_t nowMs, bool newEmgSample,
                       bool emgDataFault, bool newCurrentSample);
  bool startTrainingGroup(uint32_t nowMs);
  bool startTrainingTrajectory(uint32_t nowMs, bool resumed);
  void beginTrainingStallConfirmation(uint32_t nowMs);
  void beginTrainingReturn(
      uint32_t nowMs, bool applyImmediateRelief,
      TrainingReturnCause cause = TrainingReturnCause::ATTEMPT);
  void recordTrainingAttempt(uint8_t fingerMask, bool success);
  bool processTrainingParticipation(uint32_t nowMs, bool newEmgSample);
  void resetParticipationEvidence();
  float calculateParticipationRequirement(uint8_t mask,
                                          float triggerReference) const;
  void recordTrainingCurrentSafety(const char* phase);
  void finishTrainingSession(const char* reason);
  void sendTrainingSessionMetrics(uint32_t fixedSequence = 0);
  void sendPendingTrainingSessionSummary(uint32_t nowMs);
  bool relieveTrainingGroup();
  uint8_t getIncompleteTrainingMask() const;
  float calculateGroupSoftCurrentLimit(uint8_t mask) const;
  int calculateAverageTargetAngle(uint8_t mask) const;
  uint8_t getMaximumAssistLevel(uint8_t mask) const;

  void triggerSafetyReturn(const char* reason, SystemState destination,
                           bool clearSelection, uint32_t nowMs);
  void processSafetyReturn(uint32_t nowMs);
  void enterFault(const char* reason);

  StallStatus updateStallStatus(
      uint32_t nowMs, uint32_t confirmTimeMs = STALL_CONFIRM_TIME_MS,
      float thresholdMa = STALL_CURRENT_THRES_MA);
  void resetStallEvidence();
  bool isReturnSettled(uint32_t nowMs, StallStatus stall);
  bool isCocontractionConfirmed(uint32_t nowMs, bool newEmgSample);
  void resetActivationEvidence();
  bool refreshEmgEvidence(uint32_t nowMs, bool newEmgSample);

  void resetFingerProfiles();
  bool allSelectedFingersCalibrated(uint8_t mask) const;
  bool isActuationState() const;
  bool isCalibrationState() const;
  bool parseFingerMask(const String& value, uint8_t& mask) const;
  void sendJsonError(const char* code);
  void sendJsonState(const char* state);
  void sendCalibrationJson(const char* stage, const char* state);
  void sendRealtimeTelemetry(uint32_t nowMs);

  static float clampUnit(float value);
  static float smoothStep(float value);
  static void sortFloatSamples(float* values, uint16_t count);
  static float medianOfSorted(const float* values, uint16_t count);
  static float percentileOfSorted(const float* values, uint16_t count,
                                  float percentile);
  static float calculateRestThreshold(float* values, uint16_t count);
  static float calculateRobustMvc(float* values, uint16_t count);
};
