#include "CurrentSensor.h"
#include <math.h>

CurrentSensor::CurrentSensor()
    : isInitialized(false),
      hasValidSample(false),
      rawCurrentmA(0.0f),
      filteredCurrentmA(0.0f),
      currentStallThreshold(STALL_CURRENT_THRES_MA),
      lastSampleTimeMs(0) {}

bool CurrentSensor::begin() {
  isInitialized = false;
  hasValidSample = false;
  rawCurrentmA = 0.0f;
  filteredCurrentmA = 0.0f;
  lastSampleTimeMs = 0;

  const bool invalidConfiguration =
      CURRENT_SAMPLE_INTERVAL_MS == 0 ||
      I2C_FREQUENCY_HZ == 0 ||
      !isfinite(CURRENT_FILTER_ALPHA) ||
      CURRENT_FILTER_ALPHA <= 0.0f ||
      CURRENT_FILTER_ALPHA > 1.0f ||
      !isfinite(currentStallThreshold) ||
      currentStallThreshold <= 0.0f ||
      !isfinite(HARD_CURRENT_THRES_MA) ||
      HARD_CURRENT_THRES_MA <= currentStallThreshold;

  if (invalidConfiguration) {
    Serial.println("[CURRENT] Invalid current sensor configuration");
    return false;
  }

  if (!Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN, I2C_FREQUENCY_HZ)) {
    Serial.println("[CURRENT] Failed to initialize I2C bus");
    return false;
  }

  if (!ina219.begin(&Wire)) {
    Serial.println("[CURRENT] INA219 not found");
    return false;
  }

  ina219.setCalibration_32V_2A();
  isInitialized = true;
  Serial.println("[CURRENT] INA219 initialized");
  return true;
}

bool CurrentSensor::update(uint32_t nowMs) {
  if (!isInitialized) {
    return false;
  }

  if (hasValidSample &&
      static_cast<uint32_t>(nowMs - lastSampleTimeMs) < CURRENT_SAMPLE_INTERVAL_MS) {
    return false;
  }

  lastSampleTimeMs = nowMs;
  const float measuredCurrentmA = fabsf(ina219.getCurrent_mA());

  if (!isfinite(measuredCurrentmA)) {
    Serial.println("[CURRENT] Invalid INA219 sample");
    isInitialized = false;
    hasValidSample = false;
    rawCurrentmA = 0.0f;
    filteredCurrentmA = 0.0f;
    return false;
  }

  rawCurrentmA = measuredCurrentmA;

  if (!hasValidSample) {
    filteredCurrentmA = measuredCurrentmA;
    hasValidSample = true;
  } else {
    filteredCurrentmA =
        (CURRENT_FILTER_ALPHA * measuredCurrentmA) +
        ((1.0f - CURRENT_FILTER_ALPHA) * filteredCurrentmA);
  }

  return true;
}

float CurrentSensor::getCurrentmA() const {
  return filteredCurrentmA;
}

float CurrentSensor::getRawCurrentmA() const {
  return rawCurrentmA;
}

bool CurrentSensor::isStallDetected() const {
  return !isReady() || filteredCurrentmA >= currentStallThreshold;
}

bool CurrentSensor::isHardOverCurrent() const {
  return !isReady() || rawCurrentmA >= HARD_CURRENT_THRES_MA;
}

bool CurrentSensor::isReady() const {
  return isInitialized && hasValidSample;
}

bool CurrentSensor::setStallThreshold(float thresholdmA) {
  if (!isfinite(thresholdmA) ||
      thresholdmA <= 0.0f ||
      !isfinite(HARD_CURRENT_THRES_MA) ||
      thresholdmA >= HARD_CURRENT_THRES_MA) {
    return false;
  }

  currentStallThreshold = thresholdmA;
  return true;
}
