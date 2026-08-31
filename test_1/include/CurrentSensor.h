#pragma once
#include <Wire.h>
#include <Adafruit_INA219.h>
#include "Config.h"

class CurrentSensor {
private:
  Adafruit_INA219 ina219;
  bool isInitialized;
  bool hasValidSample;
  float rawCurrentmA;
  float filteredCurrentmA;
  float currentStallThreshold;
  uint32_t lastSampleTimeMs;

public:
  CurrentSensor();

  // 지정된 I2C 버스에서 INA219를 초기화한다.
  bool begin();

  // 샘플 주기가 지난 경우 INA219를 한 번 읽어 EMA를 갱신한다.
  // 새 유효 샘플이 반영된 경우에만 true를 반환한다.
  bool update(uint32_t nowMs);

  // 마지막 EMA 필터 전류 (mA)
  float getCurrentmA() const;
  float getRawCurrentmA() const;

  // 센서가 준비되지 않은 경우에도 fail-closed로 true를 반환한다.
  bool isStallDetected() const;
  bool isHardOverCurrent() const;
  bool isReady() const;

  // 유한한 양수이며 hard limit보다 작은 값만 허용한다.
  bool setStallThreshold(float thresholdmA);
};
