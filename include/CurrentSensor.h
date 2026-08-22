#pragma once
#include <Wire.h>
#include <Adafruit_INA219.h>
#include "Config.h"

class CurrentSensor {
private:
  Adafruit_INA219 ina219;
  bool isInitialized;
  float currentStallThreshold;

public:
  CurrentSensor();
  
  // INA219 초기화
  bool begin();
  
  // 실시간 전류 측정 (mA 단위)
  float getCurrentmA();
  
  // 스톨 전류 발생 여부 검사 (설정된 임계값 초과 시 true)
  bool isStallDetected();
  
  // 임계값 변경 (필요 시)
  void setStallThreshold(float thresholdmA);
};