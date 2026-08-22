#pragma once
#include "Config.h"
#include "ServoManager.h"
#include "CurrentSensor.h"
#include "EmgSensor.h"
#include "BleManager.h"

class RehabSystem {
private:
  ServoManager servo;
  CurrentSensor currentSensor;
  EmgSensor emgSensor;
  BleManager ble;

  SystemState currentState;
  uint32_t stateStartTime;
  uint8_t currentLevel; // 훈련 레벨 (1~10)

  // 캘리브레이션 임시 누적용 변수
  float sumExtensor;
  float sumFlexor;
  uint32_t sampleCount;

public:
  RehabSystem();
  void begin();
  void update(); // 메인 루프에서 지속 호출

private:
  void handleBleCommand(String cmd);
  void processCalibration();
  void processTraining();
  void triggerSafetyReturn(String reason);
};