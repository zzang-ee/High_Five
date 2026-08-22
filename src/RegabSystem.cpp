#include "RehabSystem.h"

RehabSystem::RehabSystem() {
  currentState = SystemState::IDLE;
  stateStartTime = 0;
  currentLevel = 1;
  sumExtensor = 0.0;
  sumFlexor = 0.0;
  sampleCount = 0;
}

void RehabSystem::begin() {
  Serial.begin(115200);
  
  servo.begin();
  currentSensor.begin();
  emgSensor.begin();
  ble.begin();

  Serial.println("[SYSTEM] 재활 장갑 시스템 초기화 완료");
}

void RehabSystem::update() {
  // 1. BLE 명령어 수신 처리
  if (ble.available()) {
    handleBleCommand(ble.readData());
  }

  // 2. 스톨 전류 비상 정지 상시 감지 (훈련/캘리브레이션 모드 작동 중일 때만)
  if (currentState == SystemState::TRAINING_ACTIVE || currentState == SystemState::CALIB_PASSIVE_RANGE) {
    if (currentSensor.isStallDetected()) {
      triggerSafetyReturn("STALL_CURRENT_DETECTED");
      return;
    }
  }

  // 3. 상태 머신 실행
  switch (currentState) {
    case SystemState::IDLE:
      // 대기 상태
      break;

    case SystemState::CALIB_REST:
    case SystemState::CALIB_EXTENSOR_MAX:
    case SystemState::CALIB_FLEXOR_MAX:
      processCalibration();
      break;

    case SystemState::TRAINING_ACTIVE:
      processTraining();
      break;

    case SystemState::TRAINING_RETURNING:
      servo.returnToInitialPosition();
      currentState = SystemState::IDLE;
      ble.sendData("STATE:IDLE");
      break;

    default:
      break;
  }
}

void RehabSystem::handleBleCommand(String cmd) {
  cmd.trim();

  if (cmd == "CALIB_START") {
    currentState = SystemState::CALIB_REST;
    stateStartTime = millis();
    sumExtensor = 0.0;
    sumFlexor = 0.0;
    sampleCount = 0;
    ble.sendData("CALIB:REST_START");
  } 
  else if (cmd.startsWith("LEVEL:")) {
    currentLevel = cmd.substring(6).toInt();
    if (currentLevel < 1) currentLevel = 1;
    if (currentLevel > 10) currentLevel = 10;
    ble.sendData("SET_LEVEL:" + String(currentLevel));
  }
  // [JSON control 연동] DB에서 가져온 threshold 및 mvc 주입 command
  // 예시: SET_PARAM:800.0,750.0,1200.0,1100.0
  else if (cmd.startsWith("SET_PARAM:")) {
    String params = cmd.substring(10);
    int p1 = params.indexOf(',');
    int p2 = params.indexOf(',', p1 + 1);
    int p3 = params.indexOf(',', p2 + 1);

    if (p1 > 0 && p2 > 0 && p3 > 0) {
      float thExt = params.substring(0, p1).toFloat();
      float thFlex = params.substring(p1 + 1, p2).toFloat();
      float mvcExt = params.substring(p2 + 1, p3).toFloat();
      float mvcFlex = params.substring(p3 + 1).toFloat();

      emgSensor.setThresholdsAndMVC(thExt, thFlex, mvcExt, mvcFlex);
      ble.sendData("PARAM:UPDATED");
    }
  } // <--- [수정] SET_PARAM 조건문의 닫는 중괄호 추가
  else if (cmd == "TRAIN_START") {
    emgSensor.resetPoints();
    currentState = SystemState::TRAINING_ACTIVE;
    ble.sendData("TRAIN:STARTED");
  }
  else if (cmd == "STOP" || cmd == "EMERGENCY") {
    triggerSafetyReturn("USER_STOP");
  }
}

void RehabSystem::processCalibration() {
  uint32_t elapsedTime = millis() - stateStartTime;

  // 5초간 샘플 수집 (10ms 주기로 누적)
  if (elapsedTime < CALIBRATION_DURATION_MS) {
    sumExtensor += emgSensor.getExtensorRMS();
    sumFlexor += emgSensor.getFlexorRMS();
    sampleCount++;
    delay(10);
  } else {
    // 5초 종료 후 단계별 처리
    float avgExt = (sampleCount > 0) ? (sumExtensor / sampleCount) : 0;
    float avgFlex = (sampleCount > 0) ? (sumFlexor / sampleCount) : 0;

    if (currentState == SystemState::CALIB_REST) {
      emgSensor.setRestThreshold(avgExt, avgFlex);
      currentState = SystemState::CALIB_EXTENSOR_MAX;
      ble.sendData("CALIB:EXT_MAX_START");
    } 
    else if (currentState == SystemState::CALIB_EXTENSOR_MAX) {
      emgSensor.setExtensorMax(avgExt);
      currentState = SystemState::CALIB_FLEXOR_MAX;
      ble.sendData("CALIB:FLEX_MAX_START");
    } 
    else if (currentState == SystemState::CALIB_FLEXOR_MAX) {
      emgSensor.setFlexorMax(avgFlex);
      currentState = SystemState::TRAINING_READY;
      ble.sendData("CALIB:COMPLETE");
    }

    // 다음 단계를 위한 초기화
    stateStartTime = millis();
    sumExtensor = 0.0;
    sumFlexor = 0.0;
    sampleCount = 0;
  }
}

void RehabSystem::processTraining() {
  emgSensor.updateTrainingPoints();

  // 굴곡근 동시수축(Co-contraction) 감지 시 비상 정지
  if (emgSensor.isFlexorActivated()) {
    triggerSafetyReturn("CO_CONTRACTION_DETECTED");
    return;
  }

  // 신전근 100% 활성화 달성 시 모터 가동
  if (emgSensor.isExtensorActivated()) {
    int targetAngle = map(currentLevel, 1, 10, 30, 180);
    int targetAngles[5] = {targetAngle, targetAngle, targetAngle, targetAngle, targetAngle};
    
    ble.sendData("TRAIN:MOTOR_MOVING");
    servo.moveToAnglesStaggered(targetAngles, 10);
    
    delay(3000); // 팽창 유지를 위한 대기 시간
    
    // 동작 완료 후 복귀
    servo.returnToInitialPosition();
    emgSensor.resetPoints();
    ble.sendData("TRAIN:CYCLE_COMPLETE");
  }
}

void RehabSystem::triggerSafetyReturn(String reason) {
  Serial.println("[SAFETY] 비상 복귀 발동 원인: " + reason);
  ble.sendData("SAFETY_STOP:" + reason);
  currentState = SystemState::TRAINING_RETURNING;
}