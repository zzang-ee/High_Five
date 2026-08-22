#include "CurrentSensor.h"

CurrentSensor::CurrentSensor() {
  isInitialized = false;
  currentStallThreshold = STALL_CURRENT_THRES_MA;
}

bool CurrentSensor::begin() {
  // ESP32 기본 I2C 핀(SDA: GPIO 21, SCL: GPIO 22) 사용
  // 서보모터 핀과 중복되지 않도록 확인 필요
  if (!ina219.begin()) {
    Serial.println("Error: INA219 전류 센서를 찾을 수 없습니다.");
    isInitialized = false;
    return false;
  }
  
  // MG92B 서보모터 측정 범위에 맞게 교정 (32V, 2A 범위)
  ina219.setCalibration_32V_2A();
  isInitialized = true;
  Serial.println("INA219 전류 센서 초기화 완료");
  return true;
}

float CurrentSensor::getCurrentmA() {
  if (!isInitialized) return 0.0;
  
  float current_mA = ina219.getCurrent_mA();
  // 부하 흐름 방향에 따라 음수가 나올 수 있으므로 절대값 처리
  return abs(current_mA);
}

bool CurrentSensor::isStallDetected() {
  if (!isInitialized) return false;

  // 노이즈 방지를 위해 연속 3회 읽어 평균 검사
  float sum = 0.0;
  for (int i = 0; i < 3; i++) {
    sum += getCurrentmA();
    delayMicroseconds(500);
  }
  float avgCurrent = sum / 3.0;

  if (avgCurrent >= currentStallThreshold) {
    Serial.printf("[WARNING] 스톨 전류 감지! 현재 전류: %.2f mA\n", avgCurrent);
    return true;
  }
  
  return false;
}

void CurrentSensor::setStallThreshold(float thresholdmA) {
  currentStallThreshold = thresholdmA;
}