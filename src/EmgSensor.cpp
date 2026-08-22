#include "EmgSensor.h"

EmgSensor::EmgSensor() {
  extensorThreshold = 0.0;
  flexorThreshold = 0.0;
  extensorMVC = 0.0;
  flexorMVC = 0.0;
  sampleIndex = 0;
  extensorPoints = 0;
  flexorPoints = 0;

  for (int i = 0; i < 25; i++) {
    extensorSamples[i] = 0.0;
    flexorSamples[i] = 0.0;
  }
}

void EmgSensor::begin() {
  analogReadResolution(12); // ESP32 12-bit ADC (0 ~ 4095)
}

// 25개 샘플의 RMS(실효치) 계산 (논문 식 1 반영)
float EmgSensor::getExtensorRMS() {
  extensorSamples[sampleIndex] = (float)analogRead(EMG_EXTENSOR_PIN);
  
  float sumSq = 0.0;
  for (int i = 0; i < 25; i++) {
    sumSq += extensorSamples[i] * extensorSamples[i];
  }
  
  return sqrt(sumSq / 25.0);
}

float EmgSensor::getFlexorRMS() {
  flexorSamples[sampleIndex] = (float)analogRead(EMG_FLEXOR_PIN);

  float sumSq = 0.0;
  for (int i = 0; i < 25; i++) {
    sumSq += flexorSamples[i] * flexorSamples[i];
  }

  // 링 버퍼 인덱스 업데이트
  sampleIndex = (sampleIndex + 1) % 25;

  return sqrt(sumSq / 25.0);
}

// 1. 휴식 상태 측정 후 Threshold 설정 (논문 식 2 반영)
void EmgSensor::setRestThreshold(float extensorAvgRest, float flexorAvgRest) {
  extensorThreshold = extensorAvgRest * scalingFactor;
  flexorThreshold = flexorAvgRest * scalingFactor;
  Serial.printf("[EMG] Threshold 설정 - 신전근: %.2f, 굴곡근: %.2f\n", extensorThreshold, flexorThreshold);
}

// 2. 최대 신전근 측정값(MVC) 반영
void EmgSensor::setExtensorMax(float extensorMaxVal) {
  extensorMVC = extensorMaxVal;
  Serial.printf("[EMG] 신전근 MVC 설정: %.2f\n", extensorMVC);
}

// 3. 최대 굴곡근 측정값(MVC) 반영
void EmgSensor::setFlexorMax(float flexorMaxVal) {
  flexorMVC = flexorMaxVal;
  Serial.printf("[EMG] 굴곡근 MVC 설정: %.2f\n", flexorMVC);
}

// [JSON 연동] DB/앱에서 읽어온 임계값 및 MVC 직접 주입
  void EmgSensor::setThresholdsAndMVC(float thExt, float thFlex, float mvcExt, float mvcFlex) {
    extensorThreshold = thExt;
    flexorThreshold = thFlex;
    extensorMVC = mvcExt;
    flexorMVC = mvcFlex;
  }

// 4. Threshold~MVC 구간 10등분 후 포인트 부여 (논문 식 6 반영)
void EmgSensor::updateTrainingPoints() {
  float extRMS = getExtensorRMS();
  float flexRMS = getFlexorRMS();

  // --- 신전근 포인트 산출 ---
  if (extRMS > extensorThreshold && extensorMVC > extensorThreshold) {
    float step = (extensorMVC - extensorThreshold) / 10.0;
    if (step > 0) {
      int level = (int)((extRMS - extensorThreshold) / step) + 1;
      if (level > 10) level = 10;
      extensorPoints += (level * 5); // 구간별 가중치 부여 (5, 10, ... 50 point)
    }
  }

  // --- 굴곡근 포인트 산출 ---
  if (flexRMS > flexorThreshold && flexorMVC > flexorThreshold) {
    float step = (flexorMVC - flexorThreshold) / 10.0;
    if (step > 0) {
      int level = (int)((flexRMS - flexorThreshold) / step) + 1;
      if (level > 10) level = 10;
      flexorPoints += (level * 5);
    }
  }
}

bool EmgSensor::isExtensorActivated() {
  return extensorPoints >= 100;
}

bool EmgSensor::isFlexorActivated() {
  return flexorPoints >= 100; // 동시수축 활성화 조건
}

void EmgSensor::resetPoints() {
  extensorPoints = 0;
  flexorPoints = 0;
}