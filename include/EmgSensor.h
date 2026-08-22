#pragma once
#include <Arduino.h>
#include <math.h>
#include "Config.h"

class EmgSensor {
private:
  // 신전근(Extensor), 굴곡근(Flexor) 매개변수
  float extensorThreshold;
  float flexorThreshold;
  float extensorMVC;
  float flexorMVC;

  // 논문 기준 Scaling Factor (기본값 1.15)
  const float scalingFactor = 1.15;

  // 25샘플 RMS 계산용 링 버퍼
  float extensorSamples[25];
  float flexorSamples[25];
  uint8_t sampleIndex;

  // 훈련 모드 포인트 누적 변수
  int extensorPoints;
  int flexorPoints;

public:
  EmgSensor();
  void begin();

  // 1. 실시간 ADC 샘플링 및 RMS 계산 (N=25)
  float getExtensorRMS();
  float getFlexorRMS();

  // 2. 캘리브레이션 단계별 데이터 처리
  void setRestThreshold(float extensorAvgRest, float flexorAvgRest);
  void setExtensorMax(float extensorMaxVal);
  void setFlexorMax(float flexorMaxVal);

  // [JSON 연동] DB/앱에서 읽어온 임계값 및 MVC 직접 주입
  void setThresholdsAndMVC(float thExt, float thFlex, float mvcExt, float mvcFlex);

  // 3. 훈련 시 활성화 비율 및 포인트 업데이트 (100점 달성 검사)
  void updateTrainingPoints();
  bool isExtensorActivated(); // 신전근 100% 달성 여부
  bool isFlexorActivated();   // 굴곡근(동시수축) 100% 달성 여부
  void resetPoints();

  // Getter
  float getExtensorThreshold() const { return extensorThreshold; }
  float getFlexorThreshold() const { return flexorThreshold; }
};