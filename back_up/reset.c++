#include <Arduino.h>
#include <ESP32Servo.h>

// 4개 손가락 서보모터 GPIO 핀
const int SERVO_PINS[4] = {16, 17, 18, 19};
Servo fingers[4];

void setup() {
    Serial.begin(115200);

    // ESP32 PWM 타이머 할당
    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);

    Serial.println(">>> 모든 서보모터를 0도로 이동합니다...");

    for (int i = 0; i < 4; i++) {
        fingers[i].setPeriodHertz(50);
        // 서보모터 연결 (핀번호, 최소 펄스폭, 최대 펄스폭)
        fingers[i].attach(SERVO_PINS[i], 500, 2400);
        
        // 0도로 위치 이동
        fingers[i].write(0);
    }

    Serial.println(">>> 0도 초기화 완료! 이제 손가락 관절을 조립하셔도 됩니다.");
}

void loop() {
    // 0도 위치를 계속 유지 (아무 동작도 하지 않음)
    delay(1000);
}