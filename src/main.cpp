// #include <Arduino.h>
// #include <ESP32Servo.h>
// #include <Wire.h>
// #include <Adafruit_INA219.h>

// // Hardware pins
// constexpr int EMG_CH1 = 34;  // extensor: open-hand command
// constexpr int EMG_CH2 = 35;  // flexor: emergency stop command
// constexpr int SERVO_COUNT = 4;
// const int SERVO_PIN[SERVO_COUNT] = {16, 17, 18, 19};

// // Motion and safety settings.  Tune MAX_SAFE_CURRENT_MA after measuring the
// // normal running current of the complete servo supply.
// // MG92B is a 180-degree positional servo.  Use its electrical centre as the
// // mechanical home; commanding 0 degrees at startup drives it to an end stop.
// // Fit each servo horn while this firmware is holding HOME_ANGLE.
// constexpr int HOME_ANGLE = 90;
// constexpr int OPEN_ANGLE = 135;
// constexpr uint32_t STEP_INTERVAL_MS = 45;   // 1 degree / 45 ms: slow motion
// constexpr uint32_t HOLD_TIME_MS = 3000;
// constexpr uint32_t EMG_SAMPLE_PERIOD_MS = 10;
// constexpr uint32_t EMG_DEBUG_PERIOD_MS = 500;
// constexpr float MAX_SAFE_CURRENT_MA = 1200.0f;
// constexpr uint8_t CURRENT_TRIP_SAMPLES = 2; // reject one noisy INA219 sample

// Servo servos[SERVO_COUNT];
// Adafruit_INA219 ina219;
// bool ina219Available = false;

// int ch1Threshold = 2400;
// int ch2Threshold = 2000;
// int currentAngle[SERVO_COUNT] = {HOME_ANGLE, HOME_ANGLE, HOME_ANGLE, HOME_ANGLE};
// int targetAngle = HOME_ANGLE;

// enum class State { WAIT_FOR_COMMAND, OPENING, HOLDING, RETURNING };
// State state = State::WAIT_FOR_COMMAND;
// uint32_t lastStepAt = 0;
// uint32_t holdStartedAt = 0;
// uint32_t lastEmgSampleAt = 0;
// uint32_t lastEmgDebugAt = 0;
// int filteredCh1 = 0;
// int filteredCh2 = 0;
// uint8_t overCurrentSamples = 0;

// int readAverage(int pin, uint8_t samples = 4) {
//   uint32_t total = 0;
//   for (uint8_t i = 0; i < samples; ++i) {
//     total += analogRead(pin);
//     delay(2);
//   }
//   return total / samples;
// }

// void calibrateEMG() {
//   Serial.println("\n[EMG calibration]");
//   Serial.println("Relax your hand. Measuring baseline in 2 seconds...");
//   delay(2000);

//   uint32_t ch1RestSum = 0, ch2RestSum = 0;
//   for (uint8_t i = 0; i < 100; ++i) {
//     ch1RestSum += analogRead(EMG_CH1);
//     ch2RestSum += analogRead(EMG_CH2);
//     delay(10);
//   }
//   const int ch1Rest = ch1RestSum / 100;
//   const int ch2Rest = ch2RestSum / 100;

//   Serial.println("Extend/open your fingers firmly for 3 seconds...");
//   delay(1000);
//   int ch1Max = ch1Rest;
//   const uint32_t ch1StartedAt = millis();
//   while (millis() - ch1StartedAt < 3000) {
//     // Use the same averaged value used in loop(), not a short ADC spike.
//     ch1Max = max(ch1Max, readAverage(EMG_CH1));
//     delay(5);
//   }

//   Serial.println("Make a fist firmly for 3 seconds...");
//   delay(1000);
//   int ch2Max = ch2Rest;
//   const uint32_t ch2StartedAt = millis();
//   while (millis() - ch2StartedAt < 3000) {
//     ch2Max = max(ch2Max, readAverage(EMG_CH2));
//     delay(5);
//   }

//   // CH1 requires a deliberate contraction. CH2 uses a lower threshold so it
//   // remains a sensitive emergency-stop signal.
//   // 30% is intentionally sensitive: EMG modules vary substantially between
//   // users and electrode placements. The filtered input suppresses brief noise.
//   ch1Threshold = ch1Rest + (ch1Max - ch1Rest) * 0.30f;
//   ch2Threshold = ch2Rest + (ch2Max - ch2Rest) * 0.35f;
//   Serial.printf("CH1: rest=%d max=%d threshold=%d\n", ch1Rest, ch1Max, ch1Threshold);
//   Serial.printf("CH2: rest=%d max=%d threshold=%d\n", ch2Rest, ch2Max, ch2Threshold);
// }

// void updateEMG() {
//   if (millis() - lastEmgSampleAt < EMG_SAMPLE_PERIOD_MS) return;
//   lastEmgSampleAt = millis();
//   // Light low-pass filtering prevents a single ADC spike from starting motion.
//   const int raw1 = readAverage(EMG_CH1);
//   const int raw2 = readAverage(EMG_CH2);
//   filteredCh1 = (filteredCh1 * 3 + raw1) / 4;
//   filteredCh2 = (filteredCh2 * 3 + raw2) / 4;
// }

// bool isOverCurrent() {
//   if (!ina219Available) return false;
//   const float current = ina219.getCurrent_mA();
//   if (isnan(current) || isinf(current)) return false;

//   if (fabsf(current) >= MAX_SAFE_CURRENT_MA) {
//     ++overCurrentSamples;
//   } else {
//     overCurrentSamples = 0;
//   }
//   if (overCurrentSamples >= CURRENT_TRIP_SAMPLES) {
//     Serial.printf("Emergency: over-current %.0f mA\n", current);
//     return true;
//   }
//   return false;
// }

// bool safetyStopRequested() {
//   if (filteredCh2 >= ch2Threshold) {
//     Serial.printf("Emergency: CH2 detected (%d)\n", filteredCh2);
//     return true;
//   }
//   return isOverCurrent();
// }

// void beginReturn(const char *reason) {
//   if (state == State::RETURNING) return;
//   Serial.printf("%s: stopping and returning slowly.\n", reason);
//   targetAngle = HOME_ANGLE;
//   state = State::RETURNING;
//   // Do not wait a full step period after an emergency command.
//   lastStepAt = millis() - STEP_INTERVAL_MS;
// }

// bool allAtTarget() {
//   for (int i = 0; i < SERVO_COUNT; ++i)
//     if (currentAngle[i] != targetAngle) return false;
//   return true;
// }

// void moveOneStep() {
//   if (millis() - lastStepAt < STEP_INTERVAL_MS) return;
//   lastStepAt = millis();
//   for (int i = 0; i < SERVO_COUNT; ++i) {
//     if (currentAngle[i] < targetAngle) ++currentAngle[i];
//     else if (currentAngle[i] > targetAngle) --currentAngle[i];
//     servos[i].write(currentAngle[i]);
//   }
// }

// void setup() {
//   Serial.begin(115200);
//   Wire.begin(23, 22); // GPIO 23 -> SDA, GPIO 22 -> SCL
//   analogReadResolution(12);
//   Wire.setTimeOut(10);
//   ina219Available = ina219.begin();
//   Serial.println(ina219Available ? "INA219 connected." : "WARNING: INA219 not found; current protection is unavailable.");

//   ESP32PWM::allocateTimer(0);
//   ESP32PWM::allocateTimer(1);
//   ESP32PWM::allocateTimer(2);
//   ESP32PWM::allocateTimer(3);
//   for (int i = 0; i < SERVO_COUNT; ++i) {
//     servos[i].setPeriodHertz(50);
//     // 1000..2000 us is the normal, safe 0..180-degree control range.
//     // The previous 500..2400 us range can force an MG92B into its end stops.
//     servos[i].attach(SERVO_PIN[i], 1000, 2000);
//     servos[i].write(HOME_ANGLE);
//   }

//   calibrateEMG();
//   filteredCh1 = readAverage(EMG_CH1);
//   filteredCh2 = readAverage(EMG_CH2);
//   Serial.println("Ready. Contract CH1 to start.");
// }

// void loop() {
//   updateEMG();

//   switch (state) {
//     case State::WAIT_FOR_COMMAND:
//       if (millis() - lastEmgDebugAt >= EMG_DEBUG_PERIOD_MS) {
//         lastEmgDebugAt = millis();
//         Serial.printf("EMG CH1=%d/%d, CH2=%d/%d\n", filteredCh1, ch1Threshold,
//                       filteredCh2, ch2Threshold);
//       }
//       if (filteredCh1 >= ch1Threshold && filteredCh2 < ch2Threshold) {
//         Serial.println("CH1 detected: opening slowly.");
//         targetAngle = OPEN_ANGLE;
//         state = State::OPENING;
//         lastStepAt = millis() - STEP_INTERVAL_MS;
//       }
//       break;

//     case State::OPENING:
//       if (safetyStopRequested()) beginReturn("Safety stop");
//       else {
//         moveOneStep();
//         if (allAtTarget()) {
//           Serial.println("90 degrees reached: holding for 3 seconds.");
//           holdStartedAt = millis();
//           state = State::HOLDING;
//         }
//       }
//       break;

//     case State::HOLDING:
//       // Safety is also checked while holding; no blocking delay is used.
//       if (safetyStopRequested()) beginReturn("Safety stop");
//       else if (millis() - holdStartedAt >= HOLD_TIME_MS) beginReturn("Hold complete");
//       break;

//     case State::RETURNING:
//       // CH2 remains an emergency condition, but return motion continues safely.
//       // A new high current is logged but must not prevent returning home.
//       if (isOverCurrent()) Serial.println("Warning: current high while returning.");
//       moveOneStep();
//       if (allAtTarget()) {
//         overCurrentSamples = 0;
//         state = State::WAIT_FOR_COMMAND;
//         Serial.println("Returned home. Ready.");
//       }
//       break;
//   }
// }
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <ESP32Servo.h>

// ===============================
// 서보 / LED 설정
// ===============================
Servo myServo;

const int servoPin = 18;   // 서보모터 신호선
const int statusLed = 2;   // ESP32 내장 LED

// ===============================
// BLE UUID
// Flutter 코드와 반드시 동일해야 함
// ===============================
#define SERVICE_UUID \
  "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"

#define CHARACTERISTIC_UUID_RX \
  "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"


// ===============================
// 앱 연결 / 연결 해제 처리
// ===============================
class MyServerCallbacks : public BLEServerCallbacks {

  void onConnect(BLEServer* pServer) override {

    digitalWrite(statusLed, HIGH);

    Serial.println("==========================");
    Serial.println("Flutter 앱 연결됨!");
    Serial.println("==========================");
  }

  void onDisconnect(BLEServer* pServer) override {

    digitalWrite(statusLed, LOW);

    Serial.println("==========================");
    Serial.println("Flutter 앱 연결 해제");
    Serial.println("재연결 대기 중...");
    Serial.println("==========================");

    // 다시 BLE 검색 가능하도록 광고 재시작
    pServer->startAdvertising();
  }
};


// ===============================
// Flutter → ESP32 데이터 수신
// ===============================
class MyCallbacks : public BLECharacteristicCallbacks {

  void onWrite(BLECharacteristic* pCharacteristic) override {

    String rxValue = pCharacteristic->getValue().c_str();

    rxValue.trim();

    if (rxValue.length() == 0) {
      return;
    }

    Serial.print("Flutter에서 받은 값: ");
    Serial.println(rxValue);


    // ---------------------------
    // 1. TEST 명령
    // ---------------------------
    if (rxValue == "TEST") {

      Serial.println("BLE 명령 수신 성공!");

      return;
    }


    // ---------------------------
    // 2. 숫자인지 확인
    // ---------------------------
    bool isNumber = true;

    for (int i = 0; i < rxValue.length(); i++) {

      if (!isDigit(rxValue[i])) {

        isNumber = false;

        break;
      }
    }


    if (!isNumber) {

      Serial.println("잘못된 명령입니다.");

      return;
    }


    // ---------------------------
    // 3. 문자열 → 각도 변환
    // ---------------------------
    int angle = rxValue.toInt();


    // ---------------------------
    // 4. 서보 범위 확인
    // ---------------------------
    if (angle >= 0 && angle <= 180) {

      myServo.write(angle);

      Serial.print("서보모터 이동: ");
      Serial.print(angle);
      Serial.println("도");
    }

    else {

      Serial.println(
        "각도는 0~180 사이여야 합니다."
      );
    }
  }
};


// ===============================
// SETUP
// ===============================
void setup() {

  Serial.begin(115200);

  delay(500);


  // ---------------------------
  // LED
  // ---------------------------
  pinMode(statusLed, OUTPUT);

  digitalWrite(statusLed, LOW);


  // ---------------------------
  // Servo 설정
  // ---------------------------
  ESP32PWM::allocateTimer(0);

  myServo.setPeriodHertz(50);

  myServo.attach(
    servoPin,
    500,
    2400
  );


  // 처음 시작 위치
  myServo.write(90);


  // ---------------------------
  // BLE 설정
  // ---------------------------

  // Flutter에서 찾는 이름과 동일
  BLEDevice::init("RehabGlove");


  // BLE Server 생성
  BLEServer* pServer =
    BLEDevice::createServer();


  pServer->setCallbacks(
    new MyServerCallbacks()
  );


  // BLE Service 생성
  BLEService* pService =
    pServer->createService(
      SERVICE_UUID
    );


  // Flutter → ESP32 WRITE용
  BLECharacteristic* pRxCharacteristic =
    pService->createCharacteristic(
      CHARACTERISTIC_UUID_RX,
      BLECharacteristic::PROPERTY_WRITE
    );


  pRxCharacteristic->setCallbacks(
    new MyCallbacks()
  );


  // Service 시작
  pService->start();


  // BLE Advertising 시작
  pServer->getAdvertising()->start();


  Serial.println();
  Serial.println("==========================");
  Serial.println("ESP32 준비 완료");
  Serial.println("BLE 이름 : RehabGlove");
  Serial.println("Flutter 연결 대기 중...");
  Serial.println("==========================");
}


// ===============================
// LOOP
// ===============================
void loop() {

  delay(100);
}