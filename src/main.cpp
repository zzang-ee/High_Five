#include <Arduino.h>
#include <WiFi.h>
#include <FirebaseESP32.h>
#include <ESP32Servo.h>

// ==========================================
// 1. 네트워크 및 Firebase 설정
// ==========================================
#define WIFI_SSID "YOUR_WIFI_SSID"
#define WIFI_PASSWORD "YOUR_WIFI_PASSWORD"
#define FIREBASE_HOST "YOUR_PROJECT_ID.firebaseio.com" // https:// 제외
#define FIREBASE_AUTH "YOUR_FIREBASE_DATABASE_SECRET"

FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;

// ==========================================
// 2. 핀 정의 & 객체 생성
// ==========================================
const int PIN_EMG_CH1 = 34;   // Extensor (신전근)
const int PIN_EMG_CH2 = 35;   // Flexor (굴곡근)
const int PIN_ANGLE_SENSOR = 32; // PIP 관절 각도 센서 (포텐쇼미터)
const int PIN_SERVO = 18;      // 모터 1개

Servo motor1;

// ==========================================
// 3. 내부 상태 및 제어 변수 (DB 노드 동기화)
// ==========================================
String mode = "STOP";
int target_angle = 55;
int hold_time_sec = 3;
int motor_level = 1;
int threshold_ch1 = 1800;
int flexor_limit = 2500;

bool is_triggered = false;
bool is_cocontraction = false;

// 보조 타스크 및 타이머 변수
unsigned long mode_timer = 0;
unsigned long response_start_time = 0;

// ==========================================
// 4. 함수 선언
// ==========================================
int readAngle();
void updateControlFromDB();
void runDailyTest();
void runTraining();
void setMotorAngle(int angle);

void setup() {
  Serial.begin(115200);

  // 핀 모드 설정
  pinMode(PIN_EMG_CH1, INPUT);
  pinMode(PIN_EMG_CH2, INPUT);
  pinMode(PIN_ANGLE_SENSOR, INPUT);

  // 서보모터 초기화
  ESP32PWM::allocateTimer(0);
  motor1.setPeriodHertz(50); // 표준 50Hz 서보
  motor1.attach(PIN_SERVO, 500, 2400);
  motor1.write(0); // 시작 시 Rest 위치 (0도)

  // WiFi 연결
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(300);
  }
  Serial.println("\nConnected to Wi-Fi!");

  // Firebase 설정
  config.host = FIREBASE_HOST;
  config.signer.tokens.legacy_token = FIREBASE_AUTH;
  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(true);

  Serial.println("ESP32 Firmware Ready.");
}

void loop() {
  // 1. Firebase control 노드 동기화 (0.2초 마다)
  static unsigned long last_db_check = 0;
  if (millis() - last_db_check > 200) {
    updateControlFromDB();
    last_db_check = millis();
  }

  // 2. 모드별 동작 수행
  if (mode == "STOP") {
    setMotorAngle(0); // 0도 복귀
    if (is_triggered || is_cocontraction) {
      is_triggered = false;
      is_cocontraction = false;
      Firebase.setBool(fbdo, "/users/USER_001/realtime/is_triggered", false);
      Firebase.setBool(fbdo, "/users/USER_001/realtime/is_cocontraction", false);
    }
  } 
  else if (mode == "TEST") {
    runDailyTest();
  } 
  else if (mode == "TRAIN") {
    runTraining();
  }

  delay(10); // 안정적인 스케줄링을 위한 짧은 딜레이
}

// ==========================================
// 5. 세부 동작 함수 구현
// ==========================================

// 관절 각도 획득 (ADC 0~4095 -> Angle 0~90도 매핑)
int readAngle() {
  int raw = analogRead(PIN_ANGLE_SENSOR);
  int angle = map(raw, 0, 4095, 0, 90);
  return constrain(angle, 0, 90);
}

// 모터 구동 함수
void setMotorAngle(int angle) {
  angle = constrain(angle, 0, 90);
  motor1.write(angle);
}

// Firebase의 control 노드 읽어오기
void updateControlFromDB() {
  if (Firebase.getString(fbdo, "/users/USER_001/control/mode")) {
    mode = fbdo.stringValue();
  }
  if (Firebase.getInt(fbdo, "/users/USER_001/control/target_angle")) {
    target_angle = fbdo.intValue();
  }
  if (Firebase.getInt(fbdo, "/users/USER_001/control/hold_time_sec")) {
    hold_time_sec = fbdo.intValue();
  }
  if (Firebase.getInt(fbdo, "/users/USER_001/control/threshold_ch1")) {
    threshold_ch1 = fbdo.intValue();
  }
  if (Firebase.getInt(fbdo, "/users/USER_001/control/flexor_limit")) {
    flexor_limit = fbdo.intValue();
  }
}

// 데일리 테스트 모드 구현
void runDailyTest() {
  Serial.println("[TEST MODE] Starting Daily Test...");
  
  // 1. Baseline 노이즈 측정 (2초간)
  long sum_ch1 = 0, sum_ch2 = 0;
  for (int i = 0; i < 20; i++) {
    sum_ch1 += analogRead(PIN_EMG_CH1);
    sum_ch2 += analogRead(PIN_EMG_CH2);
    delay(100);
  }
  int baseline_ch1 = sum_ch1 / 20;
  int baseline_ch2 = sum_ch2 / 20;

  // 2. 환자의 자량 펴기 시도 모니터링 (경직 또는 근력 한계 측정)
  int active_max_angle = 0;
  unsigned long test_start = millis();
  int last_angle = readAngle();
  unsigned long plateau_start = millis();

  while (millis() - test_start < 10000) { // 최대 10초간 검사
    int current_ch2 = analogRead(PIN_EMG_CH2);
    int current_angle = readAngle();

    if (current_angle > active_max_angle) {
      active_max_angle = current_angle;
    }

    // 조건 A: 굴곡근 경직 감지 (급증)
    if (current_ch2 > (baseline_ch2 + 1000)) { 
      Serial.println("[TEST] Spasm Detected!");
      break;
    }

    // 조건 B: 근력 한계 (각도 정체 Plateau 2초 유지)
    if (abs(current_angle - last_angle) <= 2) {
      if (millis() - plateau_start > 2000) {
        Serial.println("[TEST] Muscle Strength Plateau Reached.");
        break;
      }
    } else {
      last_angle = current_angle;
      plateau_start = millis();
    }

    delay(50);
  }

  // 3. DB daily_tests 노드에 결과 업로드
  String path = "/users/USER_001/daily_tests/2026-07-26"; // 예시 날짜
  Firebase.setInt(fbdo, path + "/baseline_ch1", baseline_ch1);
  Firebase.setInt(fbdo, path + "/baseline_ch2", baseline_ch2);
  Firebase.setInt(fbdo, path + "/active_max_angle", active_max_angle);

  // 테스트 완료 후 자동으로 STOP 모드로 전환
  Firebase.setString(fbdo, "/users/USER_001/control/mode", "STOP");
  mode = "STOP";
  Serial.println("[TEST MODE] Completed and Uploaded.");
}

// 재활 훈련 모드 구현
void runTraining() {
  int emg1 = analogRead(PIN_EMG_CH1);
  int emg2 = analogRead(PIN_EMG_CH2);

  // 1. 안전 장치: 굴곡근 경직(Co-contraction) 감지 시 비상 정지
  if (emg2 > flexor_limit) {
    if (!is_cocontraction) {
      is_cocontraction = true;
      Firebase.setBool(fbdo, "/users/USER_001/realtime/is_cocontraction", true);
      Serial.println("⚠️ EMERGENCY: Co-contraction Detected!");
    }
    setMotorAngle(0); // 0도로 즉시 비상 복귀
    return;
  }

  // 2. 신전근(CH1) 의지 감지 -> 모터 보조 구동
  if (emg1 > threshold_ch1 && !is_triggered) {
    is_triggered = true;
    response_start_time = millis();
    Firebase.setBool(fbdo, "/users/USER_001/realtime/is_triggered", true);
    Serial.println(" Triggered! Motor moving to target_angle...");

    // [핵심 제어] target_angle까지 모터 이동
    setMotorAngle(target_angle);

    // hold_time_sec (예: 3초) 동안 유지
    delay(hold_time_sec * 1000);

    // 천천히 0도로 복귀
    setMotorAngle(0);

    // 훈련 1회 완료 후 상태 리셋
    is_triggered = false;
    Firebase.setBool(fbdo, "/users/USER_001/realtime/is_triggered", false);
  }
}