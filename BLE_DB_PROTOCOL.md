# 재활 장갑 BLE · 앱 · DB 연동 규격 v8

이 문서는 펌웨어 `BLE_PROTOCOL_VERSION = 8`의 실제 송수신 형식이다.
앱에서 장갑으로 보내는 제어는 짧은 텍스트이고, 장갑에서 앱으로 보내는
상태·결과는 모두 완전한 JSON 객체이다. 한 번의 BLE notification은 하나의
JSON 객체이며 문자열 조각을 이어 붙이는 방식은 사용하지 않는다.

v8은 v7과 명령 의미가 호환되지 않는다. `STOP`과 소프트 전류 안전은 더
이상 세션 종료가 아니므로 앱은 v8 장치에서 이 두 사건을 DB 최종 저장
신호로 취급하면 안 된다.

## 1. BLE 연결 정보

| 항목 | 값 |
|---|---|
| 장치 이름 | `RehabGlove` |
| Service UUID | `6E400001-B5A3-F393-E0A9-E50E24DCCA9E` |
| RX UUID | `6E400002-B5A3-F393-E0A9-E50E24DCCA9E` |
| TX UUID | `6E400003-B5A3-F393-E0A9-E50E24DCCA9E` |
| RX 속성 | Write / Write Without Response |
| TX 속성 | Notify |
| 요구 ATT MTU | 247 이상(notification payload 244 bytes) |
| 페어링 | 암호화, MITM, bonding |
| 개발용 PIN | `654321` |

앱 연결 순서는 다음과 같다.

1. 장갑에 연결하고 TX notification을 구독한다.
2. ATT MTU 247을 요청한다.
3. `MTU?`를 보내고 응답의 `mtu`가 247 이상인지 확인한다.
4. 그 뒤에만 다른 명령을 보낸다.

앱 명령은 ASCII 64바이트 이하이며 한 번의 write에 한 명령만 보낸다.
제품 배포 전에는 개발용 고정 PIN을 변경해야 한다.

## 2. 앱 → 장갑 텍스트 명령

| 명령 | 의미 |
|---|---|
| `MTU?` | 현재 협상된 MTU 확인 |
| `FINGERS:HH` | 0이 아닌 16진수 손가락 mask 선택 |
| `CALIB_START` | EMG 안정·신전 MVC·굴곡 MVC·5손가락 ROM 전체 보정 |
| `TRAIN_START` | 선택되고 보정된 손가락으로 훈련 시작 |
| `STOP` | 현재 시도만 중단하고 홈 복귀, 같은 훈련 세션 계속 |
| `EMERGENCY` | 활성 훈련 세션 종료 및 모든 모터 홈 복귀 |
| `STATUS?` | 실시간 JSON 즉시 요청 |
| `SES_GET` | ACK되지 않은 최종 세션 JSON 재요청 |
| `SES_ACK:boot,sid,q` | 최종 DB 저장 완료 확인 |

손가락 mask는 bit 0부터 엄지, 검지, 중지, 약지, 소지 순서이다.
예를 들어 엄지+중지는 `FINGERS:05`, 전체 손가락은
`FINGERS:1F`이다.

`SET_PARAM`과 `ROM_START`는 v7부터 삭제되었다. 앱/DB는 이전 부팅의
EMG threshold, MVC, ROM 목표각도 또는 레벨을 다음 부팅에 복원 명령으로
보내면 안 된다. 전원을 켠 뒤 `CALIB_START`로 전체 보정을 수행해야 한다.
`control`과 `profile`의 보정값·레벨은 현재 부팅의 읽기 전용 상태이다.
앱에서 레벨을 직접 변경하는 명령은 없다.

### Temporary index-servo lockout

While the index servo is faulty, firmware excludes finger bit 1 (`0x02`) from
all servo commands and ROM calibration. `FINGERS:1F` is accepted but the
`fingers_selected` response reports the effective mask `29` (`0x1D`: thumb,
middle, ring, pinky). `FINGERS:02` returns `{"error":"FINGER_DISABLED"}`;
the previous valid selection is retained. During `CALIB_START`, index ROM is
skipped and an `{"event":"rom","state":"skipped","finger":1,
"reason":"disabled"}` event is sent. `calibration_done.mask` is `29` and
`target_angles.index` stays `0`. The app must accept this four-finger mask as
complete calibration and must not require the index target or index profile.
After repairing the servo, set `TEMP_DISABLED_FINGER_MASK` in `Config.h` to
`0`, rebuild, and run full calibration again.

## 3. boot, sid, q

- `boot`: 전원을 켤 때 새로 만들어지는 장치 부팅 식별자
- `sid`: 해당 부팅 안에서 증가하는 훈련 세션 번호
- `q`: 같은 실시간 snapshot 또는 최종 로그 조각을 합칠 때 쓰는 순번

DB idempotency key는 `boot + sid`이다. `sid`만 사용하면 재부팅 뒤의
다른 세션과 충돌할 수 있다. 장갑의 `device_ms`, `start_ms`,
`end_ms`는 부팅 후 경과시간이므로 Unix 시간이 아니다. 날짜와 Unix
시간은 앱 또는 서버가 수신 시각으로 기록한다.

## 4. 실시간 JSON

같은 `q`를 가진 객체를 앱에서 `realtime` 아래에 병합한다.

```json
{
  "v": 8,
  "boot": 123456,
  "q": 10,
  "device_ms": 84020,
  "status": {
    "mode": "START",
    "is_triggered": true,
    "is_cocontraction": false,
    "is_participation_low": false,
    "battery": 85,
    "stall_detected": false
  }
}
```

`stall_detected`는 v8에서 전류 안전 상태를 뜻하며 훈련 실패를 뜻하지
않는다. `battery`는 2S LiPo의 추정 잔량(0~100 정수)이다. GPIO32의
배터리 분압 입력이 아직 준비되지 않았거나 측정 전압이 유효 범위를
벗어나면 `null`이다. 서보 구동 중 전압 강하로 표시가 흔들리지 않도록
홈 위치에서 얻은 최신 잔량을 훈련 중에도 유지한다.

배터리 측정 회로는 `배터리+ -> 47 kΩ -> GPIO32 -> 22 kΩ -> GND`이며,
22 kΩ과 병렬로 100 nF를 연결한다. 배터리 음극, ESP32 GND, 전류센서
GND는 공통이어야 한다. 2S LiPo 전압을 GPIO32에 직접 연결하면 안 된다.

```json
{
  "q": 10,
  "angles": {
    "thumb": 45,
    "index": 50,
    "middle": 48,
    "ring": 52,
    "pinky": 46
  },
  "target_angles": {
    "thumb": 84,
    "index": 92,
    "middle": 95,
    "ring": 81,
    "pinky": 76
  }
}
```

각도는 앱에 통일된 논리각이다. 홈은 모두 0이고 펴지는 방향이 증가하여
최대 179가 된다. 실제 서보각은 엄지·중지가 `logical + 1`,
검지·약지·소지가 `180 - logical`이다. 손가락 목표 도달률은 보정된
목표가 0보다 클 때 다음처럼 표시한다.

```text
도달률(%) = clamp(angles[finger] / target_angles[finger] * 100, 0, 100)
```

이 값은 별도 관절각 센서의 실측값이 아니라 펌웨어가 서보에 명령한
논리각이다.

```json
{"q":10,"emg":{"ch1":1024,"ch2":980}}
```

`ch1`은 신전근, `ch2`는 굴곡근의 필터링된 RMS이다.

```json
{
  "q": 10,
  "control": {
    "mode": "START",
    "target_angle": 90,
    "hold_time_sec": 3,
    "motor_level": 3,
    "threshold_extensor": 800,
    "threshold_flexor": 800,
    "mvc_extensor": 900,
    "mvc_flexor": 900,
    "participation_required": 0.126
  }
}
```

`target_angle`과 `motor_level`은 선택 손가락의 평균 목표각과 최대
레벨이다. `motor_level`은 보조 강도 1~10단계(1: 낮은 보조,
10: 높은 보조)이며, 앱에서 레벨로 표시할 때 이 값을 사용한다.
`participation_required`는 현재 사이클의 정규화된 CH1 참여 기준(0~1)으로,
레벨과 같은 값이 아니다. 해당 사이클 시작 시 CH1 활성도를 기준으로
`motor_level` 1에서는 70%, 10에서는 35%를 요구하고 중간 레벨은
선형 보간한다. 최소 참여 기준은 5%이며, 두 값 모두 앱 입력값이 아닌
표시·진단용이다. 캘리브레이션의 CH1 휴식 임계값~MVC 최소 간격은
15 RMS count와 휴식 임계값의 25% 중 큰 값이다.

회전 전송되는 `profile`에는 손가락별 `target_angle`,
`motor_level`, `failure_stack`, `calibrated`,
`current_baseline_ma`, `current_trip_ma`가 들어온다. 이 값들은 현재
부팅 안에서만 유효하며 다음 부팅 복원용 DB 데이터로 사용하지 않는다.

## 5. CH1 참여 경고 · 회복 · 실패

모든 레벨의 정상 이동시간은 12초로 같다. 레벨은 속도나 토크 명령을
높이지 않고 CH1 참여 유지 기준만 바꾼다.

- 레벨 1: 트리거 시 CH1의 70% 유지
- 레벨 10: 트리거 시 CH1의 35% 유지
- 레벨 2~9: 위 두 값 사이 선형 보간
- 절대 최저 기준: 정규화 활성도 5%

이동 중 CH1이 기준보다 낮아진 뒤 500 ms가 지나면:

```json
{
  "event": "participation",
  "state": "warning",
  "sid": 1,
  "mask": 31,
  "level": 3,
  "actual_pct": 8,
  "required_pct": 12,
  "grace_ms": 1000
}
```

앱은 경고 팝업을 표시하되 모터는 계속 보조한다. 기준 이상으로 회복한 뒤
3%p 여유를 300 ms 유지하면:

```json
{"event":"participation","state":"recovered","sid":1,"mask":31}
```

경고 후에도 총 저하시간 1500 ms까지 회복하지 못하면:

```json
{"event":"participation","state":"failed","sid":1,"mask":31,"level":3}
```

```json
{"event":"attempt","sid":1,"success":false,"reason":"participation","mask":31}
```

모든 선택 손가락이 함께 중지·복귀한다. 위 두 JSON은 같은 한 번의
사건이다. 앱은 팝업 처리를 위해 `participation.failed`를 사용하고,
훈련 통계는 `attempt` 또는 최종 `training_log`만 사용하여 두 번
집계하지 않는다.

참여 실패 한 번은 선택 그룹의 attempt 한 번이며 선택된 각 손가락의
`failure_stack`을 1씩 증가시킨다. 같은 레벨에서 3회가 되면 해당
손가락 레벨을 1 올리고 스택을 0으로 만든다. 레벨 10에서는 스택을 3으로
유지하며 더 이상 올리지 않는다.

## 6. 전류 안전

훈련 중 공용 INA219는 손가락별 실패 판별기가 아니라 선택된 모터 전체의
안전 감시 장치이다. 공용 센서 하나로는 어느 손가락 전류인지 신뢰성 있게
분리할 수 없으므로 `finger`는 항상 `null`이다.

soft group 기준은 보정 때 측정한 각 손가락의 홈 전류와 ROM 전류 기준을
사용한다. 한 손가락 선택 시 해당 손가락의 ROM 기준과 같고, 여러 손가락
선택 시 각 상승 허용량의 합에 0.60을 곱한 뒤 선택 손가락의 홈 전류 중
가장 큰 값을 한 번만
더한다. 선택한 손가락 중 가장 높은 단일 기준보다 낮아지지는 않는다.
따라서 기준은 손가락 조합과 해당 부팅의 보정 결과에 따라 달라진다.

예를 들어 홈 전류가 모두 약 18.4 mA이고 상승 허용량이
엄지~약지 100 mA, 소지 85 mA이면 5개 동시 기준은 약 309.4 mA이다.

1600 mA hard 기준은 배선·전원·센서의 절대 상한이므로 손가락 수만큼
곱하지 않는다. 위 수치는 초기 벤치값이며 사람 안전이 검증된 의료 기준이
아니다.

짧은 PWM 피크는 무시하고 지속 전류가 확인되면:

```json
{
  "event": "current_safety",
  "state": "triggered",
  "sid": 1,
  "mask": 31,
  "finger": null,
  "current_ma": 320.2,
  "limit_ma": 309.4,
  "scope": "group",
  "phase": "moving"
}
```

가능하면 전체 손가락을 조금 이완한 뒤 함께 홈으로 복귀한다. 이때 현재
시도는 집계하지 않고 같은 `boot/sid`, 세션 시작시간과 누적 훈련 기록을
유지한다.

```json
{"event":"cycle_interrupted","sid":1,"reason":"current_safety","counted_attempt":false,"session_continues":true}
```

홈 위치와 전류 안정이 확인되면 다음 이완·트리거를 기다리며 같은 세션을
계속한다.

```json
{"event":"training","state":"waiting_release","sid":1,"resume_after":"current_safety","session_continues":true,"success_count":2}
```

복귀 중에도 전류가 높거나 1600 mA hard 기준을 넘으면 강제로 계속 당기지
않고 FAULT로 출력 차단하며, 이때만 세션이 종료된다.

전류 사건은 `attempt`, `total_attempts`, `success_count`,
`participation_failure_count`, 손가락 실패 스택, 성공률에 포함하지
않는다. 오직 `current_safety_count`만 증가한다.

## 7. 사이클과 서보 유지

선택 모터는 돌입전류를 분산하기 위해 세션 시작 때 홈 위치에서 30 ms
간격으로 부착된다. 이것은 훈련 동작 시차가 아니다. 펴기와 복귀에서는
선택한 모든 손가락을 같은 궤적 시작시각으로 두고 10 ms 주기 한 번의
스케줄러 실행 안에서 모두 갱신한다. 훈련 신전은 12초의 전용 궤적으로
초반에 와이어 유격을 잡고 목표각 근처에서 점차 느려진다. 모두 목표
명령각에 도달하면 3초 유지한 뒤 함께 복귀한다. ROM과 홈 복귀는 기존
quintic 궤적을 사용한다.

일반 사이클이 홈에 도착한 뒤에는 서보를 detach하지 않고 홈 PWM을
유지한다. 따라서 다음 CH1 트리거를 기다리는 동안에도 케이블 구속이
유지된다. `STOP`과 소프트 전류 안전 복귀에서도 같은 세션을 이어가므로
홈 PWM을 유지한다. 다섯 번째 성공과 `EMERGENCY`로 세션이 끝나면 홈
복귀가 확인된 뒤 detach한다. 하드 과전류에서는 즉시 detach한다. 센서
없는 RC 서보이므로 실제 손가락 위치와 케이블 장력은 별도 하드웨어 검증이
필요하다.

훈련 중 `STOP`은 현재 사이클을 시도 횟수에 넣지 않고 복귀시킨다.

```json
{"event":"cycle_interrupted","sid":1,"reason":"user_stop","counted_attempt":false,"session_continues":true}
```

복귀 뒤에는 새 `TRAIN_START` 없이 이완 300 ms와 다음 CH1 트리거를 다시
만족하면 훈련을 계속한다. 반대로 `EMERGENCY`는 활성 세션을 실제로
종료하므로 최종 `session_end` 및 두 로그 조각을 저장해야 한다. 이때 완성된
캘리브레이션, 목표각도, 레벨과 실패 스택은 삭제하지 않는다.

```json
{"event":"attempt","sid":1,"success":true,"reason":"target","mask":31}
```

```json
{"event":"cycle_done","sid":1,"success":true,"success_count":5,"success_goal":5,"goal_reached":true}
```

성공 5회가 되면 장갑이 자동으로 세션을 종료하므로 앱의 별도 STOP은
필요하지 않다.

## 8. 세션 로그와 DB 저장

세션 시작:

```json
{"event":"session_start","v":8,"boot":123456,"sid":1,"start_ms":90210,"mask":31}
```

앱은 `mask`를 `selected_finger_mask`로 저장한다. 한 attempt는 손가락
수가 아니라 선택 그룹의 한 사이클이다.

MTU 244바이트 안에 안전하게 들어가도록 로그는 같은 `sid/q`를 가진 두
객체로 나뉜다.

```json
{
  "sid": 1,
  "q": 12,
  "part": "outcome",
  "training_log": {
    "total_attempts": 7,
    "success_count": 5,
    "participation_failure_count": 2,
    "average_angle": 60.5,
    "success_rate": 71.4
  }
}
```

```json
{
  "sid": 1,
  "q": 12,
  "part": "safety_emg",
  "training_log": {
    "cocontraction_cnt": 2,
    "current_safety_count": 1,
    "max_extensor_rms": 1000,
    "max_flexor_rms": 1000
  }
}
```

다음 불변식을 DB 검증에 사용할 수 있다.

```text
total_attempts = success_count + participation_failure_count
success_rate = success_count / total_attempts * 100
```

전류 안전은 이 식에 포함되지 않는다.
기존 DB의 `stall_count`는 의미가 혼동되므로 v7부터 제거하고
`current_safety_count`로 마이그레이션한다.

## 9. 신뢰성 있는 세션 종료

최종 저장 단위는 다음 세 객체이다.

```json
{"event":"session_end","v":8,"boot":123456,"sid":1,"q":20,"start_ms":90210,"end_ms":105000,"mask":31,"reason":"GOAL"}
```

```json
{"sid":1,"q":20,"part":"outcome","training_log":{"total_attempts":7,"success_count":5,"participation_failure_count":2,"average_angle":60.5,"success_rate":71.4}}
```

```json
{"sid":1,"q":20,"part":"safety_emg","training_log":{"cocontraction_cnt":2,"current_safety_count":1,"max_extensor_rms":1000,"max_flexor_rms":1000}}
```

앱은 같은 `sid/q`의 `session_end`, `outcome`, `safety_emg`를 모두
DB에 upsert하고 commit이 성공한 뒤에만 다음 ACK를 보낸다.

```text
SES_ACK:123456,1,20
```

ACK 전까지 장갑은 세 객체를 2초마다 재전송하고 새 `TRAIN_START`를
`{"error":"SES_PENDING"}`으로 거부한다. 중복 수신은 append하지 말고
`boot + sid + q + part` 기준으로 upsert한다.

## 10. 앱/서버가 생성하는 값

장갑은 UID, 사용자 이름, 이메일, 날짜, Unix 시간, 일간·주간 통계를 알 수
없다. 다음은 앱 또는 서버가 생성·갱신한다.

- `users/{uid}/profile`
- `realtime/timestamp`
- 세션 key, `date`, `start_time`, `end_time`
- `progress/daily`, `progress/weekly`

`STOP`과 소프트 전류 안전에서는 `session_end`가 오지 않으므로 DB 세션을
확정하거나 `SES_ACK`를 보내면 안 된다. 최종 종료 이유의 대표값은
`GOAL`, `EMERG`, `LINK`, `COCON`, `RESIST`, `EMG_BAD` 및 fault reason이다.

권장 전체 예시는 `DB_SCHEMA_V8.json`을 따른다.
