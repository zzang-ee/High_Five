# BLE Smoke Test after measurement 01/02

이 프로젝트는 `01_EMG_Raw_Check`, `02_EMG_Calibration`까지만 끝난 상태에서
앱과 장갑 사이의 최소 경로를 확인하기 위한 독립 펌웨어다. 전류센서를
초기화하거나 판정에 사용하지 않는다.

## 반드시 지킬 안전 조건

- 장갑을 사람 손에 착용하지 말고 와이어 장력을 풀어 둔 벤치 상태에서만 실행한다.
- 서보 전원 스위치를 바로 끌 수 있게 준비한다.
- 시험 이동은 논리각 0도에서 15도까지만 수행한다.
- 전류센서 기준 시험이 끝나기 전에는 목표각을 늘리지 않는다.
- 이 코드는 통신 확인용이며 본 훈련이나 안전 검증을 대체하지 않는다.

## 생산 코드와 같은 BLE 연결 정보

- 장치 이름: `RehabGlove`
- PIN: `654321`
- Service: `6E400001-B5A3-F393-E0A9-E50E24DCCA9E`
- RX(write): `6E400002-B5A3-F393-E0A9-E50E24DCCA9E`
- TX(notify): `6E400003-B5A3-F393-E0A9-E50E24DCCA9E`
- 요구 MTU: 247
- 시리얼 속도: 115200

`BleManager`, `ServoManager`, `EmgSensor`는 상위 메인 펌웨어의 소스를 직접
컴파일하므로 UUID, 보안, 서보 방향, EMG 필터가 메인 코드와 동일하다.

## 지원 명령

| 순서 | 앱에서 RX로 보낼 텍스트 | 기대 결과 |
|---|---|---|
| 1 | `MTU?` | `{"mtu":247}` 확인 |
| 2 | `STATUS?` | status, angles, emg, smoke_test JSON 수신 |
| 3 | `DAILY_TEST_START` | 실제 측정 없이 데일리 테스트 성공 JSON 수신 |
| 4 | `CALIB_START` | 약 4초 동안 가상 EMG·ROM 보정 이벤트 수신 |
| 5 | `FINGERS:01` 또는 `FINGERS:1F` | 손가락 선택 후 `fingers_selected` 수신 |
| 6 | `TRAIN_START` | 선택 손가락이 15도까지 이동·1초 유지·홈 복귀 |
| 7 | `STOP` | 동작 중이면 즉시 홈 복귀, 선택은 유지 |
| 8 | `EMERGENCY` | 홈 복귀 후 PWM 분리 |

손가락 비트는 엄지 `01`, 검지 `02`, 중지 `04`, 약지 `08`, 소지 `10`이다.
가상 보정 완료 시에는 모든 손가락의 목표각도를 안전한 벤치 시험용 15도로
전송한다. 다중 손가락 선택도 수신 시험을 위해 허용하지만 전류센서가 없으므로
사람이 착용한 상태에서는 절대 실행하지 않는다.

데일리 테스트 명령은 `DAILY`로 시작하는 문자열과 `SELF_TEST`,
`SELF_TEST_START`, `TEST_START`도 같은 성공 응답으로 처리한다. `SES_GET`,
`SES_ACK` 세션 저장 기능은 본 펌웨어에서만 시험하며 여기서는
`{"error":"NOT_IN_SMOKE_TEST"}`를 반환한다.

## 업로드

```powershell
cd C:\PlatformIO\High_Five\BLE_Smoke_Test_01_02
pio run -t upload
pio device monitor -b 115200 -p COM5
```

앱은 연결·페어링 후 MTU 247을 요청하고 TX notification을 활성화한 다음
위 명령을 순서대로 전송해야 한다.
