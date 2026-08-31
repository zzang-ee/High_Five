# 키보드 입력 재활 장갑

이 프로젝트는 상위 `High_Five` 펌웨어의 EMG 보정, 손가락별 ROM 보정,
훈련 상태 머신, 서보 궤적 및 전류 안전 로직을 그대로 사용합니다. BLE와
앱/DB 연결만 제거하고, 앱이 RX 특성으로 보내던 **동일한 명령 문자열**을
USB 시리얼에서 한 줄씩 받습니다. 별도의 키보드 명령이나 단축키는 없습니다.

## 빌드 및 실행

ESP32를 연결한 뒤 다음 명령을 실행합니다.

```powershell
& "$env:USERPROFILE\.platformio\penv\Scripts\pio.exe" run -d test_1
& "$env:USERPROFILE\.platformio\penv\Scripts\pio.exe" run -d test_1 -t upload --upload-port COM5
& "$env:USERPROFILE\.platformio\penv\Scripts\pio.exe" device monitor -d test_1 -e esp32dev -p COM5 -b 115200 --echo --eol LF
```

시리얼 모니터는 `115200 baud`, 줄 끝은 `LF` 또는 `CRLF`로 설정합니다.
한 줄이 앱의 BLE write 한 번과 같습니다. 입력은 대소문자를 구분하며,
앞뒤 공백을 포함해 출력 가능한 ASCII 64바이트 이하여야 합니다. 앞뒤 공백은
명령 처리 전에 제거됩니다.

## 앱과 동일한 입력 명령

| 입력 | 동작 |
|---|---|
| `MTU?` | 호환용 논리 MTU 확인 (`247`) |
| `FINGERS:HH` | 손가락 비트 마스크 선택 |
| `CALIB_START` | EMG 및 5개 손가락 ROM 전체 보정 시작 |
| `TRAIN_START` | 선택 및 보정된 손가락으로 훈련 시작 |
| `STOP` | 현재 사이클만 중단하고 홈 복귀, 세션은 계속 |
| `EMERGENCY` | 활성 세션 종료 및 모든 모터 홈 복귀 |
| `STATUS?` | 현재 JSON 상태 즉시 출력 |
| `SES_GET` | 아직 확인되지 않은 최종 세션 JSON 재출력 |
| `SES_ACK:boot,sid,q` | 표시된 세션 저장 완료를 수동 확인 |

손가락 마스크의 bit 0부터 엄지, 검지, 중지, 약지, 소지입니다. 예를 들어
엄지+중지는 `FINGERS:05`, 전체 손가락은 `FINGERS:1F`입니다. 삭제된 구형
명령 `SET_PARAM`과 `ROM_START`는 지원하지 않습니다.

기본 사용 순서는 다음과 같습니다.

```text
MTU?
FINGERS:1F
CALIB_START
```

시리얼 JSON 안내에 따라 안정/신전/굴곡/ROM 보정을 마친 뒤 입력합니다.

```text
TRAIN_START
STATUS?
```

장갑이 내보내는 각 JSON 객체는 시리얼의 한 줄로 출력됩니다. DB가 없으므로
세션 종료 후 다음 훈련을 시작하려면 최종 JSON의 `boot`, `sid`, `q`를 사용해
`SES_ACK:boot,sid,q`를 직접 입력해야 합니다. 이는 앱의 ACK 동작을 생략하지
않고 동일하게 재현하기 위한 것입니다.

예를 들어 최종 JSON 값이 `"boot":123`, `"sid":1`, `"q":42`이면
`SES_ACK:123,1,42`를 입력합니다.

## 하드웨어 및 안전

- 서보 핀: 엄지부터 `16, 17, 18, 19, 21`
- EMG: 신전근 `GPIO34`, 굴곡근 `GPIO35`
- INA219 I2C: SDA `GPIO23`, SCL `GPIO22`
- 서보 전원과 ESP32 전원은 하드웨어 설계에 맞게 분리하고 GND를 공통으로 연결

`STOP`과 `EMERGENCY`는 일반 입력 큐보다 먼저 처리되고, 대기 중인 일반 명령을
비우도록 원본과 동일하게 구현되어 있습니다. 최초 구동은 장갑을 손에서 분리한
상태에서 수행하고, ROM 전류 한계는 실제 기구와 사용자에 맞게 검증해야 합니다.
