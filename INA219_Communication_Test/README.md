# INA219 납땜 후 통신 전용 확인

이 프로젝트는 INA219의 전류·전압을 측정하지 않고 `0x40` 주소의 I2C ACK
응답만 확인한다. INA219 측정 레지스터는 읽지 않는다.
서보, BLE 및 측정 단자 `VIN+`, `VIN-`는 사용하지 않는다.

## 배선

| INA219 | ESP32 |
|---|---|
| `VCC` | `3.3V` |
| `GND` | `GND` |
| `SDA` | `GPIO23` |
| `SCL` | `GPIO22` |

ESP32 전원을 끈 상태에서 배선한다. INA219의 `VCC`에는 배터리나 서보용
5V를 연결하지 말고 ESP32의 3.3V만 연결한다. 이번 시험에서는 `VIN+`,
`VIN-`를 비워 둔다.

## 업로드와 확인

```powershell
cd C:\PlatformIO\High_Five\INA219_Communication_Test
& "$env:USERPROFILE\.platformio\penv\Scripts\platformio.exe" run -t upload
& "$env:USERPROFILE\.platformio\penv\Scripts\platformio.exe" device monitor -b 115200 -p COM5
```

정상 통신이면 2초마다 다음 메시지가 출력된다.

```text
I2C_BUS,OK
INA219_COMM,OK,address=0x40
```

통신이 되지 않으면 다음 메시지가 출력된다.

```text
INA219_COMM,FAIL,address=0x40,reason=NO_I2C_ACK
```

실행 중 연결 상태가 변하면 다음 메시지도 출력된다.

```text
INA219_COMM_CHANGED,CONNECTED
INA219_COMM_CHANGED,DISCONNECTED
```

실패 시에는 VCC/GND 극성, SDA/SCL 순서, 납땜 불량 및 핀 사이 납땜 단락을
확인한다. 기본 주소 점퍼를 변경하지 않은 INA219의 주소는 `0x40`이다.
