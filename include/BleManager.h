#pragma once
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <BLESecurity.h>
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include "Config.h"

enum class BleSafetyCommand : uint8_t {
  NONE,
  STOP,
  DISCONNECT,
  EMERGENCY
};

class BleManager : public BLEServerCallbacks, public BLECharacteristicCallbacks {
private:
  struct CommandMessage {
    char data[BLE_MAX_COMMAND_LENGTH + 1];
  };

  struct TxMessage {
    // A negotiated 247-byte ATT MTU carries at most 244 payload bytes.
    char data[BLE_LOCAL_MTU - 2U];
  };

  BLEServer* pServer;
  BLECharacteristic* pTxCharacteristic;
  BLECharacteristic* pRxCharacteristic;
  BLESecurity* pSecurity;
  volatile bool deviceConnected;
  volatile uint16_t negotiatedMtu;
  QueueHandle_t commandQueue;
  QueueHandle_t safetyQueue;
  QueueHandle_t txQueue;
  uint32_t lastTxTime;

public:
  BleManager();
  void begin();
  void update(uint32_t nowMs);
  
  // 상태 데이터 앱으로 전송 (TX)
  // State/events are important by default. Realtime telemetry passes false
  // so it can retry instead of displacing an event when the queue is full.
  bool sendData(const String& message, bool important = true);
  
  // 데이터 수신 여부 확인 및 가져오기 (RX)
  bool available();
  String readData();
  BleSafetyCommand consumeSafetyCommand();
  
  // BLE 연결 상태 반환
  bool isConnected() const { return deviceConnected; }
  bool hasRequiredMtu() const { return negotiatedMtu >= BLE_LOCAL_MTU; }
  uint16_t getNegotiatedMtu() const { return negotiatedMtu; }

  // BLE 콜백 함수 구현
  void onConnect(BLEServer* pServer) override;
  void onDisconnect(BLEServer* pServer) override;
  void onMtuChanged(BLEServer* pServer,
                    esp_ble_gatts_cb_param_t* param) override;
  void onWrite(BLECharacteristic* pCharacteristic) override;
};
