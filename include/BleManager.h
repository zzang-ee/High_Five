#pragma once
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <Arduino.h>
#include "Config.h"

class BleManager : public BLEServerCallbacks, public BLECharacteristicCallbacks {
private:
  BLEServer* pServer;
  BLECharacteristic* pTxCharacteristic;
  BLECharacteristic* pRxCharacteristic;
  bool deviceConnected;
  String receivedData;
  bool hasNewData;

public:
  BleManager();
  void begin();
  
  // 상태 데이터 앱으로 전송 (TX)
  void sendData(String message);
  
  // 데이터 수신 여부 확인 및 가져오기 (RX)
  bool available();
  String readData();
  
  // BLE 연결 상태 반환
  bool isConnected() const { return deviceConnected; }

  // BLE 콜백 함수 구현
  void onConnect(BLEServer* pServer) override;
  void onDisconnect(BLEServer* pServer) override;
  void onWrite(BLECharacteristic* pCharacteristic) override;
};