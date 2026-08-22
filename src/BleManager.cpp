#include "BleManager.h"

BleManager::BleManager() {
  pServer = nullptr;
  pTxCharacteristic = nullptr;
  pRxCharacteristic = nullptr;
  deviceConnected = false;
  hasNewData = false;
  receivedData = "";
}

void BleManager::begin() {
  // BLE 장치 초기화
  BLEDevice::init(BLE_DEVICE_NAME);

  // BLE 서버 생성
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(this);

  // BLE 서비스 생성
  BLEService* pService = pServer->createService(SERVICE_UUID);

  // TX 특성 (ESP32 -> App)
  pTxCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID_TX,
    BLECharacteristic::PROPERTY_NOTIFY
  );
  pTxCharacteristic->addDescriptor(new BLE2902());

  // RX 특성 (App -> ESP32)
  pRxCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID_RX,
    BLECharacteristic::PROPERTY_WRITE
  );
  pRxCharacteristic->setCallbacks(this);

  // 서비스 시작 및 어드버타이징 시작
  pService->start();
  BLEAdvertising* pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06); // iOS 연결 안정화용
  pAdvertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising();

  Serial.println("[BLE] 어드버타이징 시작: 앱 연결 대기 중...");
}

void BleManager::onConnect(BLEServer* pServer) {
  deviceConnected = true;
  Serial.println("[BLE] 중앙 장치(앱) 연결됨");
}

void BleManager::onDisconnect(BLEServer* pServer) {
  deviceConnected = false;
  Serial.println("[BLE] 중앙 장치(앱) 연결 해제됨");
  
  // 재연결을 위한 어드버타이징 재시작
  pServer->startAdvertising();
  Serial.println("[BLE] 어드버타이징 재시작");
}

void BleManager::onWrite(BLECharacteristic* pCharacteristic) {
  std::string rxValue = pCharacteristic->getValue();

  if (rxValue.length() > 0) {
    receivedData = "";
    for (int i = 0; i < rxValue.length(); i++) {
      receivedData += rxValue[i];
    }
    hasNewData = true;
    Serial.printf("[BLE 수신]: %s\n", receivedData.c_str());
  }
}

void BleManager::sendData(String message) {
  if (deviceConnected && pTxCharacteristic != nullptr) {
    pTxCharacteristic->setValue(message.c_str());
    pTxCharacteristic->notify();
    Serial.printf("[BLE 송신]: %s\n", message.c_str());
  }
}

bool BleManager::available() {
  return hasNewData;
}

String BleManager::readData() {
  hasNewData = false;
  return receivedData;
}