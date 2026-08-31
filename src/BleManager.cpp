#include "BleManager.h"
#include <cctype>
#include <cstring>

namespace {
uint8_t safetyPriority(BleSafetyCommand command) {
  switch (command) {
    case BleSafetyCommand::EMERGENCY:
      return 3;
    case BleSafetyCommand::DISCONNECT:
      return 2;
    case BleSafetyCommand::STOP:
      return 1;
    case BleSafetyCommand::NONE:
    default:
      return 0;
  }
}

bool latchSafetyCommand(QueueHandle_t queue, BleSafetyCommand command) {
  if (queue == nullptr || command == BleSafetyCommand::NONE) {
    return false;
  }

  BleSafetyCommand pending = BleSafetyCommand::NONE;
  if (xQueuePeek(queue, &pending, 0) == pdPASS &&
      safetyPriority(pending) >= safetyPriority(command)) {
    return true;
  }

  return xQueueOverwrite(queue, &command) == pdPASS;
}
}  // namespace

BleManager::BleManager() {
  pServer = nullptr;
  pTxCharacteristic = nullptr;
  pRxCharacteristic = nullptr;
  pSecurity = nullptr;
  deviceConnected = false;
  negotiatedMtu = 23;
  commandQueue = nullptr;
  safetyQueue = nullptr;
}

void BleManager::begin() {
  negotiatedMtu = 23;
  commandQueue = xQueueCreate(BLE_COMMAND_QUEUE_LENGTH, sizeof(CommandMessage));
  safetyQueue = xQueueCreate(1, sizeof(BleSafetyCommand));

  if (commandQueue == nullptr || safetyQueue == nullptr) {
    Serial.println("[BLE] Failed to create command queues; BLE will not start.");
    if (commandQueue != nullptr) {
      vQueueDelete(commandQueue);
      commandQueue = nullptr;
    }
    if (safetyQueue != nullptr) {
      vQueueDelete(safetyQueue);
      safetyQueue = nullptr;
    }
    return;
  }

  // BLE 장치 초기화
  BLEDevice::init(BLE_DEVICE_NAME);
  if (BLEDevice::setMTU(BLE_LOCAL_MTU) != ESP_OK) {
    Serial.println("[BLE] Failed to set the local MTU.");
  }

  // The app must pair with the static PIN before accessing the characteristics.
  pSecurity = new BLESecurity();
  if (pSecurity == nullptr) {
    Serial.println("[BLE] Failed to create security context; BLE will not start.");
    vQueueDelete(commandQueue);
    vQueueDelete(safetyQueue);
    commandQueue = nullptr;
    safetyQueue = nullptr;
    return;
  }
  pSecurity->setStaticPIN(BLE_STATIC_PASSKEY);
  pSecurity->setAuthenticationMode(ESP_LE_AUTH_REQ_SC_MITM_BOND);

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
  pTxCharacteristic->setAccessPermissions(ESP_GATT_PERM_READ_ENC_MITM);

  BLE2902* pCccd = new BLE2902();
  pCccd->setAccessPermissions(
    static_cast<esp_gatt_perm_t>(ESP_GATT_PERM_READ_ENC_MITM |
                                 ESP_GATT_PERM_WRITE_ENC_MITM)
  );
  pTxCharacteristic->addDescriptor(pCccd);

  // RX 특성 (App -> ESP32)
  pRxCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID_RX,
    BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR
  );
  pRxCharacteristic->setAccessPermissions(ESP_GATT_PERM_WRITE_ENC_MITM);
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
  negotiatedMtu = 23;
  Serial.println("[BLE] 중앙 장치(앱) 연결됨");
}

void BleManager::onDisconnect(BLEServer* pServer) {
  deviceConnected = false;
  negotiatedMtu = 23;
  Serial.println("[BLE] 중앙 장치(앱) 연결 해제됨");

  // Commands from the old link must never execute after a reconnect. A
  // disconnect is also a safety event, but it must not replace an emergency
  // that is already latched.
  if (commandQueue != nullptr) {
    xQueueReset(commandQueue);
  }
  if (safetyQueue != nullptr) {
    const BleSafetyCommand disconnect = BleSafetyCommand::DISCONNECT;
    if (!latchSafetyCommand(safetyQueue, disconnect)) {
      Serial.println("[BLE] Failed to latch disconnect safety event.");
    }
  }
  
  // 재연결을 위한 어드버타이징 재시작
  pServer->startAdvertising();
  Serial.println("[BLE] 어드버타이징 재시작");
}

void BleManager::onMtuChanged(BLEServer* pServer,
                              esp_ble_gatts_cb_param_t* param) {
  (void)pServer;
  if (param == nullptr) {
    return;
  }
  negotiatedMtu = param->mtu.mtu;
  Serial.printf("[BLE] Negotiated MTU: %u\n",
                static_cast<unsigned>(negotiatedMtu));
}

void BleManager::onWrite(BLECharacteristic* pCharacteristic) {
  if (commandQueue == nullptr || safetyQueue == nullptr) {
    return;
  }

  const std::string rxValue = pCharacteristic->getValue();
  size_t first = 0;
  size_t last = rxValue.size();

  while (first < last && std::isspace(static_cast<unsigned char>(rxValue[first]))) {
    ++first;
  }
  while (last > first && std::isspace(static_cast<unsigned char>(rxValue[last - 1]))) {
    --last;
  }

  const size_t commandLength = last - first;
  if (commandLength == 0) {
    return;
  }

  if (commandLength > BLE_MAX_COMMAND_LENGTH) {
    Serial.printf(
      "[BLE] Command too long; discarded: %u bytes\n",
      static_cast<unsigned>(commandLength)
    );
    return;
  }

  // The protocol is deliberately ASCII-only. Reject embedded NULs and other
  // control bytes so C-string parsing cannot see a different command than the
  // received BLE payload.
  for (size_t i = first; i < last; ++i) {
    const unsigned char byte = static_cast<unsigned char>(rxValue[i]);
    if (byte < 0x20U || byte > 0x7EU) {
      Serial.println("[BLE] Non-ASCII/control byte in command; discarded.");
      return;
    }
  }

  CommandMessage message{};
  std::memcpy(message.data, rxValue.data() + first, commandLength);
  message.data[commandLength] = '\0';

  BleSafetyCommand safetyCommand = BleSafetyCommand::NONE;
  if (std::strcmp(message.data, "STOP") == 0) {
    safetyCommand = BleSafetyCommand::STOP;
  } else if (std::strcmp(message.data, "EMERGENCY") == 0) {
    safetyCommand = BleSafetyCommand::EMERGENCY;
  }

  if (safetyCommand != BleSafetyCommand::NONE) {
    xQueueReset(commandQueue);
    if (!latchSafetyCommand(safetyQueue, safetyCommand)) {
      Serial.println("[BLE] Failed to latch safety command.");
      return;
    }
    Serial.printf("[BLE safety command]: %s\n", message.data);
    return;
  }

  if (xQueueSend(commandQueue, &message, 0) != pdPASS) {
    CommandMessage droppedMessage{};
    xQueueReceive(commandQueue, &droppedMessage, 0);
    if (xQueueSend(commandQueue, &message, 0) != pdPASS) {
      Serial.println("[BLE] Failed to enqueue command.");
      return;
    }
  }

  Serial.printf("[BLE 수신]: %s\n", message.data);
}

void BleManager::sendData(String message) {
  if (deviceConnected && pTxCharacteristic != nullptr) {
    const uint16_t mtu = negotiatedMtu;
    const size_t maximumPayload = mtu > 3U ? mtu - 3U : 0U;
    if (message.length() > maximumPayload) {
      static const char mtuError[] = "{\"error\":\"MTU\"}";
      if (maximumPayload >= sizeof(mtuError) - 1U) {
        pTxCharacteristic->setValue(mtuError);
        pTxCharacteristic->notify();
      }
      Serial.printf("[BLE] TX frame exceeds negotiated MTU (%u): %s\n",
                    static_cast<unsigned>(mtu), message.c_str());
      return;
    }
    pTxCharacteristic->setValue(message.c_str());
    pTxCharacteristic->notify();
    Serial.printf("[BLE 송신]: %s\n", message.c_str());
  }
}

bool BleManager::available() {
  return commandQueue != nullptr && uxQueueMessagesWaiting(commandQueue) > 0;
}

String BleManager::readData() {
  if (commandQueue == nullptr) {
    return String();
  }

  CommandMessage message{};
  if (xQueueReceive(commandQueue, &message, 0) != pdPASS) {
    return String();
  }

  return String(message.data);
}

BleSafetyCommand BleManager::consumeSafetyCommand() {
  if (safetyQueue == nullptr) {
    return BleSafetyCommand::NONE;
  }

  BleSafetyCommand command = BleSafetyCommand::NONE;
  if (xQueueReceive(safetyQueue, &command, 0) != pdPASS) {
    return BleSafetyCommand::NONE;
  }

  return command;
}
