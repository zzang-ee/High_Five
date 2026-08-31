#include <Arduino.h>

#include "RehabSystem.h"

namespace {
constexpr size_t SERIAL_TX_BUFFER_SIZE = 4096;
}

RehabSystem systemManager;

void setup() {
  // The app protocol emits several JSON frames at once. A TX ring buffer keeps
  // UART draining from delaying the 35 ms rehabilitation control loop.
  Serial.setTxBufferSize(SERIAL_TX_BUFFER_SIZE);
  systemManager.begin();
}

void loop() {
  systemManager.update();
}
