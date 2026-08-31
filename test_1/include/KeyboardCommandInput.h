#pragma once

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>

#include "Config.h"

// These values preserve the safety-command interface used by RehabSystem.
enum class BleSafetyCommand : uint8_t {
  NONE,
  STOP,
  DISCONNECT,
  EMERGENCY
};

class KeyboardCommandInput {
 public:
  KeyboardCommandInput();

  void begin();
  void sendData(const String& message);
  bool available();
  String readData();
  BleSafetyCommand consumeSafetyCommand();

  // The serial link is presented as an always-connected app transport. This
  // keeps every command inside the existing v8 parser and telemetry path.
  bool isConnected() const { return inputReady; }
  bool hasRequiredMtu() const { return negotiatedMtu >= BLE_LOCAL_MTU; }
  uint16_t getNegotiatedMtu() const { return negotiatedMtu; }

 private:
  struct CommandMessage {
    char data[BLE_MAX_COMMAND_LENGTH + 1];
  };

  struct OutputMessage {
    char data[BLE_LOCAL_MTU - 2];
  };

  QueueHandle_t commandQueue;
  QueueHandle_t safetyQueue;
  QueueHandle_t outputQueue;
  TaskHandle_t outputTask;
  bool inputReady;
  uint16_t negotiatedMtu;

  char inputBuffer[BLE_MAX_COMMAND_LENGTH + 1];
  size_t inputLength;
  bool inputOverflow;
  bool inputInvalid;

  void pumpInput();
  void finishInputLine();
  void resetInputLine();
  void enqueueCommand(const char* command);
  bool latchSafetyCommand(BleSafetyCommand command);
  static uint8_t safetyPriority(BleSafetyCommand command);
  static void outputTaskEntry(void* context);
};
