#include "KeyboardCommandInput.h"

#include <ctype.h>
#include <string.h>

namespace {
constexpr uint8_t OUTPUT_QUEUE_LENGTH = 24;
constexpr uint16_t OUTPUT_TASK_STACK = 3072;
constexpr UBaseType_t OUTPUT_TASK_PRIORITY = 1;
constexpr BaseType_t OUTPUT_TASK_CORE = 0;
}  // namespace

KeyboardCommandInput::KeyboardCommandInput()
    : commandQueue(nullptr),
      safetyQueue(nullptr),
      outputQueue(nullptr),
      outputTask(nullptr),
      inputReady(false),
      negotiatedMtu(BLE_LOCAL_MTU),
      inputBuffer{},
      inputLength(0),
      inputOverflow(false),
      inputInvalid(false) {}

void KeyboardCommandInput::begin() {
  commandQueue =
      xQueueCreate(BLE_COMMAND_QUEUE_LENGTH, sizeof(CommandMessage));
  safetyQueue = xQueueCreate(1, sizeof(BleSafetyCommand));
  outputQueue = xQueueCreate(OUTPUT_QUEUE_LENGTH, sizeof(OutputMessage));

  if (commandQueue == nullptr || safetyQueue == nullptr ||
      outputQueue == nullptr) {
    Serial.println("[KEYBOARD] Failed to create command queues.");
    if (commandQueue != nullptr) {
      vQueueDelete(commandQueue);
      commandQueue = nullptr;
    }
    if (safetyQueue != nullptr) {
      vQueueDelete(safetyQueue);
      safetyQueue = nullptr;
    }
    if (outputQueue != nullptr) {
      vQueueDelete(outputQueue);
      outputQueue = nullptr;
    }
    return;
  }

  const BaseType_t taskCreated = xTaskCreatePinnedToCore(
      outputTaskEntry, "serial_app_tx", OUTPUT_TASK_STACK, this,
      OUTPUT_TASK_PRIORITY, &outputTask, OUTPUT_TASK_CORE);
  if (taskCreated != pdPASS) {
    Serial.println("[KEYBOARD] Failed to start the output task.");
    vQueueDelete(commandQueue);
    vQueueDelete(safetyQueue);
    vQueueDelete(outputQueue);
    commandQueue = nullptr;
    safetyQueue = nullptr;
    outputQueue = nullptr;
    outputTask = nullptr;
    return;
  }

  negotiatedMtu = BLE_LOCAL_MTU;
  inputReady = true;
  Serial.println(
      "[KEYBOARD] Ready. Enter one exact app command per line at 115200 baud.");
}

void KeyboardCommandInput::sendData(const String& message) {
  if (!inputReady || outputQueue == nullptr) {
    return;
  }

  const size_t maximumPayload =
      negotiatedMtu > 3U ? negotiatedMtu - 3U : 0U;
  if (message.length() > maximumPayload ||
      message.length() >= sizeof(OutputMessage::data)) {
    static const char mtuError[] = "{\"error\":\"MTU\"}";
    OutputMessage errorMessage{};
    memcpy(errorMessage.data, mtuError, sizeof(mtuError));
    xQueueSend(outputQueue, &errorMessage, 0);
    return;
  }

  OutputMessage outgoing{};
  memcpy(outgoing.data, message.c_str(), message.length());
  outgoing.data[message.length()] = '\0';

  if (xQueueSend(outputQueue, &outgoing, 0) != pdPASS) {
    // Prefer the newest state/safety frame when the terminal is not draining.
    OutputMessage dropped{};
    xQueueReceive(outputQueue, &dropped, 0);
    xQueueSend(outputQueue, &outgoing, 0);
  }
}

bool KeyboardCommandInput::available() {
  pumpInput();
  return commandQueue != nullptr &&
         uxQueueMessagesWaiting(commandQueue) > 0;
}

String KeyboardCommandInput::readData() {
  pumpInput();
  if (commandQueue == nullptr) {
    return String();
  }

  CommandMessage message{};
  if (xQueueReceive(commandQueue, &message, 0) != pdPASS) {
    return String();
  }
  return String(message.data);
}

BleSafetyCommand KeyboardCommandInput::consumeSafetyCommand() {
  pumpInput();
  if (safetyQueue == nullptr) {
    return BleSafetyCommand::NONE;
  }

  BleSafetyCommand command = BleSafetyCommand::NONE;
  if (xQueueReceive(safetyQueue, &command, 0) != pdPASS) {
    return BleSafetyCommand::NONE;
  }
  return command;
}

void KeyboardCommandInput::pumpInput() {
  if (!inputReady) {
    return;
  }

  while (Serial.available() > 0) {
    const int raw = Serial.read();
    if (raw < 0) {
      return;
    }

    const unsigned char byte = static_cast<unsigned char>(raw);
    if (byte == '\r' || byte == '\n') {
      finishInputLine();
      continue;
    }

    if (inputOverflow || inputInvalid) {
      continue;
    }
    if (byte < 0x20U || byte > 0x7EU) {
      inputInvalid = true;
      continue;
    }
    if (inputLength >= BLE_MAX_COMMAND_LENGTH) {
      inputOverflow = true;
      continue;
    }

    inputBuffer[inputLength++] = static_cast<char>(byte);
    inputBuffer[inputLength] = '\0';
  }
}

void KeyboardCommandInput::finishInputLine() {
  if (inputOverflow) {
    Serial.println("[KEYBOARD] Command too long; discarded.");
    resetInputLine();
    return;
  }
  if (inputInvalid) {
    Serial.println("[KEYBOARD] Non-ASCII/control byte; command discarded.");
    resetInputLine();
    return;
  }

  size_t first = 0;
  size_t last = inputLength;
  while (first < last && inputBuffer[first] == ' ') {
    ++first;
  }
  while (last > first && inputBuffer[last - 1] == ' ') {
    --last;
  }

  const size_t commandLength = last - first;
  if (commandLength > 0) {
    char command[BLE_MAX_COMMAND_LENGTH + 1]{};
    memcpy(command, inputBuffer + first, commandLength);
    command[commandLength] = '\0';
    enqueueCommand(command);
  }
  resetInputLine();
}

void KeyboardCommandInput::resetInputLine() {
  inputLength = 0;
  inputOverflow = false;
  inputInvalid = false;
  inputBuffer[0] = '\0';
}

void KeyboardCommandInput::enqueueCommand(const char* command) {
  if (command == nullptr || commandQueue == nullptr ||
      safetyQueue == nullptr) {
    return;
  }

  CommandMessage message{};
  strncpy(message.data, command, BLE_MAX_COMMAND_LENGTH);
  message.data[BLE_MAX_COMMAND_LENGTH] = '\0';

  BleSafetyCommand safetyCommand = BleSafetyCommand::NONE;
  if (strcmp(message.data, "STOP") == 0) {
    safetyCommand = BleSafetyCommand::STOP;
  } else if (strcmp(message.data, "EMERGENCY") == 0) {
    safetyCommand = BleSafetyCommand::EMERGENCY;
  }

  if (safetyCommand != BleSafetyCommand::NONE) {
    xQueueReset(commandQueue);
    latchSafetyCommand(safetyCommand);
    Serial.printf("[KEYBOARD APP_RX SAFETY] %s\n", message.data);
    return;
  }

  if (xQueueSend(commandQueue, &message, 0) != pdPASS) {
    CommandMessage dropped{};
    xQueueReceive(commandQueue, &dropped, 0);
    if (xQueueSend(commandQueue, &message, 0) != pdPASS) {
      Serial.println("[KEYBOARD] Failed to enqueue command.");
      return;
    }
  }
  Serial.printf("[KEYBOARD APP_RX] %s\n", message.data);
}

bool KeyboardCommandInput::latchSafetyCommand(
    BleSafetyCommand command) {
  if (safetyQueue == nullptr || command == BleSafetyCommand::NONE) {
    return false;
  }

  BleSafetyCommand pending = BleSafetyCommand::NONE;
  if (xQueuePeek(safetyQueue, &pending, 0) == pdPASS &&
      safetyPriority(pending) >= safetyPriority(command)) {
    return true;
  }
  return xQueueOverwrite(safetyQueue, &command) == pdPASS;
}

uint8_t KeyboardCommandInput::safetyPriority(
    BleSafetyCommand command) {
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

void KeyboardCommandInput::outputTaskEntry(void* context) {
  auto* input = static_cast<KeyboardCommandInput*>(context);
  OutputMessage message{};
  for (;;) {
    if (xQueueReceive(input->outputQueue, &message, portMAX_DELAY) ==
        pdPASS) {
      // Write the JSON and line ending under one UART lock so diagnostics from
      // the control task cannot split an app notification across lines.
      const size_t payloadLength = strnlen(message.data, sizeof(message.data));
      char frame[BLE_LOCAL_MTU]{};
      memcpy(frame, message.data, payloadLength);
      frame[payloadLength] = '\r';
      frame[payloadLength + 1] = '\n';
      Serial.write(reinterpret_cast<const uint8_t*>(frame),
                   payloadLength + 2);
    }
  }
}
