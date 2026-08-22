#include <Arduino.h>
#include "RehabSystem.h"

RehabSystem systemManager;

void setup() {
  systemManager.begin();
}

void loop() {
  systemManager.update();
}