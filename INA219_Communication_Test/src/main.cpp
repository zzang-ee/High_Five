#include <Arduino.h>
#include <Wire.h>

namespace {
constexpr uint8_t I2C_SDA_PIN = 23;
constexpr uint8_t I2C_SCL_PIN = 22;
constexpr uint32_t I2C_FREQUENCY_HZ = 100000;
constexpr uint8_t SENSOR_I2C_ADDRESS = 0x40;
constexpr uint32_t CHECK_INTERVAL_MS = 2000;

uint32_t lastCheckMs = 0;
bool lastCommunicationOk = false;

bool addressResponds(uint8_t address) {
  Wire.beginTransmission(address);
  return Wire.endTransmission() == 0;
}

bool checkIna219Communication() {
  if (!addressResponds(SENSOR_I2C_ADDRESS)) {
    Serial.println("INA219_COMM,FAIL,address=0x40,reason=NO_I2C_ACK");
    return false;
  }
  Serial.println("INA219_COMM,OK,address=0x40");
  return true;
}
}  // namespace

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println();
  Serial.println("INA219_COMMUNICATION_ONLY_TEST_V1");
  Serial.printf("I2C_CONFIG,SDA=%u,SCL=%u,FREQ_HZ=%lu\n", I2C_SDA_PIN,
                I2C_SCL_PIN,
                static_cast<unsigned long>(I2C_FREQUENCY_HZ));

  if (!Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN, I2C_FREQUENCY_HZ)) {
    Serial.println("I2C_BUS,FAIL");
    return;
  }

  Serial.println("I2C_BUS,OK");
  lastCommunicationOk = checkIna219Communication();
  lastCheckMs = millis();
}

void loop() {
  const uint32_t nowMs = millis();
  if (static_cast<uint32_t>(nowMs - lastCheckMs) < CHECK_INTERVAL_MS) {
    return;
  }
  lastCheckMs = nowMs;

  const bool communicationOk = checkIna219Communication();
  if (communicationOk != lastCommunicationOk) {
    Serial.printf("INA219_COMM_CHANGED,%s\n",
                  communicationOk ? "CONNECTED" : "DISCONNECTED");
  }
  lastCommunicationOk = communicationOk;
}
