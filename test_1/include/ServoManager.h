#pragma once

#include <ESP32Servo.h>
#include "Config.h"

class ServoManager {
public:
  ServoManager();

  bool begin();
  void update(uint32_t nowMs);

  bool setAngle(uint8_t index, int logicalAngle);
  int getAngle(uint8_t index) const;
  int getHomeAngleForFinger(uint8_t index) const;
  int getSoftwareEndAngleForFinger(uint8_t index) const;

  bool setMaxAngleForFinger(uint8_t index, int logicalAngle);
  int getMaxAngleForFinger(uint8_t index) const;

  // Starts one bounded, non-blocking quintic S-curve trajectory.
  bool startMoveFinger(uint8_t index, int targetAngle,
                       uint32_t durationMs, uint32_t nowMs);

  // Starts a coordinated multi-finger trajectory. Every selected finger is
  // updated in the same scheduler pass from one shared trajectory clock.
  bool startMoveFingers(uint8_t mask,
                        const int targetAngles[FINGER_COUNT],
                        const uint32_t durationMs[FINGER_COUNT],
                        uint32_t nowMs,
                        uint8_t maxCommandStepDeg =
                            SERVO_MAX_COMMAND_STEP_DEG);

  // Returns every finger in mask from its commanded position to home.
  bool startReturn(uint8_t mask, uint32_t durationMs, uint32_t nowMs);

  void stopMotion();
  void detachAll();
  bool setEnabledMask(uint8_t mask);
  bool isBusy() const { return motionActive; }
  bool isAttached() const;
  uint8_t getEnabledMask() const;
  bool allAtHome(uint8_t mask = ALL_FINGERS_MASK) const;
  uint8_t getActiveMask() const { return activeMask; }

private:
  Servo servos[FINGER_COUNT];
  int currentAngles[FINGER_COUNT];
  int maxAngles[FINGER_COUNT];
  bool attached[FINGER_COUNT];

  int motionStartAngles[FINGER_COUNT];
  int motionTargetAngles[FINGER_COUNT];
  uint32_t motionDurationMs[FINGER_COUNT];
  uint8_t activeMask;
  uint32_t motionStartTimeMs;
  uint32_t lastMotionUpdateTimeMs;
  uint8_t motionMaxCommandStepDeg;
  bool motionActive;

  int logicalToPhysical(uint8_t index, int logicalAngle) const;
  bool attachFinger(uint8_t index);
  bool updateTrajectoryFinger(uint8_t index, uint32_t elapsed);
  bool allActiveTargetsReached() const;
  static float quinticSmoothStep(float progress);
};
