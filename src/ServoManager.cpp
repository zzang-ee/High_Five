#include "ServoManager.h"

#include <math.h>

ServoManager::ServoManager()
    : activeMask(0),
      motionStartTimeMs(0),
      lastMotionUpdateTimeMs(0),
      motionMaxCommandStepDeg(SERVO_MAX_COMMAND_STEP_DEG),
      motionShape(TrajectoryShape::QUINTIC),
      motionActive(false) {
  for (uint8_t i = 0; i < FINGER_COUNT; ++i) {
    currentAngles[i] = SERVO_LOGICAL_HOME_ANGLE;
    maxAngles[i] = SERVO_LOGICAL_MAX_ANGLE;
    attached[i] = false;
    motionStartAngles[i] = SERVO_LOGICAL_HOME_ANGLE;
    motionTargetAngles[i] = SERVO_LOGICAL_HOME_ANGLE;
    motionDurationMs[i] = 0;
  }
}

bool ServoManager::begin() {
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);

  for (uint8_t i = 0; i < FINGER_COUNT; ++i) {
    servos[i].setPeriodHertz(50);
    attached[i] = false;
    currentAngles[i] = SERVO_LOGICAL_HOME_ANGLE;
    maxAngles[i] = SERVO_LOGICAL_MAX_ANGLE;
  }

  motionActive = false;
  activeMask = 0;
  motionMaxCommandStepDeg = SERVO_MAX_COMMAND_STEP_DEG;
  motionShape = TrajectoryShape::QUINTIC;
  return true;
}

void ServoManager::update(uint32_t nowMs) {
  if (!motionActive) {
    return;
  }
  if (static_cast<uint32_t>(nowMs - lastMotionUpdateTimeMs) <
      SERVO_COMMAND_INTERVAL_MS) {
    return;
  }
  lastMotionUpdateTimeMs = nowMs;

  const uint32_t elapsed = static_cast<uint32_t>(nowMs - motionStartTimeMs);
  bool trajectoryValid = true;
  for (uint8_t i = 0; i < FINGER_COUNT; ++i) {
    if ((activeMask & (1U << i)) != 0 &&
        !updateTrajectoryFinger(i, elapsed)) {
      trajectoryValid = false;
      break;
    }
  }

  if (!trajectoryValid || allActiveTargetsReached()) {
    motionActive = false;
    activeMask = 0;
  }
}

bool ServoManager::setAngle(uint8_t index, int logicalAngle) {
  if (index >= FINGER_COUNT ||
      (AVAILABLE_FINGERS_MASK & (1U << index)) == 0 || !attached[index]) {
    return false;
  }

  const int boundedAngle =
      constrain(logicalAngle, SERVO_LOGICAL_HOME_ANGLE, maxAngles[index]);
  servos[index].write(logicalToPhysical(index, boundedAngle));
  currentAngles[index] = boundedAngle;
  return true;
}

int ServoManager::getAngle(uint8_t index) const {
  return index < FINGER_COUNT ? currentAngles[index] : 0;
}

int ServoManager::getHomeAngleForFinger(uint8_t index) const {
  return index < FINGER_COUNT ? SERVO_LOGICAL_HOME_ANGLE : 0;
}

int ServoManager::getSoftwareEndAngleForFinger(uint8_t index) const {
  return index < FINGER_COUNT ? SERVO_LOGICAL_MAX_ANGLE : 0;
}

bool ServoManager::setMaxAngleForFinger(uint8_t index, int logicalAngle) {
  if (index >= FINGER_COUNT ||
      logicalAngle < SERVO_LOGICAL_HOME_ANGLE + ROM_MIN_TARGET_ANGLE ||
      logicalAngle > SERVO_LOGICAL_MAX_ANGLE) {
    return false;
  }

  maxAngles[index] = logicalAngle;
  if (currentAngles[index] > maxAngles[index]) {
    return setAngle(index, maxAngles[index]);
  }
  return true;
}

int ServoManager::getMaxAngleForFinger(uint8_t index) const {
  return index < FINGER_COUNT ? maxAngles[index] : 0;
}

bool ServoManager::startMoveFinger(uint8_t index, int targetAngle,
                                   uint32_t durationMs, uint32_t nowMs) {
  if (index >= FINGER_COUNT) {
    return false;
  }

  int targetAngles[FINGER_COUNT];
  uint32_t durations[FINGER_COUNT];
  for (uint8_t i = 0; i < FINGER_COUNT; ++i) {
    targetAngles[i] = currentAngles[i];
    durations[i] = durationMs;
  }
  targetAngles[index] = targetAngle;
  return startMoveFingers(static_cast<uint8_t>(1U << index), targetAngles,
                          durations, nowMs);
}

bool ServoManager::startMoveFingers(
    uint8_t mask, const int targetAngles[FINGER_COUNT],
    const uint32_t durationMs[FINGER_COUNT], uint32_t nowMs,
    uint8_t maxCommandStepDeg, TrajectoryShape shape) {
  if (motionActive || targetAngles == nullptr || durationMs == nullptr ||
      maxCommandStepDeg == 0) {
    return false;
  }

  mask &= ALL_FINGERS_MASK;
  if (mask == 0 || (mask & TEMP_DISABLED_FINGER_MASK) != 0) {
    return false;
  }

  // Validate the complete request before changing any trajectory state.
  for (uint8_t i = 0; i < FINGER_COUNT; ++i) {
    if ((mask & (1U << i)) == 0) {
      continue;
    }
    if (!attached[i] || durationMs[i] == 0 ||
        targetAngles[i] < SERVO_LOGICAL_HOME_ANGLE ||
        targetAngles[i] > maxAngles[i]) {
      return false;
    }
  }

  uint8_t movingMask = 0;
  for (uint8_t i = 0; i < FINGER_COUNT; ++i) {
    motionStartAngles[i] = currentAngles[i];
    motionTargetAngles[i] = currentAngles[i];
    motionDurationMs[i] = 0;
    if ((mask & (1U << i)) == 0) {
      continue;
    }

    motionTargetAngles[i] = targetAngles[i];
    motionDurationMs[i] = durationMs[i];
    if (motionTargetAngles[i] != motionStartAngles[i]) {
      movingMask |= static_cast<uint8_t>(1U << i);
    }
  }

  activeMask = movingMask;
  motionStartTimeMs = nowMs;
  lastMotionUpdateTimeMs = nowMs;
  motionMaxCommandStepDeg = maxCommandStepDeg;
  motionShape = shape;
  motionActive = movingMask != 0;
  return true;
}

bool ServoManager::startReturn(uint8_t mask, uint32_t durationMs,
                               uint32_t nowMs) {
  if (motionActive || durationMs == 0) {
    return false;
  }

  mask &= AVAILABLE_FINGERS_MASK;
  uint8_t movingMask = 0;
  int targetAngles[FINGER_COUNT];
  uint32_t durations[FINGER_COUNT];
  for (uint8_t i = 0; i < FINGER_COUNT; ++i) {
    targetAngles[i] = SERVO_LOGICAL_HOME_ANGLE;
    durations[i] = durationMs;
    if ((mask & (1U << i)) != 0 &&
        currentAngles[i] != SERVO_LOGICAL_HOME_ANGLE) {
      if (!attached[i]) {
        return false;
      }
      movingMask |= static_cast<uint8_t>(1U << i);
    }
  }

  if (movingMask == 0) {
    activeMask = 0;
    motionActive = false;
    return true;
  }
  return startMoveFingers(movingMask, targetAngles, durations, nowMs);
}

void ServoManager::stopMotion() {
  motionActive = false;
  activeMask = 0;
}

void ServoManager::detachAll() {
  stopMotion();
  for (uint8_t i = 0; i < FINGER_COUNT; ++i) {
    if (attached[i]) {
      servos[i].detach();
      attached[i] = false;
    }
  }
}

bool ServoManager::setEnabledMask(uint8_t mask) {
  mask &= ALL_FINGERS_MASK;
  if ((mask & TEMP_DISABLED_FINGER_MASK) != 0) {
    return false;
  }

  // Never remove PWM from a finger that still needs a controlled return.
  for (uint8_t i = 0; i < FINGER_COUNT; ++i) {
    if ((mask & (1U << i)) == 0 && attached[i] &&
        currentAngles[i] != SERVO_LOGICAL_HOME_ANGLE) {
      return false;
    }
  }

  for (uint8_t i = 0; i < FINGER_COUNT; ++i) {
    if ((mask & (1U << i)) == 0 && attached[i]) {
      servos[i].detach();
      attached[i] = false;
    }
  }

  for (uint8_t i = 0; i < FINGER_COUNT; ++i) {
    if ((mask & (1U << i)) != 0 && !attachFinger(i)) {
      return false;
    }
  }
  return true;
}

bool ServoManager::isAttached() const {
  for (uint8_t i = 0; i < FINGER_COUNT; ++i) {
    if ((AVAILABLE_FINGERS_MASK & (1U << i)) != 0 && !attached[i]) {
      return false;
    }
  }
  return true;
}

uint8_t ServoManager::getEnabledMask() const {
  uint8_t mask = 0;
  for (uint8_t i = 0; i < FINGER_COUNT; ++i) {
    if (attached[i]) {
      mask |= static_cast<uint8_t>(1U << i);
    }
  }
  return mask;
}

bool ServoManager::allAtHome(uint8_t mask) const {
  mask &= ALL_FINGERS_MASK;
  for (uint8_t i = 0; i < FINGER_COUNT; ++i) {
    if ((mask & (1U << i)) != 0 &&
        currentAngles[i] != SERVO_LOGICAL_HOME_ANGLE) {
      return false;
    }
  }
  return true;
}

int ServoManager::logicalToPhysical(uint8_t index, int logicalAngle) const {
  if (SERVO_REVERSED[index]) {
    return constrain(SERVO_PHYSICAL_MAX_ANGLE - logicalAngle,
                     SERVO_PHYSICAL_MIN_ANGLE,
                     SERVO_PHYSICAL_MAX_ANGLE);
  }
  return constrain(SERVO_PHYSICAL_MIN_ANGLE + logicalAngle,
                   SERVO_PHYSICAL_MIN_ANGLE,
                   SERVO_PHYSICAL_MAX_ANGLE);
}

bool ServoManager::attachFinger(uint8_t index) {
  if (index >= FINGER_COUNT ||
      (AVAILABLE_FINGERS_MASK & (1U << index)) == 0) {
    return false;
  }
  if (attached[index]) {
    return true;
  }

  servos[index].setPeriodHertz(50);
  servos[index].attach(SERVO_PINS[index], SERVO_MIN_PULSE_US,
                       SERVO_MAX_PULSE_US);
  attached[index] = servos[index].attached();
  if (!attached[index]) {
    return false;
  }

  servos[index].write(logicalToPhysical(index, currentAngles[index]));
  return true;
}

bool ServoManager::updateTrajectoryFinger(uint8_t index, uint32_t elapsed) {
  if (index >= FINGER_COUNT || (activeMask & (1U << index)) == 0 ||
      motionDurationMs[index] == 0) {
    return false;
  }

  float progress = 1.0f;
  if (elapsed < motionDurationMs[index]) {
    progress = static_cast<float>(elapsed) /
               static_cast<float>(motionDurationMs[index]);
  }
  const float shapedProgress =
      motionShape == TrajectoryShape::TRAINING_EASE_OUT
          ? trainingEaseOut(progress)
          : quinticSmoothStep(progress);
  const float interpolated =
      static_cast<float>(motionStartAngles[index]) +
      static_cast<float>(motionTargetAngles[index] -
                         motionStartAngles[index]) *
          shapedProgress;
  int desiredAngle = static_cast<int>(lroundf(interpolated));
  const int delta = desiredAngle - currentAngles[index];
  if (delta > static_cast<int>(motionMaxCommandStepDeg)) {
    desiredAngle = currentAngles[index] + motionMaxCommandStepDeg;
  } else if (delta < -static_cast<int>(motionMaxCommandStepDeg)) {
    desiredAngle = currentAngles[index] - motionMaxCommandStepDeg;
  }

  if (desiredAngle == currentAngles[index]) {
    return true;
  }
  if (!setAngle(index, desiredAngle)) {
    return false;
  }

  return true;
}

bool ServoManager::allActiveTargetsReached() const {
  for (uint8_t i = 0; i < FINGER_COUNT; ++i) {
    if ((activeMask & (1U << i)) != 0 &&
        currentAngles[i] != motionTargetAngles[i]) {
      return false;
    }
  }
  return true;
}

float ServoManager::quinticSmoothStep(float progress) {
  if (progress <= 0.0f) {
    return 0.0f;
  }
  if (progress >= 1.0f) {
    return 1.0f;
  }

  const float p2 = progress * progress;
  const float p3 = p2 * progress;
  return p3 * (10.0f + progress * (-15.0f + 6.0f * progress));
}

float ServoManager::trainingEaseOut(float progress) {
  if (progress <= 0.0f) {
    return 0.0f;
  }
  if (progress >= 1.0f) {
    return 1.0f;
  }

  // Integral of 12*p*(1-p)^2: zero speed at both ends, peak speed at 1/3
  // of the time, and progressively slower movement toward the target.
  const float p2 = progress * progress;
  return p2 * (6.0f + progress * (-8.0f + 3.0f * progress));
}
