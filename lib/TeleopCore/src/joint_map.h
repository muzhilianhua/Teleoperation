// Exoskeleton encoder -> servo position mapping (two-point linear calibration).
//
// The table is indexed by servo: every servo names the encoder that drives it,
// so spare encoders (9 encoders, 8 servos) are simply not referenced.

#pragma once

#include <stdint.h>

namespace teleop {

constexpr int32_t kEncoderCounts = 16384;  // AS5048B, 14 bit per turn
constexpr float kDegreesPerCount = 360.0f / kEncoderCounts;

// Signed distance from ref in counts, folded into [-8192, 8191].
// Valid as long as the joint travels less than half a turn around ref.
int32_t unwrapCounts(uint16_t raw, uint16_t ref);

struct JointMap {
  const char* name = "";
  uint8_t encoderIndex = 0;
  uint8_t servoId = 0;
  // Operator side: exoRef is the raw reading at endpoint A; exoA/exoB are
  // continuous counts relative to exoRef, possibly multi-turn (exoA is 0).
  // Input positions come from ExoTracker, not from unwrapCounts().
  uint16_t exoRef = 0;
  int32_t exoA = 0;
  int32_t exoB = 0;
  // Robot side: servo counts that correspond to exoA / exoB.
  int16_t servoA = 0;
  int16_t servoB = 0;
  // Hard travel limits in servo counts; every output is clamped to them.
  int16_t servoMin = 0;
  int16_t servoMax = 0;

  bool exoCalibrated() const { return exoA != exoB; }
  bool servoRangeValid() const {
    return servoMin < servoMax && servoA >= servoMin && servoA <= servoMax &&
           servoB >= servoMin && servoB <= servoMax && servoA != servoB;
  }
  bool ready() const { return exoCalibrated() && servoRangeValid(); }
};

float clampToTravel(const JointMap& joint, float servoCounts);
// exoCounts: unwrapped (relative to exoRef) and filtered, may be fractional.
// Outside [exoA, exoB] the output saturates at servoA / servoB.
float mapToServo(const JointMap& joint, float exoCounts);

}  // namespace teleop
