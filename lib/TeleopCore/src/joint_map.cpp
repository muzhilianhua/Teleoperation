#include "joint_map.h"

namespace teleop {

int32_t unwrapCounts(uint16_t raw, uint16_t ref) {
  int32_t d = static_cast<int32_t>(raw % kEncoderCounts) - static_cast<int32_t>(ref % kEncoderCounts);
  if (d >= kEncoderCounts / 2) d -= kEncoderCounts;
  if (d < -kEncoderCounts / 2) d += kEncoderCounts;
  return d;
}

float clampToTravel(const JointMap& joint, float servoCounts) {
  if (servoCounts < joint.servoMin) return joint.servoMin;
  if (servoCounts > joint.servoMax) return joint.servoMax;
  return servoCounts;
}

float mapToServo(const JointMap& joint, float exoCounts) {
  if (!joint.exoCalibrated()) return clampToTravel(joint, joint.servoA);
  float t = (exoCounts - joint.exoA) / static_cast<float>(joint.exoB - joint.exoA);
  if (t < 0.0f) t = 0.0f;
  if (t > 1.0f) t = 1.0f;
  const float out = joint.servoA + t * static_cast<float>(joint.servoB - joint.servoA);
  return clampToTravel(joint, out);
}

}  // namespace teleop
