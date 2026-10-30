#include "hls_offset.h"

namespace teleop {

namespace {

int16_t wrapPosition(int32_t position, int32_t turn) {
  int32_t p = position % turn;
  if (p < 0) p += turn;
  return static_cast<int16_t>(p);
}

}  // namespace

int16_t foldOffset(int32_t offset) {
  const int32_t turn = feetech::kHlsMaxPositionOffset + 1;
  while (offset > feetech::kHlsMaxPositionOffset) offset -= turn;
  while (offset < -feetech::kHlsMaxPositionOffset) offset += turn;
  return static_cast<int16_t>(offset);
}

int16_t centerOffset(int16_t offset, int16_t position) {
  return foldOffset(static_cast<int32_t>(offset) + kZeroCenter - position);
}

int16_t mirroredOffset(int16_t offset, int16_t written) {
  return foldOffset(2 * static_cast<int32_t>(offset) - written);
}

bool shiftForOffset(JointSettings& s, int32_t delta, int16_t positionMax) {
  const int32_t turn = static_cast<int32_t>(positionMax) + 1;
  const bool calibrated = s.servoA != s.servoB;
  if (travelDefined(s)) {
    const int16_t lo = wrapPosition(s.travelMin + delta, turn);
    const int16_t hi = wrapPosition(s.travelMax + delta, turn);
    if (lo >= hi) {
      s.travelMin = s.travelMax = 0;
      s.servoA = s.servoB = 0;
      s.enabled = false;
      return false;
    }
    s.travelMin = lo;
    s.travelMax = hi;
  }
  if (calibrated) {
    s.servoA = wrapPosition(s.servoA + delta, turn);
    s.servoB = wrapPosition(s.servoB + delta, turn);
  } else {
    s.servoA = s.servoB = s.travelMin;
  }
  return true;
}

}  // namespace teleop
