// Rejects encoder samples that imply a physically impossible exoskeleton
// motion between two control cycles (magnet slip, bus glitch, bad unwrap).
//
// After `resyncAfter` consecutive rejections the new value is taken as the
// baseline (Resynced): the jump was real or persistent, and the caller treats
// it as an encoder fault so the joint goes through HOLD -> ALIGN instead of
// snapping to the new position at full speed.

#pragma once

#include <math.h>
#include <stdint.h>

namespace teleop {

class JumpGuard {
 public:
  enum class Result : uint8_t { Accepted, Rejected, Resynced };

  explicit JumpGuard(float maxStep = 0.0f, uint8_t resyncAfter = 3)
      : maxStep_(maxStep), resyncAfter_(resyncAfter) {}

  void setMaxStep(float maxStep) { maxStep_ = maxStep; }
  float maxStep() const { return maxStep_; }
  void reset() {
    hasLast_ = false;
    rejects_ = 0;
  }

  Result check(float value) {
    if (!hasLast_ || maxStep_ <= 0.0f || fabsf(value - last_) <= maxStep_) {
      last_ = value;
      hasLast_ = true;
      rejects_ = 0;
      return Result::Accepted;
    }
    if (++rejects_ >= resyncAfter_) {
      last_ = value;
      rejects_ = 0;
      return Result::Resynced;
    }
    return Result::Rejected;
  }

 private:
  float maxStep_;
  uint8_t resyncAfter_;
  bool hasLast_ = false;
  uint8_t rejects_ = 0;
  float last_ = 0.0f;
};

}  // namespace teleop
