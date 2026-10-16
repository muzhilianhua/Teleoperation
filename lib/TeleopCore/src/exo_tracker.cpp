#include "exo_tracker.h"

#include <stdlib.h>

#include "joint_map.h"

namespace teleop {

namespace {

constexpr uint8_t kLostAfterRejects = 3;
constexpr int32_t kMaxTurnsSearched = 4;

}  // namespace

void ExoTracker::configure(uint16_t exoRef, int32_t exoB, int32_t deadZoneCounts) {
  ref_ = exoRef;
  exoB_ = exoB;
  deadZone_ = deadZoneCounts < 0 ? 0 : deadZoneCounts;
  invalidate();
}

bool ExoTracker::singleTurn() const {
  return calibrated() && labs(exoB_) + 2 * deadZone_ < kEncoderCounts;
}

int32_t ExoTracker::rangeLo() const { return (exoB_ < 0 ? exoB_ : 0) - deadZone_; }
int32_t ExoTracker::rangeHi() const { return (exoB_ > 0 ? exoB_ : 0) + deadZone_; }

void ExoTracker::invalidate() {
  homed_ = false;
  continuous_ = false;
  rejects_ = 0;
}

void ExoTracker::accept(int32_t x, uint16_t raw, uint32_t nowMs) {
  x_ = x;
  lastRaw_ = raw;
  lastSampleMs_ = nowMs;
  rejects_ = 0;
  homed_ = true;
}

bool ExoTracker::resolve(uint16_t raw, uint32_t nowMs) {
  const int32_t d = unwrapCounts(raw, ref_);
  if (!calibrated()) {
    // Before `cal b` there is no range: nearest to A is the only sensible guess.
    accept(d, raw, nowMs);
    continuous_ = false;
    return true;
  }
  if (!singleTurn()) return false;  // ambiguous: needs home()
  int32_t found = 0;
  uint8_t count = 0;
  for (int32_t k = -kMaxTurnsSearched; k <= kMaxTurnsSearched; ++k) {
    const int32_t c = d + k * kEncoderCounts;
    if (c >= rangeLo() && c <= rangeHi()) {
      found = c;
      ++count;
    }
  }
  if (count != 1) return false;  // in the unused arc (or impossible overlap)
  accept(found, raw, nowMs);
  continuous_ = false;
  return true;
}

ExoTracker::Result ExoTracker::update(uint16_t raw, uint32_t nowMs) {
  if (homed_ && nowMs - lastSampleMs_ > lossMs_) invalidate();
  if (!homed_) return resolve(raw, nowMs) ? Result::Accepted : Result::Unhomed;

  const int32_t delta = unwrapCounts(raw, lastRaw_);
  if (maxStep_ > 0 && labs(delta) > maxStep_) {
    if (++rejects_ >= kLostAfterRejects) {
      invalidate();
      return Result::Lost;
    }
    return Result::Rejected;
  }
  accept(x_ + delta, raw, nowMs);
  return Result::Accepted;
}

void ExoTracker::noSample(uint32_t nowMs) {
  if (homed_ && nowMs - lastSampleMs_ > lossMs_) invalidate();
}

const char* ExoTracker::home(uint16_t raw, uint32_t nowMs) {
  if (!calibrated()) return "joint not calibrated (cal <j> a / b)";
  const int32_t c = unwrapCounts(raw, ref_);  // the turn nearest A
  if (c < rangeLo() || c > rangeHi()) return "position outside the calibrated range: is the glove at pose A?";
  accept(c, raw, nowMs);
  continuous_ = true;
  return nullptr;
}

void ExoTracker::homeAtA(uint16_t raw, uint32_t nowMs) {
  ref_ = raw;
  accept(0, raw, nowMs);
  continuous_ = true;
}

}  // namespace teleop
