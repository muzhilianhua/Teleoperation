// Continuous (multi-turn) exoskeleton position from a single-turn encoder.
//
// The position x is in encoder counts relative to endpoint A (x = 0 at A) and
// may span several turns, e.g. when a transmission makes the encoder turn more
// than once over the joint's travel. It is tracked by summing the shortest
// signed step between consecutive samples, which is exact as long as the
// encoder turns less than half a turn between samples (18000 deg/s at 100 Hz).
//
// After power-up the turn count is unknown ("not homed"):
//   - single-turn joints (calibrated range plus dead zones < one turn) resolve
//     it automatically: exactly one candidate raw + k * 16384 lies in range;
//   - multi-turn joints need home(): the operator holds the glove near pose A
//     and the turn nearest A is taken.
// Losing samples for longer than the loss time, or three rejected jumps in a
// row, drops the home: single-turn joints re-resolve on the next sample,
// multi-turn joints wait for home() again.

#pragma once

#include <stdint.h>

namespace teleop {

class ExoTracker {
 public:
  enum class Result : uint8_t {
    Accepted,  // position updated
    Rejected,  // jump larger than maxStep, sample ignored
    Lost,      // tracking given up (repeated jumps); home dropped
    Unhomed,   // no valid position: multi-turn joint not homed, or sample in the dead zone
  };

  // Calibration changed: reference raw count at A, span to B (counts, 0 =
  // uncalibrated) and dead zone beyond each end. Drops the home.
  void configure(uint16_t exoRef, int32_t exoB, int32_t deadZoneCounts);
  // `cal b`: the span to B is now known. Unlike configure() the position and
  // home are kept, since they were tracked continuously from A.
  void setSpan(int32_t exoB) { exoB_ = exoB; }
  void setMaxStep(int32_t counts) { maxStep_ = counts; }  // <= 0 disables the jump check
  void setLossMs(uint32_t ms) { lossMs_ = ms; }

  bool calibrated() const { return exoB_ != 0; }
  bool singleTurn() const;
  bool homed() const { return homed_; }
  // True while the position has been tracked without a break since the last
  // explicit home (cal a / home()); required to capture endpoint B.
  bool continuous() const { return homed_ && continuous_; }
  int32_t position() const { return x_; }

  Result update(uint16_t raw, uint32_t nowMs);
  // Call when a cycle produced no valid sample; drops the home after the loss time.
  void noSample(uint32_t nowMs);
  // Operator holds the glove near A. nullptr on success, else why it was refused.
  const char* home(uint16_t raw, uint32_t nowMs);
  // `cal a`: the current raw reading becomes A (x = 0).
  void homeAtA(uint16_t raw, uint32_t nowMs);
  void invalidate();

 private:
  bool resolve(uint16_t raw, uint32_t nowMs);
  void accept(int32_t x, uint16_t raw, uint32_t nowMs);
  int32_t rangeLo() const;
  int32_t rangeHi() const;

  uint16_t ref_ = 0;
  int32_t exoB_ = 0;
  int32_t deadZone_ = 0;
  int32_t maxStep_ = 0;
  uint32_t lossMs_ = 100;

  bool homed_ = false;
  bool continuous_ = false;
  int32_t x_ = 0;
  uint16_t lastRaw_ = 0;
  uint32_t lastSampleMs_ = 0;
  uint8_t rejects_ = 0;
};

}  // namespace teleop
