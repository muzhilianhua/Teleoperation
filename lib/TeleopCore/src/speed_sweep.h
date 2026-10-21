// On-device speed sweep: measures a joint's saturation speed in both
// directions and suggests vmax / vsat. Port of servo_bringup verify-motion
// check F (speed sweep), driven one control cycle at a time.
//
// Sequence: move to the low end at the approach speed, then for each
// speed tier: low -> high, settle, high -> low, settle. Each move is a
// single goal write at that speed (like servo_bringup's single-write
// measurement). A move's speed is taken between 25 % and 75 % of its distance
// when it arrives, or as the average progress after the first 0.2 s when it
// times out.
//
// Saturation per direction = fastest move that arrived. A direction with no
// arriving move from the second speed tier on is force-limited (friction / preload):
// no suggestion is made, the mechanics need fixing first.

#pragma once

#include <stdint.h>

namespace teleop {

struct SweepConfig {
  int16_t lo = 0;  // sweep range in servo counts, already inside travel
  int16_t hi = 0;
  // Commanded speed tiers in the servo's own steps/s, slowest first. The
  // defaults are SCS0009 steps; an HLS3606 sweep passes them x3.333. The
  // first tier can be limited by the command itself, so only arrivals from
  // the second tier on prove a direction is not force-limited.
  float speeds[4] = {100.0f, 300.0f, 600.0f, 1000.0f};
  uint8_t speedCount = 4;
  float approachSpeed = 50.0f;
  uint32_t approachTimeoutMs = 5000;  // then start from wherever the joint got to
  uint32_t settleMs = 300;
  uint32_t ignoreStartMs = 200;       // start-up excluded from timed-out averages
  float reachTolerance = 3.0f;
  float loadAbortPct = 90.0f;         // sweep-only protection against pushing a stuck joint
  uint32_t loadAbortMs = 300;
  float minSuggestedVmax = 50.0f;     // suggestion bounds, in the servo's steps/s
  float maxSuggestedSpeed = 1000.0f;
};

struct SweepRun {
  float commandSpeed = 0.0f;  // steps/s
  uint8_t level = 0;          // index into SweepConfig::speeds
  int8_t direction = 0;  // +1 low -> high, -1 high -> low
  // False when the move had less than half the range to cover because the
  // previous move never got back (a stuck direction): too short to time, so
  // it is skipped and ignored by the result.
  bool valid = true;
  float speed = 0.0f;    // steps/s
  bool reached = false;
  float maxLoadPct = 0.0f;
};

struct SweepResult {
  static constexpr uint8_t kMaxRuns = 8;
  SweepRun runs[kMaxRuns];
  uint8_t runCount = 0;
  bool completed = false;
  bool aborted = false;
  const char* abortReason = "";
  float saturationPlus = 0.0f;   // 0 when no move in that direction arrived
  float saturationMinus = 0.0f;
  bool limitedPlus = false;
  bool limitedMinus = false;
  uint16_t suggestedVmax = 0;    // 0 when force-limited or incomplete
  uint16_t suggestedVsat = 0;

  bool forceLimited() const { return limitedPlus || limitedMinus; }
  bool applicable() const { return completed && !forceLimited() && suggestedVmax > 0; }
};

struct SweepStep {
  bool write = false;  // send `goal` at `speed` this cycle
  int16_t goal = 0;
  float speed = 0.0f;  // steps/s
  bool runFinished = false;  // result().runs[runCount - 1] is new
  bool done = false;         // completed or aborted; switch the joint off
};

class SpeedSweep {
 public:
  void start(const SweepConfig& config, int16_t position, uint32_t nowMs);
  // position / load come from this cycle's servo read; valid = the read succeeded.
  SweepStep update(uint32_t nowMs, int16_t position, float loadPct, bool valid);
  void abort(const char* reason);

  bool active() const { return phase_ != Phase::Idle && phase_ != Phase::Done; }
  const SweepResult& result() const { return result_; }

 private:
  enum class Phase : uint8_t { Idle, Approach, Settle, Run, Done };

  // Returns false (and records a skipped run) when the move would be too short.
  bool beginRun(uint32_t nowMs, int16_t position, SweepStep& step);
  void finishRun(int16_t position, uint32_t nowMs, bool reached);
  void finishSweep();
  uint32_t runTimeoutMs() const;

  SweepConfig config_;
  SweepResult result_;
  Phase phase_ = Phase::Idle;
  uint32_t phaseStartMs_ = 0;
  bool approachSent_ = false;
  uint8_t nextRun_ = 0;  // index into the speed tier x direction sequence
  // Current move
  int16_t runStart_ = 0;
  int16_t runTarget_ = 0;
  int8_t runDirection_ = 0;
  float runSpeed_ = 0.0f;
  uint8_t runLevel_ = 0;
  float runMaxLoad_ = 0.0f;
  uint32_t t25_ = 0;
  uint32_t t75_ = 0;
  bool have25_ = false;
  bool have75_ = false;
  bool haveIgnorePos_ = false;
  int16_t ignorePos_ = 0;
  uint32_t ignoreMs_ = 0;
  // Load abort
  bool overload_ = false;
  uint32_t overloadSinceMs_ = 0;
};

}  // namespace teleop
