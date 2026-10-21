#include "speed_sweep.h"

#include <math.h>
#include <stdlib.h>

namespace teleop {

namespace {

constexpr uint8_t kLimitLevel = 1;  // the first tier can itself be the bottleneck

uint16_t clampU16(float v, uint16_t lo, uint16_t hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return static_cast<uint16_t>(v);
}

}  // namespace

void SpeedSweep::start(const SweepConfig& config, int16_t position, uint32_t nowMs) {
  config_ = config;
  if (config_.speedCount > SweepResult::kMaxRuns / 2) config_.speedCount = SweepResult::kMaxRuns / 2;
  result_ = SweepResult();
  nextRun_ = 0;
  overload_ = false;
  approachSent_ = false;
  phase_ = Phase::Approach;
  phaseStartMs_ = nowMs;
  (void)position;
}

void SpeedSweep::abort(const char* reason) {
  if (!active()) return;
  result_.aborted = true;
  result_.abortReason = reason;
  phase_ = Phase::Done;
}

uint32_t SpeedSweep::runTimeoutMs() const {
  // 1.5x the time the commanded speed needs for the distance, within 2..6 s.
  const float distance = fabsf(static_cast<float>(runTarget_ - runStart_));
  const float ms = 1500.0f * distance / runSpeed_ + 500.0f;
  if (ms < 2000.0f) return 2000;
  if (ms > 6000.0f) return 6000;
  return static_cast<uint32_t>(ms);
}

bool SpeedSweep::beginRun(uint32_t nowMs, int16_t position, SweepStep& step) {
  runLevel_ = nextRun_ / 2;
  runSpeed_ = config_.speeds[runLevel_];
  runDirection_ = (nextRun_ % 2 == 0) ? 1 : -1;
  runStart_ = position;
  runTarget_ = runDirection_ > 0 ? config_.hi : config_.lo;
  runMaxLoad_ = 0.0f;
  const float distance = static_cast<float>(runTarget_ - runStart_) * runDirection_;
  if (distance < 0.5f * static_cast<float>(config_.hi - config_.lo)) {
    SweepRun& run = result_.runs[result_.runCount++];
    run.commandSpeed = runSpeed_;
    run.level = runLevel_;
    run.direction = runDirection_;
    run.valid = false;
    ++nextRun_;
    step.runFinished = true;
    phaseStartMs_ = nowMs;  // stay in Settle; the next move starts after it
    return false;
  }
  have25_ = have75_ = haveIgnorePos_ = false;
  phase_ = Phase::Run;
  phaseStartMs_ = nowMs;
  step.write = true;
  step.goal = runTarget_;
  step.speed = runSpeed_;
  return true;
}

void SpeedSweep::finishRun(int16_t position, uint32_t nowMs, bool reached) {
  SweepRun& run = result_.runs[result_.runCount++];
  run.commandSpeed = runSpeed_;
  run.level = runLevel_;
  run.direction = runDirection_;
  run.reached = reached;
  run.maxLoadPct = runMaxLoad_;
  const float distance = fabsf(static_cast<float>(runTarget_ - runStart_));
  if (reached && have25_ && have75_ && t75_ > t25_) {
    run.speed = 0.5f * distance * 1000.0f / static_cast<float>(t75_ - t25_);
  } else if (reached) {
    const uint32_t elapsed = nowMs - phaseStartMs_;
    run.speed = elapsed > 0 ? distance * 1000.0f / static_cast<float>(elapsed) : 0.0f;
  } else if (haveIgnorePos_ && nowMs > ignoreMs_) {
    run.speed = fabsf(static_cast<float>(position - ignorePos_)) * 1000.0f /
                static_cast<float>(nowMs - ignoreMs_);
  }
  ++nextRun_;
}

void SpeedSweep::finishSweep() {
  float satPlus = 0.0f, satMinus = 0.0f;
  bool fastPlus = false, fastMinus = false;  // arrived at a tier >= kLimitLevel
  for (uint8_t i = 0; i < result_.runCount; ++i) {
    const SweepRun& r = result_.runs[i];
    if (!r.valid || !r.reached) continue;
    float& sat = r.direction > 0 ? satPlus : satMinus;
    if (r.speed > sat) sat = r.speed;
    if (r.level >= kLimitLevel) (r.direction > 0 ? fastPlus : fastMinus) = true;
  }
  result_.saturationPlus = satPlus;
  result_.saturationMinus = satMinus;
  result_.limitedPlus = !fastPlus;
  result_.limitedMinus = !fastMinus;
  result_.completed = true;
  if (!result_.forceLimited()) {
    const float faster = satPlus > satMinus ? satPlus : satMinus;
    const float slower = satPlus < satMinus ? satPlus : satMinus;
    const uint16_t maxSpeed = static_cast<uint16_t>(config_.maxSuggestedSpeed);
    result_.suggestedVmax =
        clampU16(ceilf(1.2f * faster), static_cast<uint16_t>(config_.minSuggestedVmax), maxSpeed);
    result_.suggestedVsat = clampU16(floorf(0.9f * slower), 1, maxSpeed);
  }
  phase_ = Phase::Done;
}

SweepStep SpeedSweep::update(uint32_t nowMs, int16_t position, float loadPct, bool valid) {
  SweepStep step;
  if (!active()) {
    step.done = phase_ == Phase::Done;
    return step;
  }

  if (valid) {
    const bool high = fabsf(loadPct) >= config_.loadAbortPct;
    if (high && !overload_) {
      overload_ = true;
      overloadSinceMs_ = nowMs;
    } else if (!high) {
      overload_ = false;
    }
    if (overload_ && nowMs - overloadSinceMs_ >= config_.loadAbortMs) {
      abort("load >= 90 % for 0.3 s (joint pushing against something)");
      step.done = true;
      return step;
    }
  }

  switch (phase_) {
    case Phase::Approach:
      if (!approachSent_) {
        approachSent_ = true;
        phaseStartMs_ = nowMs;
        step.write = true;
        step.goal = config_.lo;
        step.speed = config_.approachSpeed;
        break;
      }
      if (!valid) break;
      if (fabsf(static_cast<float>(position - config_.lo)) <= config_.reachTolerance ||
          nowMs - phaseStartMs_ >= config_.approachTimeoutMs) {
        // Hold where it is and settle; a stuck joint is measured from where it stopped.
        step.write = true;
        step.goal = position;
        step.speed = config_.approachSpeed;
        phase_ = Phase::Settle;
        phaseStartMs_ = nowMs;
      }
      break;

    case Phase::Settle:
      if (nowMs - phaseStartMs_ < config_.settleMs || !valid) break;
      if (nextRun_ >= 2 * config_.speedCount) {
        finishSweep();
        step.done = true;
        break;
      }
      beginRun(nowMs, position, step);
      break;

    case Phase::Run: {
      if (!valid) break;
      const float load = fabsf(loadPct);
      if (load > runMaxLoad_) runMaxLoad_ = load;
      const uint32_t elapsed = nowMs - phaseStartMs_;
      if (!haveIgnorePos_ && elapsed >= config_.ignoreStartMs) {
        haveIgnorePos_ = true;
        ignorePos_ = position;
        ignoreMs_ = nowMs;
      }
      const float travelled = static_cast<float>(position - runStart_) * runDirection_;
      const float distance = fabsf(static_cast<float>(runTarget_ - runStart_));
      if (!have25_ && travelled >= 0.25f * distance) {
        have25_ = true;
        t25_ = nowMs;
      }
      if (!have75_ && travelled >= 0.75f * distance) {
        have75_ = true;
        t75_ = nowMs;
      }
      const bool reached = fabsf(static_cast<float>(position - runTarget_)) <= config_.reachTolerance;
      if (reached || elapsed >= runTimeoutMs()) {
        finishRun(position, nowMs, reached);
        step.runFinished = true;
        // Hold where it ended; never leave a stuck joint pushing toward the target.
        step.write = true;
        step.goal = position;
        step.speed = config_.approachSpeed;
        phase_ = Phase::Settle;
        phaseStartMs_ = nowMs;
      }
      break;
    }

    case Phase::Idle:
    case Phase::Done:
      break;
  }
  return step;
}

}  // namespace teleop
