#include "rate_limiter.h"

#include <math.h>

namespace teleop {

namespace {

float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

}  // namespace

RateLimiter::RateLimiter(float position, float maxSpeed, float acceleration, float lo, float hi)
    : maxSpeed_(maxSpeed), acceleration_(acceleration), lo_(lo), hi_(hi) {
  reset(position);
}

void RateLimiter::setTarget(float target) { target_ = clamp(target); }

void RateLimiter::reset(float position) {
  position_ = target_ = previousTarget_ = clamp(position);
  velocity_ = 0.0f;
}

float RateLimiter::step(float dt) {
  if (dt <= 0.0f) return position_;
  const float targetVelocity = (target_ - previousTarget_) / dt;
  previousTarget_ = target_;
  const float error = target_ - position_;

  // Feasible target: follow it exactly and leave with the target's own velocity.
  const float needed = error / dt;
  const float reach = acceleration_ * dt + 1e-6f;
  if (fabsf(needed) <= maxSpeed_ + 1e-6f && fabsf(needed - velocity_) <= reach &&
      fabsf(needed - targetVelocity) <= reach) {
    velocity_ = fabsf(error) > 1e-9f ? needed : 0.0f;
    position_ = target_;
    return position_;
  }

  const bool movingTarget = fabsf(targetVelocity) > 1e-6f;
  if (!movingTarget && fabsf(error) < 0.5f && fabsf(velocity_) <= acceleration_ * dt) {
    position_ = target_;
    velocity_ = 0.0f;
    return position_;
  }

  // Fastest speed that can still stop at the target, linear when close.
  const float approachMagnitude =
      fminf(sqrtf(2.0f * acceleration_ * fabsf(error)), kApproachGain * fabsf(error));
  const float approach = copysignf(approachMagnitude, error);
  const float desired = clampf(targetVelocity + approach, -maxSpeed_, maxSpeed_);
  const float dv = clampf(desired - velocity_, -acceleration_ * dt, acceleration_ * dt);
  velocity_ += dv;
  float next = position_ + velocity_ * dt;
  if (!movingTarget && (target_ - next) * error < 0.0f) {
    next = target_;  // never overshoot a stationary target
    velocity_ = 0.0f;
  }
  position_ = clamp(next);
  return position_;
}

}  // namespace teleop
