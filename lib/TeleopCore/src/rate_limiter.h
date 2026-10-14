// Velocity- and acceleration-limited tracking of a target inside [lo, hi].
// Port of servo_bringup/dexhand_servo/control/trajectory.py RateLimiter, so the
// firmware and the bring-up tool share the same motion semantics.

#pragma once

namespace teleop {

class RateLimiter {
 public:
  static constexpr float kApproachGain = 10.0f;  // 1/s, linear correction near the target

  RateLimiter() = default;
  RateLimiter(float position, float maxSpeed, float acceleration, float lo, float hi);

  void setLimits(float maxSpeed, float acceleration) {
    maxSpeed_ = maxSpeed;
    acceleration_ = acceleration;
  }
  void setTravel(float lo, float hi) {
    lo_ = lo;
    hi_ = hi;
  }
  void setTarget(float target);
  void reset(float position);
  float step(float dt);

  float position() const { return position_; }
  float velocity() const { return velocity_; }
  float target() const { return target_; }
  float maxSpeed() const { return maxSpeed_; }

 private:
  float clamp(float v) const { return v < lo_ ? lo_ : (v > hi_ ? hi_ : v); }

  float maxSpeed_ = 0.0f;
  float acceleration_ = 0.0f;
  float lo_ = 0.0f;
  float hi_ = 0.0f;
  float position_ = 0.0f;
  float target_ = 0.0f;
  float previousTarget_ = 0.0f;
  float velocity_ = 0.0f;
};

}  // namespace teleop
