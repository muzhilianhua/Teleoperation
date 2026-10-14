// Grip force limiter for position-controlled servos.
//
// A finger stopped by an object keeps receiving the full mapped goal, so its
// P loop drives the servo to 100 % and the servo's own overload protection
// (80 % for 1 s on SCS0009) drops the grip. Limiting how far the sent goal may
// lead the measured position bounds the squeeze (load ~ P x error). The lead
// limit L adapts on measured load, so the held squeeze settles near loadPct
// whatever the joint's P gain:
//   contact (|load| >= loadPct and |speed| < stallSpeed): L -= shrinkStep
//   relaxed (|load| <  loadPct - hysteresisPct):          L += growStep
// In free motion the load stays low and L stays at maxLead, which is larger
// than what a following servo lags, so tracking is not slowed.

#pragma once

#include <math.h>

namespace teleop {

class GripLimiter {
 public:
  struct Config {
    float maxLead = 60.0f;  // steps; <= 0 disables the limiter
    float loadPct = 60.0f;
    float hysteresisPct = 10.0f;
    float shrinkStep = 2.0f;  // per servo read
    float growStep = 4.0f;    // per servo read; release should not feel sticky
    float minLead = 3.0f;
    float stallSpeed = 20.0f;  // steps/s
  };

  void setConfig(const Config& config) {
    config_ = config;
    if (lead_ > config_.maxLead) lead_ = config_.maxLead;
  }
  const Config& config() const { return config_; }

  void reset() {
    lead_ = config_.maxLead;
    contact_ = false;
  }

  // Call once per fresh servo reading; returns whether the finger is in contact.
  bool update(float loadPct, float speed) {
    if (!enabled()) {
      contact_ = false;
      return false;
    }
    const float load = fabsf(loadPct);
    contact_ = load >= config_.loadPct && fabsf(speed) < config_.stallSpeed;
    if (contact_) {
      lead_ = fmaxf(config_.minLead, lead_ - config_.shrinkStep);
    } else if (load < config_.loadPct - config_.hysteresisPct) {
      lead_ = fminf(config_.maxLead, lead_ + config_.growStep);
    }
    return contact_;
  }

  // The goal actually sent: at most `lead` steps away from the measured position.
  float clamp(float goal, float position) const {
    if (!enabled()) return goal;
    return fminf(fmaxf(goal, position - lead_), position + lead_);
  }

  bool enabled() const { return config_.maxLead > 0.0f; }
  bool contact() const { return contact_; }
  float lead() const { return lead_; }

 private:
  Config config_;
  float lead_ = 60.0f;
  bool contact_ = false;
};

}  // namespace teleop
