// One Euro filter (Casiez et al., CHI 2012): low jitter at rest, low lag in motion.
// Cutoff adapts as minCutoff + beta * |filtered derivative|, so beta carries the
// unit of the input (here degrees: beta in 1/deg).

#pragma once

#include <math.h>

namespace teleop {

class OneEuroFilter {
 public:
  OneEuroFilter(float minCutoffHz = 1.0f, float beta = 0.01f, float dCutoffHz = 1.0f)
      : minCutoff_(minCutoffHz), beta_(beta), dCutoff_(dCutoffHz) {}

  void setParams(float minCutoffHz, float beta, float dCutoffHz) {
    minCutoff_ = minCutoffHz;
    beta_ = beta;
    dCutoff_ = dCutoffHz;
  }
  float minCutoff() const { return minCutoff_; }
  float beta() const { return beta_; }

  void reset() { initialized_ = false; }

  float filter(float x, float dt) {
    if (!initialized_ || dt <= 0.0f) {
      x_ = x;
      dx_ = 0.0f;
      initialized_ = true;
      return x_;
    }
    const float rawDx = (x - x_) / dt;
    dx_ += alpha(dCutoff_, dt) * (rawDx - dx_);
    const float cutoff = minCutoff_ + beta_ * fabsf(dx_);
    x_ += alpha(cutoff, dt) * (x - x_);
    return x_;
  }

  float value() const { return x_; }
  float derivative() const { return dx_; }

 private:
  static float alpha(float cutoffHz, float dt) {
    const float tau = 1.0f / (2.0f * static_cast<float>(M_PI) * cutoffHz);
    return 1.0f / (1.0f + tau / dt);
  }

  float minCutoff_;
  float beta_;
  float dCutoff_;
  bool initialized_ = false;
  float x_ = 0.0f;
  float dx_ = 0.0f;
};

}  // namespace teleop
