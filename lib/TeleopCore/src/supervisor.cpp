#include "supervisor.h"

#include <math.h>
#include <string.h>

namespace teleop {

const char* faultToString(Fault fault) {
  switch (fault) {
    case Fault::None: return "none";
    case Fault::CommLost: return "comm lost";
    case Fault::ServoError: return "servo error";
    case Fault::TorqueMismatch: return "torque mismatch";
    case Fault::LoadHard: return "load hard limit";
    case Fault::LoadTrip: return "load trip";
    case Fault::Overheat: return "overheat";
    case Fault::SupplyVoltage: return "supply voltage";
    case Fault::TrackingError: return "tracking error";
  }
  return "?";
}

Fault JointSupervisor::update(const ServoSample& s, bool armed, float setpoint, uint32_t nowMs) {
  if (!s.readOk) {
    if (failures_ < 0xFF) ++failures_;
    return failures_ >= config_.commFailCount ? Fault::CommLost : Fault::None;
  }
  failures_ = 0;

  if (s.errorByte != 0 || s.status != 0) return Fault::ServoError;
  if (s.torqueOn != armed) return Fault::TorqueMismatch;

  const float load = fabsf(s.loadPct);
  // Evaluate every timer each cycle so their start times stay correct.
  const bool hard = loadHard_.update(load >= config_.loadHardPct, nowMs, config_.loadHardMs);
  const bool trip = loadTrip_.update(load >= config_.loadTripPct, nowMs, config_.loadTripMs);
  const bool hot = overheat_.update(s.temperatureC >= config_.temperatureTripC, nowMs,
                                    config_.temperatureTripMs);
  const bool lagging =
      armed && tracking_.update(fabsf(s.position - setpoint) > config_.trackingErrorSteps, nowMs,
                                config_.trackingErrorMs);
  if (!armed) tracking_.reset();

  if (hard) return Fault::LoadHard;
  if (trip) return Fault::LoadTrip;
  if (hot) return Fault::Overheat;
  if (lagging) return Fault::TrackingError;
  return Fault::None;
}

void JointSupervisor::reset() {
  failures_ = 0;
  loadTrip_.reset();
  loadHard_.reset();
  tracking_.reset();
  overheat_.reset();
}

float median(const float* values, size_t count) {
  if (count == 0) return NAN;
  float sorted[16];
  if (count > 16) count = 16;
  memcpy(sorted, values, count * sizeof(float));
  for (size_t i = 1; i < count; ++i) {
    const float v = sorted[i];
    size_t j = i;
    while (j > 0 && sorted[j - 1] > v) {
      sorted[j] = sorted[j - 1];
      --j;
    }
    sorted[j] = v;
  }
  return count % 2 ? sorted[count / 2] : 0.5f * (sorted[count / 2 - 1] + sorted[count / 2]);
}

Fault SupplySupervisor::update(const float* voltages, size_t count, uint32_t nowMs) {
  if (count == 0) {
    outOfRange_.reset();
    return Fault::None;
  }
  const float v = median(voltages, count);
  const bool bad = v < config_.supplyMinV || v > config_.supplyMaxV;
  return outOfRange_.update(bad, nowMs, config_.supplyMs) ? Fault::SupplyVoltage : Fault::None;
}

}  // namespace teleop
