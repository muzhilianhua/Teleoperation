// Per-joint safety supervisor (M0 subset of servo_bringup's supervisor).
// Thresholds default to servo_bringup/dexhand_servo/config.py MotionConfig.

#pragma once

#include <stddef.h>
#include <stdint.h>

namespace teleop {

// True once a condition has held continuously for holdMs.
class Persistence {
 public:
  bool update(bool condition, uint32_t nowMs, uint32_t holdMs) {
    if (!condition) {
      active_ = false;
      return false;
    }
    if (!active_) {
      active_ = true;
      sinceMs_ = nowMs;
    }
    return nowMs - sinceMs_ >= holdMs;
  }
  void reset() { active_ = false; }

 private:
  bool active_ = false;
  uint32_t sinceMs_ = 0;
};

enum class Fault : uint8_t {
  None = 0,
  CommLost,        // consecutive servo read failures
  ServoError,      // non-zero status error byte (other than voltage) or status register
  ServoVoltage,    // the servo itself reports an input-voltage error (error bit 0x01)
  TorqueMismatch,  // servo torque state disagrees with the firmware
  LoadHard,
  LoadTrip,
  Overheat,
  SupplyVoltage,
  TrackingError,
  OverCurrent,  // HLS: |current| near its goal-current cap for too long
};

const char* faultToString(Fault fault);

// Joint: only the joint that raised it loses torque, the rest keep following.
// Hand: the cause is shared (supply, bus) or the firmware state can no longer
// be trusted (torque mismatch), so every servo is switched off.
// Matches the per-fault actions of servo_bringup's supervisor.
enum class FaultScope : uint8_t { Joint, Hand };
FaultScope faultScope(Fault fault);

constexpr uint8_t kServoErrorVoltageBit = 0x01;  // status-packet error byte

struct SupervisorConfig {
  // Off: LoadTrip/LoadHard are never raised and only the servo's own overload
  // protection (reported as ServoError) ends a high-load episode.
  bool loadTripEnabled = true;
  float loadTripPct = 75.0f;
  uint32_t loadTripMs = 300;
  float loadHardPct = 80.0f;
  uint32_t loadHardMs = 100;
  float trackingErrorSteps = 40.0f;
  uint32_t trackingErrorMs = 400;
  int temperatureTripC = 65;
  uint32_t temperatureTripMs = 1000;
  float supplyMinV = 5.5f;
  float supplyMaxV = 6.5f;
  uint32_t supplyMs = 200;
  uint8_t commFailCount = 3;
  // HLS3606: trip when |current| >= currentTripMa for currentTripMs
  // (DEX_hand_software: 0.9 x the effective current cap, 0.2 s). 0 = off.
  float currentTripMa = 0.0f;
  uint32_t currentTripMs = 200;
};

struct ServoSample {
  bool readOk = false;
  bool torqueOn = false;
  int16_t position = 0;
  float loadPct = 0.0f;  // signed, percent of max torque
  float voltage = 0.0f;  // volts
  int temperatureC = 0;
  uint8_t errorByte = 0;
  uint8_t status = 0;
  // The grip limiter sees the finger pressing on an object: falling behind the
  // goal is expected then, so the tracking check is suspended.
  bool inContact = false;
  bool hasCurrent = false;  // HLS3606 reports current, SCS0009 does not
  float currentMa = 0.0f;
};

class JointSupervisor {
 public:
  void setConfig(const SupervisorConfig& config) { config_ = config; }
  const SupervisorConfig& config() const { return config_; }

  // armed: firmware believes torque is on. goal: position sent last cycle.
  // reference: where a servo moving at its measured capability would be
  // (a rate limiter chasing the goal at vsat). The tracking check flags only
  // lag beyond that model, lag = |goal - position| - |goal - reference|, so a
  // goal that outruns the servo's physical speed is not a fault by itself.
  // Returns the first fault found this cycle (None if healthy).
  Fault update(const ServoSample& sample, bool armed, float goal, float reference, uint32_t nowMs);
  void reset();
  uint8_t consecutiveFailures() const { return failures_; }

 private:
  SupervisorConfig config_;
  uint8_t failures_ = 0;
  Persistence loadTrip_, loadHard_, tracking_, overheat_, overCurrent_;
};

// Supply check on the median voltage across all servos, so one bad reading
// does not trip the hand while a real sag (all servos) does.
class SupplySupervisor {
 public:
  void setConfig(const SupervisorConfig& config) { config_ = config; }
  // Returns SupplyVoltage once the median has been out of range for supplyMs.
  Fault update(const float* voltages, size_t count, uint32_t nowMs);
  void reset() { outOfRange_.reset(); }

 private:
  SupervisorConfig config_;
  Persistence outOfRange_;
};

float median(const float* values, size_t count);

}  // namespace teleop
