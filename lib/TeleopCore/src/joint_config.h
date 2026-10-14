// Per-joint configuration: compile-time robot facts (JointDefaults) and the
// persisted, operator-entered settings (JointSettings).
//
// Travel limits deliberately exist only in JointDefaults so no console command
// can widen them; everything an operator calibrates lives in JointSettings.

#pragma once

#include <stddef.h>
#include <stdint.h>

#include "joint_map.h"

namespace teleop {

constexpr uint8_t kMaxJoints = 9;
constexpr uint8_t kNoEncoder = 0xFF;
constexpr int32_t kMinExoSpanCounts = 182;  // 4 deg

struct JointDefaults {
  uint8_t servoId;
  const char* name;
  int16_t travelMin;
  int16_t travelMax;
  uint16_t vmax;  // steps/s, rate-limiter ceiling
  uint16_t vsat;  // steps/s, measured capability used by the tracking check
  uint16_t amax;  // steps/s^2
  uint8_t encoderIndex;
  bool enabled;
};

struct JointSettings {
  bool enabled = false;
  uint8_t encoderIndex = kNoEncoder;
  // Operator side: raw encoder count at endpoint A, and endpoint B as unwrapped
  // counts relative to it (endpoint A is 0 by construction; 0 = uncalibrated).
  uint16_t exoRef = 0;
  int16_t exoB = 0;
  // Robot side: servo counts matching exo A / B (equal = uncalibrated).
  int16_t servoA = 0;
  int16_t servoB = 0;
  uint16_t vmax = 0;
  uint16_t amax = 0;
  uint16_t vsat = 0;
  uint16_t trackSteps = 40;
};

struct GlobalSettings {
  float leadMs = 0.0f;
  float minCutoffHz = 1.0f;
  float beta = 0.01f;
  float jumpDeg = 25.0f;  // largest plausible exo change per control cycle
};

JointSettings defaultSettings(const JointDefaults& defaults);

// Travel min >= max marks a joint whose limits have not been measured yet.
inline bool travelDefined(const JointDefaults& d) { return d.travelMin < d.travelMax; }
inline bool exoCalibrated(const JointSettings& s) { return s.exoB != 0; }
bool servoCalibrated(const JointSettings& s, const JointDefaults& d);
bool jointReady(const JointSettings& s, const JointDefaults& d, uint8_t encoderCount);

// nullptr when the settings are acceptable, otherwise the first problem.
// Uncalibrated endpoints are acceptable; malformed ones are not.
const char* validateJoint(const JointSettings& s, const JointDefaults& d, uint8_t encoderCount);
const char* validateGlobal(const GlobalSettings& g);

// Mapping view for joint_map.h; exoA is always 0.
JointMap toJointMap(const JointSettings& s, const JointDefaults& d);

// Raw encoder count of endpoint B, i.e. exoRef + exoB folded into one turn.
uint16_t exoRawB(const JointSettings& s);

}  // namespace teleop
