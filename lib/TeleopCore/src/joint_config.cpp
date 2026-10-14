#include "joint_config.h"

#include <stdlib.h>

namespace teleop {

namespace {

bool inRange(int32_t v, int32_t lo, int32_t hi) { return v >= lo && v <= hi; }

}  // namespace

JointSettings defaultSettings(const JointDefaults& d) {
  JointSettings s;
  s.enabled = d.enabled;
  s.encoderIndex = d.encoderIndex;
  s.exoRef = 0;
  s.exoB = 0;
  s.servoA = d.travelMin;
  s.servoB = d.travelMin;
  s.vmax = d.vmax;
  s.amax = d.amax;
  s.vsat = d.vsat;
  s.trackSteps = 40;
  return s;
}

bool servoCalibrated(const JointSettings& s, const JointDefaults& d) {
  return travelDefined(d) && s.servoA != s.servoB && inRange(s.servoA, d.travelMin, d.travelMax) &&
         inRange(s.servoB, d.travelMin, d.travelMax);
}

bool jointReady(const JointSettings& s, const JointDefaults& d, uint8_t encoderCount) {
  return travelDefined(d) && s.enabled && s.encoderIndex < encoderCount && exoCalibrated(s) &&
         servoCalibrated(s, d);
}

const char* validateJoint(const JointSettings& s, const JointDefaults& d, uint8_t encoderCount) {
  if (!travelDefined(d) && s.enabled) return "travel not defined in config.h: joint cannot be enabled";
  if (s.encoderIndex != kNoEncoder && s.encoderIndex >= encoderCount) return "encoder index out of range";
  if (s.exoRef >= kEncoderCounts) return "exo reference out of range";
  if (s.exoB != 0 && abs(s.exoB) < kMinExoSpanCounts) return "exo span below 4 deg";
  if (s.servoA != s.servoB && !servoCalibrated(s, d)) return "servo endpoint outside travel";
  if (!inRange(s.vmax, 1, 1000)) return "vmax outside 1..1000";
  if (!inRange(s.vsat, 1, 1000)) return "vsat outside 1..1000";
  if (!inRange(s.amax, 10, 20000)) return "amax outside 10..20000";
  if (!inRange(s.trackSteps, 5, 1000)) return "trk outside 5..1000";
  return nullptr;
}

const char* validateGlobal(const GlobalSettings& g) {
  if (!(g.leadMs >= 0.0f && g.leadMs <= 300.0f)) return "lead outside 0..300 ms";
  if (!(g.minCutoffHz >= 0.01f && g.minCutoffHz <= 30.0f)) return "mincut outside 0.01..30 Hz";
  if (!(g.beta >= 0.0f && g.beta <= 10.0f)) return "beta outside 0..10";
  if (!(g.jumpDeg >= 1.0f && g.jumpDeg <= 180.0f)) return "jump outside 1..180 deg";
  return nullptr;
}

JointMap toJointMap(const JointSettings& s, const JointDefaults& d) {
  JointMap m;
  m.name = d.name;
  m.encoderIndex = s.encoderIndex;
  m.servoId = d.servoId;
  m.exoRef = s.exoRef;
  m.exoA = 0;
  m.exoB = s.exoB;
  m.servoA = s.servoA;
  m.servoB = s.servoB;
  m.servoMin = d.travelMin;
  m.servoMax = d.travelMax;
  return m;
}

uint16_t exoRawB(const JointSettings& s) {
  int32_t raw = static_cast<int32_t>(s.exoRef) + s.exoB;
  raw %= kEncoderCounts;
  if (raw < 0) raw += kEncoderCounts;
  return static_cast<uint16_t>(raw);
}

}  // namespace teleop
