#include "joint_config.h"

#include <math.h>
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
  s.trackSteps = static_cast<uint16_t>(lroundf(scaledSteps(d, 40.0f)));
  s.gripSteps = static_cast<uint16_t>(lroundf(scaledSteps(d, 60.0f)));
  s.currentLimitMa = 0;
  s.currentTripPct = kDefaultCurrentTripPct;
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
  if (s.exoB != 0 && labs(s.exoB) < kMinExoSpanCounts) return "exo span below 4 deg";
  if (labs(s.exoB) > kMaxExoSpanCounts) return "exo span above 3 encoder turns";
  if (s.servoA != s.servoB && !servoCalibrated(s, d)) return "servo endpoint outside travel";
  // Bounds follow the servo model: its fastest speed register and its step size.
  const feetech::ModelInfo& m = feetech::modelInfo(d.model);
  const int32_t maxSpeed = static_cast<int32_t>(m.maxSpeedRegister * m.stepsPerSpeedUnit);
  const float scale = stepScale(d);
  if (!inRange(s.vmax, 1, maxSpeed)) return "vmax outside 1..max speed of the servo model";
  if (!inRange(s.vsat, 1, maxSpeed)) return "vsat outside 1..max speed of the servo model";
  if (!inRange(s.amax, 10, lroundf(fminf(20000.0f * scale, 65000.0f)))) return "amax outside 10..20000 SCS steps/s^2";
  if (!inRange(s.trackSteps, 5, lroundf(1000.0f * scale))) return "trk outside 5..1000 SCS steps";
  if (s.gripSteps > lroundf(400.0f * scale)) return "grip outside 0..400 SCS steps";
  if (s.currentLimitMa != 0) {
    if (d.model != feetech::ServoModel::Hls3606) return "ilimit applies to HLS3606 joints only";
    if (!inRange(s.currentLimitMa, kMinCurrentLimitMa, kMaxCurrentLimitMa)) return "ilimit outside 7..13306 mA";
  }
  if (s.currentTripPct != kDefaultCurrentTripPct) {
    if (d.model != feetech::ServoModel::Hls3606) return "itrip applies to HLS3606 joints only";
    if (s.currentTripPct != 0 && !inRange(s.currentTripPct, kMinCurrentTripPct, kMaxCurrentTripPct)) {
      return "itrip must be 0 (off) or 50..100 %";
    }
  }
  return nullptr;
}

const char* validateGlobal(const GlobalSettings& g) {
  if (!(g.leadMs >= 0.0f && g.leadMs <= 300.0f)) return "lead outside 0..300 ms";
  if (!(g.minCutoffHz >= 0.01f && g.minCutoffHz <= 30.0f)) return "mincut outside 0.01..30 Hz";
  if (!(g.beta >= 0.0f && g.beta <= 10.0f)) return "beta outside 0..10";
  if (!(g.jumpDeg >= 1.0f && g.jumpDeg <= 180.0f)) return "jump outside 1..180 deg";
  if (!(g.gripLoadPct >= 20.0f && g.gripLoadPct <= 95.0f)) return "gripload outside 20..95 %";
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
