// Per-joint configuration: compile-time robot facts and defaults
// (JointDefaults) and the persisted, operator-entered settings (JointSettings).
//
// Travel (the mechanical limits) differs from robot to robot with assembly, so
// it is a setting too: config.h only provides the default, `set <j> travel`
// changes it (disarmed only, within the servo model's position range), and it
// is stored, dumped and restored with the rest of the calibration.

#pragma once

#include <servo_models.h>
#include <stddef.h>
#include <stdint.h>

#include "joint_map.h"

namespace teleop {

constexpr uint8_t kMaxJoints = 9;
constexpr uint8_t kNoEncoder = 0xFF;
constexpr int32_t kMinExoSpanCounts = 182;  // 4 deg
constexpr int32_t kMaxExoSpanCounts = 3 * kEncoderCounts;  // 3 encoder turns

// Every position, speed and step tolerance of a joint is in its servo's own
// steps (SCS0009 ~0.293 deg, HLS3606 ~0.088 deg per step).
struct JointDefaults {
  uint8_t servoId;
  const char* name;
  feetech::ServoModel model;
  int16_t travelMin;  // default travel; JointSettings holds the robot's own
  int16_t travelMax;
  uint16_t vmax;  // steps/s, rate-limiter ceiling
  uint16_t vsat;  // steps/s, measured capability used by the tracking check
  uint16_t amax;  // steps/s^2
  uint8_t encoderIndex;
  bool enabled;
};

struct JointSettings {
  // Mechanical travel of this robot, in servo steps: the range every goal is
  // clamped to and what `limits write` stores in the servo. min >= max = not
  // measured yet (monitor only).
  int16_t travelMin = 0;
  int16_t travelMax = 0;
  bool enabled = false;
  uint8_t encoderIndex = kNoEncoder;
  // Operator side: raw encoder count at endpoint A, and endpoint B as the
  // continuous (possibly multi-turn) count from A (A is 0; 0 = uncalibrated).
  uint16_t exoRef = 0;
  int32_t exoB = 0;
  // Robot side: servo counts matching exo A / B (equal = uncalibrated).
  int16_t servoA = 0;
  int16_t servoB = 0;
  uint16_t vmax = 0;
  uint16_t amax = 0;
  uint16_t vsat = 0;
  uint16_t trackSteps = 40;
  // Grip limiter: largest lead of the goal over the measured position, in
  // steps. Contact shrinks it to hold the squeeze near gripLoadPct. 0 = off.
  uint16_t gripSteps = 60;
  // HLS3606 only: goal current cap in mA sent with every goal (the servo
  // limits its torque to it). 0 = the servo's own protection current.
  uint16_t currentLimitMa = 0;
  // HLS3606 only: OverCurrent trips when |current| stays at or above this %
  // of the goal-current cap for 0.2 s; 0 = off (the servo's own over-current
  // cut-out still acts). A bench setting: not stored in NVS or in `dump`, so
  // it is back to the default after boot, `load` or `defaults`.
  uint8_t currentTripPct = 90;
};

constexpr uint16_t kMinCurrentLimitMa = 7;      // 1 unit of 6.5 mA
constexpr uint16_t kMaxCurrentLimitMa = 13306;  // 2047 units
constexpr uint8_t kDefaultCurrentTripPct = 90;  // DEX_hand_software current_trip_frac
constexpr uint8_t kMinCurrentTripPct = 50;
constexpr uint8_t kMaxCurrentTripPct = 100;

struct GlobalSettings {
  float leadMs = 0.0f;
  float minCutoffHz = 1.0f;
  float beta = 0.01f;
  float jumpDeg = 25.0f;      // largest plausible exo change per control cycle
  // Sustained squeeze target: % load on an SCS0009 (below its 80 % / 1 s
  // overload cut-out), % of the goal-current cap on an HLS3606 (below itrip).
  float gripLoadPct = 60.0f;
  // Firmware load trips (75 % / 0.3 s, 80 % / 0.1 s). Off by default so a
  // grasp can load the servo; the servo's own overload protection then decides.
  bool loadTripEnabled = false;
  // Servo supply window: the hand is switched off when the median servo
  // voltage stays outside [supplyMinV, supplyMaxV] for supplyMs.
  float supplyMinV = 5.5f;
  float supplyMaxV = 6.5f;
  uint16_t supplyMs = 200;
};

// Bounds of the supply window. SCS0009 (spec A/0): input 4-7.4 V, its own
// under-voltage cut-out below 4 V. HLS3606 limits are not documented here.
constexpr float kMinSupplyMinV = 4.5f;
constexpr float kMaxSupplyMinV = 6.0f;
constexpr float kMinSupplyMaxV = 6.0f;
constexpr float kMaxSupplyMaxV = 7.4f;
constexpr uint16_t kMinSupplyMs = 50;
constexpr uint16_t kMaxSupplyMs = 1000;

JointSettings defaultSettings(const JointDefaults& defaults);

// Native steps per SCS0009 step, to express SCS-calibrated tolerances
// (tracking 40 steps, grip lead 60 steps, ...) in a joint's own steps.
inline float stepScale(const JointDefaults& d) { return feetech::modelInfo(d.model).stepScale; }
inline float scaledSteps(const JointDefaults& d, float scsSteps) { return scsSteps * stepScale(d); }

// Travel min >= max marks a joint whose limits have not been measured yet.
inline bool travelDefined(const JointDefaults& d) { return d.travelMin < d.travelMax; }
inline bool travelDefined(const JointSettings& s) { return s.travelMin < s.travelMax; }
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
