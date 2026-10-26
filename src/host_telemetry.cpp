#include "host_telemetry.h"

#include <math.h>

#include "hand.h"

#ifndef FW_GIT_REV
#define FW_GIT_REV __DATE__ " " __TIME__
#endif

namespace {

// Index = hostlink::EventCode.
const char* const kEventNames[] = {"generic",    "refused",      "armed",        "disarmed",   "torque_off",
                                   "fault",      "joint_state",  "cal_captured", "homed",      "ping_result",
                                   "limits_row", "sweep_run",    "sweep_done",   "config_saved", "config_loaded",
                                   "defaults",   "setting_changed", "monitor"};
static_assert(sizeof(kEventNames) / sizeof(kEventNames[0]) == static_cast<size_t>(hostlink::EventCode::Count),
              "one name per event code");

constexpr uint8_t kFaultCount = static_cast<uint8_t>(teleop::Fault::OverCurrent) + 1;
constexpr uint8_t kStateCount = static_cast<uint8_t>(JointState::Sweep) + 1;
constexpr uint8_t kModelCount = static_cast<uint8_t>(feetech::ServoModel::Hls3606) + 1;

int16_t toI16(float v) {
  const float r = roundf(v);
  return static_cast<int16_t>(r > 32767.0f ? 32767.0f : (r < -32768.0f ? -32768.0f : r));
}

uint16_t toU16(float v) {
  const float r = roundf(v);
  return static_cast<uint16_t>(r > 65535.0f ? 65535.0f : (r < 0.0f ? 0.0f : r));
}

uint16_t jointFlags(const JointSnapshot& j) {
  uint16_t f = 0;
  if (j.settings.enabled) f |= hostlink::kJointEnabled;
  if (j.ready) f |= hostlink::kJointReady;
  if (j.refCaptured) f |= hostlink::kJointRefCaptured;
  if (j.homed) f |= hostlink::kJointHomed;
  if (j.singleTurn) f |= hostlink::kJointSingleTurn;
  if (j.encFresh) f |= hostlink::kJointEncFresh;
  if (j.contact) f |= hostlink::kJointContact;
  if (j.goalSent) f |= hostlink::kJointGoalSent;
  if (j.modelOk) f |= hostlink::kJointModelOk;
  if (j.servoOk) f |= hostlink::kJointServoOk;
  if (j.servo.torqueOn) f |= hostlink::kJointTorqueOn;
  if (j.servo.moving) f |= hostlink::kJointMoving;
  if (j.servo.hasCurrent) f |= hostlink::kJointHasCurrent;
  if (j.readAgeMs <= config::kServoCommStaleMs) f |= hostlink::kJointServoFresh;
  return f;
}

}  // namespace

const hostlink::DeviceInfo& deviceInfo() {
  static hostlink::JointInfo joints[teleop::kMaxJoints];
  static const char* faults[kFaultCount];
  static const char* states[kStateCount];
  static const char* models[kModelCount];
  static hostlink::DeviceInfo info;
  static bool built = false;
  if (built) return info;
  for (uint8_t i = 0; i < teleop::kMaxJoints; ++i) {
    const teleop::JointDefaults& d = hand::joints()[i];
    const feetech::ModelInfo& m = feetech::modelInfo(d.model);
    hostlink::JointInfo& j = joints[i];
    j.servoId = d.servoId;
    j.model = static_cast<uint8_t>(d.model);
    j.travelMin = d.travelMin;
    j.travelMax = d.travelMax;
    j.vmax = d.vmax;
    j.vsat = d.vsat;
    j.amax = d.amax;
    j.maxSpeed = toU16(m.maxSpeedRegister * m.stepsPerSpeedUnit);
    j.encoderDefault = d.encoderIndex;
    j.enabledDefault = d.enabled ? 1 : 0;
    j.stepScale = teleop::stepScale(d);
    j.degreesPerStep = (300.0f / 1024.0f) / j.stepScale;  // SCS0009: ~300 deg over 1024 steps
    j.name = d.name;
  }
  for (uint8_t i = 0; i < kFaultCount; ++i) faults[i] = teleop::faultToString(static_cast<teleop::Fault>(i));
  for (uint8_t i = 0; i < kStateCount; ++i) states[i] = jointStateToString(static_cast<JointState>(i));
  for (uint8_t i = 0; i < kModelCount; ++i) models[i] = feetech::modelName(static_cast<feetech::ServoModel>(i));
  info.firmware = FW_GIT_REV;
  info.controlHz = config::kControlHz;
  info.degreesPerCount = teleop::kDegreesPerCount;
  info.encoderCount = config::kEncoderCount;
  info.minCurrentLimitMa = teleop::kMinCurrentLimitMa;
  info.maxCurrentLimitMa = teleop::kMaxCurrentLimitMa;
  info.minCurrentTripPct = teleop::kMinCurrentTripPct;
  info.maxCurrentTripPct = teleop::kMaxCurrentTripPct;
  info.defaultCurrentTripPct = teleop::kDefaultCurrentTripPct;
  info.minExoSpanCounts = teleop::kMinExoSpanCounts;
  info.maxExoSpanCounts = teleop::kMaxExoSpanCounts;
  info.joints = joints;
  info.jointCount = teleop::kMaxJoints;
  info.faults = {faults, kFaultCount};
  info.states = {states, kStateCount};
  info.events = {kEventNames, static_cast<uint8_t>(hostlink::EventCode::Count)};
  info.models = {models, kModelCount};
  built = true;
  return info;
}

void fillTelemetry(const Snapshot& s, hostlink::Telemetry& out) {
  static_assert(teleop::kMaxJoints <= hostlink::kMaxJoints, "joint table larger than the wire format");
  out.cycle = s.cycle;
  out.tUs = s.tUs;
  out.uptimeMs = s.uptimeMs;
  out.armedCount = s.armedCount;
  out.handFault = static_cast<uint8_t>(s.lastHandFault);
  out.handFlags = 0;
  if (s.torqueOffConfirmed) out.handFlags |= hostlink::kHandTorqueOffConfirmed;
  if (s.monitor) out.handFlags |= hostlink::kHandMonitor;
  if (s.sweepActive) out.handFlags |= hostlink::kHandSweepActive;
  out.execUs = toU16(static_cast<float>(s.timing.lastExecUs));
  out.jointCount = teleop::kMaxJoints;
  for (uint8_t i = 0; i < teleop::kMaxJoints; ++i) {
    const JointSnapshot& j = s.joints[i];
    hostlink::TelemetryJoint& w = out.joints[i];
    w.state = static_cast<uint8_t>(j.state);
    w.fault = static_cast<uint8_t>(j.lastFault);
    w.flags = jointFlags(j);
    w.encRaw = j.encRaw;
    w.exoCounts = j.exoCounts;
    w.exoFilteredCounts = j.exoFilteredCounts;
    w.target = j.target;
    w.goal = j.goal;
    w.sentGoal = j.sentGoal;
    w.reference = j.reference;
    w.speedCommand = toU16(j.speed);
    w.gripLead = toI16(j.gripLead);
    w.position = j.servo.position;
    w.speed = j.servo.speed;
    w.loadDeciPct = toI16(j.servo.loadPct * 10.0f);
    w.currentMa = toI16(j.servo.currentMa);
    const uint16_t deciVolts = toU16(j.servo.voltage * 10.0f);
    w.voltageDeciV = deciVolts > 255 ? 255 : static_cast<uint8_t>(deciVolts);
    w.temperatureC = static_cast<int8_t>(j.servo.temperatureC > 127 ? 127 : j.servo.temperatureC);
    w.status = j.servo.status;
    w.errorByte = j.servo.errorByte;
    w.readAgeMs = j.readAgeMs > 65535 ? 65535 : static_cast<uint16_t>(j.readAgeMs);
  }
}

void fillSlowStatus(const Snapshot& s, hostlink::SlowStatus& out) {
  static_assert(config::kEncoderCount <= hostlink::kMaxEncoders, "encoder table larger than the wire format");
  const teleop::GlobalSettings& g = s.global;
  out.leadMs = g.leadMs;
  out.minCutoffHz = g.minCutoffHz;
  out.beta = g.beta;
  out.jumpDeg = g.jumpDeg;
  out.gripLoadPct = g.gripLoadPct;
  out.loadTrip = g.loadTripEnabled ? 1 : 0;
  const TimingStats& t = s.timing;
  out.cycles = t.cycles;
  out.overruns = t.overruns;
  out.lastPeriodUs = t.lastPeriodUs;
  out.maxJitterUs = t.maxJitterUs;
  out.p99JitterUs = t.p99JitterUs;
  out.lastExecUs = t.lastExecUs;
  out.maxExecUs = t.maxExecUs;
  out.averageExecUs = t.averageExecUs();
  out.busSent = s.port.sent;
  out.busReceived = s.port.received;
  out.busTimeouts = s.port.timeouts;
  out.busParseErrors = s.port.parseErrors;
  out.busEchoErrors = s.port.echoErrors;
  out.busLastReplyUs = s.port.lastReplyUs;
  out.busMaxReplyUs = s.port.maxReplyUs;
  out.busEcho = s.echo ? 1 : 0;
  out.encoderAverageUs = s.encoderCycle.averageCycleUs();
  out.encoderMaxUs = s.encoderCycle.maxCycleUs;
  out.encoderMaxSkewUs = s.encoderCycle.maxSkewUs;
  out.droppedEvents = s.droppedEvents;
  out.droppedTelemetry = s.droppedTelemetry;
  out.monitor = s.monitor ? 1 : 0;
  out.monitorOkMask = 0;
  out.jointCount = teleop::kMaxJoints;
  for (uint8_t i = 0; i < teleop::kMaxJoints; ++i) {
    if (s.monitorOk[i]) out.monitorOkMask |= static_cast<uint16_t>(1u << i);
    out.monitorPosition[i] = s.monitorPosition[i];
    const JointSnapshot& j = s.joints[i];
    const teleop::JointSettings& st = j.settings;
    hostlink::SlowJoint& w = out.joints[i];
    w.enabled = st.enabled ? 1 : 0;
    w.encoderIndex = st.encoderIndex;
    w.exoRef = st.exoRef;
    w.exoB = st.exoB;
    w.servoA = st.servoA;
    w.servoB = st.servoB;
    w.vmax = st.vmax;
    w.amax = st.amax;
    w.vsat = st.vsat;
    w.trackSteps = st.trackSteps;
    w.gripSteps = st.gripSteps;
    w.currentLimitMa = st.currentLimitMa;
    w.currentTripPct = st.currentTripPct;
    w.detectedModel = static_cast<uint8_t>(j.detectedModel);
    w.effectiveCurrentMa = toU16(j.currentLimitMa);
    w.jumpRejects = j.jumpRejects;
  }
  out.encoderCount = config::kEncoderCount;
  for (uint8_t i = 0; i < config::kEncoderCount; ++i) {
    const EncoderSummary& e = s.encoders[i];
    hostlink::SlowEncoder& w = out.encoders[i];
    w.address = e.address;
    w.state = static_cast<uint8_t>(e.state);
    w.valid = e.valid ? 1 : 0;
    w.angle = e.angle;
    w.agc = e.agc;
    w.magnetOk = e.magnetOk ? 1 : 0;
    w.errors = e.errors;
  }
}
