#include "host_telemetry.h"

#include <math.h>

namespace {

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
