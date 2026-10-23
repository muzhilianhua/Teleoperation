#include "host_messages.h"

#include <string.h>

#include "byte_buffer.h"

namespace hostlink {

namespace {

void writeNames(ByteWriter& w, const NameTable& t) {
  w.u8(t.count);
  for (uint8_t i = 0; i < t.count; ++i) w.str8(t.names[i]);
}

size_t finish(const ByteWriter& w) { return w.ok() ? w.length() : 0; }

}  // namespace

size_t encodeHello(const DeviceInfo& info, uint8_t* out, size_t capacity) {
  ByteWriter w(out, capacity);
  w.u8(kProtocolVersion);
  w.str8(info.firmware);
  w.u16(info.controlHz);
  w.f32(info.degreesPerCount);
  w.u8(info.encoderCount);
  w.u16(info.minCurrentLimitMa);
  w.u16(info.maxCurrentLimitMa);
  w.u8(info.minCurrentTripPct);
  w.u8(info.maxCurrentTripPct);
  w.u8(info.defaultCurrentTripPct);
  w.i32(info.minExoSpanCounts);
  w.i32(info.maxExoSpanCounts);
  w.u8(info.jointCount);
  for (uint8_t i = 0; i < info.jointCount; ++i) {
    const JointInfo& j = info.joints[i];
    w.u8(j.servoId);
    w.u8(j.model);
    w.i16(j.travelMin);
    w.i16(j.travelMax);
    w.u16(j.vmax);
    w.u16(j.vsat);
    w.u16(j.amax);
    w.u16(j.maxSpeed);
    w.u8(j.encoderDefault);
    w.u8(j.enabledDefault);
    w.f32(j.stepScale);
    w.f32(j.degreesPerStep);
    w.str8(j.name);
  }
  writeNames(w, info.faults);
  writeNames(w, info.states);
  writeNames(w, info.events);
  writeNames(w, info.models);
  return finish(w);
}

// ---- TELEMETRY ---------------------------------------------------------------

size_t encodeTelemetry(const Telemetry& t, uint8_t* out, size_t capacity) {
  if (t.jointCount > kMaxJoints) return 0;
  ByteWriter w(out, capacity);
  w.u32(t.cycle);
  w.u32(t.tUs);
  w.u32(t.uptimeMs);
  w.u8(t.armedCount);
  w.u8(t.handFault);
  w.u8(t.handFlags);
  w.u16(t.execUs);
  w.u8(t.jointCount);
  for (uint8_t i = 0; i < t.jointCount; ++i) {
    const TelemetryJoint& j = t.joints[i];
    w.u8(j.state);
    w.u8(j.fault);
    w.u16(j.flags);
    w.u16(j.encRaw);
    w.f32(j.exoCounts);
    w.f32(j.exoFilteredCounts);
    w.f32(j.target);
    w.f32(j.goal);
    w.f32(j.sentGoal);
    w.f32(j.reference);
    w.u16(j.speedCommand);
    w.i16(j.gripLead);
    w.i16(j.position);
    w.i16(j.speed);
    w.i16(j.loadDeciPct);
    w.i16(j.currentMa);
    w.u8(j.voltageDeciV);
    w.i8(j.temperatureC);
    w.u8(j.status);
    w.u8(j.errorByte);
    w.u16(j.readAgeMs);
  }
  return finish(w);
}

bool decodeTelemetry(const uint8_t* in, size_t length, Telemetry& t) {
  ByteReader r(in, length);
  t.cycle = r.u32();
  t.tUs = r.u32();
  t.uptimeMs = r.u32();
  t.armedCount = r.u8();
  t.handFault = r.u8();
  t.handFlags = r.u8();
  t.execUs = r.u16();
  t.jointCount = r.u8();
  if (!r.ok() || t.jointCount > kMaxJoints || length != telemetrySize(t.jointCount)) return false;
  for (uint8_t i = 0; i < t.jointCount; ++i) {
    TelemetryJoint& j = t.joints[i];
    j.state = r.u8();
    j.fault = r.u8();
    j.flags = r.u16();
    j.encRaw = r.u16();
    j.exoCounts = r.f32();
    j.exoFilteredCounts = r.f32();
    j.target = r.f32();
    j.goal = r.f32();
    j.sentGoal = r.f32();
    j.reference = r.f32();
    j.speedCommand = r.u16();
    j.gripLead = r.i16();
    j.position = r.i16();
    j.speed = r.i16();
    j.loadDeciPct = r.i16();
    j.currentMa = r.i16();
    j.voltageDeciV = r.u8();
    j.temperatureC = r.i8();
    j.status = r.u8();
    j.errorByte = r.u8();
    j.readAgeMs = r.u16();
  }
  return r.ok();
}

// ---- SLOW_STATUS ---------------------------------------------------------------

size_t encodeSlowStatus(const SlowStatus& s, uint8_t* out, size_t capacity) {
  if (s.jointCount > kMaxJoints || s.encoderCount > kMaxEncoders) return 0;
  ByteWriter w(out, capacity);
  w.f32(s.leadMs);
  w.f32(s.minCutoffHz);
  w.f32(s.beta);
  w.f32(s.jumpDeg);
  w.f32(s.gripLoadPct);
  w.u8(s.loadTrip);
  w.u32(s.cycles);
  w.u32(s.overruns);
  w.u32(s.lastPeriodUs);
  w.u32(s.maxJitterUs);
  w.u32(s.p99JitterUs);
  w.u32(s.lastExecUs);
  w.u32(s.maxExecUs);
  w.u32(s.averageExecUs);
  w.u32(s.busSent);
  w.u32(s.busReceived);
  w.u32(s.busTimeouts);
  w.u32(s.busParseErrors);
  w.u32(s.busEchoErrors);
  w.u32(s.busLastReplyUs);
  w.u32(s.busMaxReplyUs);
  w.u8(s.busEcho);
  w.u32(s.encoderAverageUs);
  w.u32(s.encoderMaxUs);
  w.u32(s.encoderMaxSkewUs);
  w.u32(s.droppedEvents);
  w.u32(s.droppedTelemetry);
  w.u32(s.rxCrcErrors);
  w.u32(s.rxMalformed);
  w.u8(s.monitor);
  w.u16(s.monitorOkMask);
  w.u8(s.jointCount);
  for (uint8_t i = 0; i < s.jointCount; ++i) w.i16(s.monitorPosition[i]);
  for (uint8_t i = 0; i < s.jointCount; ++i) {
    const SlowJoint& j = s.joints[i];
    w.u8(j.enabled);
    w.u8(j.encoderIndex);
    w.u16(j.exoRef);
    w.i32(j.exoB);
    w.i16(j.servoA);
    w.i16(j.servoB);
    w.u16(j.vmax);
    w.u16(j.amax);
    w.u16(j.vsat);
    w.u16(j.trackSteps);
    w.u16(j.gripSteps);
    w.u16(j.currentLimitMa);
    w.u8(j.currentTripPct);
    w.u8(j.detectedModel);
    w.u16(j.effectiveCurrentMa);
    w.u32(j.jumpRejects);
  }
  w.u8(s.encoderCount);
  for (uint8_t i = 0; i < s.encoderCount; ++i) {
    const SlowEncoder& e = s.encoders[i];
    w.u8(e.address);
    w.u8(e.state);
    w.u8(e.valid);
    w.u16(e.angle);
    w.u8(e.agc);
    w.u8(e.magnetOk);
    w.u32(e.errors);
  }
  return finish(w);
}

bool decodeSlowStatus(const uint8_t* in, size_t length, SlowStatus& s) {
  ByteReader r(in, length);
  s.leadMs = r.f32();
  s.minCutoffHz = r.f32();
  s.beta = r.f32();
  s.jumpDeg = r.f32();
  s.gripLoadPct = r.f32();
  s.loadTrip = r.u8();
  s.cycles = r.u32();
  s.overruns = r.u32();
  s.lastPeriodUs = r.u32();
  s.maxJitterUs = r.u32();
  s.p99JitterUs = r.u32();
  s.lastExecUs = r.u32();
  s.maxExecUs = r.u32();
  s.averageExecUs = r.u32();
  s.busSent = r.u32();
  s.busReceived = r.u32();
  s.busTimeouts = r.u32();
  s.busParseErrors = r.u32();
  s.busEchoErrors = r.u32();
  s.busLastReplyUs = r.u32();
  s.busMaxReplyUs = r.u32();
  s.busEcho = r.u8();
  s.encoderAverageUs = r.u32();
  s.encoderMaxUs = r.u32();
  s.encoderMaxSkewUs = r.u32();
  s.droppedEvents = r.u32();
  s.droppedTelemetry = r.u32();
  s.rxCrcErrors = r.u32();
  s.rxMalformed = r.u32();
  s.monitor = r.u8();
  s.monitorOkMask = r.u16();
  s.jointCount = r.u8();
  if (!r.ok() || s.jointCount > kMaxJoints) return false;
  for (uint8_t i = 0; i < s.jointCount; ++i) s.monitorPosition[i] = r.i16();
  for (uint8_t i = 0; i < s.jointCount; ++i) {
    SlowJoint& j = s.joints[i];
    j.enabled = r.u8();
    j.encoderIndex = r.u8();
    j.exoRef = r.u16();
    j.exoB = r.i32();
    j.servoA = r.i16();
    j.servoB = r.i16();
    j.vmax = r.u16();
    j.amax = r.u16();
    j.vsat = r.u16();
    j.trackSteps = r.u16();
    j.gripSteps = r.u16();
    j.currentLimitMa = r.u16();
    j.currentTripPct = r.u8();
    j.detectedModel = r.u8();
    j.effectiveCurrentMa = r.u16();
    j.jumpRejects = r.u32();
  }
  s.encoderCount = r.u8();
  if (!r.ok() || s.encoderCount > kMaxEncoders) return false;
  for (uint8_t i = 0; i < s.encoderCount; ++i) {
    SlowEncoder& e = s.encoders[i];
    e.address = r.u8();
    e.state = r.u8();
    e.valid = r.u8();
    e.angle = r.u16();
    e.agc = r.u8();
    e.magnetOk = r.u8();
    e.errors = r.u32();
  }
  return r.ok() && r.remaining() == 0;
}

// ---- EVENT / ACK / DONE / CONFIG / LOG ----------------------------------------

size_t encodeEvent(const Event& e, uint8_t* out, size_t capacity) {
  ByteWriter w(out, capacity);
  w.u8(static_cast<uint8_t>(e.code));
  w.u8(static_cast<uint8_t>(e.severity));
  w.u8(e.joint);
  w.u16(e.cmdSeq);
  for (int32_t a : e.args) w.i32(a);
  w.bytes(reinterpret_cast<const uint8_t*>(e.text), strnlen(e.text, kMaxText - 1));
  return finish(w);
}

bool decodeEvent(const uint8_t* in, size_t length, Event& e) {
  ByteReader r(in, length);
  e.code = static_cast<EventCode>(r.u8());
  e.severity = static_cast<Severity>(r.u8());
  e.joint = r.u8();
  e.cmdSeq = r.u16();
  for (int32_t& a : e.args) a = r.i32();
  if (!r.ok()) return false;
  r.text(e.text, sizeof(e.text));
  return true;
}

size_t encodeAck(uint16_t ackSeq, AckStatus status, const char* text, uint8_t* out, size_t capacity) {
  ByteWriter w(out, capacity);
  w.u16(ackSeq);
  w.u8(static_cast<uint8_t>(status));
  w.text(text);
  return finish(w);
}

size_t encodeDone(uint16_t ackSeq, bool ok, uint8_t* out, size_t capacity) {
  ByteWriter w(out, capacity);
  w.u16(ackSeq);
  w.u8(ok ? 0 : 1);
  return finish(w);
}

size_t encodeConfig(uint16_t ackSeq, ConfigFormat format, const char* text, uint8_t* out, size_t capacity) {
  ByteWriter w(out, capacity);
  w.u16(ackSeq);
  w.u8(static_cast<uint8_t>(format));
  w.text(text);
  return finish(w);
}

size_t encodeLog(LogSource source, const char* text, size_t length, uint8_t* out, size_t capacity) {
  ByteWriter w(out, capacity);
  w.u8(static_cast<uint8_t>(source));
  w.bytes(reinterpret_cast<const uint8_t*>(text), length);
  return finish(w);
}

// ---- host -> device ------------------------------------------------------------

bool decodeHostHello(const uint8_t* in, size_t length, HostHello& h) {
  ByteReader r(in, length);
  h.protocol = r.u8();
  h.flags = r.u8();
  h.heartbeatTimeoutMs = r.u16();
  h.telemetryDivider = r.u8();
  h.slowDivider = r.u8();
  return r.ok();
}

bool decodeTelemConfig(const uint8_t* in, size_t length, TelemConfig& c) {
  ByteReader r(in, length);
  c.telemetryDivider = r.u8();
  c.slowDivider = r.u8();
  return r.ok();
}

}  // namespace hostlink
