// Host link v1 message types and payload layouts (docs/host-protocol.md).
//
// These are wire structs only: they know nothing about the controller. The
// firmware fills them from its Snapshot (src/host_telemetry.cpp); the Python
// host (host/src/teleop_host/protocol/messages.py) mirrors every layout, and
// both sides are tested against the same golden byte vectors.
//
// All integers are little-endian. str8 = u8 length + bytes. A trailing text
// field runs to the end of the payload.

#pragma once

#include <stddef.h>
#include <stdint.h>

#include "frame.h"

namespace hostlink {

constexpr uint8_t kMaxJoints = 9;
constexpr uint8_t kMaxEncoders = 9;
constexpr size_t kMaxText = 160;  // event / log text, NUL included

enum class MsgType : uint8_t {
  // host -> device
  HostHello = 0x01,
  Heartbeat = 0x02,
  Command = 0x03,
  Estop = 0x04,
  TelemConfig = 0x05,
  HostExit = 0x06,
  GetInfo = 0x07,
  // device -> host
  Hello = 0x81,
  Telemetry = 0x82,
  SlowStatus = 0x83,
  Event = 0x84,
  Ack = 0x85,
  Done = 0x86,
  Config = 0x87,
  Log = 0x88,
  Bye = 0x89,
};

constexpr uint8_t kFlagReplyExpected = 0x01;  // header flags

// ---- host -> device --------------------------------------------------------

constexpr uint8_t kHelloEstopOnLinkLoss = 0x01;

struct HostHello {  // 6 B
  uint8_t protocol = kProtocolVersion;
  uint8_t flags = kHelloEstopOnLinkLoss;
  uint16_t heartbeatTimeoutMs = 1000;
  uint8_t telemetryDivider = 1;  // control cycles per TELEMETRY frame, 0 = off
  uint8_t slowDivider = 10;      // control cycles per SLOW_STATUS frame, 0 = off
};

struct TelemConfig {  // 2 B
  uint8_t telemetryDivider = 1;
  uint8_t slowDivider = 10;
};

// ---- device -> host: HELLO -------------------------------------------------

struct JointInfo {
  uint8_t servoId = 0;
  uint8_t model = 0;  // feetech::ServoModel
  int16_t travelMin = 0;
  int16_t travelMax = 0;
  uint16_t vmax = 0;  // defaults from config.h
  uint16_t vsat = 0;
  uint16_t amax = 0;
  uint16_t maxSpeed = 0;  // fastest speed of the servo model, steps/s
  uint8_t encoderDefault = 0xFF;
  uint8_t enabledDefault = 0;
  float stepScale = 1.0f;     // native steps per SCS0009 step
  float degreesPerStep = 0.0f;
  const char* name = "";
};

struct NameTable {
  const char* const* names = nullptr;
  uint8_t count = 0;
};

struct DeviceInfo {
  const char* firmware = "";
  uint16_t controlHz = 100;
  float degreesPerCount = 0.0f;  // encoder
  uint8_t encoderCount = 0;
  uint16_t minCurrentLimitMa = 0;
  uint16_t maxCurrentLimitMa = 0;
  uint8_t minCurrentTripPct = 0;
  uint8_t maxCurrentTripPct = 0;
  uint8_t defaultCurrentTripPct = 0;
  int32_t minExoSpanCounts = 0;
  int32_t maxExoSpanCounts = 0;
  const JointInfo* joints = nullptr;
  uint8_t jointCount = 0;
  NameTable faults;   // index = teleop::Fault
  NameTable states;   // index = JointState
  NameTable events;   // index = EventCode
  NameTable models;   // index = feetech::ServoModel
};

// Returns the payload length, 0 if it does not fit.
size_t encodeHello(const DeviceInfo& info, uint8_t* out, size_t capacity);

// ---- device -> host: TELEMETRY (every control cycle) ------------------------

enum JointFlag : uint16_t {
  kJointEnabled = 1u << 0,
  kJointReady = 1u << 1,
  kJointRefCaptured = 1u << 2,
  kJointHomed = 1u << 3,
  kJointSingleTurn = 1u << 4,
  kJointEncFresh = 1u << 5,
  kJointContact = 1u << 6,
  kJointGoalSent = 1u << 7,
  kJointModelOk = 1u << 8,
  kJointServoOk = 1u << 9,     // last read attempt succeeded
  kJointTorqueOn = 1u << 10,
  kJointMoving = 1u << 11,
  kJointHasCurrent = 1u << 12,
  kJointServoFresh = 1u << 13,  // last good read within the stale limit
};

enum HandFlag : uint8_t {
  kHandTorqueOffConfirmed = 1u << 0,
  kHandMonitor = 1u << 1,
  kHandSweepActive = 1u << 2,
  kHandTelemetryDropped = 1u << 3,  // at least one cycle was lost since the previous frame
};

struct TelemetryJoint {  // 48 B on the wire
  uint8_t state = 0;
  uint8_t fault = 0;  // last fault that switched the joint off
  uint16_t flags = 0;
  uint16_t encRaw = 0;
  float exoCounts = 0.0f;  // unwrapped, relative to exoRef
  float exoFilteredCounts = 0.0f;
  float target = 0.0f;
  float goal = 0.0f;
  float sentGoal = 0.0f;
  float reference = 0.0f;
  uint16_t speedCommand = 0;  // steps/s
  int16_t gripLead = 0;
  int16_t position = 0;
  int16_t speed = 0;
  int16_t loadDeciPct = 0;  // 0.1 %
  int16_t currentMa = 0;
  uint8_t voltageDeciV = 0;  // 0.1 V
  int8_t temperatureC = 0;
  uint8_t status = 0;
  uint8_t errorByte = 0;
  uint16_t readAgeMs = 0;
};

struct Telemetry {  // 18 B + 48 B per joint
  uint32_t cycle = 0;
  uint32_t tUs = 0;  // micros() at the cycle's wake-up
  uint32_t uptimeMs = 0;
  uint8_t armedCount = 0;
  uint8_t handFault = 0;
  uint8_t handFlags = 0;
  uint16_t execUs = 0;
  uint8_t jointCount = 0;
  TelemetryJoint joints[kMaxJoints];
};

constexpr size_t kTelemetryHandSize = 18;
constexpr size_t kTelemetryJointSize = 48;
constexpr size_t telemetrySize(uint8_t joints) { return kTelemetryHandSize + joints * kTelemetryJointSize; }

size_t encodeTelemetry(const Telemetry& t, uint8_t* out, size_t capacity);
bool decodeTelemetry(const uint8_t* in, size_t length, Telemetry& t);

// ---- device -> host: SLOW_STATUS --------------------------------------------

struct SlowJoint {  // 32 B
  uint8_t enabled = 0;
  uint8_t encoderIndex = 0xFF;
  uint16_t exoRef = 0;
  int32_t exoB = 0;
  int16_t servoA = 0;
  int16_t servoB = 0;
  uint16_t vmax = 0;
  uint16_t amax = 0;
  uint16_t vsat = 0;
  uint16_t trackSteps = 0;
  uint16_t gripSteps = 0;
  uint16_t currentLimitMa = 0;
  uint8_t currentTripPct = 0;
  uint8_t detectedModel = 0;
  uint16_t effectiveCurrentMa = 0;
  uint32_t jumpRejects = 0;
};

struct SlowEncoder {  // 11 B
  uint8_t address = 0;
  uint8_t state = 0;  // AS5048BChannelState
  uint8_t valid = 0;
  uint16_t angle = 0;
  uint8_t agc = 0;
  uint8_t magnetOk = 0;
  uint32_t errors = 0;
};

struct SlowStatus {
  // Global settings (21 B)
  float leadMs = 0.0f;
  float minCutoffHz = 0.0f;
  float beta = 0.0f;
  float jumpDeg = 0.0f;
  float gripLoadPct = 0.0f;
  uint8_t loadTrip = 0;
  // Loop timing (32 B)
  uint32_t cycles = 0;
  uint32_t overruns = 0;
  uint32_t lastPeriodUs = 0;
  uint32_t maxJitterUs = 0;
  uint32_t p99JitterUs = 0;
  uint32_t lastExecUs = 0;
  uint32_t maxExecUs = 0;
  uint32_t averageExecUs = 0;
  // Servo bus (29 B)
  uint32_t busSent = 0;
  uint32_t busReceived = 0;
  uint32_t busTimeouts = 0;
  uint32_t busParseErrors = 0;
  uint32_t busEchoErrors = 0;
  uint32_t busLastReplyUs = 0;
  uint32_t busMaxReplyUs = 0;
  uint8_t busEcho = 0;
  // Encoder pass (12 B)
  uint32_t encoderAverageUs = 0;
  uint32_t encoderMaxUs = 0;
  uint32_t encoderMaxSkewUs = 0;
  // Host link health (16 B)
  uint32_t droppedEvents = 0;
  uint32_t droppedTelemetry = 0;
  uint32_t rxCrcErrors = 0;
  uint32_t rxMalformed = 0;
  // Monitor (4 B + 2 B per joint)
  uint8_t monitor = 0;
  uint16_t monitorOkMask = 0;
  // Per joint / encoder
  uint8_t jointCount = 0;
  int16_t monitorPosition[kMaxJoints] = {0};
  SlowJoint joints[kMaxJoints];
  uint8_t encoderCount = 0;
  SlowEncoder encoders[kMaxEncoders];
};

size_t encodeSlowStatus(const SlowStatus& s, uint8_t* out, size_t capacity);
bool decodeSlowStatus(const uint8_t* in, size_t length, SlowStatus& s);

// ---- device -> host: events, replies, text ----------------------------------

enum class Severity : uint8_t { Info = 0, Warning = 1, Error = 2 };

// Structured meaning of an EVENT; the text is the console message as printed
// in text mode. Args per code are listed in docs/host-protocol.md.
enum class EventCode : uint8_t {
  Generic = 0,
  Refused,         // a command was refused or rejected
  Armed,           // args: armed joint count
  Disarmed,        // joint (0 = all); args: fault, torque-off confirmed
  TorqueOff,       // E-stop / boot / link loss; args: confirmed
  Fault,           // idle / hand fault reported; args: fault
  JointState,      // joint state change; args: new state
  CalCaptured,     // args: 0 exo A, 1 exo B, 2 servo A, 3 servo B; value
  Homed,           // args: turn offset
  PingResult,      // joint; args: model, fw major, fw minor, ok
  LimitsRow,       // joint; args: servo lo, servo hi, match
  SweepRun,        // joint; args: command speed, measured speed x10, reached, max load x10
  SweepDone,       // joint; args: suggested vmax, suggested vsat, applicable, aborted
  ConfigSaved,     // args: ok
  ConfigLoaded,    // args: blob status
  Defaults,
  SettingChanged,  // joint (0 = global)
  Monitor,         // args: on
  Count
};

struct Event {
  EventCode code = EventCode::Generic;
  Severity severity = Severity::Info;
  uint8_t joint = 0;    // servo id, 0 = none
  uint16_t cmdSeq = 0;  // command that caused it, 0 = unsolicited
  int32_t args[4] = {0, 0, 0, 0};
  char text[kMaxText] = "";
};

size_t encodeEvent(const Event& e, uint8_t* out, size_t capacity);
bool decodeEvent(const uint8_t* in, size_t length, Event& e);

enum class AckStatus : uint8_t { Accepted = 0, ParseError = 1, Busy = 2, BadFrame = 3 };

size_t encodeAck(uint16_t ackSeq, AckStatus status, const char* text, uint8_t* out, size_t capacity);
size_t encodeDone(uint16_t ackSeq, bool ok, uint8_t* out, size_t capacity);

enum class ConfigFormat : uint8_t { DumpText = 0 };
size_t encodeConfig(uint16_t ackSeq, ConfigFormat format, const char* text, uint8_t* out, size_t capacity);

enum class LogSource : uint8_t { Console = 0, EspLog = 1, Boot = 2 };
size_t encodeLog(LogSource source, const char* text, size_t length, uint8_t* out, size_t capacity);

enum class ByeReason : uint8_t { HostExit = 0, HeartbeatTimeout = 1 };

// ---- host -> device decoding -------------------------------------------------

bool decodeHostHello(const uint8_t* in, size_t length, HostHello& h);
bool decodeTelemConfig(const uint8_t* in, size_t length, TelemConfig& c);

}  // namespace hostlink
