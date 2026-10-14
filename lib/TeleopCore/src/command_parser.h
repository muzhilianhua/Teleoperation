// Console command grammar, settings edits and the `dump` format.
//
// Kept free of Arduino so the round trip "settings -> dump text -> parse ->
// apply -> settings" is unit-tested on the host: a dump pasted back into the
// console must reproduce the configuration exactly.
//
// Joints are addressed by servo id (1..8 on this hand).

#pragma once

#include <stddef.h>
#include <stdint.h>

#include "joint_config.h"

namespace teleop {

enum class Verb : uint8_t {
  None,      // empty line
  Comment,   // line starting with '#'
  Help,
  Status,
  Stats,
  StatsReset,
  Enc,
  Ping,
  Arm,       // joint 0 = every enabled joint
  Disarm,    // joint 0 = every joint
  Estop,
  Joints,
  Monitor,   // flag = on/off
  Plot,      // joint (0 = off), flag = on/off
  Cal,       // joint, flag = endpoint B
  CalHand,   // joint, flag = endpoint B
  Set,
  Save,
  Load,
  Defaults,
  Dump,
  LimitsCheck,
  LimitsWrite,  // joint 0 = all joints
};

enum class SetKey : uint8_t {
  None,
  Enable,     // flag
  Encoder,    // values[0], kNoEncoder for "none"
  Exo,        // values[0..1] = raw counts at A and B
  Servo,      // values[0..1] = servo counts at A and B
  VMax,
  AMax,
  VSat,
  Track,
  Grip,
  Lead,       // global
  MinCutoff,  // global
  Beta,       // global
  Jump,       // global
  GripLoad,   // global
  LoadTrip,   // global, values[0] = 1 (on) / 0 (off)
};

struct ParsedCommand {
  Verb verb = Verb::None;
  uint8_t joint = 0;  // servo id, 0 = none
  SetKey key = SetKey::None;
  bool flag = false;
  float values[2] = {0.0f, 0.0f};
  uint8_t valueCount = 0;
};

// nullptr on success, otherwise a message for the operator.
const char* parseCommand(const char* line, ParsedCommand& out);

bool isGlobalKey(SetKey key);
// Commands that change calibration, mapping or persistent state, or that need
// the bus for long transactions, are refused while torque is on.
bool requiresDisarmed(const ParsedCommand& command);

int jointIndexForServo(const JointDefaults* defaults, uint8_t count, uint8_t servoId);

// Applies a Set command; validates the result and leaves everything untouched
// on error. Returns nullptr on success.
const char* applySetting(const ParsedCommand& command, JointSettings* joints,
                         const JointDefaults* defaults, uint8_t count, GlobalSettings& global,
                         uint8_t encoderCount);

// Writes the whole configuration as console commands, starting with
// `defaults`. Returns the text length, or 0 if capacity was too small.
size_t formatDump(const JointSettings* joints, const JointDefaults* defaults, uint8_t count,
                  const GlobalSettings& global, char* out, size_t capacity);

}  // namespace teleop
