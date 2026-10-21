// Servo models on the hand's bus: what differs between the SCS0009 and the
// HLS3606. Both speak the same Feetech packet protocol (feetech_packet.h), so
// one transport serves both; the models differ in the meaning of the bytes.
//
//                     SCS0009                     HLS3606
//   identify          END (reg 2) = 1, model 1284 END = 0, servo major (reg 3) = 10
//   byte order        big-endian                  little-endian
//   position          0..1023 over ~300 deg       0..4095 per turn, BIT15 = sign
//   goal block 42..47 position, time (0), speed   position, goal current, speed
//   speed register    ~steps/s, 0 = full speed    ~49.97 steps/s per unit, 0 = stop
//   status from 40    27 bytes                    31 bytes (+ present goal, current)
//   lock register     48                          55
//
// Facts from DEX_hand_software (models.py, docs/hls-memory-table.md) and a
// real HLS3606 register dump (firmware 3.43). Pure C++ for host tests.

#pragma once

#include <stddef.h>
#include <stdint.h>

#include "feetech_packet.h"

// One decoded status block (registers from 40), in the servo's own steps.
struct ServoState {
  bool torqueOn = false;
  int16_t goal = 0;
  uint16_t goalSpeed = 0;   // raw speed register
  int16_t position = 0;     // native steps (HLS may be negative in multi-turn mode)
  int16_t speed = 0;        // signed, native steps/s
  float loadPct = 0.0f;     // signed, percent of max torque
  float voltage = 0.0f;     // volts
  int16_t temperatureC = 0;
  uint8_t status = 0;       // servo status register, 0 when healthy
  bool moving = false;
  uint8_t errorByte = 0;    // status-packet error byte
  bool hasCurrent = false;  // HLS only
  float currentMa = 0.0f;   // signed
};

namespace feetech {

enum class ServoModel : uint8_t { Unknown = 0, Scs0009 = 1, Hls3606 = 2 };

struct ModelInfo {
  const char* name;
  Endian endian;
  int16_t positionMax;        // largest single-turn position
  uint8_t statusBlockLength;  // bytes read from register 40
  uint8_t lockRegister;
  float stepScale;            // native steps per SCS0009 step (same angle)
  float stepsPerSpeedUnit;    // native steps/s per speed-register unit
  uint16_t maxSpeedRegister;
  float currentMaPerUnit;     // 0: no usable current reading / goal current
};

constexpr uint8_t kRegVersion = 0;          // registers 0..4 identify the model
constexpr uint8_t kVersionLength = 5;
constexpr uint8_t kRegAngleLimits = 9;      // min 9..10, max 11..12
constexpr uint8_t kRegTorqueEnable = 40;
constexpr uint8_t kRegGoalBlock = 42;       // 6 bytes on both models
constexpr uint8_t kGoalBlockLength = 6;
constexpr uint8_t kRegStatusBlock = 40;
// HLS arming info: registers 9..33 (limits, protection current 28, resolution 30, mode 33).
constexpr uint8_t kRegArmInfo = 9;
constexpr uint8_t kArmInfoLength = 25;
constexpr uint16_t kHlsMaxGoalCurrent = 2047;  // 6.5 mA units

const ModelInfo& modelInfo(ServoModel model);
const char* modelName(ServoModel model);

// From registers 0..4 (kVersionLength bytes).
ServoModel identifyModel(const uint8_t* versionRegs);

// Speed register for a speed in native steps/s: rounded up, never 0 (SCS 0 =
// full speed, HLS 0 = stop), never above the model's maximum.
uint16_t speedRegister(ServoModel model, float stepsPerSec);
float stepsPerSecond(ServoModel model, uint16_t speedRegister);

// Registers 42..47 for one servo. currentRaw is the HLS goal current in
// 6.5 mA units (ignored on the SCS, whose word is the goal time, always 0).
void encodeGoal(ServoModel model, int16_t position, uint16_t currentRaw, float stepsPerSec,
                uint8_t out[kGoalBlockLength]);

// Status block read from register 40 (modelInfo().statusBlockLength bytes).
bool decodeStatus(ServoModel model, const uint8_t* block, size_t length, ServoState& out);

struct ArmInfo {
  int16_t minLimit = 0;
  int16_t maxLimit = 0;
  uint16_t protectionCurrent = 0;  // 6.5 mA units
  uint8_t resolution = 0;
  uint8_t mode = 0;
};
// HLS registers 9..33 (kArmInfoLength bytes).
bool decodeArmInfo(ServoModel model, const uint8_t* regs, size_t length, ArmInfo& out);

}  // namespace feetech
