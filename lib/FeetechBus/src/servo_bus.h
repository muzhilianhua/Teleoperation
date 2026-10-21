// Servo bus interface used by the control loop, calibration and safety.
// Each servo id has its own model (SCS0009 or HLS3606, see servo_models.h);
// register addresses, byte order, position range and speed units stay in the
// implementation. Positions are in the servo's own steps and speeds in its
// own steps per second.

#pragma once

#include <servo_models.h>  // ServoState, ServoModel, ArmInfo
#include <stdint.h>

class ServoBus {
 public:
  virtual ~ServoBus() = default;

  virtual feetech::ServoModel model(uint8_t id) const = 0;
  virtual void setModel(uint8_t id, feetech::ServoModel model) = 0;
  // Reads registers 0..4: the model actually on the bus (Unknown if
  // unsupported) and its firmware version. Does not change setModel().
  virtual bool identify(uint8_t id, feetech::ServoModel& model, uint8_t& firmwareMajor,
                        uint8_t& firmwareMinor) = 0;

  virtual bool ping(uint8_t id) = 0;
  virtual bool readState(uint8_t id, ServoState& out) = 0;
  virtual bool writeTorque(uint8_t id, bool on) = 0;
  // Broadcast torque-off; servos do not reply, verify with readState().
  virtual void syncTorqueOff(const uint8_t* ids, uint8_t count) = 0;

  // HLS goal current (6.5 mA units) sent with every goal of that id.
  virtual void setGoalCurrent(uint8_t id, uint16_t currentRaw) = 0;
  // One frame for all listed ids, each entry encoded for its own model.
  // Speeds are native steps/s; the implementation never sends a speed
  // register of 0 (SCS: full speed, HLS: stop).
  virtual void syncWriteGoal(const uint8_t* ids, const int16_t* positions, const float* stepsPerSec,
                             uint8_t count) = 0;
  // Single goal write with reply; used by arming and the speed sweep.
  virtual bool writeGoal(uint8_t id, int16_t position, float stepsPerSec) = 0;

  // HLS limits / protection current / resolution / mode (registers 9..33).
  virtual bool readArmInfo(uint8_t id, feetech::ArmInfo& out) = 0;

  // The servo's own angle limits (EEPROM): a backstop that clamps goals even
  // if the firmware were wrong; on a factory HLS3606 (0/0, multi-turn) writing
  // them switches it to single-turn. Writing requires torque off; the
  // implementation unlocks, writes, re-locks and reads back.
  virtual bool readAngleLimits(uint8_t id, int16_t& lo, int16_t& hi) = 0;
  virtual bool writeAngleLimits(uint8_t id, int16_t lo, int16_t hi) = 0;
};
