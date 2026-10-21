// ServoBus for one Feetech bus carrying SCS0009 and HLS3606 servos at once.
// Same packet format and baud rate for both; every id is encoded and decoded
// with its own model (servo_models.h).

#pragma once

#include "feetech_port.h"
#include "servo_bus.h"

class FeetechMixedBus : public ServoBus {
 public:
  explicit FeetechMixedBus(FeetechPort& port) : port_(port) {}

  feetech::ServoModel model(uint8_t id) const override;
  void setModel(uint8_t id, feetech::ServoModel model) override;
  bool identify(uint8_t id, feetech::ServoModel& model, uint8_t& firmwareMajor,
                uint8_t& firmwareMinor) override;

  bool ping(uint8_t id) override;
  bool readState(uint8_t id, ServoState& out) override;
  bool writeTorque(uint8_t id, bool on) override;
  void syncTorqueOff(const uint8_t* ids, uint8_t count) override;

  void setGoalCurrent(uint8_t id, uint16_t currentRaw) override;
  void syncWriteGoal(const uint8_t* ids, const int16_t* positions, const float* stepsPerSec,
                     uint8_t count) override;
  bool writeGoal(uint8_t id, int16_t position, float stepsPerSec) override;

  bool readArmInfo(uint8_t id, feetech::ArmInfo& out) override;
  bool readAngleLimits(uint8_t id, int16_t& lo, int16_t& hi) override;
  bool writeAngleLimits(uint8_t id, int16_t lo, int16_t hi) override;

 private:
  static constexpr uint8_t kIds = 254;  // 0..253

  bool readRegisters(uint8_t id, uint8_t address, uint8_t length, uint8_t* out, uint8_t* error);
  bool writeRegisters(uint8_t id, uint8_t address, const uint8_t* data, uint8_t length);

  FeetechPort& port_;
  feetech::ServoModel models_[kIds] = {};
  uint16_t goalCurrent_[kIds] = {};
};
