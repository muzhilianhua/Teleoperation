// Feetech SCS series backend (SCS0009): big-endian, 0..1023 over ~300 deg.
// Register map: servo_bringup/dexhand_servo/registers.py.

#pragma once

#include "feetech_port.h"
#include "servo_bus.h"

class ScsBus : public ServoBus {
 public:
  explicit ScsBus(FeetechPort& port) : port_(port) {}

  const char* seriesName() const override { return "SCS"; }
  uint16_t expectedModel() const override { return 1284; }  // SCS0009 reports 0x0504
  int16_t positionMin() const override { return 0; }
  int16_t positionMax() const override { return 1023; }

  bool ping(uint8_t id, uint16_t* model = nullptr) override;
  bool readState(uint8_t id, ServoState& out) override;
  bool writeTorque(uint8_t id, bool on) override;
  void syncTorqueOff(const uint8_t* ids, uint8_t count) override;
  void syncWriteGoal(const uint8_t* ids, const int16_t* positions, const uint16_t* speeds,
                     uint8_t count) override;
  bool writeGoal(uint8_t id, int16_t position, uint16_t speed) override;

 private:
  bool readRegisters(uint8_t id, uint8_t address, uint8_t length, uint8_t* out, uint8_t* error);
  bool writeRegisters(uint8_t id, uint8_t address, const uint8_t* data, uint8_t length);
  void fillGoalBlock(uint8_t* block, int16_t position, uint16_t speed) const;

  FeetechPort& port_;
};
