// Series-independent servo bus interface. Upper layers (control loop,
// calibration, safety) use only this; register addresses, byte order and
// position resolution live in the backend (ScsBus now, HlsBus in M5).

#pragma once

#include <servo_models.h>  // ServoState
#include <stdint.h>

class ServoBus {
 public:
  virtual ~ServoBus() = default;

  virtual const char* seriesName() const = 0;
  virtual uint16_t expectedModel() const = 0;
  virtual int16_t positionMin() const = 0;
  virtual int16_t positionMax() const = 0;

  // model is read from the servo when non-null.
  virtual bool ping(uint8_t id, uint16_t* model = nullptr) = 0;
  virtual bool readState(uint8_t id, ServoState& out) = 0;
  virtual bool writeTorque(uint8_t id, bool on) = 0;
  // Broadcast torque-off; servos do not reply, verify with readState().
  virtual void syncTorqueOff(const uint8_t* ids, uint8_t count) = 0;
  // Goals go to every listed id in one frame; speed is clamped to >= 1
  // because 0 means "maximum speed" on Feetech servos.
  virtual void syncWriteGoal(const uint8_t* ids, const int16_t* positions,
                             const uint16_t* speeds, uint8_t count) = 0;
  // Single goal write with reply; used only by the arming sequence.
  virtual bool writeGoal(uint8_t id, int16_t position, uint16_t speed) = 0;

  // The servo's own angle limits (EEPROM): a backstop that clamps goals even
  // if the firmware were wrong. Writing requires torque off; the backend
  // unlocks, writes, re-locks and reads back. Returns false on any failure.
  virtual bool readAngleLimits(uint8_t id, int16_t& lo, int16_t& hi) = 0;
  virtual bool writeAngleLimits(uint8_t id, int16_t lo, int16_t hi) = 0;
};
