// AS5048B 14-bit magnetic rotary encoder - I2C driver for Arduino (ESP32-S3).
//
// Based on the ams AS5048A/AS5048B datasheet v1-11 (2018-Jan-29) and the
// register handling of the open-source sosandroid/AMS_AS5048B library.
//
// Angle convention:
//   - "raw" values are exactly what the chip reports (already corrected by the
//     chip's zero-position register, 0..16383). Verify the physical rotation
//     direction on your mechanics and use setReversed() if needed.
//   - Non-raw values additionally apply the software direction (setReversed).

#pragma once

#include <Arduino.h>
#include <Wire.h>

#ifndef AS5048B_DEFAULT_SDA
#define AS5048B_DEFAULT_SDA SDA
#endif
#ifndef AS5048B_DEFAULT_SCL
#define AS5048B_DEFAULT_SCL SCL
#endif
#ifndef AS5048B_DEFAULT_I2C_FREQ
// The ESP32-S3 I2C controller tops out at 1 MHz; the chip's 3.4 MHz HS mode is not usable.
#define AS5048B_DEFAULT_I2C_FREQ 400000UL
#endif

enum class AS5048BStatus : uint8_t {
  Ok = 0,
  I2cDataTooLong = 1,   // Wire.endTransmission() == 1
  I2cNackAddress = 2,   // Wire.endTransmission() == 2
  I2cNackData = 3,      // Wire.endTransmission() == 3
  I2cOther = 4,         // Wire.endTransmission() == 4
  I2cTimeout = 5,       // Wire.endTransmission() == 5
  ReadShort,            // requestFrom() returned fewer bytes than requested (incl. NACK on ESP32)
  InvalidArgument,
  NotConfirmed,         // OTP call without the correct confirmation key
  MagnetError,          // diagnostics report an unusable field (COF / weak / strong / OCF not set)
  VerifyFailed,         // read-back after write / burn did not match
  BurnTimeout,          // OTP Burn bit did not clear in time; power-cycle and rescan the bus
};

struct AS5048BDiagnostics {
  uint8_t raw = 0;        // register 0xFB
  bool ocf = false;       // Offset Compensation Finished, must be 1 after power-up
  bool cof = false;       // CORDIC overflow, angle/magnitude invalid while set
  bool compLow = false;   // magnetic field too strong
  bool compHigh = false;  // magnetic field too weak

  bool magnetOk() const { return ocf && !cof && !compLow && !compHigh; }
};

struct AS5048BReading {
  uint16_t angleRaw = 0;   // 0..16383, chip zero-corrected
  uint16_t angle = 0;      // angleRaw with software direction applied
  uint16_t magnitude = 0;  // 14-bit CORDIC magnitude
  uint8_t agc = 0;         // 0 = strong field ... 255 = weak field
  AS5048BDiagnostics diag;

  float angleDegrees() const;
};

class AS5048B {
 public:
  static constexpr uint8_t kBaseAddress = 0x40;
  static constexpr uint16_t kResolution = 16384;  // 2^14 counts per turn
  static constexpr uint16_t kMask = kResolution - 1;
  // Pass this to the burn*OTP() functions to confirm an irreversible operation.
  static constexpr uint32_t kOtpConfirmKey = 0xA5048B0Fu;

  // Register map (I2C variant, datasheet Figure 31)
  static constexpr uint8_t kRegProgControl = 0x03;
  static constexpr uint8_t kRegI2cAddress = 0x15;
  static constexpr uint8_t kRegZeroHigh = 0x16;  // zero<13:6>
  static constexpr uint8_t kRegZeroLow = 0x17;   // zero<5:0>
  static constexpr uint8_t kRegAgc = 0xFA;
  static constexpr uint8_t kRegDiagnostics = 0xFB;
  static constexpr uint8_t kRegMagnitudeHigh = 0xFC;
  static constexpr uint8_t kRegMagnitudeLow = 0xFD;
  static constexpr uint8_t kRegAngleHigh = 0xFE;
  static constexpr uint8_t kRegAngleLow = 0xFF;

  explicit AS5048B(uint8_t address = kBaseAddress, TwoWire& wire = Wire);

  // 7-bit address for factory OTP content and the given A1/A2 pin levels.
  static uint8_t addressFromPins(bool a1High, bool a2High);
  // Value register 0x15 must hold for a 7-bit address (bit 4 is inverted in the chip).
  static uint8_t addressRegisterValue(uint8_t address);

  // Probes addresses [first, last] on the bus; returns the number found.
  static uint8_t scan(TwoWire& wire, uint8_t* found, uint8_t maxFound,
                      uint8_t first = kBaseAddress, uint8_t last = kBaseAddress + 3);

  static float countsToDegrees(uint16_t counts);
  static float countsToRadians(uint16_t counts);
  static const char* statusToString(AS5048BStatus status);

  // ---- Lifecycle ----------------------------------------------------------
  // initBus=false: the caller already ran wire.begin() (e.g. shared bus), pins/freq ignored.
  // Returns true when the chip ACKs and offset compensation (OCF) has finished.
  bool begin(int sda = AS5048B_DEFAULT_SDA, int scl = AS5048B_DEFAULT_SCL,
             uint32_t frequency = AS5048B_DEFAULT_I2C_FREQ, bool initBus = true);
  bool isConnected();

  uint8_t address() const { return address_; }
  AS5048BStatus lastStatus() const { return lastStatus_; }

  // ---- Measurements -------------------------------------------------------
  AS5048BStatus readAngleRaw(uint16_t& counts);
  AS5048BStatus readAngle(uint16_t& counts);  // direction applied
  AS5048BStatus readMagnitude(uint16_t& magnitude);
  AS5048BStatus readAgc(uint8_t& agc);
  AS5048BStatus readDiagnostics(AS5048BDiagnostics& diag);
  // AGC + diagnostics + magnitude + angle in a single 6-byte burst (0xFA..0xFF).
  AS5048BStatus readAll(AS5048BReading& reading);

  // Convenience wrappers; return NAN on I2C failure (check lastStatus()).
  float getAngleDegrees();
  float getAngleRadians();

  // Software direction: true flips the sign so clockwise becomes positive.
  // Changing it restarts multi-turn tracking (turn count is lost).
  void setReversed(bool reversed);
  bool isReversed() const { return reversed_; }

  // ---- Multi-turn tracking ------------------------------------------------
  // Call update() periodically; the shaft must turn less than half a revolution
  // between two calls or the turn count will be wrong. The first update() after
  // begin() seeds the cumulative value with the current absolute angle.
  AS5048BStatus update();
  // Feeds an angle obtained elsewhere (e.g. readAll() or a group read) into the
  // multi-turn tracker; must be direction-applied, like readAngle().
  void trackAngle(uint16_t angle);
  AS5048BStatus resetTurns();  // makes the current position cumulative 0
  int32_t getCumulativeCounts() const { return cumulative_; }
  int32_t getTurns() const;
  float getCumulativeDegrees() const;

  // ---- Zero position (volatile, lost on power cycle) ----------------------
  AS5048BStatus readZeroPosition(uint16_t& zero);
  AS5048BStatus writeZeroPosition(uint16_t zero);
  // Datasheet programming steps 1-3: clear zero, read angle, write it back.
  // capturedZero (optional) receives the value written.
  AS5048BStatus setZeroHere(uint16_t* capturedZero = nullptr);

  // ---- I2C address (volatile) ---------------------------------------------
  AS5048BStatus readAddressRegister(uint8_t& value);
  // Moves the chip to newAddress (7-bit). The two LSBs are set by the A1/A2
  // pins and must match the current address. On success address() is updated.
  AS5048BStatus changeAddress(uint8_t newAddress);

  // ---- OTP (IRREVERSIBLE, one-time) ---------------------------------------
  // Both burns program the whole customer OTP block from the current register
  // contents (zero position AND slave address). Requires VDD5V operation, or
  // 3.3 V mode at >= 3.3 V with 10 uF on VDD3V. Keep the magnet still.
  //
  // Burns the zero position currently held in 0x16/0x17 (use setZeroHere()
  // first), then runs the datasheet verify sequence (steps 4-8).
  AS5048BStatus burnZeroPositionOTP(uint32_t confirmKey);
  // Burns the slave address currently held in 0x15. Call changeAddress() in the
  // same power session first; the chip is then addressed at the new address.
  // Refuses (InvalidArgument) unless 0x15 matches address(), the address is not
  // the factory default and the zero registers are 0 (so no zero gets burned).
  // Sequence per ams AN000386: 0x03<-0xFD (special programming mode),
  // 0x03<-0x08 (burn), poll Burn bit until clear, 0x03<-0x00, then
  // 0x03<-0x40 (Verify: reload OTP) and compare 0x15. Power-cycle afterwards.
  AS5048BStatus burnAddressOTP(uint32_t confirmKey);

  // ---- Low level ----------------------------------------------------------
  AS5048BStatus readRegisters(uint8_t reg, uint8_t* buffer, size_t length);
  AS5048BStatus writeRegister(uint8_t reg, uint8_t value);

 private:
  static uint16_t combine14(uint8_t high, uint8_t low);
  uint16_t applyDirection(uint16_t raw) const;
  AS5048BStatus read14(uint8_t highReg, uint16_t& value);
  AS5048BStatus setStatus(AS5048BStatus status);

  TwoWire& wire_;
  uint8_t address_;
  AS5048BStatus lastStatus_ = AS5048BStatus::Ok;
  bool reversed_ = false;

  bool tracking_ = false;
  uint16_t lastAngle_ = 0;
  int32_t cumulative_ = 0;
};
