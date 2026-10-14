#include "AS5048B.h"

#include <math.h>

namespace {

constexpr uint8_t kProgEnable = 0x01;
constexpr uint8_t kProgBurn = 0x08;
constexpr uint8_t kProgVerify = 0x40;
// Special programming mode used for slave-address OTP (ams app note / sosandroid).
constexpr uint8_t kProgAddressEnable = 0xFD;
constexpr uint8_t kProgIdle = 0x00;

// Address register bit 4 is inverted inside the chip (datasheet Figure 32).
constexpr uint8_t kAddressInvertBit = 0x10;
constexpr uint8_t kAddressRegMask = 0x1F;

constexpr uint32_t kStartupTimeoutMs = 20;  // datasheet t_startup max 10 ms
constexpr uint32_t kOtpStepDelayMs = 10;
// AN000386 gives no burn duration; poll generously before declaring a timeout.
constexpr uint32_t kBurnPollTimeoutMs = 200;
// 0x00-0x07 and 0x78-0x7F are reserved by the I2C specification.
constexpr uint8_t kFirstUsableAddress = 0x08;
constexpr uint8_t kLastUsableAddress = 0x77;
// Output noise is ~2.7 LSB rms, so allow a few sigma around zero after a burn.
constexpr int32_t kZeroVerifyTolerance = 32;
constexpr uint8_t kZeroCaptureSamples = 8;

int32_t wrapDelta(int32_t delta) {
  const int32_t half = AS5048B::kResolution / 2;
  if (delta >= half) delta -= AS5048B::kResolution;
  if (delta < -half) delta += AS5048B::kResolution;
  return delta;
}

}  // namespace

float AS5048BReading::angleDegrees() const { return AS5048B::countsToDegrees(angle); }

AS5048B::AS5048B(uint8_t address, TwoWire& wire) : wire_(wire), address_(address) {}

uint8_t AS5048B::addressFromPins(bool a1High, bool a2High) {
  return kBaseAddress | (a2High ? 0x02 : 0x00) | (a1High ? 0x01 : 0x00);
}

uint8_t AS5048B::addressRegisterValue(uint8_t address) {
  return ((address >> 2) ^ kAddressInvertBit) & kAddressRegMask;
}

uint8_t AS5048B::scan(TwoWire& wire, uint8_t* found, uint8_t maxFound, uint8_t first,
                      uint8_t last) {
  uint8_t count = 0;
  for (uint16_t addr = first; addr <= last && addr <= 0x7F; ++addr) {
    wire.beginTransmission(static_cast<uint8_t>(addr));
    if (wire.endTransmission() == 0) {
      if (found != nullptr && count < maxFound) found[count] = static_cast<uint8_t>(addr);
      ++count;
    }
  }
  return count;
}

float AS5048B::countsToDegrees(uint16_t counts) {
  return static_cast<float>(counts & kMask) * (360.0f / kResolution);
}

float AS5048B::countsToRadians(uint16_t counts) {
  return static_cast<float>(counts & kMask) * (TWO_PI / kResolution);
}

const char* AS5048B::statusToString(AS5048BStatus status) {
  switch (status) {
    case AS5048BStatus::Ok: return "OK";
    case AS5048BStatus::I2cDataTooLong: return "I2C data too long";
    case AS5048BStatus::I2cNackAddress: return "I2C NACK on address";
    case AS5048BStatus::I2cNackData: return "I2C NACK on data";
    case AS5048BStatus::I2cOther: return "I2C error";
    case AS5048BStatus::I2cTimeout: return "I2C timeout";
    case AS5048BStatus::ReadShort: return "I2C short read (device not responding?)";
    case AS5048BStatus::InvalidArgument: return "invalid argument";
    case AS5048BStatus::NotConfirmed: return "OTP not confirmed";
    case AS5048BStatus::MagnetError: return "magnet/diagnostics error";
    case AS5048BStatus::VerifyFailed: return "verify failed";
    case AS5048BStatus::BurnTimeout: return "OTP burn timeout (power-cycle, rescan)";
  }
  return "unknown";
}

// ---- Lifecycle --------------------------------------------------------------

bool AS5048B::begin(int sda, int scl, uint32_t frequency, bool initBus) {
  tracking_ = false;
  if (initBus && !wire_.begin(sda, scl, frequency)) {
    setStatus(AS5048BStatus::I2cOther);
    return false;
  }
  if (!isConnected()) return false;

  const uint32_t start = millis();
  AS5048BDiagnostics diag;
  do {
    if (readDiagnostics(diag) != AS5048BStatus::Ok) return false;
    if (diag.ocf) return true;
    delay(1);
  } while (millis() - start < kStartupTimeoutMs);

  setStatus(AS5048BStatus::MagnetError);
  return false;
}

bool AS5048B::isConnected() {
  wire_.beginTransmission(address_);
  const uint8_t err = wire_.endTransmission();
  setStatus(err == 0 ? AS5048BStatus::Ok
                     : static_cast<AS5048BStatus>(err <= 5 ? err : 4));
  return err == 0;
}

// ---- Measurements -----------------------------------------------------------

AS5048BStatus AS5048B::readAngleRaw(uint16_t& counts) {
  return read14(kRegAngleHigh, counts);
}

AS5048BStatus AS5048B::readAngle(uint16_t& counts) {
  uint16_t raw = 0;
  const AS5048BStatus status = readAngleRaw(raw);
  if (status == AS5048BStatus::Ok) counts = applyDirection(raw);
  return status;
}

AS5048BStatus AS5048B::readMagnitude(uint16_t& magnitude) {
  return read14(kRegMagnitudeHigh, magnitude);
}

AS5048BStatus AS5048B::readAgc(uint8_t& agc) { return readRegisters(kRegAgc, &agc, 1); }

AS5048BStatus AS5048B::readDiagnostics(AS5048BDiagnostics& diag) {
  uint8_t value = 0;
  const AS5048BStatus status = readRegisters(kRegDiagnostics, &value, 1);
  if (status != AS5048BStatus::Ok) return status;
  diag.raw = value;
  diag.ocf = value & 0x01;
  diag.cof = value & 0x02;
  diag.compLow = value & 0x04;
  diag.compHigh = value & 0x08;
  return status;
}

AS5048BStatus AS5048B::readAll(AS5048BReading& reading) {
  uint8_t buf[6];
  const AS5048BStatus status = readRegisters(kRegAgc, buf, sizeof(buf));
  if (status != AS5048BStatus::Ok) return status;
  reading.agc = buf[0];
  reading.diag.raw = buf[1];
  reading.diag.ocf = buf[1] & 0x01;
  reading.diag.cof = buf[1] & 0x02;
  reading.diag.compLow = buf[1] & 0x04;
  reading.diag.compHigh = buf[1] & 0x08;
  reading.magnitude = combine14(buf[2], buf[3]);
  reading.angleRaw = combine14(buf[4], buf[5]);
  reading.angle = applyDirection(reading.angleRaw);
  return status;
}

float AS5048B::getAngleDegrees() {
  uint16_t counts = 0;
  return readAngle(counts) == AS5048BStatus::Ok ? countsToDegrees(counts) : NAN;
}

float AS5048B::getAngleRadians() {
  uint16_t counts = 0;
  return readAngle(counts) == AS5048BStatus::Ok ? countsToRadians(counts) : NAN;
}

void AS5048B::setReversed(bool reversed) {
  if (reversed != reversed_) tracking_ = false;
  reversed_ = reversed;
}

// ---- Multi-turn tracking ----------------------------------------------------

AS5048BStatus AS5048B::update() {
  uint16_t angle = 0;
  const AS5048BStatus status = readAngle(angle);
  if (status == AS5048BStatus::Ok) trackAngle(angle);
  return status;
}

void AS5048B::trackAngle(uint16_t angle) {
  angle &= kMask;
  if (!tracking_) {
    cumulative_ = angle;
    tracking_ = true;
  } else {
    cumulative_ += wrapDelta(static_cast<int32_t>(angle) - lastAngle_);
  }
  lastAngle_ = angle;
}

AS5048BStatus AS5048B::resetTurns() {
  uint16_t angle = 0;
  const AS5048BStatus status = readAngle(angle);
  if (status != AS5048BStatus::Ok) return status;
  lastAngle_ = angle;
  cumulative_ = 0;
  tracking_ = true;
  return status;
}

int32_t AS5048B::getTurns() const {
  // Floor division so that -1 count reads as turn -1, not 0.
  return cumulative_ >= 0 ? cumulative_ / kResolution
                          : -((-cumulative_ + kResolution - 1) / kResolution);
}

float AS5048B::getCumulativeDegrees() const {
  return static_cast<float>(cumulative_) * (360.0f / kResolution);
}

// ---- Zero position ----------------------------------------------------------

AS5048BStatus AS5048B::readZeroPosition(uint16_t& zero) { return read14(kRegZeroHigh, zero); }

AS5048BStatus AS5048B::writeZeroPosition(uint16_t zero) {
  if (zero > kMask) return setStatus(AS5048BStatus::InvalidArgument);
  AS5048BStatus status = writeRegister(kRegZeroHigh, static_cast<uint8_t>(zero >> 6));
  if (status != AS5048BStatus::Ok) return status;
  status = writeRegister(kRegZeroLow, static_cast<uint8_t>(zero & 0x3F));
  if (status != AS5048BStatus::Ok) return status;

  uint16_t readBack = 0;
  status = readZeroPosition(readBack);
  if (status != AS5048BStatus::Ok) return status;
  return setStatus(readBack == zero ? AS5048BStatus::Ok : AS5048BStatus::VerifyFailed);
}

AS5048BStatus AS5048B::setZeroHere(uint16_t* capturedZero) {
  AS5048BStatus status = writeZeroPosition(0);
  if (status != AS5048BStatus::Ok) return status;
  // Let at least one new sample through (propagation delay ~100 us).
  delay(2);

  AS5048BDiagnostics diag;
  status = readDiagnostics(diag);
  if (status != AS5048BStatus::Ok) return status;
  if (!diag.magnetOk()) return setStatus(AS5048BStatus::MagnetError);

  // Average a few samples around the first one so wrap-around at 0/16383 is safe.
  uint16_t first = 0;
  status = readAngleRaw(first);
  if (status != AS5048BStatus::Ok) return status;
  int32_t sum = 0;
  for (uint8_t i = 1; i < kZeroCaptureSamples; ++i) {
    delayMicroseconds(200);
    uint16_t sample = 0;
    status = readAngleRaw(sample);
    if (status != AS5048BStatus::Ok) return status;
    sum += wrapDelta(static_cast<int32_t>(sample) - first);
  }
  const uint16_t zero =
      static_cast<uint16_t>((first + sum / kZeroCaptureSamples + kResolution) & kMask);

  status = writeZeroPosition(zero);
  if (status == AS5048BStatus::Ok && capturedZero != nullptr) *capturedZero = zero;
  tracking_ = false;
  return status;
}

// ---- I2C address ------------------------------------------------------------

AS5048BStatus AS5048B::readAddressRegister(uint8_t& value) {
  return readRegisters(kRegI2cAddress, &value, 1);
}

AS5048BStatus AS5048B::changeAddress(uint8_t newAddress) {
  if (newAddress < kFirstUsableAddress || newAddress > kLastUsableAddress ||
      (newAddress & 0x03) != (address_ & 0x03)) {
    return setStatus(AS5048BStatus::InvalidArgument);
  }
  const uint8_t regValue = addressRegisterValue(newAddress);
  AS5048BStatus status = writeRegister(kRegI2cAddress, regValue);
  if (status != AS5048BStatus::Ok) return status;

  const uint8_t oldAddress = address_;
  address_ = newAddress;
  delay(1);
  uint8_t readBack = 0;
  status = readAddressRegister(readBack);
  if (status != AS5048BStatus::Ok) {
    address_ = oldAddress;
    return setStatus(AS5048BStatus::VerifyFailed);
  }
  return setStatus((readBack & kAddressRegMask) == regValue ? AS5048BStatus::Ok
                                                            : AS5048BStatus::VerifyFailed);
}

// ---- OTP --------------------------------------------------------------------

AS5048BStatus AS5048B::burnZeroPositionOTP(uint32_t confirmKey) {
  if (confirmKey != kOtpConfirmKey) return setStatus(AS5048BStatus::NotConfirmed);

  uint16_t zero = 0;
  AS5048BStatus status = readZeroPosition(zero);
  if (status != AS5048BStatus::Ok) return status;
  // Nothing to burn; also guards against burning before setZeroHere().
  if (zero == 0) return setStatus(AS5048BStatus::InvalidArgument);

  AS5048BDiagnostics diag;
  status = readDiagnostics(diag);
  if (status != AS5048BStatus::Ok) return status;
  if (!diag.magnetOk()) return setStatus(AS5048BStatus::MagnetError);

  // Datasheet "Programming Sequence with Verification", steps 4-8.
  uint16_t angle = 0;
  if ((status = writeRegister(kRegProgControl, kProgEnable)) != AS5048BStatus::Ok) return status;
  delay(kOtpStepDelayMs);
  if ((status = writeRegister(kRegProgControl, kProgBurn)) != AS5048BStatus::Ok) return status;
  delay(kOtpStepDelayMs);
  if ((status = readAngleRaw(angle)) != AS5048BStatus::Ok) return status;
  if ((status = writeRegister(kRegProgControl, kProgVerify)) != AS5048BStatus::Ok) return status;
  delay(kOtpStepDelayMs);
  if ((status = readAngleRaw(angle)) != AS5048BStatus::Ok) return status;

  uint16_t zeroAfter = 0;
  if ((status = readZeroPosition(zeroAfter)) != AS5048BStatus::Ok) return status;
  const bool angleNearZero = abs(wrapDelta(angle)) <= kZeroVerifyTolerance;
  tracking_ = false;
  return setStatus(zeroAfter == zero && angleNearZero ? AS5048BStatus::Ok
                                                      : AS5048BStatus::VerifyFailed);
}

AS5048BStatus AS5048B::burnAddressOTP(uint32_t confirmKey) {
  if (confirmKey != kOtpConfirmKey) return setStatus(AS5048BStatus::NotConfirmed);

  // Preconditions: changeAddress() done in this power session, a non-default
  // address, and a clear zero register (the whole OTP block is burned at once).
  const uint8_t expected = addressRegisterValue(address_);
  uint8_t current = 0;
  AS5048BStatus status = readAddressRegister(current);
  if (status != AS5048BStatus::Ok) return status;
  if ((current & kAddressRegMask) != expected || expected == 0) {
    return setStatus(AS5048BStatus::InvalidArgument);
  }
  uint16_t zero = 0;
  if ((status = readZeroPosition(zero)) != AS5048BStatus::Ok) return status;
  if (zero != 0) return setStatus(AS5048BStatus::InvalidArgument);

  if ((status = writeRegister(kRegProgControl, kProgAddressEnable)) != AS5048BStatus::Ok) {
    return status;
  }
  delay(kOtpStepDelayMs);
  if ((status = writeRegister(kRegProgControl, kProgBurn)) != AS5048BStatus::Ok) return status;

  // Read errors are tolerated while the fuses are being blown.
  const uint32_t start = millis();
  bool burnDone = false;
  while (millis() - start < kBurnPollTimeoutMs) {
    delay(1);
    uint8_t control = 0;
    if (readRegisters(kRegProgControl, &control, 1) == AS5048BStatus::Ok &&
        (control & kProgBurn) == 0) {
      burnDone = true;
      break;
    }
  }
  // Writing more commands while a burn may still be running could corrupt it.
  if (!burnDone) return setStatus(AS5048BStatus::BurnTimeout);

  if ((status = writeRegister(kRegProgControl, kProgIdle)) != AS5048BStatus::Ok) return status;
  delay(kOtpStepDelayMs);
  if ((status = writeRegister(kRegProgControl, kProgVerify)) != AS5048BStatus::Ok) return status;
  delay(kOtpStepDelayMs);

  uint8_t readBack = 0;
  status = readAddressRegister(readBack);
  writeRegister(kRegProgControl, kProgIdle);
  if (status != AS5048BStatus::Ok) return setStatus(status);
  return setStatus((readBack & kAddressRegMask) == expected ? AS5048BStatus::Ok
                                                            : AS5048BStatus::VerifyFailed);
}

// ---- Low level --------------------------------------------------------------

AS5048BStatus AS5048B::readRegisters(uint8_t reg, uint8_t* buffer, size_t length) {
  if (buffer == nullptr || length == 0) return setStatus(AS5048BStatus::InvalidArgument);

  wire_.beginTransmission(address_);
  wire_.write(reg);
  // Repeated START (datasheet Figure 32: STOP only after a complete sequence).
  const uint8_t err = wire_.endTransmission(false);
  if (err != 0) return setStatus(static_cast<AS5048BStatus>(err <= 5 ? err : 4));

  const size_t received = wire_.requestFrom(static_cast<uint16_t>(address_), length, true);
  if (received != length) {
    while (wire_.available()) wire_.read();
    return setStatus(AS5048BStatus::ReadShort);
  }
  for (size_t i = 0; i < length; ++i) buffer[i] = static_cast<uint8_t>(wire_.read());
  return setStatus(AS5048BStatus::Ok);
}

AS5048BStatus AS5048B::writeRegister(uint8_t reg, uint8_t value) {
  wire_.beginTransmission(address_);
  wire_.write(reg);
  wire_.write(value);
  const uint8_t err = wire_.endTransmission(true);
  return setStatus(err == 0 ? AS5048BStatus::Ok
                            : static_cast<AS5048BStatus>(err <= 5 ? err : 4));
}

// ---- Private ----------------------------------------------------------------

uint16_t AS5048B::combine14(uint8_t high, uint8_t low) {
  return (static_cast<uint16_t>(high) << 6) | (low & 0x3F);
}

uint16_t AS5048B::applyDirection(uint16_t raw) const {
  return reversed_ ? static_cast<uint16_t>((kResolution - raw) & kMask) : raw;
}

AS5048BStatus AS5048B::read14(uint8_t highReg, uint16_t& value) {
  uint8_t buf[2];
  const AS5048BStatus status = readRegisters(highReg, buf, sizeof(buf));
  if (status == AS5048BStatus::Ok) value = combine14(buf[0], buf[1]);
  return status;
}

AS5048BStatus AS5048B::setStatus(AS5048BStatus status) {
  lastStatus_ = status;
  return status;
}
