#include "servo_models.h"

#include <math.h>

namespace feetech {

namespace {

constexpr uint16_t kScsModelNumber = 1284;  // 0x0504
constexpr uint8_t kHlsServoMajor = 10;

const ModelInfo kUnknown = {"unknown", Endian::Big, 0, 27, 48, 1.0f, 1.0f, 1, 0.0f};
const ModelInfo kScs = {"SCS0009", Endian::Big, 1023, 27, 48, 1.0f, 1.0f, 1000, 0.0f};
// 4096 steps per turn vs 1024 over 300 deg; one speed unit is 0.732 RPM.
const ModelInfo kHls = {"HLS3606", Endian::Little, 4095, 31, 55,
                        (300.0f / 1024.0f) / (360.0f / 4096.0f), 4096.0f * 0.732f / 60.0f, 250, 6.5f};

// Offsets inside the status block (from register 40), identical on both models.
constexpr uint8_t kOffTorque = 0;
constexpr uint8_t kOffGoal = 2;
constexpr uint8_t kOffGoalSpeed = 6;
constexpr uint8_t kOffPosition = 16;
constexpr uint8_t kOffSpeed = 18;
constexpr uint8_t kOffLoad = 20;
constexpr uint8_t kOffVoltage = 22;
constexpr uint8_t kOffTemperature = 23;
constexpr uint8_t kOffStatus = 25;
constexpr uint8_t kOffMoving = 26;
constexpr uint8_t kOffCurrent = 29;  // HLS register 69

int16_t clamp16(int32_t v, int32_t lo, int32_t hi) {
  return static_cast<int16_t>(v < lo ? lo : (v > hi ? hi : v));
}

}  // namespace

const ModelInfo& modelInfo(ServoModel model) {
  switch (model) {
    case ServoModel::Scs0009: return kScs;
    case ServoModel::Hls3606: return kHls;
    case ServoModel::Unknown: break;
  }
  return kUnknown;
}

const char* modelName(ServoModel model) { return modelInfo(model).name; }

ServoModel identifyModel(const uint8_t* v) {
  if (v == nullptr) return ServoModel::Unknown;
  if (v[2] == 1 && get16(v + 3, Endian::Big) == kScsModelNumber) return ServoModel::Scs0009;
  if (v[2] == 0 && v[3] == kHlsServoMajor) return ServoModel::Hls3606;
  return ServoModel::Unknown;
}

uint16_t speedRegister(ServoModel model, float stepsPerSec) {
  const ModelInfo& m = modelInfo(model);
  float units = ceilf(fabsf(stepsPerSec) / m.stepsPerSpeedUnit - 1e-4f);
  if (units < 1.0f) units = 1.0f;
  if (units > m.maxSpeedRegister) units = m.maxSpeedRegister;
  return static_cast<uint16_t>(units);
}

float stepsPerSecond(ServoModel model, uint16_t reg) { return reg * modelInfo(model).stepsPerSpeedUnit; }

void encodeGoal(ServoModel model, int16_t position, uint16_t currentRaw, float stepsPerSec,
                uint8_t out[kGoalBlockLength]) {
  const ModelInfo& m = modelInfo(model);
  // Never a negative (sign-bit) position: travel always lies in 0..positionMax.
  const uint16_t pos = static_cast<uint16_t>(clamp16(position, 0, m.positionMax));
  const uint16_t third = model == ServoModel::Hls3606
                             ? static_cast<uint16_t>(currentRaw > kHlsMaxGoalCurrent ? kHlsMaxGoalCurrent : currentRaw)
                             : 0;  // SCS goal time must stay 0 (speed mode)
  put16(out + 0, pos, m.endian);
  put16(out + 2, third, m.endian);
  put16(out + 4, speedRegister(model, stepsPerSec), m.endian);
}

bool decodeStatus(ServoModel model, const uint8_t* b, size_t length, ServoState& out) {
  const ModelInfo& m = modelInfo(model);
  if (model == ServoModel::Unknown || b == nullptr || length < m.statusBlockLength) return false;
  const Endian e = m.endian;
  const bool hls = model == ServoModel::Hls3606;
  out.torqueOn = b[kOffTorque] != 0;
  out.goal = hls ? signMagnitude(get16(b + kOffGoal, e), 15) : static_cast<int16_t>(get16(b + kOffGoal, e));
  out.goalSpeed = get16(b + kOffGoalSpeed, e);
  out.position =
      hls ? signMagnitude(get16(b + kOffPosition, e), 15) : static_cast<int16_t>(get16(b + kOffPosition, e));
  out.speed = signMagnitude(get16(b + kOffSpeed, e), 15);
  if (hls) out.speed = static_cast<int16_t>(lroundf(out.speed * m.stepsPerSpeedUnit));
  out.loadPct = 0.1f * signMagnitude(get16(b + kOffLoad, e), 10);
  out.voltage = 0.1f * b[kOffVoltage];
  out.temperatureC = b[kOffTemperature];
  out.status = b[kOffStatus];
  out.moving = (b[kOffMoving] & 0x01) != 0;
  out.hasCurrent = hls;
  out.currentMa = hls ? m.currentMaPerUnit * signMagnitude(get16(b + kOffCurrent, e), 15) : 0.0f;
  return true;
}

bool decodeArmInfo(ServoModel model, const uint8_t* r, size_t length, ArmInfo& out) {
  if (model == ServoModel::Unknown || r == nullptr || length < kArmInfoLength) return false;
  const Endian e = modelInfo(model).endian;
  // Offsets relative to register 9.
  out.minLimit = static_cast<int16_t>(get16(r + 0, e));
  out.maxLimit = static_cast<int16_t>(get16(r + 2, e));
  out.protectionCurrent = get16(r + 19, e);  // register 28
  out.resolution = r[21];                    // register 30
  out.mode = r[24];                          // register 33
  return true;
}

}  // namespace feetech
