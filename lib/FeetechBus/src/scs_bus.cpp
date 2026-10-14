#include "scs_bus.h"

#include <string.h>

using feetech::Endian;
using feetech::ParseResult;
using feetech::StatusPacket;

namespace {

constexpr Endian kEndian = Endian::Big;

// SCS register map (servo_bringup registers.py)
constexpr uint8_t kRegModel = 3;           // 2 bytes
constexpr uint8_t kRegTorqueEnable = 40;   // 1 byte
constexpr uint8_t kRegGoalBlock = 42;      // position, time, speed (2 bytes each)
constexpr uint8_t kGoalBlockLength = 6;
constexpr uint8_t kRegStatusBlock = 40;    // registers 40..66
constexpr uint8_t kStatusBlockLength = 27;

// Offsets inside the status block
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

constexpr uint8_t kSpeedSignBit = 15;
constexpr uint8_t kLoadSignBit = 10;

constexpr uint8_t kMaxSyncServos = 16;

}  // namespace

bool ScsBus::readRegisters(uint8_t id, uint8_t address, uint8_t length, uint8_t* out,
                           uint8_t* error) {
  uint8_t packet[8];
  const size_t n = feetech::buildRead(packet, sizeof(packet), id, address, length);
  if (n == 0 || !port_.send(packet, n)) return false;
  StatusPacket reply;
  if (port_.receive(id, length, reply) != ParseResult::Ok) return false;
  memcpy(out, reply.params, length);
  if (error != nullptr) *error = reply.error;
  return true;
}

bool ScsBus::writeRegisters(uint8_t id, uint8_t address, const uint8_t* data, uint8_t length) {
  uint8_t packet[feetech::kMaxPacket];
  const size_t n = feetech::buildWrite(packet, sizeof(packet), id, address, data, length);
  if (n == 0 || !port_.send(packet, n)) return false;
  if (id == feetech::kBroadcastId) return true;
  StatusPacket reply;  // response level 1: servos acknowledge writes
  return port_.receive(id, 0, reply) == ParseResult::Ok;
}

bool ScsBus::ping(uint8_t id, uint16_t* model) {
  uint8_t packet[8];
  const size_t n = feetech::buildPing(packet, sizeof(packet), id);
  if (!port_.send(packet, n)) return false;
  StatusPacket reply;
  if (port_.receive(id, 0, reply) != ParseResult::Ok) return false;
  if (model == nullptr) return true;
  uint8_t raw[2];
  if (!readRegisters(id, kRegModel, sizeof(raw), raw, nullptr)) return false;
  *model = feetech::get16(raw, kEndian);
  return true;
}

bool ScsBus::readState(uint8_t id, ServoState& out) {
  uint8_t b[kStatusBlockLength];
  uint8_t error = 0;
  if (!readRegisters(id, kRegStatusBlock, kStatusBlockLength, b, &error)) return false;
  out.torqueOn = b[kOffTorque] != 0;
  out.goal = static_cast<int16_t>(feetech::get16(b + kOffGoal, kEndian));
  out.goalSpeed = feetech::get16(b + kOffGoalSpeed, kEndian);
  out.position = static_cast<int16_t>(feetech::get16(b + kOffPosition, kEndian));
  out.speed = feetech::signMagnitude(feetech::get16(b + kOffSpeed, kEndian), kSpeedSignBit);
  out.loadPct = 0.1f * feetech::signMagnitude(feetech::get16(b + kOffLoad, kEndian), kLoadSignBit);
  out.voltage = 0.1f * b[kOffVoltage];
  out.temperatureC = b[kOffTemperature];
  out.status = b[kOffStatus];
  out.moving = b[kOffMoving] != 0;
  out.errorByte = error;
  return true;
}

bool ScsBus::writeTorque(uint8_t id, bool on) {
  const uint8_t value = on ? 1 : 0;
  return writeRegisters(id, kRegTorqueEnable, &value, 1);
}

void ScsBus::syncTorqueOff(const uint8_t* ids, uint8_t count) {
  if (count == 0 || count > kMaxSyncServos) return;
  uint8_t data[kMaxSyncServos] = {0};
  uint8_t packet[feetech::kMaxPacket];
  const size_t n =
      feetech::buildSyncWrite(packet, sizeof(packet), kRegTorqueEnable, 1, ids, data, count);
  if (n > 0) port_.send(packet, n);
}

void ScsBus::fillGoalBlock(uint8_t* block, int16_t position, uint16_t speed) const {
  if (position < positionMin()) position = positionMin();
  if (position > positionMax()) position = positionMax();
  if (speed < 1) speed = 1;  // 0 would mean "maximum speed"
  feetech::put16(block + 0, static_cast<uint16_t>(position), kEndian);
  feetech::put16(block + 2, 0, kEndian);  // goal time must stay 0 (speed mode)
  feetech::put16(block + 4, speed, kEndian);
}

void ScsBus::syncWriteGoal(const uint8_t* ids, const int16_t* positions, const uint16_t* speeds,
                           uint8_t count) {
  if (count == 0 || count > kMaxSyncServos) return;
  uint8_t data[kMaxSyncServos * kGoalBlockLength];
  for (uint8_t i = 0; i < count; ++i) {
    fillGoalBlock(data + i * kGoalBlockLength, positions[i], speeds[i]);
  }
  uint8_t packet[feetech::kMaxPacket];
  const size_t n = feetech::buildSyncWrite(packet, sizeof(packet), kRegGoalBlock,
                                           kGoalBlockLength, ids, data, count);
  if (n > 0) port_.send(packet, n);
}

bool ScsBus::writeGoal(uint8_t id, int16_t position, uint16_t speed) {
  uint8_t block[kGoalBlockLength];
  fillGoalBlock(block, position, speed);
  return writeRegisters(id, kRegGoalBlock, block, kGoalBlockLength);
}
