#include "feetech_mixed_bus.h"

#include <string.h>

using feetech::ModelInfo;
using feetech::ParseResult;
using feetech::ServoModel;
using feetech::StatusPacket;

namespace {

constexpr uint8_t kMaxSyncServos = 16;
constexpr uint8_t kMaxStatusBlock = 31;

}  // namespace

ServoModel FeetechMixedBus::model(uint8_t id) const {
  return id < kIds ? models_[id] : ServoModel::Unknown;
}

void FeetechMixedBus::setModel(uint8_t id, ServoModel model) {
  if (id < kIds) models_[id] = model;
}

void FeetechMixedBus::setGoalCurrent(uint8_t id, uint16_t currentRaw) {
  if (id < kIds) goalCurrent_[id] = currentRaw;
}

bool FeetechMixedBus::readRegisters(uint8_t id, uint8_t address, uint8_t length, uint8_t* out,
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

bool FeetechMixedBus::writeRegisters(uint8_t id, uint8_t address, const uint8_t* data, uint8_t length) {
  uint8_t packet[feetech::kMaxPacket];
  const size_t n = feetech::buildWrite(packet, sizeof(packet), id, address, data, length);
  if (n == 0 || !port_.send(packet, n)) return false;
  if (id == feetech::kBroadcastId) return true;
  StatusPacket reply;  // response level 1: servos acknowledge writes
  return port_.receive(id, 0, reply) == ParseResult::Ok;
}

bool FeetechMixedBus::identify(uint8_t id, ServoModel& model, uint8_t& firmwareMajor, uint8_t& firmwareMinor) {
  uint8_t v[feetech::kVersionLength];
  if (!readRegisters(id, feetech::kRegVersion, sizeof(v), v, nullptr)) return false;
  model = feetech::identifyModel(v);
  firmwareMajor = v[0];
  firmwareMinor = v[1];
  return true;
}

bool FeetechMixedBus::ping(uint8_t id) {
  uint8_t packet[8];
  const size_t n = feetech::buildPing(packet, sizeof(packet), id);
  if (!port_.send(packet, n)) return false;
  StatusPacket reply;
  return port_.receive(id, 0, reply) == ParseResult::Ok;
}

bool FeetechMixedBus::readState(uint8_t id, ServoState& out) {
  const ServoModel m = model(id);
  if (m == ServoModel::Unknown) return false;
  const uint8_t length = feetech::modelInfo(m).statusBlockLength;
  uint8_t b[kMaxStatusBlock];
  uint8_t error = 0;
  if (!readRegisters(id, feetech::kRegStatusBlock, length, b, &error)) return false;
  if (!feetech::decodeStatus(m, b, length, out)) return false;
  out.errorByte = error;
  return true;
}

bool FeetechMixedBus::writeTorque(uint8_t id, bool on) {
  const uint8_t value = on ? 1 : 0;  // HLS 2 (damping) is never used
  return writeRegisters(id, feetech::kRegTorqueEnable, &value, 1);
}

void FeetechMixedBus::syncTorqueOff(const uint8_t* ids, uint8_t count) {
  if (count == 0 || count > kMaxSyncServos) return;
  uint8_t data[kMaxSyncServos] = {0};
  uint8_t packet[feetech::kMaxPacket];
  const size_t n =
      feetech::buildSyncWrite(packet, sizeof(packet), feetech::kRegTorqueEnable, 1, ids, data, count);
  if (n > 0) port_.send(packet, n);
}

void FeetechMixedBus::syncWriteGoal(const uint8_t* ids, const int16_t* positions, const float* stepsPerSec,
                                    uint8_t count) {
  if (count == 0 || count > kMaxSyncServos) return;
  uint8_t sendIds[kMaxSyncServos];
  uint8_t data[kMaxSyncServos * feetech::kGoalBlockLength];
  uint8_t n = 0;
  for (uint8_t i = 0; i < count; ++i) {
    const ServoModel m = model(ids[i]);
    if (m == ServoModel::Unknown) continue;  // never guess a byte order
    sendIds[n] = ids[i];
    feetech::encodeGoal(m, positions[i], goalCurrent_[ids[i]], stepsPerSec[i],
                        data + n * feetech::kGoalBlockLength);
    ++n;
  }
  if (n == 0) return;
  uint8_t packet[feetech::kMaxPacket];
  const size_t len = feetech::buildSyncWrite(packet, sizeof(packet), feetech::kRegGoalBlock,
                                             feetech::kGoalBlockLength, sendIds, data, n);
  if (len > 0) port_.send(packet, len);
}

bool FeetechMixedBus::writeGoal(uint8_t id, int16_t position, float stepsPerSec) {
  const ServoModel m = model(id);
  if (m == ServoModel::Unknown) return false;
  uint8_t block[feetech::kGoalBlockLength];
  feetech::encodeGoal(m, position, goalCurrent_[id], stepsPerSec, block);
  return writeRegisters(id, feetech::kRegGoalBlock, block, sizeof(block));
}

bool FeetechMixedBus::readArmInfo(uint8_t id, feetech::ArmInfo& out) {
  const ServoModel m = model(id);
  if (m == ServoModel::Unknown) return false;
  uint8_t r[feetech::kArmInfoLength];
  if (!readRegisters(id, feetech::kRegArmInfo, sizeof(r), r, nullptr)) return false;
  return feetech::decodeArmInfo(m, r, sizeof(r), out);
}

bool FeetechMixedBus::readAngleLimits(uint8_t id, int16_t& lo, int16_t& hi) {
  const ServoModel m = model(id);
  if (m == ServoModel::Unknown) return false;
  uint8_t raw[4];
  if (!readRegisters(id, feetech::kRegAngleLimits, sizeof(raw), raw, nullptr)) return false;
  const feetech::Endian e = feetech::modelInfo(m).endian;
  lo = static_cast<int16_t>(feetech::get16(raw, e));
  hi = static_cast<int16_t>(feetech::get16(raw + 2, e));
  return true;
}

bool FeetechMixedBus::writeAngleLimits(uint8_t id, int16_t lo, int16_t hi) {
  const ServoModel m = model(id);
  if (m == ServoModel::Unknown) return false;
  const ModelInfo& info = feetech::modelInfo(m);
  // lo < hi also rules out 0/0 (SCS wheel mode, HLS multi-turn mode).
  if (lo < 0 || hi > info.positionMax || lo >= hi) return false;
  uint8_t torque = 0;
  if (!readRegisters(id, feetech::kRegTorqueEnable, 1, &torque, nullptr) || torque != 0) return false;

  uint8_t data[4];
  feetech::put16(data, static_cast<uint16_t>(lo), info.endian);
  feetech::put16(data + 2, static_cast<uint16_t>(hi), info.endian);
  const uint8_t unlock = 0;
  const uint8_t lock = 1;
  // Unlock, one 4-byte write (min < max at every moment), re-lock, read back:
  // the sequence of servo_bringup tune_servo and DEX_hand_software hls_setup.
  if (!writeRegisters(id, info.lockRegister, &unlock, 1)) return false;
  const bool written = writeRegisters(id, feetech::kRegAngleLimits, data, sizeof(data));
  delay(10);  // EEPROM commit before the lock write and read-back
  const bool locked = writeRegisters(id, info.lockRegister, &lock, 1);
  int16_t checkLo = 0, checkHi = 0;
  return written && locked && readAngleLimits(id, checkLo, checkHi) && checkLo == lo && checkHi == hi;
}
