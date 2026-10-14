#include "feetech_packet.h"

#include <string.h>

namespace feetech {

uint8_t checksum(const uint8_t* body, size_t length) {
  uint32_t sum = 0;
  for (size_t i = 0; i < length; ++i) sum += body[i];
  return static_cast<uint8_t>(~sum & 0xFF);
}

size_t buildPacket(uint8_t* out, size_t capacity, uint8_t id, uint8_t instruction,
                   const uint8_t* params, size_t paramCount) {
  const size_t total = paramCount + 6;
  if (out == nullptr || total > capacity || paramCount + 2 > 0xFF) return 0;
  if (paramCount > 0 && params == nullptr) return 0;
  out[0] = 0xFF;
  out[1] = 0xFF;
  out[2] = id;
  out[3] = static_cast<uint8_t>(paramCount + 2);
  out[4] = instruction;
  if (paramCount > 0) memcpy(out + 5, params, paramCount);
  out[total - 1] = checksum(out + 2, total - 3);
  return total;
}

size_t buildPing(uint8_t* out, size_t capacity, uint8_t id) {
  return buildPacket(out, capacity, id, kInstPing, nullptr, 0);
}

size_t buildRead(uint8_t* out, size_t capacity, uint8_t id, uint8_t address, uint8_t length) {
  if (id == kBroadcastId || length == 0) return 0;
  const uint8_t params[2] = {address, length};
  return buildPacket(out, capacity, id, kInstRead, params, sizeof(params));
}

size_t buildWrite(uint8_t* out, size_t capacity, uint8_t id, uint8_t address,
                  const uint8_t* data, size_t length) {
  if (length == 0 || length + 1 > kMaxPacket) return 0;
  uint8_t params[kMaxPacket];
  params[0] = address;
  memcpy(params + 1, data, length);
  return buildPacket(out, capacity, id, kInstWrite, params, length + 1);
}

size_t buildSyncWrite(uint8_t* out, size_t capacity, uint8_t address, uint8_t dataLength,
                      const uint8_t* ids, const uint8_t* data, size_t count) {
  if (count == 0 || dataLength == 0 || ids == nullptr || data == nullptr) return 0;
  const size_t paramCount = 2 + count * (dataLength + 1u);
  if (paramCount > kMaxPacket) return 0;
  uint8_t params[kMaxPacket];
  params[0] = address;
  params[1] = dataLength;
  size_t pos = 2;
  for (size_t i = 0; i < count; ++i) {
    if (ids[i] > kMaxId) return 0;
    params[pos++] = ids[i];
    memcpy(params + pos, data + i * dataLength, dataLength);
    pos += dataLength;
  }
  return buildPacket(out, capacity, kBroadcastId, kInstSyncWrite, params, paramCount);
}

size_t statusPacketSize(size_t paramCount) { return paramCount + kStatusOverhead; }

ParseResult parseStatus(const uint8_t* buffer, size_t length, uint8_t expectedId,
                        int expectedParams, StatusPacket& out) {
  if (length < kStatusOverhead) return ParseResult::Incomplete;
  if (buffer[0] != 0xFF || buffer[1] != 0xFF) return ParseResult::BadHeader;
  const uint8_t len = buffer[3];
  if (len < 2) return ParseResult::BadLength;
  const size_t total = static_cast<size_t>(len) + 4;
  if (length < total) return ParseResult::Incomplete;
  const int paramCount = len - 2;
  if (expectedParams >= 0 && paramCount != expectedParams) return ParseResult::BadLength;
  if (checksum(buffer + 2, total - 3) != buffer[total - 1]) return ParseResult::BadChecksum;
  if (buffer[2] != expectedId) return ParseResult::WrongId;
  out.id = buffer[2];
  out.error = buffer[4];
  out.params = buffer + 5;
  out.paramCount = static_cast<uint8_t>(paramCount);
  return ParseResult::Ok;
}

void put16(uint8_t* p, uint16_t value, Endian endian) {
  const uint8_t hi = static_cast<uint8_t>(value >> 8);
  const uint8_t lo = static_cast<uint8_t>(value & 0xFF);
  if (endian == Endian::Big) {
    p[0] = hi;
    p[1] = lo;
  } else {
    p[0] = lo;
    p[1] = hi;
  }
}

uint16_t get16(const uint8_t* p, Endian endian) {
  return endian == Endian::Big ? static_cast<uint16_t>((p[0] << 8) | p[1])
                               : static_cast<uint16_t>((p[1] << 8) | p[0]);
}

int16_t signMagnitude(uint16_t raw, uint8_t signBit) {
  const uint16_t mask = static_cast<uint16_t>(1u << signBit);
  const int16_t magnitude = static_cast<int16_t>(raw & static_cast<uint16_t>(~mask) & 0x7FFF);
  return (raw & mask) ? static_cast<int16_t>(-magnitude) : magnitude;
}

const char* parseResultToString(ParseResult result) {
  switch (result) {
    case ParseResult::Ok: return "ok";
    case ParseResult::Incomplete: return "incomplete";
    case ParseResult::BadHeader: return "bad header";
    case ParseResult::BadLength: return "bad length";
    case ParseResult::BadChecksum: return "bad checksum";
    case ParseResult::WrongId: return "wrong id";
  }
  return "?";
}

}  // namespace feetech
