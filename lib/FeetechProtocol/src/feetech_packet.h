// Feetech bus servo protocol (SCS / STS / HLS share the same frame format).
//
// Instruction packet: FF FF ID LEN INSTR PARAM... CHK   (LEN = #params + 2)
// Status packet:      FF FF ID LEN ERROR PARAM... CHK   (LEN = #params + 2)
// CHK = ~(ID + LEN + INSTR/ERROR + sum(PARAM)) & 0xFF
//
// Pure C++ (no Arduino dependency) so it can be unit-tested on the host.
// Series differ only in register layout and 16-bit byte order, which the
// caller selects with Endian.

#pragma once

#include <stddef.h>
#include <stdint.h>

namespace feetech {

constexpr uint8_t kBroadcastId = 0xFE;
constexpr uint8_t kMaxId = 0xFD;
constexpr size_t kHeaderSize = 2;
constexpr size_t kStatusOverhead = 6;  // FF FF ID LEN ERR ... CHK
constexpr size_t kMaxPacket = 160;

enum Instruction : uint8_t {
  kInstPing = 0x01,
  kInstRead = 0x02,
  kInstWrite = 0x03,
  kInstSyncWrite = 0x83,
};

// Status-packet error byte bits (SDK ERRBIT_*).
enum ErrorBit : uint8_t {
  kErrVoltage = 0x01,
  kErrAngleSensor = 0x02,
  kErrOverheat = 0x04,
  kErrOverCurrent = 0x08,
  kErrOverload = 0x20,
};

enum class Endian : uint8_t { Big, Little };

enum class ParseResult : uint8_t {
  Ok,
  Incomplete,   // fewer bytes than the header announces
  BadHeader,    // no FF FF at the start
  BadLength,    // LEN < 2 or does not match the expected parameter count
  BadChecksum,
  WrongId,
};

struct StatusPacket {
  uint8_t id = 0;
  uint8_t error = 0;
  const uint8_t* params = nullptr;  // points into the parsed buffer
  uint8_t paramCount = 0;
};

uint8_t checksum(const uint8_t* body, size_t length);

// Each builder writes a complete packet into out and returns its length,
// or 0 if it does not fit in capacity or the arguments are invalid.
size_t buildPacket(uint8_t* out, size_t capacity, uint8_t id, uint8_t instruction,
                   const uint8_t* params, size_t paramCount);
size_t buildPing(uint8_t* out, size_t capacity, uint8_t id);
size_t buildRead(uint8_t* out, size_t capacity, uint8_t id, uint8_t address, uint8_t length);
size_t buildWrite(uint8_t* out, size_t capacity, uint8_t id, uint8_t address,
                  const uint8_t* data, size_t length);
// data holds count * dataLength bytes, one block per id, in ids order.
size_t buildSyncWrite(uint8_t* out, size_t capacity, uint8_t address, uint8_t dataLength,
                      const uint8_t* ids, const uint8_t* data, size_t count);

// expectedParams < 0 accepts any length.
ParseResult parseStatus(const uint8_t* buffer, size_t length, uint8_t expectedId,
                        int expectedParams, StatusPacket& out);
size_t statusPacketSize(size_t paramCount);

void put16(uint8_t* p, uint16_t value, Endian endian);
uint16_t get16(const uint8_t* p, Endian endian);
// Sign-magnitude decode as used by present speed (bit 15) and load (bit 10).
int16_t signMagnitude(uint16_t raw, uint8_t signBit);

const char* parseResultToString(ParseResult result);

}  // namespace feetech
