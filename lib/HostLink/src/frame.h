// Host link framing (docs/host-protocol.md).
//
// A frame on the wire is COBS(header | payload | crc16) followed by one 0x00
// delimiter. COBS removes every zero byte from the body, so 0x00 always marks
// a frame boundary and a reader joining mid-stream (or after text output)
// resynchronises at the next delimiter.
//
//   header  5 B: version u8 | type u8 | seq u16 LE | flags u8
//   payload 0..kMaxPayload B
//   crc16   2 B LE: CRC16-CCITT-FALSE over header and payload
//
// Everything here is plain C++ so the same code is unit-tested on the host.

#pragma once

#include <stddef.h>
#include <stdint.h>

namespace hostlink {

constexpr uint8_t kProtocolVersion = 1;
constexpr size_t kHeaderSize = 5;
constexpr size_t kCrcSize = 2;
constexpr size_t kMaxFrame = 4096;  // decoded header + payload + crc
constexpr size_t kMaxPayload = kMaxFrame - kHeaderSize - kCrcSize;

// Worst-case COBS output for n input bytes (one code byte per 254 bytes).
constexpr size_t cobsMaxEncoded(size_t n) { return n + n / 254 + 1; }
// Worst-case wire size of a frame with `payload` bytes, delimiter included.
constexpr size_t maxWireSize(size_t payload) {
  return cobsMaxEncoded(kHeaderSize + payload + kCrcSize) + 1;
}

// CRC16-CCITT-FALSE: poly 0x1021, init 0xFFFF, no reflection, no final xor.
// "123456789" -> 0x29B1. Pass the previous result to continue over segments.
uint16_t crc16(const uint8_t* data, size_t length, uint16_t crc = 0xFFFF);

// Streaming COBS encoder into a caller buffer (no delimiter is written).
class CobsEncoder {
 public:
  CobsEncoder(uint8_t* out, size_t capacity) : out_(out), capacity_(capacity) {}
  void put(uint8_t byte);
  void put(const uint8_t* data, size_t length);
  // Closes the last block; returns the encoded length, 0 on overflow.
  size_t finish();

 private:
  void write(uint8_t byte);
  void openBlock();
  void closeBlock();

  uint8_t* out_;
  size_t capacity_;
  size_t length_ = 0;
  size_t codePos_ = 0;
  uint8_t code_ = 1;
  bool open_ = false;
  bool overflow_ = false;
};

size_t cobsEncode(const uint8_t* in, size_t length, uint8_t* out, size_t capacity);
// Decodes in -> out (may be the same buffer). Returns false on malformed
// input (a zero byte, a truncated block) or when out is too small.
bool cobsDecode(const uint8_t* in, size_t length, uint8_t* out, size_t capacity, size_t& outLength);

struct FrameHeader {
  uint8_t version = kProtocolVersion;
  uint8_t type = 0;
  uint16_t seq = 0;
  uint8_t flags = 0;
};

// Builds one complete wire frame (COBS body + 0x00) into out. Returns the
// number of bytes to send, or 0 if the payload is too long or out too small.
size_t encodeFrame(const FrameHeader& header, const uint8_t* payload, size_t payloadLength, uint8_t* out,
                   size_t capacity);

// Streaming frame reader: feed it every received byte.
class FrameReader {
 public:
  enum class Result : uint8_t { None, Frame, Error };

  struct Stats {
    uint32_t frames = 0;
    uint32_t crcErrors = 0;
    uint32_t malformed = 0;  // bad COBS, too short, wrong version
    uint32_t overflows = 0;  // longer than kMaxFrame before a delimiter
  };

  // Frame: header() / payload() are valid until the next feed().
  Result feed(uint8_t byte);

  const FrameHeader& header() const { return header_; }
  const uint8_t* payload() const { return buffer_ + kHeaderSize; }
  size_t payloadLength() const { return payloadLength_; }
  const Stats& stats() const { return stats_; }
  void reset();

 private:
  Result finishFrame();

  uint8_t buffer_[cobsMaxEncoded(kMaxFrame)];
  size_t length_ = 0;
  bool discarding_ = false;  // overflowed: drop bytes until the next delimiter
  FrameHeader header_;
  size_t payloadLength_ = 0;
  Stats stats_;
};

}  // namespace hostlink
