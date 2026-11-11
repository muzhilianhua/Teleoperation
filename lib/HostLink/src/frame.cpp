#include "frame.h"

namespace hostlink {

uint16_t crc16(const uint8_t* data, size_t length, uint16_t crc) {
  for (size_t i = 0; i < length; ++i) {
    crc ^= static_cast<uint16_t>(data[i]) << 8;
    for (int bit = 0; bit < 8; ++bit) {
      crc = (crc & 0x8000) ? static_cast<uint16_t>((crc << 1) ^ 0x1021) : static_cast<uint16_t>(crc << 1);
    }
  }
  return crc;
}

// ---- COBS ------------------------------------------------------------------

void CobsEncoder::write(uint8_t byte) {
  if (length_ >= capacity_) {
    overflow_ = true;
    return;
  }
  out_[length_++] = byte;
}

void CobsEncoder::openBlock() {
  codePos_ = length_;
  write(0);  // placeholder for the block's code byte
  code_ = 1;
  open_ = true;
}

void CobsEncoder::closeBlock() {
  if (codePos_ < capacity_) out_[codePos_] = code_;
  open_ = false;
}

void CobsEncoder::put(uint8_t byte) {
  if (!open_) openBlock();
  if (byte == 0) {
    closeBlock();
    openBlock();  // a zero always starts a new block, even at the very end
    return;
  }
  write(byte);
  if (++code_ == 0xFF) closeBlock();  // full block; the next byte opens a new one
}

void CobsEncoder::put(const uint8_t* data, size_t length) {
  for (size_t i = 0; i < length; ++i) put(data[i]);
}

size_t CobsEncoder::finish() {
  // An empty input still encodes as one block (0x01); a full 254-byte block
  // ending the data needs no extra code byte.
  if (!open_ && length_ == 0) openBlock();
  if (open_) closeBlock();
  return overflow_ ? 0 : length_;
}

size_t cobsEncode(const uint8_t* in, size_t length, uint8_t* out, size_t capacity) {
  CobsEncoder encoder(out, capacity);
  encoder.put(in, length);
  return encoder.finish();
}

bool cobsDecode(const uint8_t* in, size_t length, uint8_t* out, size_t capacity, size_t& outLength) {
  // The write index never passes the read index, so decoding in place is safe.
  size_t r = 0;
  size_t w = 0;
  while (r < length) {
    const uint8_t code = in[r++];
    if (code == 0) return false;
    for (uint8_t i = 1; i < code; ++i) {
      if (r >= length || in[r] == 0 || w >= capacity) return false;
      out[w++] = in[r++];
    }
    if (code != 0xFF && r < length) {
      if (w >= capacity) return false;
      out[w++] = 0;
    }
  }
  outLength = w;
  return true;
}

// ---- Frames ----------------------------------------------------------------

size_t encodeFrame(const FrameHeader& header, const uint8_t* payload, size_t payloadLength, uint8_t* out,
                   size_t capacity) {
  if (payloadLength > kMaxPayload || capacity == 0) return 0;
  const uint8_t head[kHeaderSize] = {header.version, header.type, static_cast<uint8_t>(header.seq & 0xFF),
                                     static_cast<uint8_t>(header.seq >> 8), header.flags};
  uint16_t crc = crc16(head, kHeaderSize);
  crc = crc16(payload, payloadLength, crc);
  const uint8_t tail[kCrcSize] = {static_cast<uint8_t>(crc & 0xFF), static_cast<uint8_t>(crc >> 8)};

  CobsEncoder encoder(out, capacity - 1);  // keep room for the delimiter
  encoder.put(head, kHeaderSize);
  encoder.put(payload, payloadLength);
  encoder.put(tail, kCrcSize);
  const size_t n = encoder.finish();
  if (n == 0) return 0;
  out[n] = 0;
  return n + 1;
}

void FrameReader::reset() {
  length_ = 0;
  discarding_ = false;
  payloadLength_ = 0;
}

FrameReader::Result FrameReader::feed(uint8_t byte) {
  if (byte != 0) {
    if (discarding_) return Result::None;
    if (length_ >= sizeof(buffer_)) {
      ++stats_.overflows;
      discarding_ = true;
      length_ = 0;
      return Result::Error;
    }
    buffer_[length_++] = byte;
    return Result::None;
  }
  if (discarding_) {
    discarding_ = false;
    return Result::None;
  }
  if (length_ == 0) return Result::None;  // back-to-back delimiters
  const Result result = finishFrame();
  length_ = 0;
  return result;
}

FrameReader::Result FrameReader::finishFrame() {
  size_t decoded = 0;
  if (!cobsDecode(buffer_, length_, buffer_, sizeof(buffer_), decoded) || decoded < kHeaderSize + kCrcSize ||
      decoded > kMaxFrame) {
    ++stats_.malformed;
    return Result::Error;
  }
  const size_t body = decoded - kCrcSize;
  const uint16_t expected = static_cast<uint16_t>(buffer_[body] | (buffer_[body + 1] << 8));
  if (crc16(buffer_, body) != expected) {
    ++stats_.crcErrors;
    return Result::Error;
  }
  if (buffer_[0] != kProtocolVersion) {
    ++stats_.malformed;
    return Result::Error;
  }
  header_.version = buffer_[0];
  header_.type = buffer_[1];
  header_.seq = static_cast<uint16_t>(buffer_[2] | (buffer_[3] << 8));
  header_.flags = buffer_[4];
  payloadLength_ = body - kHeaderSize;
  ++stats_.frames;
  return Result::Frame;
}

}  // namespace hostlink
