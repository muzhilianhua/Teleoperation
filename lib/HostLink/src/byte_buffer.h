// Little-endian field writer / reader for host link payloads. Both stop at
// the buffer end and remember it (ok() turns false) instead of overrunning,
// so encoders and decoders check once at the end.

#pragma once

#include <stddef.h>
#include <stdint.h>
#include <string.h>

namespace hostlink {

class ByteWriter {
 public:
  ByteWriter(uint8_t* out, size_t capacity) : out_(out), capacity_(capacity) {}

  void u8(uint8_t v) { bytes(&v, 1); }
  void i8(int8_t v) { u8(static_cast<uint8_t>(v)); }
  void u16(uint16_t v) {
    const uint8_t b[2] = {static_cast<uint8_t>(v), static_cast<uint8_t>(v >> 8)};
    bytes(b, 2);
  }
  void i16(int16_t v) { u16(static_cast<uint16_t>(v)); }
  void u32(uint32_t v) {
    const uint8_t b[4] = {static_cast<uint8_t>(v), static_cast<uint8_t>(v >> 8), static_cast<uint8_t>(v >> 16),
                          static_cast<uint8_t>(v >> 24)};
    bytes(b, 4);
  }
  void i32(int32_t v) { u32(static_cast<uint32_t>(v)); }
  void f32(float v) {
    uint32_t bits;
    memcpy(&bits, &v, 4);
    u32(bits);
  }
  // Length-prefixed string, truncated to 255 bytes.
  void str8(const char* s) {
    const size_t n = s == nullptr ? 0 : strnlen(s, 255);
    u8(static_cast<uint8_t>(n));
    bytes(reinterpret_cast<const uint8_t*>(s), n);
  }
  // Unprefixed text running to the end of the payload.
  void text(const char* s) {
    if (s != nullptr) bytes(reinterpret_cast<const uint8_t*>(s), strlen(s));
  }
  void bytes(const uint8_t* data, size_t n) {
    if (!ok_ || n > capacity_ - length_) {
      ok_ = false;
      return;
    }
    if (n > 0) memcpy(out_ + length_, data, n);
    length_ += n;
  }

  size_t length() const { return length_; }
  bool ok() const { return ok_; }

 private:
  uint8_t* out_;
  size_t capacity_;
  size_t length_ = 0;
  bool ok_ = true;
};

class ByteReader {
 public:
  ByteReader(const uint8_t* in, size_t length) : in_(in), length_(length) {}

  uint8_t u8() { return take(1) ? in_[pos_ - 1] : 0; }
  int8_t i8() { return static_cast<int8_t>(u8()); }
  uint16_t u16() { return take(2) ? static_cast<uint16_t>(in_[pos_ - 2] | (in_[pos_ - 1] << 8)) : 0; }
  int16_t i16() { return static_cast<int16_t>(u16()); }
  uint32_t u32() {
    if (!take(4)) return 0;
    const uint8_t* b = in_ + pos_ - 4;
    return static_cast<uint32_t>(b[0]) | (static_cast<uint32_t>(b[1]) << 8) | (static_cast<uint32_t>(b[2]) << 16) |
           (static_cast<uint32_t>(b[3]) << 24);
  }
  int32_t i32() { return static_cast<int32_t>(u32()); }
  float f32() {
    const uint32_t bits = u32();
    float v;
    memcpy(&v, &bits, 4);
    return v;
  }
  // Copies the rest of the payload as a NUL-terminated string (truncated to fit).
  void text(char* out, size_t capacity) {
    const size_t n = remaining() < capacity - 1 ? remaining() : capacity - 1;
    memcpy(out, in_ + pos_, n);
    out[n] = '\0';
    pos_ = length_;
  }

  size_t remaining() const { return length_ - pos_; }
  bool ok() const { return ok_; }

 private:
  bool take(size_t n) {
    if (!ok_ || n > length_ - pos_) {
      ok_ = false;
      return false;
    }
    pos_ += n;
    return true;
  }

  const uint8_t* in_;
  size_t length_;
  size_t pos_ = 0;
  bool ok_ = true;
};

}  // namespace hostlink
