#include "config_blob.h"

#include <string.h>

namespace teleop {

namespace {

class Writer {
 public:
  explicit Writer(uint8_t* out) : p_(out) {}
  void u8(uint8_t v) { *p_++ = v; }
  void u16(uint16_t v) {
    u8(static_cast<uint8_t>(v & 0xFF));
    u8(static_cast<uint8_t>(v >> 8));
  }
  void u32(uint32_t v) {
    u16(static_cast<uint16_t>(v & 0xFFFF));
    u16(static_cast<uint16_t>(v >> 16));
  }
  void f32(float v) {
    uint32_t bits;
    memcpy(&bits, &v, sizeof(bits));
    u32(bits);
  }
  uint8_t* position() const { return p_; }

 private:
  uint8_t* p_;
};

class Reader {
 public:
  explicit Reader(const uint8_t* in) : p_(in) {}
  uint8_t u8() { return *p_++; }
  uint16_t u16() {
    const uint16_t lo = u8();
    return static_cast<uint16_t>(lo | (u8() << 8));
  }
  int16_t i16() { return static_cast<int16_t>(u16()); }
  int32_t i32() { return static_cast<int32_t>(u32()); }
  uint32_t u32() {
    const uint32_t lo = u16();
    return lo | (static_cast<uint32_t>(u16()) << 16);
  }
  float f32() {
    const uint32_t bits = u32();
    float v;
    memcpy(&v, &bits, sizeof(v));
    return v;
  }

 private:
  const uint8_t* p_;
};

}  // namespace

const char* blobStatusToString(BlobStatus status) {
  switch (status) {
    case BlobStatus::Ok: return "ok";
    case BlobStatus::BadSize: return "wrong size (empty or foreign data)";
    case BlobStatus::BadMagic: return "bad magic";
    case BlobStatus::BadVersion: return "unsupported version";
    case BlobStatus::Superseded: return "saved for the hand before the rebuild (other ids / servos): discarded";
    case BlobStatus::BadCrc: return "CRC mismatch";
    case BlobStatus::BadJointCount: return "joint count mismatch";
    case BlobStatus::InvalidJoint: return "invalid joint settings (or changed servo model) reset to defaults";
    case BlobStatus::InvalidGlobal: return "invalid global settings reset to defaults";
  }
  return "?";
}

bool blobApplied(BlobStatus status) {
  return status == BlobStatus::Ok || status == BlobStatus::InvalidJoint || status == BlobStatus::InvalidGlobal;
}

uint32_t crc32(const uint8_t* data, size_t length) {
  uint32_t crc = 0xFFFFFFFFu;
  for (size_t i = 0; i < length; ++i) {
    crc ^= data[i];
    for (int bit = 0; bit < 8; ++bit) crc = (crc >> 1) ^ (0xEDB88320u & (0u - (crc & 1u)));
  }
  return ~crc;
}

size_t packConfig(const JointSettings* joints, const JointDefaults* defaults, uint8_t count,
                  const GlobalSettings& global, uint8_t* out, size_t capacity) {
  if (count != kMaxJoints || out == nullptr || capacity < kBlobSize) return 0;
  Writer w(out);
  w.u32(kBlobMagic);
  w.u16(kBlobVersion);
  w.u8(0);
  w.u8(count);
  for (uint8_t i = 0; i < count; ++i) {
    const JointSettings& s = joints[i];
    w.u8(s.enabled ? 1 : 0);
    w.u8(s.encoderIndex);
    w.u8(static_cast<uint8_t>(defaults[i].model));
    w.u16(s.exoRef);
    w.u32(static_cast<uint32_t>(s.exoB));
    w.u16(static_cast<uint16_t>(s.servoA));
    w.u16(static_cast<uint16_t>(s.servoB));
    w.u16(s.vmax);
    w.u16(s.amax);
    w.u16(s.vsat);
    w.u16(s.trackSteps);
    w.u16(s.gripSteps);
    w.u16(s.currentLimitMa);
  }
  w.f32(global.leadMs);
  w.f32(global.minCutoffHz);
  w.f32(global.beta);
  w.f32(global.jumpDeg);
  w.f32(global.gripLoadPct);
  w.u8(global.loadTripEnabled ? 1 : 0);
  w.u32(crc32(out, kBlobSize - 4));
  return kBlobSize;
}

BlobStatus unpackConfig(const uint8_t* in, size_t length, const JointDefaults* defaults, uint8_t count,
                        uint8_t encoderCount, const GlobalSettings& globalDefaults, JointSettings* joints,
                        GlobalSettings& global, uint8_t* firstInvalidJoint) {
  if (in == nullptr || length < 8) return BlobStatus::BadSize;
  if (count != kMaxJoints) return BlobStatus::BadJointCount;
  Reader r(in);
  if (r.u32() != kBlobMagic) return BlobStatus::BadMagic;
  const uint16_t version = r.u16();
  if (version >= 1 && version < kBlobVersion) return BlobStatus::Superseded;
  if (version != kBlobVersion) return BlobStatus::BadVersion;
  if (length != kBlobSize) return BlobStatus::BadSize;
  Reader crcReader(in + length - 4);
  if (crcReader.u32() != crc32(in, length - 4)) return BlobStatus::BadCrc;
  r.u8();  // reserved
  if (r.u8() != count) return BlobStatus::BadJointCount;

  JointSettings loaded[kMaxJoints];
  uint8_t storedModel[kMaxJoints];
  for (uint8_t i = 0; i < count; ++i) {
    JointSettings& s = loaded[i];
    s.enabled = r.u8() != 0;
    s.encoderIndex = r.u8();
    storedModel[i] = r.u8();
    s.exoRef = r.u16();
    s.exoB = r.i32();
    s.servoA = r.i16();
    s.servoB = r.i16();
    s.vmax = r.u16();
    s.amax = r.u16();
    s.vsat = r.u16();
    s.trackSteps = r.u16();
    s.gripSteps = r.u16();
    s.currentLimitMa = r.u16();
  }
  GlobalSettings g = globalDefaults;
  g.leadMs = r.f32();
  g.minCutoffHz = r.f32();
  g.beta = r.f32();
  g.jumpDeg = r.f32();
  g.gripLoadPct = r.f32();
  g.loadTripEnabled = r.u8() != 0;

  BlobStatus status = BlobStatus::Ok;
  bool invalidSeen = false;
  for (uint8_t i = 0; i < count; ++i) {
    const bool sameModel = storedModel[i] == static_cast<uint8_t>(defaults[i].model);
    if (sameModel && validateJoint(loaded[i], defaults[i], encoderCount) == nullptr) continue;
    loaded[i] = defaultSettings(defaults[i]);
    if (!invalidSeen && firstInvalidJoint != nullptr) *firstInvalidJoint = i;
    invalidSeen = true;
    status = BlobStatus::InvalidJoint;
  }
  if (validateGlobal(g) != nullptr) {
    g = globalDefaults;
    if (status == BlobStatus::Ok) status = BlobStatus::InvalidGlobal;
  }

  for (uint8_t i = 0; i < count; ++i) joints[i] = loaded[i];
  global = g;
  return status;
}

}  // namespace teleop
