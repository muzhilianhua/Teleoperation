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
    case BlobStatus::BadCrc: return "CRC mismatch";
    case BlobStatus::BadJointCount: return "joint count mismatch";
    case BlobStatus::Migrated: return "migrated from an older version: new joints and fields set to defaults";
    case BlobStatus::BackendMismatch: return "saved for another servo series: servo endpoints cleared";
    case BlobStatus::InvalidJoint: return "invalid joint settings reset to defaults";
    case BlobStatus::InvalidGlobal: return "invalid global settings reset to defaults";
  }
  return "?";
}

bool blobApplied(BlobStatus status) {
  return status == BlobStatus::Ok || status == BlobStatus::Migrated ||
         status == BlobStatus::BackendMismatch || status == BlobStatus::InvalidJoint ||
         status == BlobStatus::InvalidGlobal;
}

uint32_t crc32(const uint8_t* data, size_t length) {
  uint32_t crc = 0xFFFFFFFFu;
  for (size_t i = 0; i < length; ++i) {
    crc ^= data[i];
    for (int bit = 0; bit < 8; ++bit) crc = (crc >> 1) ^ (0xEDB88320u & (0u - (crc & 1u)));
  }
  return ~crc;
}

size_t packConfig(const JointSettings* joints, uint8_t count, const GlobalSettings& global,
                  ServoBackend backend, uint8_t* out, size_t capacity) {
  if (count != kMaxJoints || out == nullptr || capacity < kBlobSize) return 0;
  Writer w(out);
  w.u32(kBlobMagic);
  w.u16(kBlobVersion);
  w.u8(static_cast<uint8_t>(backend));
  w.u8(count);
  for (uint8_t i = 0; i < count; ++i) {
    const JointSettings& s = joints[i];
    w.u8(s.enabled ? 1 : 0);
    w.u8(s.encoderIndex);
    w.u16(s.exoRef);
    w.u32(static_cast<uint32_t>(s.exoB));
    w.u16(static_cast<uint16_t>(s.servoA));
    w.u16(static_cast<uint16_t>(s.servoB));
    w.u16(s.vmax);
    w.u16(s.amax);
    w.u16(s.vsat);
    w.u16(s.trackSteps);
    w.u16(s.gripSteps);
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

BlobStatus unpackConfig(const uint8_t* in, size_t length, ServoBackend backend,
                        const JointDefaults* defaults, uint8_t count, uint8_t encoderCount,
                        const GlobalSettings& globalDefaults, JointSettings* joints,
                        GlobalSettings& global, uint8_t* firstInvalidJoint) {
  if (in == nullptr || length < 8) return BlobStatus::BadSize;
  if (count != kMaxJoints) return BlobStatus::BadJointCount;
  Reader r(in);
  if (r.u32() != kBlobMagic) return BlobStatus::BadMagic;
  const uint16_t version = r.u16();
  uint8_t savedCount = 0;
  size_t expectedLength = 0;
  if (version == kBlobVersion) {
    savedCount = kMaxJoints;
    expectedLength = kBlobSize;
  } else if (version == 2) {
    savedCount = kMaxJoints;
    expectedLength = kV2BlobSize;
  } else if (version == 1) {
    savedCount = kV1JointCount;
    expectedLength = kV1BlobSize;
  } else {
    return BlobStatus::BadVersion;
  }
  const bool v1 = version == 1;
  const bool wideSpan = version >= 3;  // exoB became 32-bit in version 3
  if (length != expectedLength) return BlobStatus::BadSize;
  Reader crcReader(in + length - 4);
  if (crcReader.u32() != crc32(in, length - 4)) return BlobStatus::BadCrc;
  const uint8_t savedBackend = r.u8();
  if (r.u8() != savedCount) return BlobStatus::BadJointCount;

  JointSettings loaded[kMaxJoints];
  for (uint8_t i = savedCount; i < count; ++i) loaded[i] = defaultSettings(defaults[i]);
  for (uint8_t i = 0; i < savedCount; ++i) {
    JointSettings& s = loaded[i];
    s = defaultSettings(defaults[i]);  // fields an older version did not store
    s.enabled = r.u8() != 0;
    s.encoderIndex = r.u8();
    s.exoRef = r.u16();
    s.exoB = wideSpan ? r.i32() : r.i16();
    s.servoA = r.i16();
    s.servoB = r.i16();
    s.vmax = r.u16();
    s.amax = r.u16();
    s.vsat = r.u16();
    s.trackSteps = r.u16();
    if (!v1) s.gripSteps = r.u16();
  }
  GlobalSettings g = globalDefaults;
  g.leadMs = r.f32();
  g.minCutoffHz = r.f32();
  g.beta = r.f32();
  g.jumpDeg = r.f32();
  if (!v1) {
    g.gripLoadPct = r.f32();
    g.loadTripEnabled = r.u8() != 0;
  }

  BlobStatus status = version < kBlobVersion ? BlobStatus::Migrated : BlobStatus::Ok;
  if (savedBackend != static_cast<uint8_t>(backend)) {
    status = BlobStatus::BackendMismatch;
    for (uint8_t i = 0; i < count; ++i) loaded[i].servoA = loaded[i].servoB = defaults[i].travelMin;
  }
  bool invalidSeen = false;
  for (uint8_t i = 0; i < count; ++i) {
    if (validateJoint(loaded[i], defaults[i], encoderCount) == nullptr) continue;
    loaded[i] = defaultSettings(defaults[i]);
    if (!invalidSeen && firstInvalidJoint != nullptr) *firstInvalidJoint = i;
    invalidSeen = true;
    if (status == BlobStatus::Ok) status = BlobStatus::InvalidJoint;
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
