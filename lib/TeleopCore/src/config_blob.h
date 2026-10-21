// Fixed little-endian serialization of all joint and global settings for NVS.
//
// Layout (version 4: 9 joints, 258 bytes):
//   u32 magic 'TELE' | u16 version | u8 reserved (0) | u8 jointCount
//   jointCount x { u8 enabled, u8 encoder, u8 servo model, u16 exoRef, i32 exoB,
//                  i16 servoA, i16 servoB, u16 vmax, u16 amax, u16 vsat,
//                  u16 trackSteps, u16 gripSteps, u16 currentLimitMa }
//   f32 leadMs | f32 minCutoffHz | f32 beta | f32 jumpDeg | f32 gripLoadPct |
//   u8 loadTripEnabled
//   u32 crc32 over everything above
//
// Versions 1-3 were saved for the hand before it was rebuilt (other servo ids,
// joints and models): they are discarded as a whole (Superseded). Within
// version 4, a joint whose stored servo model differs from config.h is reset
// to defaults, since its steps and endpoints no longer mean the same thing.
//
// Fields are written one by one (not memcpy of the structs) so the format does
// not depend on compiler padding and host tests see the same bytes as the MCU.

#pragma once

#include <stddef.h>
#include <stdint.h>

#include "joint_config.h"

namespace teleop {

constexpr uint32_t kBlobMagic = 0x454C4554u;  // "TELE"
constexpr uint16_t kBlobVersion = 4;
constexpr size_t kJointRecordSize = 25;
constexpr size_t kGlobalRecordSize = 21;
constexpr size_t kBlobSize = 8 + kMaxJoints * kJointRecordSize + kGlobalRecordSize + 4;

enum class BlobStatus : uint8_t {
  Ok,
  BadSize,        // nothing applied
  BadMagic,       // nothing applied
  BadVersion,     // nothing applied
  Superseded,     // nothing applied: saved for the hand before the rebuild
  BadCrc,         // nothing applied
  BadJointCount,  // nothing applied
  InvalidJoint,   // applied; offending joint(s) (incl. changed servo model) reset to defaults
  InvalidGlobal,  // applied; global settings reset to defaults
};

const char* blobStatusToString(BlobStatus status);
bool blobApplied(BlobStatus status);

uint32_t crc32(const uint8_t* data, size_t length);

// Returns the number of bytes written (kBlobSize) or 0 if capacity is short.
size_t packConfig(const JointSettings* joints, const JointDefaults* defaults, uint8_t count,
                  const GlobalSettings& global, uint8_t* out, size_t capacity);

// On a status for which blobApplied() is true, joints/global hold the loaded
// (and corrected) settings; otherwise they are left untouched.
// firstInvalidJoint receives the index of the first joint reset to defaults.
BlobStatus unpackConfig(const uint8_t* in, size_t length, const JointDefaults* defaults, uint8_t count,
                        uint8_t encoderCount, const GlobalSettings& globalDefaults, JointSettings* joints,
                        GlobalSettings& global, uint8_t* firstInvalidJoint = nullptr);

}  // namespace teleop
