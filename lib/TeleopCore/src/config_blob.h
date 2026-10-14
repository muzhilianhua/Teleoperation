// Fixed little-endian serialization of all joint and global settings for NVS.
//
// Layout (version 1, 172 bytes):
//   u32 magic 'TELE' | u16 version | u8 backend | u8 jointCount
//   jointCount x { u8 enabled, u8 encoder, u16 exoRef, i16 exoB, i16 servoA,
//                  i16 servoB, u16 vmax, u16 amax, u16 vsat, u16 trackSteps }
//   f32 leadMs | f32 minCutoffHz | f32 beta | f32 jumpDeg
//   u32 crc32 over everything above
//
// Fields are written one by one (not memcpy of the structs) so the format does
// not depend on compiler padding and host tests see the same bytes as the MCU.

#pragma once

#include <stddef.h>
#include <stdint.h>

#include "joint_config.h"

namespace teleop {

constexpr uint32_t kBlobMagic = 0x454C4554u;  // "TELE"
constexpr uint16_t kBlobVersion = 1;
constexpr size_t kJointRecordSize = 18;
constexpr size_t kBlobSize = 8 + kMaxJoints * kJointRecordSize + 16 + 4;

enum class ServoBackend : uint8_t { Scs = 1, Hls = 2 };

enum class BlobStatus : uint8_t {
  Ok,
  BadSize,          // nothing applied
  BadMagic,         // nothing applied
  BadVersion,       // nothing applied
  BadCrc,           // nothing applied
  BadJointCount,    // nothing applied
  BackendMismatch,  // applied; servo endpoints of every joint reset to uncalibrated
  InvalidJoint,     // applied; offending joint(s) reset to defaults
  InvalidGlobal,    // applied; global settings reset to defaults
};

const char* blobStatusToString(BlobStatus status);
bool blobApplied(BlobStatus status);

uint32_t crc32(const uint8_t* data, size_t length);

// Returns the number of bytes written (kBlobSize) or 0 if capacity is short.
size_t packConfig(const JointSettings* joints, uint8_t count, const GlobalSettings& global,
                  ServoBackend backend, uint8_t* out, size_t capacity);

// On a status for which blobApplied() is true, joints/global hold the loaded
// (and corrected) settings; otherwise they are left untouched.
// firstInvalidJoint receives the index of the first joint reset to defaults.
BlobStatus unpackConfig(const uint8_t* in, size_t length, ServoBackend backend,
                        const JointDefaults* defaults, uint8_t count, uint8_t encoderCount,
                        const GlobalSettings& globalDefaults, JointSettings* joints,
                        GlobalSettings& global, uint8_t* firstInvalidJoint = nullptr);

}  // namespace teleop
