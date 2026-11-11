// HLS3606 zero point. The servo reports its raw position plus the position
// offset in register 31, so a joint whose travel crosses the 4095/0 seam can be
// re-zeroed: with the joint held mid-travel, the offset that makes it read
// kZeroCenter moves the seam half a turn away. Positions stored for the old
// offset (travel, servo endpoints) then move by the same amount.

#pragma once

#include <stdint.h>

#include "joint_config.h"

namespace teleop {

constexpr int16_t kZeroCenter = 2048;
// Read-back tolerance after a zero write: torque is off and the joint is held
// by hand, so it may move a little (~3.5 deg, as DEX_hand_software hls-setup).
constexpr int16_t kZeroTolerance = 40;

// Folds an offset by whole turns into the register's -4095..4095.
int16_t foldOffset(int32_t offset);

// The offset that makes `position`, read with `offset` in place, read kZeroCenter.
int16_t centerOffset(int16_t offset, int16_t position);

// The same change with the opposite sign: the fallback if a servo applied
// `written` the other way round.
int16_t mirroredOffset(int16_t offset, int16_t written);

// Moves the travel and servo endpoints by delta steps (readings wrap at one
// turn, positionMax + 1). Returns false, after clearing travel and endpoints and
// disabling the joint, when the moved travel would cross the seam; a joint
// without travel keeps none. The exo calibration is not affected.
bool shiftForOffset(JointSettings& s, int32_t delta, int16_t positionMax);

}  // namespace teleop
