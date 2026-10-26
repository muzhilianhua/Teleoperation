// Which hand this board drives (GPIO4 strap, see config.h) and the matching
// joint table, encoder addresses and names. selectHand() runs first in
// setup(); everything else reads the selection through these accessors.

#pragma once

#include "config.h"

namespace hand {

void select();
config::Hand which();
const char* name();  // "right" / "left"
const teleop::JointDefaults* joints();
const uint8_t* encoderAddresses();
// NVS key of this hand's settings: the right hand keeps the single-hand key,
// so calibrations stored before the second hand existed stay valid.
const char* storeKey();

}  // namespace hand
