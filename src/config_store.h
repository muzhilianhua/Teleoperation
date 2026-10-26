// NVS persistence of the packed settings blob (Preferences namespace "teleop",
// one key per hand: hand::storeKey()).
// Writing flash stalls the caches of both cores, so callers save only while
// the hand is disarmed.

#pragma once

#include <stddef.h>
#include <stdint.h>

namespace config_store {

bool save(const uint8_t* data, size_t length);
// Returns the stored length (0 if nothing is stored); copies at most capacity bytes.
size_t load(uint8_t* data, size_t capacity);

}  // namespace config_store
