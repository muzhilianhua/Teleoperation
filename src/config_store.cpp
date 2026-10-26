#include "config_store.h"

#include <Preferences.h>

#include "hand.h"

namespace config_store {

namespace {

constexpr const char* kNamespace = "teleop";

}  // namespace

bool save(const uint8_t* data, size_t length) {
  Preferences prefs;
  if (!prefs.begin(kNamespace, false)) return false;
  const size_t written = prefs.putBytes(hand::storeKey(), data, length);
  prefs.end();
  return written == length;
}

size_t load(uint8_t* data, size_t capacity) {
  Preferences prefs;
  if (!prefs.begin(kNamespace, true)) return 0;  // namespace absent on a fresh chip
  const size_t stored = prefs.isKey(hand::storeKey()) ? prefs.getBytesLength(hand::storeKey()) : 0;
  if (stored > 0 && stored <= capacity) prefs.getBytes(hand::storeKey(), data, stored);
  prefs.end();
  return stored;
}

}  // namespace config_store
