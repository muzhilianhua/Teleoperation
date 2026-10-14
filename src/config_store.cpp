#include "config_store.h"

#include <Preferences.h>

namespace config_store {

namespace {

constexpr const char* kNamespace = "teleop";
constexpr const char* kKey = "cfg";

}  // namespace

bool save(const uint8_t* data, size_t length) {
  Preferences prefs;
  if (!prefs.begin(kNamespace, false)) return false;
  const size_t written = prefs.putBytes(kKey, data, length);
  prefs.end();
  return written == length;
}

size_t load(uint8_t* data, size_t capacity) {
  Preferences prefs;
  if (!prefs.begin(kNamespace, true)) return 0;  // namespace absent on a fresh chip
  const size_t stored = prefs.isKey(kKey) ? prefs.getBytesLength(kKey) : 0;
  if (stored > 0 && stored <= capacity) prefs.getBytes(kKey, data, stored);
  prefs.end();
  return stored;
}

}  // namespace config_store
