// Several AS5048B encoders sharing one I2C bus: batch sampling, per-channel
// fault isolation (offline + periodic retry) and timing statistics.

#pragma once

#include "AS5048B.h"

enum class AS5048BChannelState : uint8_t {
  Online,    // responding, OCF set, angle valid
  NoMagnet,  // responding but OCF=0 or CORDIC overflow: angle not trustworthy
  Offline,   // not responding; skipped and retried every kRetryIntervalMs
};

struct AS5048BChannel {
  AS5048B* encoder = nullptr;
  const char* name = "";
  AS5048BChannelState state = AS5048BChannelState::Offline;
  bool valid = false;           // reading comes from the latest successful read
  AS5048BReading reading;       // angle always; agc/diag/magnitude only after a full read
  uint32_t sampleUs = 0;        // micros() at the end of this channel's last good read
  uint32_t okCount = 0;
  uint32_t errorCount = 0;
  uint32_t consecutiveErrors = 0;
  uint32_t recoveries = 0;      // Offline -> responding transitions
  AS5048BStatus lastStatus = AS5048BStatus::Ok;
  uint32_t lastReadUs = 0;
  uint32_t maxReadUs = 0;
  uint32_t nextRetryMs = 0;
};

struct AS5048BCycleStats {
  uint32_t cycles = 0;
  uint64_t totalCycleUs = 0;
  uint32_t lastCycleUs = 0;
  uint32_t maxCycleUs = 0;
  uint32_t lastSkewUs = 0;      // end of first good read -> end of last good read
  uint32_t maxSkewUs = 0;

  uint32_t averageCycleUs() const {
    return cycles ? static_cast<uint32_t>(totalCycleUs / cycles) : 0;
  }
};

class AS5048BGroup {
 public:
  static constexpr uint8_t kMaxEncoders = 16;
  static constexpr uint8_t kOfflineAfterErrors = 3;
  static constexpr uint32_t kRetryIntervalMs = 1000;

  // The group stores pointers: encoders must outlive it (globals are typical).
  bool add(AS5048B& encoder, const char* name);

  // The bus must already be started (Wire.begin). Returns channels not Offline.
  uint8_t beginAll();

  // One pass over all channels. full=false reads only the angle (2 bytes),
  // full=true reads AGC/diag/magnitude/angle (6-byte burst) and refreshes state.
  // Returns the number of successful reads.
  uint8_t sample(bool full);

  uint8_t zeroAll();        // volatile setZeroHere() per responding channel; returns #ok
  void resetTurnsAll();
  void clearStats();

  uint8_t count() const { return count_; }
  const AS5048BChannel& channel(uint8_t index) const { return channels_[index]; }
  AS5048B& encoder(uint8_t index) { return *channels_[index].encoder; }
  const AS5048BCycleStats& stats() const { return stats_; }

  static const char* stateToString(AS5048BChannelState state);

 private:
  void tryRecover(AS5048BChannel& ch, uint32_t nowMs);
  void markFailure(AS5048BChannel& ch, AS5048BStatus status, uint32_t nowMs);

  AS5048BChannel channels_[kMaxEncoders];
  uint8_t count_ = 0;
  AS5048BCycleStats stats_;
};
