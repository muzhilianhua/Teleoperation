#include "AS5048BGroup.h"

namespace {

bool timeReached(uint32_t nowMs, uint32_t deadlineMs) {
  return static_cast<int32_t>(nowMs - deadlineMs) >= 0;
}

AS5048BChannelState stateFromDiag(const AS5048BDiagnostics& diag) {
  return diag.ocf && !diag.cof ? AS5048BChannelState::Online : AS5048BChannelState::NoMagnet;
}

}  // namespace

bool AS5048BGroup::add(AS5048B& encoder, const char* name) {
  if (count_ >= kMaxEncoders) return false;
  AS5048BChannel& ch = channels_[count_++];
  ch = AS5048BChannel();
  ch.encoder = &encoder;
  ch.name = name != nullptr ? name : "";
  return true;
}

uint8_t AS5048BGroup::beginAll() {
  uint8_t responding = 0;
  const uint32_t nowMs = millis();
  for (uint8_t i = 0; i < count_; ++i) {
    AS5048BChannel& ch = channels_[i];
    ch.valid = false;
    ch.consecutiveErrors = 0;
    // Pins/frequency are ignored with initBus=false.
    if (ch.encoder->begin(0, 0, 0, false)) {
      ch.state = AS5048BChannelState::Online;
    } else if (ch.encoder->lastStatus() == AS5048BStatus::MagnetError) {
      ch.state = AS5048BChannelState::NoMagnet;
    } else {
      ch.state = AS5048BChannelState::Offline;
      ch.nextRetryMs = nowMs + kRetryIntervalMs;
    }
    ch.lastStatus = ch.encoder->lastStatus();
    if (ch.state != AS5048BChannelState::Offline) {
      ++responding;
      ch.valid = ch.encoder->readAll(ch.reading) == AS5048BStatus::Ok;
    }
  }
  return responding;
}

uint8_t AS5048BGroup::sample(bool full) {
  const uint32_t cycleStart = micros();
  const uint32_t nowMs = millis();
  uint32_t firstEndUs = 0;
  uint32_t lastEndUs = 0;
  uint8_t ok = 0;

  for (uint8_t i = 0; i < count_; ++i) {
    AS5048BChannel& ch = channels_[i];
    if (ch.state == AS5048BChannelState::Offline) {
      if (!timeReached(nowMs, ch.nextRetryMs)) continue;
      tryRecover(ch, nowMs);
      if (ch.state == AS5048BChannelState::Offline) continue;
    }

    AS5048B& enc = *ch.encoder;
    const uint32_t t0 = micros();
    AS5048BStatus status;
    if (full) {
      status = enc.readAll(ch.reading);
    } else {
      uint16_t raw = 0;
      status = enc.readAngleRaw(raw);
      if (status == AS5048BStatus::Ok) {
        ch.reading.angleRaw = raw;
        ch.reading.angle =
            enc.isReversed() ? static_cast<uint16_t>((AS5048B::kResolution - raw) & AS5048B::kMask)
                             : raw;
      }
    }
    const uint32_t t1 = micros();
    ch.lastReadUs = t1 - t0;
    if (ch.lastReadUs > ch.maxReadUs) ch.maxReadUs = ch.lastReadUs;
    ch.lastStatus = status;

    if (status != AS5048BStatus::Ok) {
      markFailure(ch, status, nowMs);
      continue;
    }

    ++ch.okCount;
    ch.consecutiveErrors = 0;
    ch.valid = true;
    ch.sampleUs = t1;
    if (full) ch.state = stateFromDiag(ch.reading.diag);
    if (ch.state == AS5048BChannelState::Online) enc.trackAngle(ch.reading.angle);

    if (ok == 0) firstEndUs = t1;
    lastEndUs = t1;
    ++ok;
  }

  stats_.lastCycleUs = micros() - cycleStart;
  stats_.lastSkewUs = ok > 1 ? lastEndUs - firstEndUs : 0;
  if (stats_.lastCycleUs > stats_.maxCycleUs) stats_.maxCycleUs = stats_.lastCycleUs;
  if (stats_.lastSkewUs > stats_.maxSkewUs) stats_.maxSkewUs = stats_.lastSkewUs;
  stats_.totalCycleUs += stats_.lastCycleUs;
  ++stats_.cycles;
  return ok;
}

uint8_t AS5048BGroup::zeroAll() {
  uint8_t ok = 0;
  for (uint8_t i = 0; i < count_; ++i) {
    AS5048BChannel& ch = channels_[i];
    if (ch.state == AS5048BChannelState::Offline) continue;
    ch.lastStatus = ch.encoder->setZeroHere();
    if (ch.lastStatus == AS5048BStatus::Ok) ++ok;
  }
  return ok;
}

void AS5048BGroup::resetTurnsAll() {
  for (uint8_t i = 0; i < count_; ++i) {
    AS5048BChannel& ch = channels_[i];
    if (ch.state != AS5048BChannelState::Offline) ch.lastStatus = ch.encoder->resetTurns();
  }
}

void AS5048BGroup::clearStats() {
  for (uint8_t i = 0; i < count_; ++i) {
    AS5048BChannel& ch = channels_[i];
    ch.okCount = ch.errorCount = ch.recoveries = 0;
    ch.maxReadUs = 0;
  }
  stats_ = AS5048BCycleStats();
}

const char* AS5048BGroup::stateToString(AS5048BChannelState state) {
  switch (state) {
    case AS5048BChannelState::Online: return "Online";
    case AS5048BChannelState::NoMagnet: return "NoMagnet";
    case AS5048BChannelState::Offline: return "Offline";
  }
  return "?";
}

void AS5048BGroup::tryRecover(AS5048BChannel& ch, uint32_t nowMs) {
  ch.nextRetryMs = nowMs + kRetryIntervalMs;
  if (!ch.encoder->isConnected()) return;
  if (ch.encoder->begin(0, 0, 0, false)) {
    ch.state = AS5048BChannelState::Online;
  } else if (ch.encoder->lastStatus() == AS5048BStatus::MagnetError) {
    ch.state = AS5048BChannelState::NoMagnet;
  } else {
    return;
  }
  ch.consecutiveErrors = 0;
  ++ch.recoveries;
}

void AS5048BGroup::markFailure(AS5048BChannel& ch, AS5048BStatus status, uint32_t nowMs) {
  ++ch.errorCount;
  ++ch.consecutiveErrors;
  ch.lastStatus = status;
  if (ch.consecutiveErrors >= kOfflineAfterErrors) {
    ch.state = AS5048BChannelState::Offline;
    ch.valid = false;
    ch.nextRetryMs = nowMs + kRetryIntervalMs;
  }
}
