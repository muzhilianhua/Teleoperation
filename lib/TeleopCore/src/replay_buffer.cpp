#include "replay_buffer.h"

#include <string.h>

namespace teleop {

uint8_t ReplayBuffer::jointsIn(uint16_t mask) {
  uint8_t n = 0;
  for (uint8_t i = 0; i < kMaxJoints; ++i) n += (mask >> i) & 1u;
  return n;
}

void ReplayBuffer::reset() {
  head_ = 0;
  count_ = 0;
  playIndex_ = 0;
  mask_ = 0;
  ended_ = false;
}

ReplayBuffer::Push ReplayBuffer::push(uint32_t firstIndex, uint16_t mask, const int16_t* positions,
                                      uint8_t count, bool end) {
  if (ended_) return Push::Ended;
  if (mask_ == 0 && playIndex_ == 0 && count_ == 0) mask_ = mask;  // the first chunk sets the stream's joints
  if (mask != mask_) return Push::MaskChanged;
  const uint32_t next = bufferedUntil();
  if (firstIndex > next) return Push::Gap;
  // Skip the part of a resent chunk that is already buffered (or played).
  const uint32_t skip = next - firstIndex;
  if (skip > count) return Push::Ok;
  const uint8_t fresh = static_cast<uint8_t>(count - skip);
  if (count_ + fresh > kReplayCapacity) return Push::Full;
  const uint8_t width = jointsIn(mask_);
  for (uint8_t f = 0; f < fresh; ++f) {
    const int16_t* src = positions + (skip + f) * width;
    int16_t* dst = frames_[(head_ + count_) % kReplayCapacity];
    uint8_t k = 0;
    for (uint8_t i = 0; i < kMaxJoints; ++i) {
      if (mask_ & (1u << i)) dst[i] = src[k++];
    }
    ++count_;
  }
  if (end) ended_ = true;
  return Push::Ok;
}

bool ReplayBuffer::peek(int16_t* frame) const {
  if (count_ == 0) return false;
  for (uint8_t i = 0; i < kMaxJoints; ++i) {
    if (mask_ & (1u << i)) frame[i] = frames_[head_][i];
  }
  return true;
}

bool ReplayBuffer::pop(int16_t* frame) {
  if (!peek(frame)) return false;
  head_ = static_cast<uint16_t>((head_ + 1) % kReplayCapacity);
  --count_;
  ++playIndex_;
  return true;
}

}  // namespace teleop
