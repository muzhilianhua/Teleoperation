// Ring buffer of replay frames (taught motion played back on the hand).
//
// The host streams a recording ahead of playback in numbered frames, one per
// control cycle; the control task takes one frame per cycle. A frame holds a
// goal (servo steps) for every joint in the stream's joint mask (bit i =
// joint index i in the joint table). Frames must arrive in order: a chunk may
// overlap frames already buffered (a resend after a lost frame), and only its
// new tail is kept; a chunk that would leave a gap is refused, so the host
// resends from bufferedUntil().

#pragma once

#include <stddef.h>
#include <stdint.h>

#include "joint_config.h"

namespace teleop {

constexpr uint16_t kReplayCapacity = 256;  // frames, 2.56 s at 100 Hz

class ReplayBuffer {
 public:
  enum class Push : uint8_t { Ok, Gap, MaskChanged, Full, Ended };

  void reset();

  // positions: count frames of popcount(mask) values each, in joint order.
  Push push(uint32_t firstIndex, uint16_t mask, const int16_t* positions, uint8_t count, bool end);

  // Copies the next frame (all kMaxJoints slots; joints outside the mask are
  // left untouched) and advances. False when nothing is buffered.
  bool pop(int16_t* frame);
  // The next frame without consuming it (approach target).
  bool peek(int16_t* frame) const;

  uint16_t mask() const { return mask_; }
  uint32_t playIndex() const { return playIndex_; }
  uint32_t bufferedUntil() const { return playIndex_ + count_; }  // next frame the host should send
  uint16_t buffered() const { return count_; }
  bool ended() const { return ended_; }
  bool finished() const { return ended_ && count_ == 0; }

  static uint8_t jointsIn(uint16_t mask);

 private:
  int16_t frames_[kReplayCapacity][kMaxJoints] = {};
  uint16_t head_ = 0;  // slot of playIndex_
  uint16_t count_ = 0;
  uint32_t playIndex_ = 0;
  uint16_t mask_ = 0;
  bool ended_ = false;
};

}  // namespace teleop
