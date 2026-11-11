#include <replay_buffer.h>
#include <unity.h>

using namespace teleop;

void setUp() {}
void tearDown() {}

namespace {
// Joints 1 and 3 (indices 0 and 2): frames are {base + i, -(base + i)}.
constexpr uint16_t kMask = 0b101;

void chunk(int16_t* out, uint32_t first, uint8_t count) {
  for (uint8_t i = 0; i < count; ++i) {
    out[2 * i] = static_cast<int16_t>(100 + first + i);
    out[2 * i + 1] = static_cast<int16_t>(-(100 + static_cast<int32_t>(first + i)));
  }
}
}  // namespace

void test_push_pop_in_order() {
  ReplayBuffer b;
  int16_t data[20];
  chunk(data, 0, 10);
  TEST_ASSERT_EQUAL(ReplayBuffer::Push::Ok, b.push(0, kMask, data, 10, false));
  TEST_ASSERT_EQUAL(10, b.buffered());
  TEST_ASSERT_EQUAL(10, b.bufferedUntil());
  int16_t frame[kMaxJoints] = {7, 7, 7, 7, 7, 7, 7, 7, 7};
  TEST_ASSERT_TRUE(b.peek(frame));
  TEST_ASSERT_EQUAL(0, b.playIndex());
  TEST_ASSERT_TRUE(b.pop(frame));
  TEST_ASSERT_EQUAL(100, frame[0]);
  TEST_ASSERT_EQUAL(7, frame[1]);  // outside the mask: untouched
  TEST_ASSERT_EQUAL(-100, frame[2]);
  TEST_ASSERT_TRUE(b.pop(frame));
  TEST_ASSERT_EQUAL(101, frame[0]);
  TEST_ASSERT_EQUAL(2, b.playIndex());
  TEST_ASSERT_EQUAL(8, b.buffered());
}

void test_gap_is_refused_overlap_keeps_the_tail() {
  ReplayBuffer b;
  int16_t data[40];
  chunk(data, 0, 10);
  b.push(0, kMask, data, 10, false);
  chunk(data, 12, 5);
  TEST_ASSERT_EQUAL(ReplayBuffer::Push::Gap, b.push(12, kMask, data, 5, false));
  TEST_ASSERT_EQUAL(10, b.bufferedUntil());
  chunk(data, 6, 10);  // resend overlapping frames 6..9
  TEST_ASSERT_EQUAL(ReplayBuffer::Push::Ok, b.push(6, kMask, data, 10, false));
  TEST_ASSERT_EQUAL(16, b.bufferedUntil());
  int16_t frame[kMaxJoints] = {};
  for (int i = 0; i < 11; ++i) b.pop(frame);
  TEST_ASSERT_EQUAL(110, frame[0]);  // frame 10 came from the resend
  chunk(data, 0, 3);  // entirely old: ignored
  TEST_ASSERT_EQUAL(ReplayBuffer::Push::Ok, b.push(0, kMask, data, 3, false));
  TEST_ASSERT_EQUAL(16, b.bufferedUntil());
}

void test_mask_capacity_and_end() {
  ReplayBuffer b;
  int16_t data[2 * 255];
  chunk(data, 0, 200);
  TEST_ASSERT_EQUAL(ReplayBuffer::Push::Ok, b.push(0, kMask, data, 200, false));
  TEST_ASSERT_EQUAL(ReplayBuffer::Push::MaskChanged, b.push(200, 0b1, data, 1, false));
  chunk(data, 200, 100);
  TEST_ASSERT_EQUAL(ReplayBuffer::Push::Full, b.push(200, kMask, data, 100, false));
  chunk(data, 200, 50);
  TEST_ASSERT_EQUAL(ReplayBuffer::Push::Ok, b.push(200, kMask, data, 50, true));
  TEST_ASSERT_TRUE(b.ended());
  TEST_ASSERT_FALSE(b.finished());
  TEST_ASSERT_EQUAL(ReplayBuffer::Push::Ended, b.push(250, kMask, data, 1, false));
  int16_t frame[kMaxJoints] = {};
  while (b.pop(frame)) {
  }
  TEST_ASSERT_TRUE(b.finished());
  TEST_ASSERT_EQUAL(249 + 100, frame[0]);
  TEST_ASSERT_EQUAL(250, b.playIndex());
}

void test_reset_starts_a_new_stream() {
  ReplayBuffer b;
  int16_t data[4];
  chunk(data, 0, 2);
  b.push(0, kMask, data, 2, true);
  b.reset();
  TEST_ASSERT_EQUAL(0, b.buffered());
  TEST_ASSERT_EQUAL(ReplayBuffer::Push::Ok, b.push(0, 0b11, data, 2, false));  // new mask allowed
  TEST_ASSERT_EQUAL(0b11, b.mask());
  TEST_ASSERT_EQUAL(2, ReplayBuffer::jointsIn(0b11));
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_push_pop_in_order);
  RUN_TEST(test_gap_is_refused_overlap_keeps_the_tail);
  RUN_TEST(test_mask_capacity_and_end);
  RUN_TEST(test_reset_starts_a_new_stream);
  return UNITY_END();
}
