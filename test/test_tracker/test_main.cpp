#include <exo_tracker.h>
#include <joint_map.h>
#include <unity.h>

using namespace teleop;

namespace {

constexpr int32_t kTurn = kEncoderCounts;
constexpr int32_t kDeadZone = 455;  // 10 deg

uint16_t rawAt(uint16_t ref, int32_t x) {
  int32_t r = (static_cast<int32_t>(ref) + x) % kTurn;
  if (r < 0) r += kTurn;
  return static_cast<uint16_t>(r);
}

// Moves the encoder from x0 to x1 in steps no larger than `step` counts.
uint32_t sweepTo(ExoTracker& t, uint16_t ref, int32_t x0, int32_t x1, uint32_t now, int32_t step = 300) {
  int32_t x = x0;
  while (x != x1) {
    const int32_t d = x1 - x;
    x += d > step ? step : (d < -step ? -step : d);
    now += 10;
    TEST_ASSERT_EQUAL(ExoTracker::Result::Accepted, t.update(rawAt(ref, x), now));
  }
  return now;
}

}  // namespace

void setUp() {}
void tearDown() {}

void test_single_turn_300_deg_across_zero_auto_homes() {
  ExoTracker t;
  const uint16_t ref = 15000;          // A near the top: the range crosses raw 0
  const int32_t span = 13653;          // 300 deg
  t.configure(ref, span, kDeadZone);
  TEST_ASSERT_TRUE(t.singleTurn());
  TEST_ASSERT_FALSE(t.homed());
  // Boot with the joint at 250 deg from A: raw has wrapped, old unwrap would say -110 deg.
  const int32_t x = 11378;
  TEST_ASSERT_EQUAL(ExoTracker::Result::Accepted, t.update(rawAt(ref, x), 0));
  TEST_ASSERT_TRUE(t.homed());
  TEST_ASSERT_EQUAL(x, t.position());
  TEST_ASSERT_FALSE(t.continuous());  // auto-resolved, not an explicit home
}

void test_single_turn_sample_in_unused_arc_waits() {
  ExoTracker t;
  t.configure(1000, 13653, kDeadZone);  // 300 deg range, 60 deg unused
  // 30 deg beyond B: inside the unused arc, beyond the 10 deg dead zone.
  TEST_ASSERT_EQUAL(ExoTracker::Result::Unhomed, t.update(rawAt(1000, 13653 + 1365), 0));
  TEST_ASSERT_FALSE(t.homed());
}

void test_multi_turn_needs_home_and_picks_turn_nearest_a() {
  ExoTracker t;
  const uint16_t ref = 2000;
  const int32_t span = 24576;  // 1.5 turns
  t.configure(ref, span, kDeadZone);
  TEST_ASSERT_FALSE(t.singleTurn());
  TEST_ASSERT_EQUAL(ExoTracker::Result::Unhomed, t.update(rawAt(ref, 300), 0));
  TEST_ASSERT_NULL(t.home(rawAt(ref, 300), 0));  // glove near A
  TEST_ASSERT_TRUE(t.homed());
  TEST_ASSERT_TRUE(t.continuous());
  TEST_ASSERT_EQUAL(300, t.position());
}

void test_home_rejected_outside_range() {
  ExoTracker t;
  const uint16_t ref = 2000;
  t.configure(ref, 24576, kDeadZone);
  // Glove near B (1.5 turns): the turn nearest A is -0.5 turn, outside [0, B].
  TEST_ASSERT_NOT_NULL(t.home(rawAt(ref, 24576), 0));
  TEST_ASSERT_FALSE(t.homed());
  ExoTracker u;
  TEST_ASSERT_NOT_NULL(u.home(100, 0));  // uncalibrated
}

void test_tracks_across_several_turns() {
  ExoTracker t;
  const uint16_t ref = 9000;
  const int32_t span = 2 * kTurn + 4000;
  t.configure(ref, span, kDeadZone);
  TEST_ASSERT_NULL(t.home(rawAt(ref, 0), 0));
  uint32_t now = sweepTo(t, ref, 0, span, 0);
  TEST_ASSERT_EQUAL(span, t.position());
  now = sweepTo(t, ref, span, 1234, now);
  TEST_ASSERT_EQUAL(1234, t.position());
  TEST_ASSERT_TRUE(t.continuous());
}

void test_gap_drops_home_for_multi_turn_only() {
  ExoTracker multi;
  multi.setLossMs(100);
  multi.configure(2000, 24576, kDeadZone);
  multi.home(rawAt(2000, 0), 0);
  multi.noSample(50);
  TEST_ASSERT_TRUE(multi.homed());
  multi.noSample(150);
  TEST_ASSERT_FALSE(multi.homed());
  TEST_ASSERT_EQUAL(ExoTracker::Result::Unhomed, multi.update(rawAt(2000, 100), 160));

  ExoTracker single;
  single.setLossMs(100);
  single.configure(2000, 8000, kDeadZone);
  TEST_ASSERT_EQUAL(ExoTracker::Result::Accepted, single.update(rawAt(2000, 100), 0));
  // A long gap, then a sample: re-resolved absolutely, no home needed.
  TEST_ASSERT_EQUAL(ExoTracker::Result::Accepted, single.update(rawAt(2000, 7000), 500));
  TEST_ASSERT_EQUAL(7000, single.position());
}

void test_jump_rejected_then_lost() {
  ExoTracker t;
  t.setMaxStep(1138);  // 25 deg
  t.configure(2000, 24576, kDeadZone);
  t.home(rawAt(2000, 0), 0);
  TEST_ASSERT_EQUAL(ExoTracker::Result::Accepted, t.update(rawAt(2000, 500), 10));
  TEST_ASSERT_EQUAL(ExoTracker::Result::Rejected, t.update(rawAt(2000, 6000), 20));
  TEST_ASSERT_EQUAL(500, t.position());  // glitch ignored
  TEST_ASSERT_EQUAL(ExoTracker::Result::Accepted, t.update(rawAt(2000, 600), 30));
  TEST_ASSERT_EQUAL(ExoTracker::Result::Rejected, t.update(rawAt(2000, 7000), 40));
  TEST_ASSERT_EQUAL(ExoTracker::Result::Rejected, t.update(rawAt(2000, 7000), 50));
  TEST_ASSERT_EQUAL(ExoTracker::Result::Lost, t.update(rawAt(2000, 7000), 60));
  TEST_ASSERT_FALSE(t.homed());
}

void test_calibration_flow_measures_multi_turn_span() {
  ExoTracker t;
  t.configure(0, 0, kDeadZone);  // uncalibrated
  const uint16_t rawA = 5000;
  t.homeAtA(rawA, 0);  // cal a
  TEST_ASSERT_TRUE(t.continuous());
  const int32_t span = kTurn + 6000;  // 1.37 turns, beyond what unwrap alone can measure
  sweepTo(t, rawA, 0, span, 0);
  TEST_ASSERT_TRUE(t.continuous());
  TEST_ASSERT_EQUAL(span, t.position());  // what cal b stores
  t.setSpan(span);
  TEST_ASSERT_TRUE(t.homed());  // no re-home needed right after calibration
  TEST_ASSERT_FALSE(t.singleTurn());
  TEST_ASSERT_EQUAL(span, t.position());
}

void test_uncalibrated_resolves_nearest_but_not_continuous() {
  ExoTracker t;
  t.configure(4000, 0, kDeadZone);
  TEST_ASSERT_EQUAL(ExoTracker::Result::Accepted, t.update(4300, 0));
  TEST_ASSERT_EQUAL(300, t.position());
  TEST_ASSERT_FALSE(t.continuous());  // cal b must not trust this
}

int main() {
  UNITY_BEGIN();
  RUN_TEST(test_single_turn_300_deg_across_zero_auto_homes);
  RUN_TEST(test_single_turn_sample_in_unused_arc_waits);
  RUN_TEST(test_multi_turn_needs_home_and_picks_turn_nearest_a);
  RUN_TEST(test_home_rejected_outside_range);
  RUN_TEST(test_tracks_across_several_turns);
  RUN_TEST(test_gap_drops_home_for_multi_turn_only);
  RUN_TEST(test_jump_rejected_then_lost);
  RUN_TEST(test_calibration_flow_measures_multi_turn_span);
  RUN_TEST(test_uncalibrated_resolves_nearest_but_not_continuous);
  return UNITY_END();
}
