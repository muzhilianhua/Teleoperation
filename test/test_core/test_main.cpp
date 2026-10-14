#include <joint_map.h>
#include <jump_guard.h>
#include <math.h>
#include <one_euro.h>
#include <rate_limiter.h>
#include <supervisor.h>
#include <unity.h>

using namespace teleop;

void setUp() {}
void tearDown() {}

// ---- joint map ---------------------------------------------------------------

void test_unwrap_across_zero() {
  TEST_ASSERT_EQUAL(0, unwrapCounts(100, 100));
  TEST_ASSERT_EQUAL(200, unwrapCounts(16300, 16100));
  TEST_ASSERT_EQUAL(184, unwrapCounts(100, 16300));    // wraps forward through 0
  TEST_ASSERT_EQUAL(-184, unwrapCounts(16300, 100));   // wraps backward through 0
  TEST_ASSERT_EQUAL(-8192, unwrapCounts(8192, 0));     // half turn folds to the negative side
  TEST_ASSERT_EQUAL(8191, unwrapCounts(8191, 0));
}

JointMap makeJoint() {
  JointMap j;
  j.exoRef = 1000;
  j.exoA = 0;
  j.exoB = 2000;
  j.servoA = 400;
  j.servoB = 700;
  j.servoMin = 350;
  j.servoMax = 730;
  return j;
}

void test_map_endpoints_and_midpoint() {
  const JointMap j = makeJoint();
  TEST_ASSERT_TRUE(j.ready());
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 400.0f, mapToServo(j, 0.0f));
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 700.0f, mapToServo(j, 2000.0f));
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 550.0f, mapToServo(j, 1000.0f));
}

void test_map_saturates_outside_calibrated_range() {
  const JointMap j = makeJoint();
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 400.0f, mapToServo(j, -500.0f));
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 700.0f, mapToServo(j, 9000.0f));
}

void test_map_reversed_directions() {
  JointMap j = makeJoint();
  j.exoB = -2000;  // encoder turns the other way
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 550.0f, mapToServo(j, -1000.0f));
  j.servoA = 700;  // servo also reversed
  j.servoB = 400;
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 400.0f, mapToServo(j, -2000.0f));
}

void test_map_clamps_to_travel() {
  JointMap j = makeJoint();
  j.servoMin = 450;  // endpoints outside the travel: not ready, output still clamped
  TEST_ASSERT_FALSE(j.servoRangeValid());
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 450.0f, mapToServo(j, 0.0f));
}

void test_uncalibrated_joint_not_ready() {
  JointMap j = makeJoint();
  j.exoB = j.exoA;
  TEST_ASSERT_FALSE(j.ready());
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 400.0f, mapToServo(j, 1234.0f));
}

// ---- one euro ----------------------------------------------------------------

void test_one_euro_converges_on_constant() {
  OneEuroFilter f(1.0f, 0.0f, 1.0f);
  f.filter(0.0f, 0.01f);
  float y = 0.0f;
  for (int i = 0; i < 500; ++i) y = f.filter(10.0f, 0.01f);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 10.0f, y);
}

void test_one_euro_beta_reduces_lag() {
  OneEuroFilter slow(1.0f, 0.0f, 1.0f);
  OneEuroFilter fast(1.0f, 0.5f, 1.0f);
  float ys = 0.0f, yf = 0.0f;
  for (int i = 0; i <= 50; ++i) {  // 100 deg/s ramp for 0.5 s
    const float x = i * 1.0f;
    ys = slow.filter(x, 0.01f);
    yf = fast.filter(x, 0.01f);
  }
  TEST_ASSERT_TRUE(50.0f - yf < 50.0f - ys);
}

// ---- rate limiter ------------------------------------------------------------

void test_rate_limiter_respects_speed() {
  RateLimiter r(400.0f, 200.0f, 2000.0f, 350.0f, 730.0f);
  r.setTarget(700.0f);
  float prev = r.position();
  for (int i = 0; i < 300; ++i) {
    const float p = r.step(0.01f);
    TEST_ASSERT_TRUE(fabsf(p - prev) <= 200.0f * 0.01f + 1e-3f);
    prev = p;
  }
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 700.0f, r.position());
}

void test_rate_limiter_never_overshoots_stationary_target() {
  RateLimiter r(400.0f, 500.0f, 3000.0f, 350.0f, 730.0f);
  r.setTarget(600.0f);
  for (int i = 0; i < 300; ++i) TEST_ASSERT_TRUE(r.step(0.01f) <= 600.0f + 1e-3f);
}

void test_rate_limiter_clamps_target_to_travel() {
  RateLimiter r(400.0f, 200.0f, 2000.0f, 350.0f, 730.0f);
  r.setTarget(1000.0f);
  for (int i = 0; i < 500; ++i) r.step(0.01f);
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 730.0f, r.position());
}

void test_rate_limiter_follows_slow_target_exactly() {
  // 100 steps/s ramp: within the speed limit, so after the acceleration-limited
  // catch-up the limiter must track it with zero lag (velocity feed-forward).
  RateLimiter r(400.0f, 200.0f, 2000.0f, 350.0f, 730.0f);
  for (int i = 1; i <= 100; ++i) {
    r.setTarget(400.0f + i);
    r.step(0.01f);
  }
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 500.0f, r.position());
  TEST_ASSERT_FLOAT_WITHIN(1e-2f, 100.0f, r.velocity());
}

// ---- supervisor --------------------------------------------------------------

ServoSample healthy(bool torque) {
  ServoSample s;
  s.readOk = true;
  s.torqueOn = torque;
  s.position = 500;
  s.loadPct = 10.0f;
  s.voltage = 6.0f;
  s.temperatureC = 35;
  return s;
}

void test_supervisor_healthy() {
  JointSupervisor sup;
  TEST_ASSERT_EQUAL(Fault::None, sup.update(healthy(true), true, 500.0f, 500.0f, 0));
}

void test_supervisor_comm_lost_after_three_failures() {
  JointSupervisor sup;
  ServoSample bad;
  TEST_ASSERT_EQUAL(Fault::None, sup.update(bad, true, 500.0f, 500.0f, 0));
  TEST_ASSERT_EQUAL(Fault::None, sup.update(bad, true, 500.0f, 500.0f, 10));
  TEST_ASSERT_EQUAL(Fault::CommLost, sup.update(bad, true, 500.0f, 500.0f, 20));
  TEST_ASSERT_EQUAL(Fault::None, sup.update(healthy(true), true, 500.0f, 500.0f, 30));
}

void test_supervisor_torque_mismatch_and_servo_error() {
  JointSupervisor sup;
  TEST_ASSERT_EQUAL(Fault::TorqueMismatch, sup.update(healthy(false), true, 500.0f, 500.0f, 0));
  TEST_ASSERT_EQUAL(Fault::TorqueMismatch, sup.update(healthy(true), false, 500.0f, 500.0f, 0));
  ServoSample s = healthy(true);
  s.errorByte = 0x20;
  TEST_ASSERT_EQUAL(Fault::ServoError, sup.update(s, true, 500.0f, 500.0f, 0));
}

void test_supervisor_load_trip_needs_persistence() {
  JointSupervisor sup;
  ServoSample s = healthy(true);
  s.loadPct = -76.0f;  // sign must not matter
  TEST_ASSERT_EQUAL(Fault::None, sup.update(s, true, 500.0f, 500.0f, 0));
  TEST_ASSERT_EQUAL(Fault::None, sup.update(s, true, 500.0f, 500.0f, 290));
  TEST_ASSERT_EQUAL(Fault::LoadTrip, sup.update(s, true, 500.0f, 500.0f, 300));
}

void test_supervisor_load_hard_is_faster() {
  JointSupervisor sup;
  ServoSample s = healthy(true);
  s.loadPct = 85.0f;
  TEST_ASSERT_EQUAL(Fault::None, sup.update(s, true, 500.0f, 500.0f, 0));
  TEST_ASSERT_EQUAL(Fault::LoadHard, sup.update(s, true, 500.0f, 500.0f, 100));
}

void test_supervisor_tracking_error_only_when_armed() {
  JointSupervisor sup;
  ServoSample s = healthy(true);
  TEST_ASSERT_EQUAL(Fault::None, sup.update(s, true, 560.0f, 560.0f, 0));
  TEST_ASSERT_EQUAL(Fault::TrackingError, sup.update(s, true, 560.0f, 560.0f, 400));
  sup.reset();
  ServoSample off = healthy(false);
  TEST_ASSERT_EQUAL(Fault::None, sup.update(off, false, 560.0f, 560.0f, 0));
  TEST_ASSERT_EQUAL(Fault::None, sup.update(off, false, 560.0f, 560.0f, 1000));
}

void test_supervisor_servo_faster_than_model_is_fine() {
  JointSupervisor sup;
  ServoSample s = healthy(true);
  s.position = 590;  // goal 700, model only reached 550: servo is ahead of its model
  for (uint32_t t = 0; t <= 1000; t += 10) {
    TEST_ASSERT_EQUAL(Fault::None, sup.update(s, true, 700.0f, 550.0f, t));
  }
}

void test_supervisor_goal_outrunning_capability_is_fine() {
  JointSupervisor sup;
  ServoSample s = healthy(true);
  s.position = 520;  // far behind the goal, but keeping pace with the model
  for (uint32_t t = 0; t <= 1000; t += 10) {
    TEST_ASSERT_EQUAL(Fault::None, sup.update(s, true, 700.0f, 530.0f, t));
  }
}

void test_supervisor_lag_behind_model_trips() {
  JointSupervisor sup;
  ServoSample s = healthy(true);
  s.position = 500;  // model reached 590: lag = 200 - 110 = 90 steps
  TEST_ASSERT_EQUAL(Fault::None, sup.update(s, true, 700.0f, 590.0f, 0));
  TEST_ASSERT_EQUAL(Fault::None, sup.update(s, true, 700.0f, 590.0f, 390));
  TEST_ASSERT_EQUAL(Fault::TrackingError, sup.update(s, true, 700.0f, 590.0f, 400));
}

// ---- jump guard --------------------------------------------------------------

void test_jump_guard_rejects_single_glitch() {
  JumpGuard g(25.0f, 3);
  TEST_ASSERT_EQUAL(JumpGuard::Result::Accepted, g.check(10.0f));
  TEST_ASSERT_EQUAL(JumpGuard::Result::Accepted, g.check(30.0f));   // 20 deg: fast but plausible
  TEST_ASSERT_EQUAL(JumpGuard::Result::Rejected, g.check(120.0f));  // glitch
  TEST_ASSERT_EQUAL(JumpGuard::Result::Accepted, g.check(32.0f));   // back to normal
}

void test_jump_guard_resyncs_after_persistent_jump() {
  JumpGuard g(25.0f, 3);
  g.check(0.0f);
  TEST_ASSERT_EQUAL(JumpGuard::Result::Rejected, g.check(90.0f));
  TEST_ASSERT_EQUAL(JumpGuard::Result::Rejected, g.check(90.0f));
  TEST_ASSERT_EQUAL(JumpGuard::Result::Resynced, g.check(90.0f));
  TEST_ASSERT_EQUAL(JumpGuard::Result::Accepted, g.check(91.0f));  // new baseline
}

void test_jump_guard_reset_accepts_anything() {
  JumpGuard g(25.0f, 3);
  g.check(0.0f);
  g.reset();
  TEST_ASSERT_EQUAL(JumpGuard::Result::Accepted, g.check(170.0f));
}

void test_supply_uses_median() {
  SupplySupervisor sup;
  const float oneBad[] = {6.0f, 6.0f, 4.0f};
  TEST_ASSERT_EQUAL(Fault::None, sup.update(oneBad, 3, 0));
  TEST_ASSERT_EQUAL(Fault::None, sup.update(oneBad, 3, 1000));
  const float sag[] = {5.2f, 5.3f, 5.1f};
  TEST_ASSERT_EQUAL(Fault::None, sup.update(sag, 3, 2000));
  TEST_ASSERT_EQUAL(Fault::SupplyVoltage, sup.update(sag, 3, 2200));
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 5.25f, median(sag, 2));
}

int main() {
  UNITY_BEGIN();
  RUN_TEST(test_unwrap_across_zero);
  RUN_TEST(test_map_endpoints_and_midpoint);
  RUN_TEST(test_map_saturates_outside_calibrated_range);
  RUN_TEST(test_map_reversed_directions);
  RUN_TEST(test_map_clamps_to_travel);
  RUN_TEST(test_uncalibrated_joint_not_ready);
  RUN_TEST(test_one_euro_converges_on_constant);
  RUN_TEST(test_one_euro_beta_reduces_lag);
  RUN_TEST(test_rate_limiter_respects_speed);
  RUN_TEST(test_rate_limiter_never_overshoots_stationary_target);
  RUN_TEST(test_rate_limiter_clamps_target_to_travel);
  RUN_TEST(test_rate_limiter_follows_slow_target_exactly);
  RUN_TEST(test_supervisor_healthy);
  RUN_TEST(test_supervisor_comm_lost_after_three_failures);
  RUN_TEST(test_supervisor_torque_mismatch_and_servo_error);
  RUN_TEST(test_supervisor_load_trip_needs_persistence);
  RUN_TEST(test_supervisor_load_hard_is_faster);
  RUN_TEST(test_supervisor_tracking_error_only_when_armed);
  RUN_TEST(test_supervisor_servo_faster_than_model_is_fine);
  RUN_TEST(test_supervisor_goal_outrunning_capability_is_fine);
  RUN_TEST(test_supervisor_lag_behind_model_trips);
  RUN_TEST(test_jump_guard_rejects_single_glitch);
  RUN_TEST(test_jump_guard_resyncs_after_persistent_jump);
  RUN_TEST(test_jump_guard_reset_accepts_anything);
  RUN_TEST(test_supply_uses_median);
  return UNITY_END();
}
