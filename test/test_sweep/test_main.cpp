// SpeedSweep against a simulated servo: it moves toward its goal at
// min(speed register, saturation in that direction), sampled at 100 Hz like
// the control loop.

#include <math.h>
#include <speed_sweep.h>
#include <unity.h>

using namespace teleop;

namespace {

struct SimServo {
  float position = 500.0f;
  float goal = 500.0f;
  float speed = 0.0f;  // register
  float satPlus = 550.0f;
  float satMinus = 270.0f;
  float load = 20.0f;  // constant load the servo reports

  void command(int16_t g, uint16_t s) {
    goal = g;
    speed = s;
  }
  void step(float dt) {
    const float error = goal - position;
    if (fabsf(error) < 1e-3f) return;
    const float sat = error > 0 ? satPlus : satMinus;
    const float v = fminf(speed, sat);
    const float move = fminf(fabsf(error), v * dt);
    position += error > 0 ? move : -move;
  }
};

SweepConfig makeConfig() {
  SweepConfig c;
  c.lo = 360;
  c.hi = 720;
  return c;
}

// Runs the sweep to completion (or a 200 s safety cap); returns the cycles used.
uint32_t runSweep(SpeedSweep& sweep, SimServo& servo, uint32_t* finishedRuns = nullptr) {
  uint32_t now = 1000;
  sweep.start(makeConfig(), static_cast<int16_t>(servo.position), now);
  uint32_t runs = 0;
  for (uint32_t cycle = 0; cycle < 20000; ++cycle) {
    now += 10;
    servo.step(0.01f);
    const SweepStep s = sweep.update(now, static_cast<int16_t>(lroundf(servo.position)), servo.load, true);
    if (s.write) servo.command(s.goal, s.speed);
    if (s.runFinished) ++runs;
    if (s.done) {
      if (finishedRuns) *finishedRuns = runs;
      return cycle;
    }
  }
  return 0;
}

}  // namespace

void setUp() {}
void tearDown() {}

void test_free_joint_measures_both_directions() {
  SpeedSweep sweep;
  SimServo servo;
  uint32_t runs = 0;
  TEST_ASSERT_TRUE(runSweep(sweep, servo, &runs) > 0);
  const SweepResult& r = sweep.result();
  TEST_ASSERT_TRUE(r.completed);
  TEST_ASSERT_FALSE(r.aborted);
  TEST_ASSERT_EQUAL(8, r.runCount);
  TEST_ASSERT_EQUAL(8, runs);
  TEST_ASSERT_FALSE(r.forceLimited());
  TEST_ASSERT_FLOAT_WITHIN(0.05f * 550.0f, 550.0f, r.saturationPlus);
  TEST_ASSERT_FLOAT_WITHIN(0.05f * 270.0f, 270.0f, r.saturationMinus);
  TEST_ASSERT_UINT16_WITHIN(35, 660, r.suggestedVmax);  // 1.2 x 550
  TEST_ASSERT_UINT16_WITHIN(15, 243, r.suggestedVsat);  // 0.9 x 270
  TEST_ASSERT_TRUE(r.applicable());
  // At register 100 the register itself is the limit.
  TEST_ASSERT_FLOAT_WITHIN(5.0f, 100.0f, r.runs[0].speed);
  TEST_ASSERT_EQUAL_FLOAT(100.0f, r.runs[0].commandSpeed);
  TEST_ASSERT_EQUAL(0, r.runs[0].level);
  TEST_ASSERT_EQUAL(1, r.runs[0].direction);
  TEST_ASSERT_EQUAL(-1, r.runs[1].direction);
}

void test_fully_force_limited_joint() {
  SpeedSweep sweep;
  SimServo servo;
  servo.satPlus = servo.satMinus = 5.0f;  // creeps like index_pip did
  TEST_ASSERT_TRUE(runSweep(sweep, servo) > 0);
  const SweepResult& r = sweep.result();
  TEST_ASSERT_TRUE(r.completed);
  TEST_ASSERT_TRUE(r.limitedPlus);
  TEST_ASSERT_TRUE(r.limitedMinus);
  TEST_ASSERT_FALSE(r.applicable());
  TEST_ASSERT_EQUAL(0, r.suggestedVmax);
  uint8_t valid = 0;
  for (uint8_t i = 0; i < r.runCount; ++i) {
    TEST_ASSERT_FALSE(r.runs[i].reached);
    if (!r.runs[i].valid) continue;  // never got back far enough to start a full move
    ++valid;
    TEST_ASSERT_FLOAT_WITHIN(1.5f, 5.0f, r.runs[i].speed);  // creep speed is still reported
  }
  TEST_ASSERT_TRUE(valid >= 1);
}

void test_one_direction_stuck() {
  SpeedSweep sweep;
  SimServo servo;
  servo.satPlus = 170.0f;  // like thumb_mcp: moves one way, stuck the other
  servo.satMinus = 2.0f;
  TEST_ASSERT_TRUE(runSweep(sweep, servo) > 0);
  const SweepResult& r = sweep.result();
  TEST_ASSERT_TRUE(r.completed);
  TEST_ASSERT_TRUE(r.limitedMinus);
  TEST_ASSERT_TRUE(r.forceLimited());
  TEST_ASSERT_FALSE(r.applicable());
  // The first + move is real; later + moves start next to the target because
  // the - moves never came back. They are skipped instead of reported as fast.
  TEST_ASSERT_TRUE(r.runs[0].valid);
  TEST_ASSERT_TRUE(r.runs[0].reached);
  TEST_ASSERT_FLOAT_WITHIN(5.0f, 100.0f, r.runs[0].speed);
  for (uint8_t i = 2; i < r.runCount; i += 2) TEST_ASSERT_FALSE(r.runs[i].valid);
  TEST_ASSERT_TRUE(r.saturationPlus <= 105.0f);
}

void test_sustained_high_load_aborts() {
  SpeedSweep sweep;
  SimServo servo;
  servo.load = 95.0f;
  const uint32_t cycles = runSweep(sweep, servo);
  TEST_ASSERT_TRUE(cycles > 0 && cycles < 40);  // ~0.3 s
  TEST_ASSERT_TRUE(sweep.result().aborted);
  TEST_ASSERT_FALSE(sweep.result().completed);
  TEST_ASSERT_FALSE(sweep.result().applicable());
}

void test_external_abort_stops_and_holds_position() {
  SpeedSweep sweep;
  sweep.start(makeConfig(), 500, 0);
  sweep.update(10, 500, 20.0f, true);
  TEST_ASSERT_TRUE(sweep.active());
  sweep.abort("operator");
  TEST_ASSERT_FALSE(sweep.active());
  const SweepStep s = sweep.update(20, 500, 20.0f, true);
  TEST_ASSERT_TRUE(s.done);
  TEST_ASSERT_FALSE(s.write);
  TEST_ASSERT_TRUE(sweep.result().aborted);
}

void test_failed_reads_do_not_corrupt_measurement() {
  SpeedSweep sweep;
  SimServo servo;
  uint32_t now = 0;
  sweep.start(makeConfig(), 500, now);
  for (uint32_t cycle = 0; cycle < 20000; ++cycle) {
    now += 10;
    servo.step(0.01f);
    const bool valid = cycle % 7 != 3;  // every 7th read fails
    const SweepStep s = sweep.update(now, static_cast<int16_t>(lroundf(servo.position)), servo.load, valid);
    if (s.write) servo.command(s.goal, s.speed);
    if (s.done) break;
  }
  TEST_ASSERT_TRUE(sweep.result().completed);
  TEST_ASSERT_FLOAT_WITHIN(0.07f * 550.0f, 550.0f, sweep.result().saturationPlus);
}

int main() {
  UNITY_BEGIN();
  RUN_TEST(test_free_joint_measures_both_directions);
  RUN_TEST(test_fully_force_limited_joint);
  RUN_TEST(test_one_direction_stuck);
  RUN_TEST(test_sustained_high_load_aborts);
  RUN_TEST(test_external_abort_stops_and_holds_position);
  RUN_TEST(test_failed_reads_do_not_corrupt_measurement);
  return UNITY_END();
}
