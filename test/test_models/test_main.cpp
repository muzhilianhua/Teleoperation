#include <feetech_packet.h>
#include <servo_models.h>
#include <unity.h>

using namespace feetech;

namespace {

// Registers 0..70 of a real HLS3606 (ID 1, firmware 3.43), from
// DEX_hand_software tests/hls_fixture.py: multi-turn (limits 0/0), position 4,
// 4.5 V, 29 C, protection current 800, goal speed 250, current -32.5 mA.
const uint8_t kRealHls[71] = {
    0x03, 0x2B, 0x00, 0x0A, 0x0A, 0x01, 0x00, 0xFD, 0x01, 0x00, 0x00, 0x00, 0x00, 0x50, 0x64, 0x28,
    0xE8, 0x03, 0x76, 0x0D, 0x00, 0x20, 0x20, 0x00, 0x18, 0x00, 0x00, 0x00, 0x20, 0x03, 0x01, 0x00,
    0x00, 0x00, 0x64, 0x19, 0xFF, 0x32, 0x64, 0x19, 0x00, 0x00, 0x00, 0x00, 0x20, 0x03, 0xFA, 0x00,
    0xE8, 0x03, 0x20, 0x20, 0x00, 0x00, 0x00, 0x01, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2D, 0x1D,
    0x00, 0x00, 0x00, 0x04, 0x00, 0x05, 0x80,
};

}  // namespace

void setUp() {}
void tearDown() {}

void test_identify_models() {
  TEST_ASSERT_EQUAL(ServoModel::Hls3606, identifyModel(kRealHls));
  const uint8_t scs[5] = {0x00, 0x05, 0x01, 0x05, 0x04};  // fw 0.5, END=1, model 0x0504 big-endian
  TEST_ASSERT_EQUAL(ServoModel::Scs0009, identifyModel(scs));
  const uint8_t other[5] = {0x03, 0x2B, 0x00, 0x09, 0x0A};
  TEST_ASSERT_EQUAL(ServoModel::Unknown, identifyModel(other));
  TEST_ASSERT_EQUAL_STRING("HLS3606", modelName(ServoModel::Hls3606));
}

void test_decode_real_hls_status() {
  ServoState s;
  TEST_ASSERT_TRUE(decodeStatus(ServoModel::Hls3606, kRealHls + 40, 31, s));
  TEST_ASSERT_FALSE(s.torqueOn);
  TEST_ASSERT_EQUAL(4, s.position);
  TEST_ASSERT_EQUAL(250, s.goalSpeed);
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 4.5f, s.voltage);
  TEST_ASSERT_EQUAL(29, s.temperatureC);
  TEST_ASSERT_TRUE(s.hasCurrent);
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, -32.5f, s.currentMa);  // 0x8005: BIT15 sign, 5 x 6.5 mA
  TEST_ASSERT_FALSE(decodeStatus(ServoModel::Hls3606, kRealHls + 40, 27, s));  // SCS length is too short
}

void test_decode_real_hls_arm_info() {
  ArmInfo a;
  TEST_ASSERT_TRUE(decodeArmInfo(ServoModel::Hls3606, kRealHls + kRegArmInfo, kArmInfoLength, a));
  TEST_ASSERT_EQUAL(0, a.minLimit);  // factory multi-turn mode
  TEST_ASSERT_EQUAL(0, a.maxLimit);
  TEST_ASSERT_EQUAL(800, a.protectionCurrent);  // 5.2 A
  TEST_ASSERT_EQUAL(1, a.resolution);
  TEST_ASSERT_EQUAL(0, a.mode);
}

void test_decode_scs_status_big_endian_and_signs() {
  uint8_t b[27] = {0};
  b[0] = 1;
  put16(b + 2, 512, Endian::Big);                 // goal
  put16(b + 16, 515, Endian::Big);                // position
  put16(b + 18, 120 | 0x8000, Endian::Big);       // speed -120
  put16(b + 20, 300 | (1 << 10), Endian::Big);    // load -30 %
  b[22] = 60;
  b[23] = 35;
  ServoState s;
  TEST_ASSERT_TRUE(decodeStatus(ServoModel::Scs0009, b, sizeof(b), s));
  TEST_ASSERT_TRUE(s.torqueOn);
  TEST_ASSERT_EQUAL(512, s.goal);
  TEST_ASSERT_EQUAL(515, s.position);
  TEST_ASSERT_EQUAL(-120, s.speed);
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, -30.0f, s.loadPct);
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 6.0f, s.voltage);
  TEST_ASSERT_FALSE(s.hasCurrent);
}

void test_hls_negative_position_and_speed_units() {
  uint8_t b[31] = {0};
  put16(b + 16, 40 | 0x8000, Endian::Little);  // multi-turn: -40 steps
  put16(b + 18, 2, Endian::Little);            // 2 units = 1.464 RPM ~ 100 steps/s
  ServoState s;
  TEST_ASSERT_TRUE(decodeStatus(ServoModel::Hls3606, b, sizeof(b), s));
  TEST_ASSERT_EQUAL(-40, s.position);
  TEST_ASSERT_INT_WITHIN(1, 100, s.speed);
}

void test_speed_register_never_zero_and_capped() {
  TEST_ASSERT_EQUAL(1, speedRegister(ServoModel::Hls3606, 0.0f));   // 0 would stop an HLS
  TEST_ASSERT_EQUAL(1, speedRegister(ServoModel::Hls3606, 10.0f));
  TEST_ASSERT_EQUAL(21, speedRegister(ServoModel::Hls3606, 1000.0f));  // 1000 / 49.97
  TEST_ASSERT_EQUAL(20, speedRegister(ServoModel::Hls3606, 999.4f));
  TEST_ASSERT_EQUAL(250, speedRegister(ServoModel::Hls3606, 1e6f));
  TEST_ASSERT_EQUAL(1, speedRegister(ServoModel::Scs0009, 0.0f));    // 0 would be full speed on SCS
  TEST_ASSERT_EQUAL(300, speedRegister(ServoModel::Scs0009, 300.0f));
  TEST_ASSERT_EQUAL(1000, speedRegister(ServoModel::Scs0009, 5000.0f));
  TEST_ASSERT_FLOAT_WITHIN(0.1f, 3.333f, modelInfo(ServoModel::Hls3606).stepScale);
}

void test_encode_goal_blocks() {
  uint8_t g[6];
  encodeGoal(ServoModel::Hls3606, 2048, 800, 1000.0f, g);
  const uint8_t hls[6] = {0x00, 0x08, 0x20, 0x03, 0x15, 0x00};  // little-endian; speed 21
  TEST_ASSERT_EQUAL_HEX8_ARRAY(hls, g, 6);
  encodeGoal(ServoModel::Scs0009, 512, 800, 300.0f, g);
  const uint8_t scs[6] = {0x02, 0x00, 0x00, 0x00, 0x01, 0x2C};  // big-endian; time forced to 0
  TEST_ASSERT_EQUAL_HEX8_ARRAY(scs, g, 6);
  encodeGoal(ServoModel::Hls3606, -5, 5000, 100.0f, g);  // no sign bit, current capped
  TEST_ASSERT_EQUAL_HEX8(0x00, g[0]);
  TEST_ASSERT_EQUAL_HEX8(0x00, g[1]);
  TEST_ASSERT_EQUAL_HEX16(kHlsMaxGoalCurrent, get16(g + 2, Endian::Little));
  encodeGoal(ServoModel::Scs0009, 2000, 0, 100.0f, g);  // clamped to the SCS range
  TEST_ASSERT_EQUAL(1023, get16(g, Endian::Big));
}

void test_mixed_sync_write_frame() {
  // One SYNC_WRITE carries an SCS entry (big-endian) and an HLS entry (little-endian).
  uint8_t data[12];
  encodeGoal(ServoModel::Scs0009, 512, 0, 300.0f, data);
  encodeGoal(ServoModel::Hls3606, 2048, 800, 1000.0f, data + 6);
  const uint8_t ids[2] = {3, 2};
  uint8_t pkt[64];
  const size_t n = buildSyncWrite(pkt, sizeof(pkt), kRegGoalBlock, kGoalBlockLength, ids, data, 2);
  const uint8_t expected[] = {0xFF, 0xFF, 0xFE, 0x12, 0x83, 0x2A, 0x06,
                              0x03, 0x02, 0x00, 0x00, 0x00, 0x01, 0x2C,
                              0x02, 0x00, 0x08, 0x20, 0x03, 0x15, 0x00};
  TEST_ASSERT_EQUAL(sizeof(expected) + 1, n);
  TEST_ASSERT_EQUAL_HEX8_ARRAY(expected, pkt, sizeof(expected));
  TEST_ASSERT_EQUAL_HEX8(checksum(pkt + 2, n - 3), pkt[n - 1]);
}

int main() {
  UNITY_BEGIN();
  RUN_TEST(test_identify_models);
  RUN_TEST(test_decode_real_hls_status);
  RUN_TEST(test_decode_real_hls_arm_info);
  RUN_TEST(test_decode_scs_status_big_endian_and_signs);
  RUN_TEST(test_hls_negative_position_and_speed_units);
  RUN_TEST(test_speed_register_never_zero_and_capped);
  RUN_TEST(test_encode_goal_blocks);
  RUN_TEST(test_mixed_sync_write_frame);
  return UNITY_END();
}
