#include <feetech_packet.h>
#include <unity.h>

using namespace feetech;

void setUp() {}
void tearDown() {}

void test_ping_frame() {
  uint8_t buf[16];
  const uint8_t expected[] = {0xFF, 0xFF, 0x01, 0x02, 0x01, 0xFB};
  TEST_ASSERT_EQUAL(sizeof(expected), buildPing(buf, sizeof(buf), 1));
  TEST_ASSERT_EQUAL_HEX8_ARRAY(expected, buf, sizeof(expected));
}

void test_read_status_block_frame() {
  // READ id 1, address 40, 27 bytes (status block used by the control loop)
  uint8_t buf[16];
  const uint8_t expected[] = {0xFF, 0xFF, 0x01, 0x04, 0x02, 0x28, 0x1B, 0xB5};
  TEST_ASSERT_EQUAL(sizeof(expected), buildRead(buf, sizeof(buf), 1, 40, 27));
  TEST_ASSERT_EQUAL_HEX8_ARRAY(expected, buf, sizeof(expected));
}

void test_read_rejects_broadcast() {
  uint8_t buf[16];
  TEST_ASSERT_EQUAL(0, buildRead(buf, sizeof(buf), kBroadcastId, 40, 2));
}

void test_write_torque_frame() {
  uint8_t buf[16];
  const uint8_t value = 1;
  const uint8_t expected[] = {0xFF, 0xFF, 0x01, 0x04, 0x03, 0x28, 0x01, 0xCE};
  TEST_ASSERT_EQUAL(sizeof(expected), buildWrite(buf, sizeof(buf), 1, 40, &value, 1));
  TEST_ASSERT_EQUAL_HEX8_ARRAY(expected, buf, sizeof(expected));
}

void test_sync_write_goal_frame() {
  // Goal block (pos 512, time 0, speed 200), big-endian, for ids 1 and 2.
  uint8_t data[12];
  for (int i = 0; i < 2; ++i) {
    put16(data + i * 6 + 0, 512, Endian::Big);
    put16(data + i * 6 + 2, 0, Endian::Big);
    put16(data + i * 6 + 4, 200, Endian::Big);
  }
  const uint8_t ids[] = {1, 2};
  uint8_t buf[64];
  const size_t n = buildSyncWrite(buf, sizeof(buf), 42, 6, ids, data, 2);
  TEST_ASSERT_EQUAL(6 + 2 + 2 * 7, n);
  TEST_ASSERT_EQUAL_HEX8(0xFE, buf[2]);
  TEST_ASSERT_EQUAL_HEX8(2 + 2 * 7 + 2, buf[3]);  // LEN = params + 2
  TEST_ASSERT_EQUAL_HEX8(kInstSyncWrite, buf[4]);
  TEST_ASSERT_EQUAL_HEX8(42, buf[5]);
  TEST_ASSERT_EQUAL_HEX8(6, buf[6]);
  TEST_ASSERT_EQUAL_HEX8(1, buf[7]);
  TEST_ASSERT_EQUAL_HEX8(0x02, buf[8]);  // 512 big-endian high byte
  TEST_ASSERT_EQUAL_HEX8(0x00, buf[9]);
  TEST_ASSERT_EQUAL_HEX8(2, buf[14]);
  TEST_ASSERT_EQUAL_HEX8(checksum(buf + 2, n - 3), buf[n - 1]);
}

void test_sync_write_rejects_bad_id() {
  const uint8_t ids[] = {kBroadcastId};
  const uint8_t data[1] = {0};
  uint8_t buf[32];
  TEST_ASSERT_EQUAL(0, buildSyncWrite(buf, sizeof(buf), 40, 1, ids, data, 1));
}

void test_parse_ping_reply() {
  const uint8_t reply[] = {0xFF, 0xFF, 0x01, 0x02, 0x00, 0xFC};
  StatusPacket pkt;
  TEST_ASSERT_EQUAL(ParseResult::Ok, parseStatus(reply, sizeof(reply), 1, 0, pkt));
  TEST_ASSERT_EQUAL(1, pkt.id);
  TEST_ASSERT_EQUAL(0, pkt.error);
  TEST_ASSERT_EQUAL(0, pkt.paramCount);
}

void test_parse_read_reply() {
  // Model number 1284 = 0x0504, big-endian
  uint8_t reply[] = {0xFF, 0xFF, 0x03, 0x04, 0x00, 0x05, 0x04, 0x00};
  reply[7] = checksum(reply + 2, 5);
  StatusPacket pkt;
  TEST_ASSERT_EQUAL(ParseResult::Ok, parseStatus(reply, sizeof(reply), 3, 2, pkt));
  TEST_ASSERT_EQUAL(1284, get16(pkt.params, Endian::Big));
}

void test_parse_errors() {
  StatusPacket pkt;
  uint8_t reply[] = {0xFF, 0xFF, 0x01, 0x02, 0x00, 0xFC};
  TEST_ASSERT_EQUAL(ParseResult::Incomplete, parseStatus(reply, 5, 1, 0, pkt));
  TEST_ASSERT_EQUAL(ParseResult::WrongId, parseStatus(reply, sizeof(reply), 2, 0, pkt));
  TEST_ASSERT_EQUAL(ParseResult::BadLength, parseStatus(reply, sizeof(reply), 1, 2, pkt));
  reply[5] ^= 0x01;
  TEST_ASSERT_EQUAL(ParseResult::BadChecksum, parseStatus(reply, sizeof(reply), 1, 0, pkt));
  reply[0] = 0x00;
  TEST_ASSERT_EQUAL(ParseResult::BadHeader, parseStatus(reply, sizeof(reply), 1, 0, pkt));
}

void test_endian_and_sign_magnitude() {
  uint8_t b[2];
  put16(b, 0x1234, Endian::Little);
  TEST_ASSERT_EQUAL_HEX8(0x34, b[0]);
  TEST_ASSERT_EQUAL_HEX16(0x1234, get16(b, Endian::Little));
  put16(b, 0x1234, Endian::Big);
  TEST_ASSERT_EQUAL_HEX8(0x12, b[0]);
  TEST_ASSERT_EQUAL_HEX16(0x1234, get16(b, Endian::Big));

  TEST_ASSERT_EQUAL(300, signMagnitude(300, 10));
  TEST_ASSERT_EQUAL(-300, signMagnitude(300 | (1 << 10), 10));  // load, bit 10
  TEST_ASSERT_EQUAL(-150, signMagnitude(150 | 0x8000, 15));     // speed, bit 15
}

int main() {
  UNITY_BEGIN();
  RUN_TEST(test_ping_frame);
  RUN_TEST(test_read_status_block_frame);
  RUN_TEST(test_read_rejects_broadcast);
  RUN_TEST(test_write_torque_frame);
  RUN_TEST(test_sync_write_goal_frame);
  RUN_TEST(test_sync_write_rejects_bad_id);
  RUN_TEST(test_parse_ping_reply);
  RUN_TEST(test_parse_read_reply);
  RUN_TEST(test_parse_errors);
  RUN_TEST(test_endian_and_sign_magnitude);
  return UNITY_END();
}
