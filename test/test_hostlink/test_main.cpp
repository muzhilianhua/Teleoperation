#include <byte_buffer.h>
#include <frame.h>
#include <string.h>
#include <unity.h>

#include <vector>

using namespace hostlink;

void setUp() {}
void tearDown() {}

namespace {

std::vector<uint8_t> encoded(const std::vector<uint8_t>& in) {
  std::vector<uint8_t> out(cobsMaxEncoded(in.size()));
  const size_t n = cobsEncode(in.data(), in.size(), out.data(), out.size());
  out.resize(n);
  return out;
}

void assertCobs(const std::vector<uint8_t>& raw, const std::vector<uint8_t>& wire) {
  const std::vector<uint8_t> out = encoded(raw);
  TEST_ASSERT_EQUAL(wire.size(), out.size());
  TEST_ASSERT_EQUAL_HEX8_ARRAY(wire.data(), out.data(), wire.size());
  std::vector<uint8_t> back(raw.size() + 1);
  size_t n = 0;
  TEST_ASSERT_TRUE(cobsDecode(wire.data(), wire.size(), back.data(), back.size(), n));
  TEST_ASSERT_EQUAL(raw.size(), n);
  if (n > 0) TEST_ASSERT_EQUAL_HEX8_ARRAY(raw.data(), back.data(), n);
}

std::vector<uint8_t> range(uint8_t first, size_t count) {
  std::vector<uint8_t> v(count);
  for (size_t i = 0; i < count; ++i) v[i] = static_cast<uint8_t>(first + i);
  return v;
}

}  // namespace

// ---- CRC / COBS ---------------------------------------------------------------

void test_crc16_check_value() {
  const char* check = "123456789";
  TEST_ASSERT_EQUAL_HEX16(0x29B1, crc16(reinterpret_cast<const uint8_t*>(check), 9));
  // Continuing over two segments gives the same result.
  const uint16_t part = crc16(reinterpret_cast<const uint8_t*>(check), 4);
  TEST_ASSERT_EQUAL_HEX16(0x29B1, crc16(reinterpret_cast<const uint8_t*>(check) + 4, 5, part));
}

// Vectors from the COBS paper / Wikipedia (delimiter not included).
void test_cobs_reference_vectors() {
  assertCobs({}, {0x01});
  assertCobs({0x00}, {0x01, 0x01});
  assertCobs({0x00, 0x00}, {0x01, 0x01, 0x01});
  assertCobs({0x00, 0x11, 0x00}, {0x01, 0x02, 0x11, 0x01});
  assertCobs({0x11, 0x22, 0x00, 0x33}, {0x03, 0x11, 0x22, 0x02, 0x33});
  assertCobs({0x11, 0x22, 0x33, 0x44}, {0x05, 0x11, 0x22, 0x33, 0x44});
  assertCobs({0x11, 0x00, 0x00, 0x00}, {0x02, 0x11, 0x01, 0x01, 0x01});
}

void test_cobs_long_blocks() {
  // 254 non-zero bytes: one full block, no trailing code byte.
  std::vector<uint8_t> wire = {0xFF};
  const std::vector<uint8_t> run = range(0x01, 254);
  wire.insert(wire.end(), run.begin(), run.end());
  assertCobs(run, wire);

  // 255 non-zero bytes: a full block, then a block with one byte.
  std::vector<uint8_t> raw = range(0x01, 255);
  raw[254] = 0xFF;
  wire.push_back(0x02);
  wire.push_back(0xFF);
  assertCobs(raw, wire);

  // A zero right after a full block.
  std::vector<uint8_t> withZero = run;
  withZero.push_back(0x00);
  std::vector<uint8_t> wire2 = {0xFF};
  wire2.insert(wire2.end(), run.begin(), run.end());
  wire2.push_back(0x01);
  wire2.push_back(0x01);
  assertCobs(withZero, wire2);
}

void test_cobs_rejects_malformed() {
  uint8_t out[8];
  size_t n = 0;
  const uint8_t zero[] = {0x02, 0x00};
  TEST_ASSERT_FALSE(cobsDecode(zero, sizeof(zero), out, sizeof(out), n));
  const uint8_t truncated[] = {0x05, 0x11, 0x22};
  TEST_ASSERT_FALSE(cobsDecode(truncated, sizeof(truncated), out, sizeof(out), n));
  const uint8_t big[] = {0x05, 0x11, 0x22, 0x33, 0x44};
  TEST_ASSERT_FALSE(cobsDecode(big, sizeof(big), out, 3, n));  // out too small
}

void test_cobs_encoder_reports_overflow() {
  uint8_t out[4];
  const uint8_t in[] = {1, 2, 3, 4, 5};
  TEST_ASSERT_EQUAL(0, cobsEncode(in, sizeof(in), out, sizeof(out)));
}

// ---- Frames -------------------------------------------------------------------

// Golden frame shared with the Python host tests (host/tests/vectors.py):
// version 1, type 0x03 (COMMAND), seq 0x1234, flags 1, payload "arm".
void test_frame_golden_vector() {
  FrameHeader h;
  h.type = 0x03;
  h.seq = 0x1234;
  h.flags = 0x01;
  const uint8_t payload[] = {'a', 'r', 'm'};
  uint8_t out[32];
  const size_t n = encodeFrame(h, payload, sizeof(payload), out, sizeof(out));
  const uint8_t body[] = {0x01, 0x03, 0x34, 0x12, 0x01, 'a', 'r', 'm'};
  const uint16_t crc = crc16(body, sizeof(body));
  TEST_ASSERT_EQUAL_HEX16(0xCE44, crc);
  const uint8_t expected[] = {0x0B, 0x01, 0x03, 0x34, 0x12, 0x01, 'a', 'r', 'm', 0x44, 0xCE, 0x00};
  TEST_ASSERT_EQUAL(sizeof(expected), n);
  TEST_ASSERT_EQUAL_HEX8_ARRAY(expected, out, n);
}

void test_frame_round_trip_with_zeros() {
  FrameHeader h;
  h.type = 0x82;
  h.seq = 0x0100;  // a zero byte inside the header
  uint8_t payload[600];
  for (size_t i = 0; i < sizeof(payload); ++i) payload[i] = static_cast<uint8_t>(i % 7 == 0 ? 0 : i);
  uint8_t wire[maxWireSize(sizeof(payload))];
  const size_t n = encodeFrame(h, payload, sizeof(payload), wire, sizeof(wire));
  TEST_ASSERT_GREATER_THAN(0, n);
  for (size_t i = 0; i + 1 < n; ++i) TEST_ASSERT_NOT_EQUAL(0, wire[i]);
  TEST_ASSERT_EQUAL(0, wire[n - 1]);

  FrameReader reader;
  FrameReader::Result last = FrameReader::Result::None;
  for (size_t i = 0; i < n; ++i) last = reader.feed(wire[i]);
  TEST_ASSERT_EQUAL(FrameReader::Result::Frame, last);
  TEST_ASSERT_EQUAL(0x82, reader.header().type);
  TEST_ASSERT_EQUAL_HEX16(0x0100, reader.header().seq);
  TEST_ASSERT_EQUAL(sizeof(payload), reader.payloadLength());
  TEST_ASSERT_EQUAL_HEX8_ARRAY(payload, reader.payload(), sizeof(payload));
}

void test_reader_resyncs_after_text_and_bad_crc() {
  FrameHeader h;
  h.type = 0x02;
  uint8_t wire[16];
  const size_t n = encodeFrame(h, nullptr, 0, wire, sizeof(wire));
  FrameReader reader;
  // Text typed before the host started framing, then a corrupted frame.
  const char* text = "host on\n";
  for (const char* p = text; *p; ++p) reader.feed(static_cast<uint8_t>(*p));
  uint8_t bad[16];
  memcpy(bad, wire, n);
  bad[2] ^= 0x40;
  int frames = 0;
  for (size_t i = 0; i < n; ++i) frames += reader.feed(bad[i]) == FrameReader::Result::Frame;
  for (size_t i = 0; i < n; ++i) frames += reader.feed(wire[i]) == FrameReader::Result::Frame;
  TEST_ASSERT_EQUAL(1, frames);
  // The text has no delimiter of its own: it and the corrupted frame form one bad chunk.
  TEST_ASSERT_EQUAL(1, reader.stats().malformed + reader.stats().crcErrors);
  TEST_ASSERT_EQUAL(1, reader.stats().frames);
}

void test_reader_drops_oversized_frames() {
  FrameReader reader;
  for (size_t i = 0; i < kMaxFrame + 64; ++i) reader.feed(0x11);
  TEST_ASSERT_EQUAL(1, reader.stats().overflows);
  TEST_ASSERT_EQUAL(FrameReader::Result::None, reader.feed(0x00));  // end of the dropped frame
  FrameHeader h;
  uint8_t wire[16];
  const size_t n = encodeFrame(h, nullptr, 0, wire, sizeof(wire));
  FrameReader::Result last = FrameReader::Result::None;
  for (size_t i = 0; i < n; ++i) last = reader.feed(wire[i]);
  TEST_ASSERT_EQUAL(FrameReader::Result::Frame, last);
}

void test_encode_frame_limits() {
  uint8_t out[8];
  FrameHeader h;
  TEST_ASSERT_EQUAL(0, encodeFrame(h, nullptr, kMaxPayload + 1, out, sizeof(out)));
  const uint8_t payload[16] = {1};
  TEST_ASSERT_EQUAL(0, encodeFrame(h, payload, sizeof(payload), out, sizeof(out)));
}

// ---- Byte buffers -------------------------------------------------------------

void test_byte_writer_reader_round_trip() {
  uint8_t buf[64];
  ByteWriter w(buf, sizeof(buf));
  w.u8(0xAB);
  w.i16(-2);
  w.u32(0xDEADBEEF);
  w.f32(1.5f);
  w.str8("id2");
  w.text("tail");
  TEST_ASSERT_TRUE(w.ok());
  ByteReader r(buf, w.length());
  TEST_ASSERT_EQUAL_HEX8(0xAB, r.u8());
  TEST_ASSERT_EQUAL(-2, r.i16());
  TEST_ASSERT_EQUAL_HEX32(0xDEADBEEF, r.u32());
  TEST_ASSERT_EQUAL_FLOAT(1.5f, r.f32());
  TEST_ASSERT_EQUAL(3, r.u8());
  char name[4] = {static_cast<char>(r.u8()), static_cast<char>(r.u8()), static_cast<char>(r.u8()), 0};
  TEST_ASSERT_EQUAL_STRING("id2", name);
  char tail[16];
  r.text(tail, sizeof(tail));
  TEST_ASSERT_EQUAL_STRING("tail", tail);
  TEST_ASSERT_TRUE(r.ok());
  r.u8();
  TEST_ASSERT_FALSE(r.ok());  // past the end
}

void test_byte_writer_stops_at_capacity() {
  uint8_t buf[3];
  ByteWriter w(buf, sizeof(buf));
  w.u16(1);
  w.u16(2);
  TEST_ASSERT_FALSE(w.ok());
  TEST_ASSERT_EQUAL(2, w.length());
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_crc16_check_value);
  RUN_TEST(test_cobs_reference_vectors);
  RUN_TEST(test_cobs_long_blocks);
  RUN_TEST(test_cobs_rejects_malformed);
  RUN_TEST(test_cobs_encoder_reports_overflow);
  RUN_TEST(test_frame_golden_vector);
  RUN_TEST(test_frame_round_trip_with_zeros);
  RUN_TEST(test_reader_resyncs_after_text_and_bad_crc);
  RUN_TEST(test_reader_drops_oversized_frames);
  RUN_TEST(test_encode_frame_limits);
  RUN_TEST(test_byte_writer_reader_round_trip);
  RUN_TEST(test_byte_writer_stops_at_capacity);
  return UNITY_END();
}
