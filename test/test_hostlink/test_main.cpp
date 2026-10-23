#include <byte_buffer.h>
#include <frame.h>
#include <host_messages.h>
#include <string.h>
#include <unity.h>

#include <string>
#include <vector>

#include "golden_vectors.h"

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

std::vector<uint8_t> fromHex(const char* hex) {
  std::vector<uint8_t> out;
  for (const char* p = hex; p[0] && p[1]; p += 2) out.push_back(static_cast<uint8_t>(std::stoi(std::string(p, 2), nullptr, 16)));
  return out;
}

void assertBytes(const char* hex, const uint8_t* bytes, size_t length) {
  const std::vector<uint8_t> expected = fromHex(hex);
  TEST_ASSERT_EQUAL(expected.size(), length);
  TEST_ASSERT_EQUAL_HEX8_ARRAY(expected.data(), bytes, length);
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


// ---- Message layouts (golden vectors shared with the host) --------------------

namespace {

hostlink::Telemetry sampleTelemetry() {
  Telemetry t;
  t.cycle = 123456;
  t.tUs = 0x89ABCDEF;
  t.uptimeMs = 1234567;
  t.armedCount = 2;
  t.handFlags = kHandTorqueOffConfirmed;
  t.execUs = 850;
  t.jointCount = 2;
  TelemetryJoint& a = t.joints[0];
  a.state = 2;
  a.flags = 0x2F23;
  a.encRaw = 8191;
  a.exoCounts = -1234.5f;
  a.exoFilteredCounts = -1230.25f;
  a.target = 512.5f;
  a.goal = 510.0f;
  a.sentGoal = 508.75f;
  a.reference = 505.5f;
  a.speedCommand = 1300;
  a.gripLead = 60;
  a.position = 507;
  a.speed = -120;
  a.loadDeciPct = -355;
  a.voltageDeciV = 60;
  a.temperatureC = 35;
  a.readAgeMs = 12;
  TelemetryJoint& b = t.joints[1];
  b.fault = 10;
  b.flags = 0x1200;
  b.position = 2048;
  b.currentMa = -2700;
  b.voltageDeciV = 74;
  b.temperatureC = -5;
  b.status = 8;
  b.readAgeMs = 65535;
  return t;
}

hostlink::SlowStatus sampleSlowStatus() {
  SlowStatus s;
  s.minCutoffHz = 1.0f;
  s.beta = 0.0078125f;
  s.jumpDeg = 25.0f;
  s.gripLoadPct = 60.0f;
  s.cycles = 1000;
  s.overruns = 1;
  s.lastPeriodUs = 10000;
  s.maxJitterUs = 12;
  s.p99JitterUs = 8;
  s.lastExecUs = 900;
  s.maxExecUs = 1800;
  s.averageExecUs = 850;
  s.busSent = 500;
  s.busReceived = 498;
  s.busTimeouts = 2;
  s.busLastReplyUs = 180;
  s.busMaxReplyUs = 950;
  s.busEcho = 1;
  s.encoderAverageUs = 1500;
  s.encoderMaxUs = 1700;
  s.encoderMaxSkewUs = 40;
  s.droppedEvents = 3;
  s.rxCrcErrors = 1;
  s.rxMalformed = 2;
  s.monitor = 1;
  s.monitorOkMask = 0x0001;
  s.jointCount = 1;
  s.monitorPosition[0] = 512;
  SlowJoint& j = s.joints[0];
  j.enabled = 1;
  j.encoderIndex = 2;
  j.exoRef = 1000;
  j.exoB = -8192;
  j.servoA = 794;
  j.servoB = 1900;
  j.vmax = 1333;
  j.amax = 26666;
  j.vsat = 167;
  j.trackSteps = 133;
  j.gripSteps = 200;
  j.currentLimitMa = 500;
  j.currentTripPct = 90;
  j.detectedModel = 2;
  j.effectiveCurrentMa = 494;
  j.jumpRejects = 7;
  s.encoderCount = 1;
  SlowEncoder& e = s.encoders[0];
  e.address = 0x40;
  e.valid = 1;
  e.angle = 4095;
  e.agc = 88;
  e.magnetOk = 1;
  e.errors = 3;
  return s;
}

}  // namespace

void test_hello_golden_vector() {
  static const char* const faults[] = {"none", "comm lost"};
  static const char* const states[] = {"OFF", "ALIGN"};
  static const char* const events[] = {"generic"};
  static const char* const models[] = {"unknown", "SCS0009", "HLS3606"};
  JointInfo joint;
  joint.servoId = 2;
  joint.model = 2;
  joint.travelMin = 794;
  joint.travelMax = 1911;
  joint.vmax = 1333;
  joint.vsat = 167;
  joint.amax = 26666;
  joint.maxSpeed = 12492;
  joint.encoderDefault = 2;
  joint.stepScale = 3.25f;
  joint.degreesPerStep = 0.087890625f;
  joint.name = "thumb_near";
  DeviceInfo info;
  info.firmware = "abc1234";
  info.degreesPerCount = 360.0f / 16384.0f;
  info.encoderCount = 9;
  info.minCurrentLimitMa = 7;
  info.maxCurrentLimitMa = 13306;
  info.minCurrentTripPct = 50;
  info.maxCurrentTripPct = 100;
  info.defaultCurrentTripPct = 90;
  info.minExoSpanCounts = 182;
  info.maxExoSpanCounts = 49152;
  info.joints = &joint;
  info.jointCount = 1;
  info.faults = {faults, 2};
  info.states = {states, 2};
  info.events = {events, 1};
  info.models = {models, 3};
  uint8_t out[512];
  const size_t n = encodeHello(info, out, sizeof(out));
  assertBytes(kGoldenHelloHex, out, n);
  TEST_ASSERT_EQUAL(0, encodeHello(info, out, 20));  // too small
}

void test_telemetry_golden_vector_and_round_trip() {
  const Telemetry t = sampleTelemetry();
  uint8_t out[telemetrySize(kMaxJoints)];
  const size_t n = encodeTelemetry(t, out, sizeof(out));
  TEST_ASSERT_EQUAL(telemetrySize(2), n);
  assertBytes(kGoldenTelemetryHex, out, n);
  Telemetry back;
  TEST_ASSERT_TRUE(decodeTelemetry(out, n, back));
  TEST_ASSERT_EQUAL(0x89ABCDEF, back.tUs);
  TEST_ASSERT_EQUAL_FLOAT(-1230.25f, back.joints[0].exoFilteredCounts);
  TEST_ASSERT_EQUAL(-2700, back.joints[1].currentMa);
  TEST_ASSERT_EQUAL(-5, back.joints[1].temperatureC);
  TEST_ASSERT_FALSE(decodeTelemetry(out, n - 1, back));  // truncated
}

void test_full_hand_telemetry_size() {
  Telemetry t;
  t.jointCount = kMaxJoints;
  uint8_t out[512];
  TEST_ASSERT_EQUAL(450, encodeTelemetry(t, out, sizeof(out)));
  TEST_ASSERT_LESS_OR_EQUAL(470, maxWireSize(450));
}

void test_slow_status_golden_vector_and_round_trip() {
  const SlowStatus s = sampleSlowStatus();
  uint8_t out[1024];
  const size_t n = encodeSlowStatus(s, out, sizeof(out));
  assertBytes(kGoldenSlowStatusHex, out, n);
  SlowStatus back;
  TEST_ASSERT_TRUE(decodeSlowStatus(out, n, back));
  TEST_ASSERT_EQUAL(-8192, back.joints[0].exoB);
  TEST_ASSERT_EQUAL(494, back.joints[0].effectiveCurrentMa);
  TEST_ASSERT_EQUAL_HEX8(0x40, back.encoders[0].address);
  TEST_ASSERT_EQUAL(3, back.encoders[0].errors);
}

void test_full_hand_slow_status_size() {
  SlowStatus s;
  s.jointCount = kMaxJoints;
  s.encoderCount = kMaxEncoders;
  uint8_t out[1024];
  // 114 B fixed (21 global, 32 timing, 29 bus, 12 encoder pass, 16 link, 4 monitor)
  // + 9 x (2 + 32) monitor positions and joints + 1 + 9 x 11 encoders = 520 B
  TEST_ASSERT_EQUAL(114 + 9 * 34 + 1 + 9 * 11, encodeSlowStatus(s, out, sizeof(out)));
}

void test_event_golden_vector_and_round_trip() {
  Event e;
  e.code = EventCode::SweepDone;
  e.joint = 4;
  e.cmdSeq = 77;
  e.args[0] = 1333;
  e.args[1] = 167;
  e.args[2] = 1;
  strcpy(e.text, "sweep j4: done");
  uint8_t out[256];
  const size_t n = encodeEvent(e, out, sizeof(out));
  assertBytes(kGoldenEventHex, out, n);
  Event back;
  TEST_ASSERT_TRUE(decodeEvent(out, n, back));
  TEST_ASSERT_EQUAL(EventCode::SweepDone, back.code);
  TEST_ASSERT_EQUAL(77, back.cmdSeq);
  TEST_ASSERT_EQUAL(167, back.args[1]);
  TEST_ASSERT_EQUAL_STRING("sweep j4: done", back.text);
}

void test_replies_and_host_messages() {
  uint8_t out[64];
  const uint8_t ack[] = {0x07, 0x00, 0x01, 'b', 'a', 'd'};
  size_t n = encodeAck(7, AckStatus::ParseError, "bad", out, sizeof(out));
  TEST_ASSERT_EQUAL(sizeof(ack), n);
  TEST_ASSERT_EQUAL_HEX8_ARRAY(ack, out, n);
  n = encodeDone(0x0102, false, out, sizeof(out));
  const uint8_t done[] = {0x02, 0x01, 0x01};
  TEST_ASSERT_EQUAL_HEX8_ARRAY(done, out, n);
  n = encodeLog(LogSource::EspLog, "W x", 3, out, sizeof(out));
  const uint8_t log[] = {0x01, 'W', ' ', 'x'};
  TEST_ASSERT_EQUAL_HEX8_ARRAY(log, out, n);

  const uint8_t hello[] = {0x01, 0x01, 0xE8, 0x03, 0x02, 0x0A};
  HostHello h;
  TEST_ASSERT_TRUE(decodeHostHello(hello, sizeof(hello), h));
  TEST_ASSERT_EQUAL(1000, h.heartbeatTimeoutMs);
  TEST_ASSERT_EQUAL(2, h.telemetryDivider);
  TEST_ASSERT_FALSE(decodeHostHello(hello, 4, h));
  TelemConfig c;
  const uint8_t config[] = {0x05, 0x00};
  TEST_ASSERT_TRUE(decodeTelemConfig(config, sizeof(config), c));
  TEST_ASSERT_EQUAL(5, c.telemetryDivider);
  TEST_ASSERT_EQUAL(0, c.slowDivider);
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
  RUN_TEST(test_hello_golden_vector);
  RUN_TEST(test_telemetry_golden_vector_and_round_trip);
  RUN_TEST(test_full_hand_telemetry_size);
  RUN_TEST(test_slow_status_golden_vector_and_round_trip);
  RUN_TEST(test_full_hand_slow_status_size);
  RUN_TEST(test_event_golden_vector_and_round_trip);
  RUN_TEST(test_replies_and_host_messages);
  return UNITY_END();
}
