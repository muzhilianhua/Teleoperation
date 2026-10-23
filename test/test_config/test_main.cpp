#include <command_parser.h>
#include <config_blob.h>
#include <joint_config.h>
#include <string.h>
#include <unity.h>

using namespace teleop;

namespace {

constexpr uint8_t kEncoders = 9;

constexpr feetech::ServoModel kScs = feetech::ServoModel::Scs0009;
constexpr feetech::ServoModel kHls = feetech::ServoModel::Hls3606;

const JointDefaults kDefaults[kMaxJoints] = {
    {1, "triple_pip", kScs, 350, 730, 660, 245, 8000, 5, true},
    {2, "triple_dip", kHls, 600, 890, 660, 370, 8000, kNoEncoder, false},
    {3, "index_pip", kScs, 250, 600, 400, 50, 8000, kNoEncoder, false},
    {4, "index_dip", kScs, 250, 590, 400, 50, 8000, kNoEncoder, false},
    {5, "index_mcp", kScs, 435, 600, 400, 50, 8000, kNoEncoder, false},
    {6, "thumb_mcp_rot", kHls, 300, 2700, 1333, 167, 26666, kNoEncoder, false},
    {7, "thumb_mcp", kScs, 650, 900, 400, 50, 8000, kNoEncoder, false},
    {8, "thumb_ip", kScs, 20, 280, 380, 230, 8000, kNoEncoder, false},
    {9, "joint9", kScs, 0, 0, 400, 50, 8000, kNoEncoder, false},  // travel not measured yet
};

void loadDefaults(JointSettings* joints) {
  for (uint8_t i = 0; i < kMaxJoints; ++i) joints[i] = defaultSettings(kDefaults[i]);
}

void configureSample(JointSettings* joints, GlobalSettings& g) {
  loadDefaults(joints);
  joints[0].exoRef = 16200;
  joints[0].exoB = 2100;  // crosses the encoder zero
  joints[0].servoA = 700;
  joints[0].servoB = 380;
  joints[1].enabled = true;
  joints[1].encoderIndex = 3;
  joints[1].exoRef = 4000;
  joints[1].exoB = -1500;
  joints[1].servoA = 610;
  joints[1].servoB = 880;
  joints[1].vmax = 500;
  joints[1].trackSteps = 60;
  joints[1].gripSteps = 80;
  g.leadMs = 80.0f;
  g.beta = 0.025f;
  g.jumpDeg = 30.0f;
  g.gripLoadPct = 70.0f;
  g.loadTripEnabled = true;
}

void assertJointsEqual(const JointSettings& a, const JointSettings& b) {
  TEST_ASSERT_EQUAL(a.enabled, b.enabled);
  TEST_ASSERT_EQUAL(a.encoderIndex, b.encoderIndex);
  TEST_ASSERT_EQUAL(a.exoB, b.exoB);
  if (a.exoB != 0) TEST_ASSERT_EQUAL(a.exoRef, b.exoRef);
  TEST_ASSERT_EQUAL(a.servoA, b.servoA);
  TEST_ASSERT_EQUAL(a.servoB, b.servoB);
  TEST_ASSERT_EQUAL(a.vmax, b.vmax);
  TEST_ASSERT_EQUAL(a.amax, b.amax);
  TEST_ASSERT_EQUAL(a.vsat, b.vsat);
  TEST_ASSERT_EQUAL(a.trackSteps, b.trackSteps);
  TEST_ASSERT_EQUAL(a.gripSteps, b.gripSteps);
  TEST_ASSERT_EQUAL(a.currentLimitMa, b.currentLimitMa);
}

}  // namespace

void setUp() {}
void tearDown() {}

// ---- joint config ------------------------------------------------------------

void test_defaults_are_valid_and_not_ready() {
  for (uint8_t i = 0; i < kMaxJoints; ++i) {
    const JointSettings s = defaultSettings(kDefaults[i]);
    TEST_ASSERT_NULL(validateJoint(s, kDefaults[i], kEncoders));
    TEST_ASSERT_FALSE(jointReady(s, kDefaults[i], kEncoders));
  }
}

void test_ready_needs_everything() {
  JointSettings joints[kMaxJoints];
  GlobalSettings g;
  configureSample(joints, g);
  TEST_ASSERT_TRUE(jointReady(joints[0], kDefaults[0], kEncoders));
  JointSettings s = joints[0];
  s.enabled = false;
  TEST_ASSERT_FALSE(jointReady(s, kDefaults[0], kEncoders));
  s = joints[0];
  s.encoderIndex = kNoEncoder;
  TEST_ASSERT_FALSE(jointReady(s, kDefaults[0], kEncoders));
}

void test_validate_rejections() {
  const JointDefaults& d = kDefaults[0];
  JointSettings s = defaultSettings(d);
  s.encoderIndex = 9;
  TEST_ASSERT_NOT_NULL(validateJoint(s, d, kEncoders));
  s = defaultSettings(d);
  s.exoB = 100;  // < 4 deg
  TEST_ASSERT_NOT_NULL(validateJoint(s, d, kEncoders));
  s = defaultSettings(d);
  s.servoA = 340;  // below travel
  s.servoB = 700;
  TEST_ASSERT_NOT_NULL(validateJoint(s, d, kEncoders));
  s = defaultSettings(d);
  s.vmax = 0;
  TEST_ASSERT_NOT_NULL(validateJoint(s, d, kEncoders));
  s = defaultSettings(d);
  s.amax = 30000;
  TEST_ASSERT_NOT_NULL(validateJoint(s, d, kEncoders));
  GlobalSettings g;
  g.jumpDeg = 0.5f;
  TEST_ASSERT_NOT_NULL(validateGlobal(g));
}

void test_exo_raw_b_wraps() {
  JointSettings s;
  s.exoRef = 16200;
  s.exoB = 2100;
  TEST_ASSERT_EQUAL(1916, exoRawB(s));
  s.exoRef = 100;
  s.exoB = -300;
  TEST_ASSERT_EQUAL(16184, exoRawB(s));
}

// ---- blob --------------------------------------------------------------------

void test_blob_round_trip() {
  JointSettings joints[kMaxJoints];
  GlobalSettings g;
  configureSample(joints, g);
  uint8_t blob[kBlobSize];
  TEST_ASSERT_EQUAL(kBlobSize, packConfig(joints, kDefaults, kMaxJoints, g, blob, sizeof(blob)));
  TEST_ASSERT_EQUAL(258, kBlobSize);

  JointSettings loaded[kMaxJoints];
  GlobalSettings lg;
  TEST_ASSERT_EQUAL(BlobStatus::Ok, unpackConfig(blob, sizeof(blob), kDefaults,
                                                 kMaxJoints, kEncoders, GlobalSettings(), loaded, lg));
  for (uint8_t i = 0; i < kMaxJoints; ++i) assertJointsEqual(joints[i], loaded[i]);
  TEST_ASSERT_EQUAL_FLOAT(80.0f, lg.leadMs);
  TEST_ASSERT_EQUAL_FLOAT(0.025f, lg.beta);
  TEST_ASSERT_EQUAL_FLOAT(30.0f, lg.jumpDeg);
  TEST_ASSERT_EQUAL_FLOAT(70.0f, lg.gripLoadPct);
  TEST_ASSERT_TRUE(lg.loadTripEnabled);
}

void test_blob_rejects_corruption_without_touching_output() {
  JointSettings joints[kMaxJoints];
  GlobalSettings g;
  configureSample(joints, g);
  uint8_t blob[kBlobSize];
  packConfig(joints, kDefaults, kMaxJoints, g, blob, sizeof(blob));

  JointSettings out[kMaxJoints];
  loadDefaults(out);
  GlobalSettings og;
  uint8_t bad[kBlobSize];

  memcpy(bad, blob, sizeof(bad));
  bad[20] ^= 0x01;
  TEST_ASSERT_EQUAL(BlobStatus::BadCrc, unpackConfig(bad, sizeof(bad), kDefaults,
                                                     kMaxJoints, kEncoders, GlobalSettings(), out, og));
  TEST_ASSERT_EQUAL(0, out[0].exoB);  // untouched

  TEST_ASSERT_EQUAL(BlobStatus::BadSize, unpackConfig(blob, 100, kDefaults,
                                                      kMaxJoints, kEncoders, GlobalSettings(), out, og));
  memcpy(bad, blob, sizeof(bad));
  bad[0] = 'X';
  TEST_ASSERT_EQUAL(BlobStatus::BadMagic, unpackConfig(bad, sizeof(bad), kDefaults,
                                                       kMaxJoints, kEncoders, GlobalSettings(), out, og));
  memcpy(bad, blob, sizeof(bad));
  bad[4] = 9;  // version
  TEST_ASSERT_EQUAL(BlobStatus::BadVersion, unpackConfig(bad, sizeof(bad), kDefaults,
                                                         kMaxJoints, kEncoders, GlobalSettings(), out, og));
}

void test_blob_invalid_joint_reset_to_defaults() {
  JointSettings joints[kMaxJoints];
  GlobalSettings g;
  configureSample(joints, g);
  joints[1].servoA = 100;  // outside travel: packed anyway to simulate an old/broken record
  uint8_t blob[kBlobSize];
  packConfig(joints, kDefaults, kMaxJoints, g, blob, sizeof(blob));
  JointSettings out[kMaxJoints];
  GlobalSettings og;
  uint8_t bad = 0xFF;
  TEST_ASSERT_EQUAL(BlobStatus::InvalidJoint,
                    unpackConfig(blob, sizeof(blob), kDefaults, kMaxJoints,
                                 kEncoders, GlobalSettings(), out, og, &bad));
  TEST_ASSERT_EQUAL(1, bad);
  assertJointsEqual(defaultSettings(kDefaults[1]), out[1]);
  assertJointsEqual(joints[0], out[0]);
}

void test_validate_span_limits() {
  JointSettings s = defaultSettings(kDefaults[0]);
  s.exoB = 3 * 16384;
  TEST_ASSERT_NULL(validateJoint(s, kDefaults[0], kEncoders));
  s.exoB = 3 * 16384 + 1;
  TEST_ASSERT_NOT_NULL(validateJoint(s, kDefaults[0], kEncoders));
  s.exoB = -30000;
  TEST_ASSERT_NULL(validateJoint(s, kDefaults[0], kEncoders));
}

void test_exospan_and_legacy_exo_syntax() {
  JointSettings joints[kMaxJoints];
  loadDefaults(joints);
  GlobalSettings g;
  ParsedCommand c;
  TEST_ASSERT_NULL(parseCommand("set 1 exospan 5000 -24576", c));
  TEST_ASSERT_EQUAL(SetKey::ExoSpan, c.key);
  TEST_ASSERT_TRUE(requiresDisarmed(c));
  TEST_ASSERT_NULL(applySetting(c, joints, kDefaults, kMaxJoints, g, kEncoders));
  TEST_ASSERT_EQUAL(5000, joints[0].exoRef);
  TEST_ASSERT_EQUAL(-24576, joints[0].exoB);
  TEST_ASSERT_NOT_NULL(parseCommand("set 1 exospan 5000 60000", c));  // > 3 turns
  // Old dumps (`exo rawA rawB`) still load for short spans.
  TEST_ASSERT_NULL(parseCommand("set 1 exo 16200 1916", c));
  TEST_ASSERT_NULL(applySetting(c, joints, kDefaults, kMaxJoints, g, kEncoders));
  TEST_ASSERT_EQUAL(2100, joints[0].exoB);
}

void test_blob_older_versions_superseded() {
  JointSettings joints[kMaxJoints];
  GlobalSettings g;
  configureSample(joints, g);
  uint8_t blob[kBlobSize];
  packConfig(joints, kDefaults, kMaxJoints, g, blob, sizeof(blob));
  JointSettings out[kMaxJoints];
  loadDefaults(out);
  GlobalSettings og;
  for (uint8_t version = 1; version <= 3; ++version) {
    blob[4] = version;  // the old hand's settings, whatever their size
    TEST_ASSERT_EQUAL(BlobStatus::Superseded,
                      unpackConfig(blob, version == 3 ? 231 : 172, kDefaults, kMaxJoints, kEncoders,
                                   GlobalSettings(), out, og));
  }
  TEST_ASSERT_FALSE(blobApplied(BlobStatus::Superseded));
  TEST_ASSERT_EQUAL(0, out[0].exoB);  // untouched
}

void test_blob_keeps_multi_turn_span_and_current_limit() {
  JointSettings joints[kMaxJoints];
  GlobalSettings g;
  configureSample(joints, g);
  joints[0].exoB = 24576;  // 1.5 encoder turns
  joints[5].currentLimitMa = 500;
  uint8_t blob[kBlobSize];
  packConfig(joints, kDefaults, kMaxJoints, g, blob, sizeof(blob));
  JointSettings out[kMaxJoints];
  GlobalSettings og;
  TEST_ASSERT_EQUAL(BlobStatus::Ok, unpackConfig(blob, sizeof(blob), kDefaults, kMaxJoints, kEncoders,
                                                 GlobalSettings(), out, og));
  TEST_ASSERT_EQUAL(24576, out[0].exoB);
  TEST_ASSERT_EQUAL(500, out[5].currentLimitMa);
}

void test_blob_changed_model_resets_that_joint() {
  JointSettings joints[kMaxJoints];
  GlobalSettings g;
  configureSample(joints, g);
  uint8_t blob[kBlobSize];
  packConfig(joints, kDefaults, kMaxJoints, g, blob, sizeof(blob));
  JointDefaults changed[kMaxJoints];
  for (uint8_t i = 0; i < kMaxJoints; ++i) changed[i] = kDefaults[i];
  changed[0].model = kHls;  // joint 1 rebuilt with an HLS3606
  JointSettings out[kMaxJoints];
  GlobalSettings og;
  uint8_t bad = 0xFF;
  TEST_ASSERT_EQUAL(BlobStatus::InvalidJoint, unpackConfig(blob, sizeof(blob), changed, kMaxJoints, kEncoders,
                                                           GlobalSettings(), out, og, &bad));
  TEST_ASSERT_EQUAL(0, bad);
  assertJointsEqual(defaultSettings(changed[0]), out[0]);
  assertJointsEqual(joints[1], out[1]);  // other joints keep their calibration
}

void test_defaults_scale_with_servo_model() {
  const JointSettings scs = defaultSettings(kDefaults[0]);
  const JointSettings hls = defaultSettings(kDefaults[1]);
  TEST_ASSERT_EQUAL(40, scs.trackSteps);
  TEST_ASSERT_EQUAL(60, scs.gripSteps);
  TEST_ASSERT_EQUAL(133, hls.trackSteps);  // same angle in 3.333x finer steps
  TEST_ASSERT_EQUAL(200, hls.gripSteps);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 3.333f, stepScale(kDefaults[1]));
  TEST_ASSERT_NULL(validateJoint(hls, kDefaults[1], kEncoders));
  JointSettings fast = defaultSettings(kDefaults[5]);
  fast.vmax = 9000;  // fine for an HLS (max ~12490 steps/s), not for an SCS
  TEST_ASSERT_NULL(validateJoint(fast, kDefaults[5], kEncoders));
  JointSettings scsFast = defaultSettings(kDefaults[0]);
  scsFast.vmax = 9000;
  TEST_ASSERT_NOT_NULL(validateJoint(scsFast, kDefaults[0], kEncoders));
}

void test_current_limit_rules() {
  JointSettings joints[kMaxJoints];
  loadDefaults(joints);
  GlobalSettings g;
  ParsedCommand c;
  TEST_ASSERT_NULL(parseCommand("set 2 ilimit 500", c));
  TEST_ASSERT_EQUAL(SetKey::CurrentLimit, c.key);
  TEST_ASSERT_FALSE(requiresDisarmed(c));  // adjustable while gripping
  TEST_ASSERT_NULL(applySetting(c, joints, kDefaults, kMaxJoints, g, kEncoders));
  TEST_ASSERT_EQUAL(500, joints[1].currentLimitMa);
  parseCommand("set 2 ilimit 0", c);  // back to the servo's protection current
  TEST_ASSERT_NULL(applySetting(c, joints, kDefaults, kMaxJoints, g, kEncoders));
  parseCommand("set 2 ilimit 5", c);  // below one 6.5 mA unit
  TEST_ASSERT_NOT_NULL(applySetting(c, joints, kDefaults, kMaxJoints, g, kEncoders));
  parseCommand("set 2 ilimit 20000", c);
  TEST_ASSERT_NOT_NULL(applySetting(c, joints, kDefaults, kMaxJoints, g, kEncoders));
  parseCommand("set 1 ilimit 500", c);  // SCS0009: no goal current
  TEST_ASSERT_NOT_NULL(applySetting(c, joints, kDefaults, kMaxJoints, g, kEncoders));
}

void test_current_trip_rules() {
  JointSettings joints[kMaxJoints];
  loadDefaults(joints);
  GlobalSettings g;
  ParsedCommand c;
  TEST_ASSERT_EQUAL(kDefaultCurrentTripPct, joints[1].currentTripPct);
  TEST_ASSERT_NULL(parseCommand("set 2 itrip 0", c));
  TEST_ASSERT_EQUAL(SetKey::CurrentTrip, c.key);
  TEST_ASSERT_FALSE(requiresDisarmed(c));  // bench measurements run armed
  TEST_ASSERT_NULL(applySetting(c, joints, kDefaults, kMaxJoints, g, kEncoders));
  TEST_ASSERT_EQUAL(0, joints[1].currentTripPct);
  parseCommand("set 2 itrip 100", c);
  TEST_ASSERT_NULL(applySetting(c, joints, kDefaults, kMaxJoints, g, kEncoders));
  parseCommand("set 2 itrip 40", c);
  TEST_ASSERT_NOT_NULL(applySetting(c, joints, kDefaults, kMaxJoints, g, kEncoders));
  parseCommand("set 2 itrip 306", c);  // would wrap to 50 as a byte
  TEST_ASSERT_NOT_NULL(applySetting(c, joints, kDefaults, kMaxJoints, g, kEncoders));
  parseCommand("set 1 itrip 80", c);  // SCS0009: no current reading
  TEST_ASSERT_NOT_NULL(applySetting(c, joints, kDefaults, kMaxJoints, g, kEncoders));

  // A bench setting: neither stored nor dumped.
  uint8_t blob[kBlobSize];
  packConfig(joints, kDefaults, kMaxJoints, g, blob, sizeof(blob));
  JointSettings out[kMaxJoints];
  GlobalSettings og;
  unpackConfig(blob, sizeof(blob), kDefaults, kMaxJoints, kEncoders, GlobalSettings(), out, og);
  TEST_ASSERT_EQUAL(kDefaultCurrentTripPct, out[1].currentTripPct);
  char text[3072];
  TEST_ASSERT_GREATER_THAN(0, formatDump(joints, kDefaults, kMaxJoints, g, text, sizeof(text)));
  TEST_ASSERT_NULL(strstr(text, "itrip"));
}

void test_undefined_travel_blocks_joint() {
  const JointDefaults& d = kDefaults[8];
  TEST_ASSERT_FALSE(travelDefined(d));
  JointSettings s = defaultSettings(d);
  TEST_ASSERT_NULL(validateJoint(s, d, kEncoders));  // default (disabled) is fine
  s.enabled = true;
  TEST_ASSERT_NOT_NULL(validateJoint(s, d, kEncoders));
  s = defaultSettings(d);
  s.servoA = 10;
  s.servoB = 20;
  TEST_ASSERT_NOT_NULL(validateJoint(s, d, kEncoders));

  JointSettings joints[kMaxJoints];
  loadDefaults(joints);
  GlobalSettings g;
  ParsedCommand c;
  parseCommand("set 9 on", c);
  TEST_ASSERT_NOT_NULL(applySetting(c, joints, kDefaults, kMaxJoints, g, kEncoders));
  parseCommand("set 9 enc 8", c);  // assigning an encoder is allowed, for monitoring
  TEST_ASSERT_NULL(applySetting(c, joints, kDefaults, kMaxJoints, g, kEncoders));
  TEST_ASSERT_FALSE(jointReady(joints[8], d, kEncoders));
}

// ---- parser --------------------------------------------------------------------

void test_parse_basic_commands() {
  ParsedCommand c;
  TEST_ASSERT_NULL(parseCommand("  ARM ", c));
  TEST_ASSERT_EQUAL(Verb::Arm, c.verb);
  TEST_ASSERT_EQUAL(0, c.joint);
  TEST_ASSERT_NULL(parseCommand("arm 4", c));
  TEST_ASSERT_EQUAL(Verb::Arm, c.verb);
  TEST_ASSERT_EQUAL(4, c.joint);
  TEST_ASSERT_NULL(parseCommand("disarm 9", c));
  TEST_ASSERT_EQUAL(Verb::Disarm, c.verb);
  TEST_ASSERT_EQUAL(9, c.joint);
  TEST_ASSERT_FALSE(requiresDisarmed(c));  // single-joint arm/disarm works while others follow
  TEST_ASSERT_NULL(parseCommand("", c));
  TEST_ASSERT_EQUAL(Verb::None, c.verb);
  TEST_ASSERT_NULL(parseCommand("# comment", c));
  TEST_ASSERT_EQUAL(Verb::Comment, c.verb);
  TEST_ASSERT_NULL(parseCommand("stats reset", c));
  TEST_ASSERT_EQUAL(Verb::StatsReset, c.verb);
  TEST_ASSERT_NULL(parseCommand("monitor on", c));
  TEST_ASSERT_EQUAL(Verb::Monitor, c.verb);
  TEST_ASSERT_TRUE(c.flag);
  TEST_ASSERT_NULL(parseCommand("plot 2 on", c));
  TEST_ASSERT_EQUAL(Verb::Plot, c.verb);
  TEST_ASSERT_EQUAL(2, c.joint);
  TEST_ASSERT_NULL(parseCommand("cal 6 hand b", c));
  TEST_ASSERT_EQUAL(Verb::CalHand, c.verb);
  TEST_ASSERT_EQUAL(6, c.joint);
  TEST_ASSERT_TRUE(c.flag);
  TEST_ASSERT_NULL(parseCommand("cal 1 a", c));
  TEST_ASSERT_EQUAL(Verb::Cal, c.verb);
  TEST_ASSERT_FALSE(c.flag);
}

void test_parse_grip_commands() {
  ParsedCommand c;
  TEST_ASSERT_NULL(parseCommand("set 2 grip 0", c));
  TEST_ASSERT_EQUAL(SetKey::Grip, c.key);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, c.values[0]);
  TEST_ASSERT_NULL(parseCommand("set gripload 70", c));
  TEST_ASSERT_EQUAL(SetKey::GripLoad, c.key);
  TEST_ASSERT_TRUE(isGlobalKey(c.key));
  TEST_ASSERT_NULL(parseCommand("set loadtrip on", c));
  TEST_ASSERT_EQUAL(SetKey::LoadTrip, c.key);
  TEST_ASSERT_EQUAL_FLOAT(1.0f, c.values[0]);
  TEST_ASSERT_NOT_NULL(parseCommand("set loadtrip maybe", c));

  JointSettings joints[kMaxJoints];
  loadDefaults(joints);
  GlobalSettings g;
  parseCommand("set loadtrip on", c);
  TEST_ASSERT_NULL(applySetting(c, joints, kDefaults, kMaxJoints, g, kEncoders));
  TEST_ASSERT_TRUE(g.loadTripEnabled);
  parseCommand("set gripload 99", c);  // above the servo's own 80 % protection margin
  TEST_ASSERT_NOT_NULL(applySetting(c, joints, kDefaults, kMaxJoints, g, kEncoders));
  parseCommand("set 1 grip 500", c);
  TEST_ASSERT_NOT_NULL(applySetting(c, joints, kDefaults, kMaxJoints, g, kEncoders));
}

void test_parse_home_command() {
  ParsedCommand c;
  TEST_ASSERT_NULL(parseCommand("home 9", c));
  TEST_ASSERT_EQUAL(Verb::Home, c.verb);
  TEST_ASSERT_EQUAL(9, c.joint);
  TEST_ASSERT_FALSE(requiresDisarmed(c));  // only the homed joint must be off
  TEST_ASSERT_NOT_NULL(parseCommand("home", c));
  TEST_ASSERT_NOT_NULL(parseCommand("home all", c));
}

void test_parse_sweep_commands() {
  ParsedCommand c;
  TEST_ASSERT_NULL(parseCommand("sweep 3", c));
  TEST_ASSERT_EQUAL(Verb::Sweep, c.verb);
  TEST_ASSERT_EQUAL(3, c.joint);
  TEST_ASSERT_TRUE(requiresDisarmed(c));
  TEST_ASSERT_NULL(parseCommand("sweep 3 apply", c));
  TEST_ASSERT_EQUAL(Verb::SweepApply, c.verb);
  TEST_ASSERT_FALSE(requiresDisarmed(c));
  TEST_ASSERT_NOT_NULL(parseCommand("sweep", c));
  TEST_ASSERT_NOT_NULL(parseCommand("sweep 3 now", c));
}

void test_parse_limits_commands() {
  ParsedCommand c;
  TEST_ASSERT_NULL(parseCommand("limits check", c));
  TEST_ASSERT_EQUAL(Verb::LimitsCheck, c.verb);
  TEST_ASSERT_NULL(parseCommand("limits write all", c));
  TEST_ASSERT_EQUAL(Verb::LimitsWrite, c.verb);
  TEST_ASSERT_EQUAL(0, c.joint);
  TEST_ASSERT_TRUE(requiresDisarmed(c));
  TEST_ASSERT_NULL(parseCommand("limits write 7", c));
  TEST_ASSERT_EQUAL(7, c.joint);
  TEST_ASSERT_NOT_NULL(parseCommand("limits write", c));
  TEST_ASSERT_NOT_NULL(parseCommand("limits erase all", c));
}

void test_parse_set_commands() {
  ParsedCommand c;
  TEST_ASSERT_NULL(parseCommand("set 2 enc none", c));
  TEST_ASSERT_EQUAL(SetKey::Encoder, c.key);
  TEST_ASSERT_EQUAL_FLOAT(kNoEncoder, c.values[0]);
  TEST_ASSERT_NULL(parseCommand("set 1 exo 16200 1916", c));
  TEST_ASSERT_EQUAL(SetKey::Exo, c.key);
  TEST_ASSERT_EQUAL(2, c.valueCount);
  TEST_ASSERT_NULL(parseCommand("set jump 30", c));
  TEST_ASSERT_EQUAL(SetKey::Jump, c.key);
  TEST_ASSERT_EQUAL(0, c.joint);
  TEST_ASSERT_NULL(parseCommand("set 3 off", c));
  TEST_ASSERT_EQUAL(SetKey::Enable, c.key);
  TEST_ASSERT_FALSE(c.flag);
}

void test_parse_errors() {
  ParsedCommand c;
  TEST_ASSERT_NOT_NULL(parseCommand("fly", c));
  TEST_ASSERT_NOT_NULL(parseCommand("set 1 vmax fast", c));
  TEST_ASSERT_EQUAL(Verb::None, c.verb);  // nothing half-parsed
  TEST_ASSERT_NOT_NULL(parseCommand("set 1 exo 20000 5", c));
  TEST_ASSERT_NOT_NULL(parseCommand("cal 1 c", c));
  TEST_ASSERT_NOT_NULL(parseCommand("arm now", c));
  TEST_ASSERT_NOT_NULL(parseCommand("set 0 on", c));
}

void test_requires_disarmed() {
  ParsedCommand c;
  parseCommand("set 1 exo 100 2000", c);
  TEST_ASSERT_TRUE(requiresDisarmed(c));
  parseCommand("set 1 vmax 500", c);
  TEST_ASSERT_FALSE(requiresDisarmed(c));
  parseCommand("set lead 50", c);
  TEST_ASSERT_FALSE(requiresDisarmed(c));
  parseCommand("save", c);
  TEST_ASSERT_TRUE(requiresDisarmed(c));
  parseCommand("monitor off", c);
  TEST_ASSERT_FALSE(requiresDisarmed(c));
}

void test_apply_validates_and_is_atomic() {
  JointSettings joints[kMaxJoints];
  loadDefaults(joints);
  GlobalSettings g;
  ParsedCommand c;
  parseCommand("set 1 servo 100 700", c);  // 100 is outside travel
  TEST_ASSERT_NOT_NULL(applySetting(c, joints, kDefaults, kMaxJoints, g, kEncoders));
  TEST_ASSERT_FALSE(servoCalibrated(joints[0], kDefaults[0]));
  parseCommand("set 9 on", c);
  TEST_ASSERT_NOT_NULL(applySetting(c, joints, kDefaults, kMaxJoints, g, kEncoders));
  parseCommand("set 1 exo 16200 1916", c);
  TEST_ASSERT_NULL(applySetting(c, joints, kDefaults, kMaxJoints, g, kEncoders));
  TEST_ASSERT_EQUAL(2100, joints[0].exoB);
  parseCommand("set mincut 50", c);
  TEST_ASSERT_NOT_NULL(applySetting(c, joints, kDefaults, kMaxJoints, g, kEncoders));
  TEST_ASSERT_EQUAL_FLOAT(1.0f, g.minCutoffHz);
}

void test_dump_round_trip() {
  JointSettings joints[kMaxJoints];
  GlobalSettings g;
  configureSample(joints, g);
  joints[2].exoRef = 300;
  joints[2].exoB = 20000;  // 1.2 encoder turns: only exospan can carry it
  joints[1].currentLimitMa = 650;  // HLS joint
  char text[3072];
  const size_t n = formatDump(joints, kDefaults, kMaxJoints, g, text, sizeof(text));
  TEST_ASSERT_TRUE(n > 0);

  // Start from a differently configured table to prove `defaults` + sets restore it.
  JointSettings restored[kMaxJoints];
  GlobalSettings rg;
  configureSample(restored, rg);
  restored[2].enabled = true;
  rg.leadMs = 5.0f;

  char* save = nullptr;
  for (char* line = strtok_r(text, "\n", &save); line != nullptr; line = strtok_r(nullptr, "\n", &save)) {
    ParsedCommand c;
    TEST_ASSERT_NULL_MESSAGE(parseCommand(line, c), line);
    if (c.verb == Verb::Defaults) {
      loadDefaults(restored);
      rg = GlobalSettings();
    } else if (c.verb == Verb::Set) {
      TEST_ASSERT_NULL_MESSAGE(applySetting(c, restored, kDefaults, kMaxJoints, rg, kEncoders), line);
    } else {
      TEST_ASSERT_EQUAL(Verb::Comment, c.verb);
    }
  }
  for (uint8_t i = 0; i < kMaxJoints; ++i) assertJointsEqual(joints[i], restored[i]);
  TEST_ASSERT_EQUAL_FLOAT(g.leadMs, rg.leadMs);
  TEST_ASSERT_EQUAL_FLOAT(g.beta, rg.beta);
  TEST_ASSERT_EQUAL_FLOAT(g.jumpDeg, rg.jumpDeg);
  TEST_ASSERT_EQUAL_FLOAT(g.gripLoadPct, rg.gripLoadPct);
  TEST_ASSERT_EQUAL(g.loadTripEnabled, rg.loadTripEnabled);
}

void test_dump_reports_short_buffer() {
  JointSettings joints[kMaxJoints];
  GlobalSettings g;
  configureSample(joints, g);
  char text[64];
  TEST_ASSERT_EQUAL(0, formatDump(joints, kDefaults, kMaxJoints, g, text, sizeof(text)));
}

int main() {
  UNITY_BEGIN();
  RUN_TEST(test_defaults_are_valid_and_not_ready);
  RUN_TEST(test_ready_needs_everything);
  RUN_TEST(test_validate_rejections);
  RUN_TEST(test_exo_raw_b_wraps);
  RUN_TEST(test_blob_round_trip);
  RUN_TEST(test_blob_rejects_corruption_without_touching_output);
  RUN_TEST(test_blob_invalid_joint_reset_to_defaults);
  RUN_TEST(test_validate_span_limits);
  RUN_TEST(test_exospan_and_legacy_exo_syntax);
  RUN_TEST(test_blob_older_versions_superseded);
  RUN_TEST(test_blob_keeps_multi_turn_span_and_current_limit);
  RUN_TEST(test_blob_changed_model_resets_that_joint);
  RUN_TEST(test_defaults_scale_with_servo_model);
  RUN_TEST(test_current_limit_rules);
  RUN_TEST(test_current_trip_rules);
  RUN_TEST(test_undefined_travel_blocks_joint);
  RUN_TEST(test_parse_basic_commands);
  RUN_TEST(test_parse_set_commands);
  RUN_TEST(test_parse_limits_commands);
  RUN_TEST(test_parse_grip_commands);
  RUN_TEST(test_parse_sweep_commands);
  RUN_TEST(test_parse_home_command);
  RUN_TEST(test_parse_errors);
  RUN_TEST(test_requires_disarmed);
  RUN_TEST(test_apply_validates_and_is_atomic);
  RUN_TEST(test_dump_round_trip);
  RUN_TEST(test_dump_reports_short_buffer);
  return UNITY_END();
}
