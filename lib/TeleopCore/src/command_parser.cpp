#include "command_parser.h"

#include <ctype.h>
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

namespace teleop {

namespace {

constexpr size_t kMaxLine = 128;
constexpr size_t kMaxTokens = 6;

struct Tokens {
  char buffer[kMaxLine];
  const char* t[kMaxTokens];
  size_t n = 0;
  bool overflow = false;
};

void tokenize(const char* line, Tokens& out) {
  size_t len = 0;
  while (line[len] != '\0' && len + 1 < kMaxLine) {
    out.buffer[len] = static_cast<char>(tolower(static_cast<unsigned char>(line[len])));
    ++len;
  }
  out.buffer[len] = '\0';
  char* p = out.buffer;
  while (*p != '\0') {
    while (*p == ' ' || *p == '\t' || *p == '\r' || *p == '\n') *p++ = '\0';
    if (*p == '\0') break;
    if (out.n == kMaxTokens) {
      out.overflow = true;
      return;
    }
    out.t[out.n++] = p;
    while (*p != '\0' && *p != ' ' && *p != '\t' && *p != '\r' && *p != '\n') ++p;
  }
}

bool eq(const char* a, const char* b) { return a != nullptr && strcmp(a, b) == 0; }

bool parseFloat(const char* s, float& v) {
  if (s == nullptr) return false;
  char* end = nullptr;
  v = strtof(s, &end);
  return end != s && *end == '\0' && isfinite(v);
}

bool parseInt(const char* s, long lo, long hi, long& v) {
  if (s == nullptr) return false;
  char* end = nullptr;
  v = strtol(s, &end, 10);
  return end != s && *end == '\0' && v >= lo && v <= hi;
}

bool parseOnOff(const char* s, bool& flag) {
  if (eq(s, "on")) {
    flag = true;
    return true;
  }
  if (eq(s, "off")) {
    flag = false;
    return true;
  }
  return false;
}

bool parseEndpoint(const char* s, bool& isB) {
  if (eq(s, "a")) {
    isB = false;
    return true;
  }
  if (eq(s, "b")) {
    isB = true;
    return true;
  }
  return false;
}

struct KeyName {
  const char* name;
  SetKey key;
};

constexpr KeyName kGlobalKeys[] = {
    {"lead", SetKey::Lead}, {"mincut", SetKey::MinCutoff}, {"beta", SetKey::Beta}, {"jump", SetKey::Jump}};
constexpr KeyName kJointNumberKeys[] = {
    {"vmax", SetKey::VMax}, {"amax", SetKey::AMax}, {"vsat", SetKey::VSat}, {"trk", SetKey::Track}};

const char* parseSet(const Tokens& tk, ParsedCommand& out) {
  out.verb = Verb::Set;
  for (const KeyName& k : kGlobalKeys) {
    if (!eq(tk.t[1], k.name)) continue;
    if (tk.n != 3 || !parseFloat(tk.t[2], out.values[0])) return "usage: set <lead|mincut|beta|jump> <value>";
    out.key = k.key;
    out.valueCount = 1;
    return nullptr;
  }
  long joint = 0;
  if (tk.n < 3 || !parseInt(tk.t[1], 1, 253, joint)) return "usage: set <joint> <key> <value...> (joint = servo id)";
  out.joint = static_cast<uint8_t>(joint);
  const char* key = tk.t[2];

  if (eq(key, "on") || eq(key, "off")) {
    if (tk.n != 3) return "usage: set <joint> on|off";
    out.key = SetKey::Enable;
    out.flag = eq(key, "on");
    return nullptr;
  }
  if (eq(key, "enc")) {
    out.key = SetKey::Encoder;
    out.valueCount = 1;
    long index = 0;
    if (tk.n == 4 && eq(tk.t[3], "none")) {
      out.values[0] = kNoEncoder;
      return nullptr;
    }
    if (tk.n != 4 || !parseInt(tk.t[3], 0, 254, index)) return "usage: set <joint> enc <index|none>";
    out.values[0] = static_cast<float>(index);
    return nullptr;
  }
  if (eq(key, "exo") || eq(key, "servo")) {
    const bool exo = eq(key, "exo");
    long a = 0, b = 0;
    const long hi = exo ? kEncoderCounts - 1 : 4095;
    if (tk.n != 5 || !parseInt(tk.t[3], 0, hi, a) || !parseInt(tk.t[4], 0, hi, b)) {
      return exo ? "usage: set <joint> exo <rawA> <rawB>" : "usage: set <joint> servo <a> <b>";
    }
    out.key = exo ? SetKey::Exo : SetKey::Servo;
    out.values[0] = static_cast<float>(a);
    out.values[1] = static_cast<float>(b);
    out.valueCount = 2;
    return nullptr;
  }
  for (const KeyName& k : kJointNumberKeys) {
    if (!eq(key, k.name)) continue;
    long v = 0;
    if (tk.n != 4 || !parseInt(tk.t[3], 0, 65535, v)) return "usage: set <joint> <vmax|amax|vsat|trk> <integer>";
    out.key = k.key;
    out.values[0] = static_cast<float>(v);
    out.valueCount = 1;
    return nullptr;
  }
  return "unknown key (on off enc exo servo vmax amax vsat trk | lead mincut beta jump)";
}

struct Simple {
  const char* name;
  Verb verb;
};

constexpr Simple kSimpleVerbs[] = {
    {"help", Verb::Help},   {"?", Verb::Help},         {"status", Verb::Status},
    {"enc", Verb::Enc},     {"ping", Verb::Ping},      {"arm", Verb::Arm},
    {"disarm", Verb::Disarm}, {"x", Verb::Estop},      {"joints", Verb::Joints},
    {"save", Verb::Save},   {"load", Verb::Load},      {"defaults", Verb::Defaults},
    {"dump", Verb::Dump},
};

class Appender {
 public:
  Appender(char* out, size_t capacity) : out_(out), capacity_(capacity) {
    if (capacity_ > 0) out_[0] = '\0';
  }
  void add(const char* format, ...) __attribute__((format(printf, 2, 3))) {
    if (failed_) return;
    va_list args;
    va_start(args, format);
    const int n = vsnprintf(out_ + length_, capacity_ - length_, format, args);
    va_end(args);
    if (n < 0 || length_ + static_cast<size_t>(n) >= capacity_) {
      failed_ = true;
      return;
    }
    length_ += static_cast<size_t>(n);
  }
  size_t result() const { return failed_ ? 0 : length_; }

 private:
  char* out_;
  size_t capacity_;
  size_t length_ = 0;
  bool failed_ = false;
};

}  // namespace

const char* parseCommand(const char* line, ParsedCommand& out) {
  out = ParsedCommand();
  if (line == nullptr) return nullptr;
  const char* first = line;
  while (*first == ' ' || *first == '\t') ++first;
  if (*first == '#') {  // before tokenizing: comments may hold any number of words
    out.verb = Verb::Comment;
    return nullptr;
  }
  Tokens tk;
  tokenize(line, tk);
  if (tk.overflow) return "too many arguments";
  if (tk.n == 0) return nullptr;
  const char* cmd = tk.t[0];

  for (const Simple& s : kSimpleVerbs) {
    if (!eq(cmd, s.name)) continue;
    if (tk.n != 1) return "this command takes no arguments";
    out.verb = s.verb;
    return nullptr;
  }
  if (eq(cmd, "stats")) {
    if (tk.n == 1) {
      out.verb = Verb::Stats;
      return nullptr;
    }
    if (tk.n == 2 && eq(tk.t[1], "reset")) {
      out.verb = Verb::StatsReset;
      return nullptr;
    }
    return "usage: stats [reset]";
  }
  if (eq(cmd, "monitor")) {
    if (tk.n != 2 || !parseOnOff(tk.t[1], out.flag)) return "usage: monitor on|off";
    out.verb = Verb::Monitor;
    return nullptr;
  }
  if (eq(cmd, "plot")) {
    out.verb = Verb::Plot;
    if (tk.n == 2 && eq(tk.t[1], "off")) return nullptr;
    long joint = 0;
    if (tk.n != 3 || !parseInt(tk.t[1], 1, 253, joint) || !parseOnOff(tk.t[2], out.flag)) {
      out.verb = Verb::None;
      return "usage: plot <joint> on|off  or  plot off";
    }
    out.joint = static_cast<uint8_t>(joint);
    return nullptr;
  }
  if (eq(cmd, "cal")) {
    long joint = 0;
    if (tk.n >= 3 && parseInt(tk.t[1], 1, 253, joint)) {
      out.joint = static_cast<uint8_t>(joint);
      if (tk.n == 3 && parseEndpoint(tk.t[2], out.flag)) {
        out.verb = Verb::Cal;
        return nullptr;
      }
      if (tk.n == 4 && eq(tk.t[2], "hand") && parseEndpoint(tk.t[3], out.flag)) {
        out.verb = Verb::CalHand;
        return nullptr;
      }
    }
    out.joint = 0;
    return "usage: cal <joint> a|b  or  cal <joint> hand a|b";
  }
  if (eq(cmd, "set")) {
    if (tk.n < 2) return "usage: set <joint> <key> <value...>  or  set <global key> <value>";
    const char* error = parseSet(tk, out);
    if (error != nullptr) out = ParsedCommand();
    return error;
  }
  return "unknown command (type `help`)";
}

bool isGlobalKey(SetKey key) {
  return key == SetKey::Lead || key == SetKey::MinCutoff || key == SetKey::Beta || key == SetKey::Jump;
}

bool requiresDisarmed(const ParsedCommand& c) {
  switch (c.verb) {
    case Verb::Cal:
    case Verb::CalHand:
    case Verb::Save:
    case Verb::Load:
    case Verb::Defaults:
    case Verb::Ping:
      return true;
    case Verb::Monitor:
      return c.flag;
    case Verb::Set:
      return c.key == SetKey::Enable || c.key == SetKey::Encoder || c.key == SetKey::Exo ||
             c.key == SetKey::Servo;
    default:
      return false;
  }
}

int jointIndexForServo(const JointDefaults* defaults, uint8_t count, uint8_t servoId) {
  for (uint8_t i = 0; i < count; ++i) {
    if (defaults[i].servoId == servoId) return i;
  }
  return -1;
}

const char* applySetting(const ParsedCommand& c, JointSettings* joints, const JointDefaults* defaults,
                         uint8_t count, GlobalSettings& global, uint8_t encoderCount) {
  if (c.verb != Verb::Set) return "not a set command";
  if (isGlobalKey(c.key)) {
    GlobalSettings g = global;
    switch (c.key) {
      case SetKey::Lead: g.leadMs = c.values[0]; break;
      case SetKey::MinCutoff: g.minCutoffHz = c.values[0]; break;
      case SetKey::Beta: g.beta = c.values[0]; break;
      case SetKey::Jump: g.jumpDeg = c.values[0]; break;
      default: break;
    }
    const char* error = validateGlobal(g);
    if (error == nullptr) global = g;
    return error;
  }

  const int index = jointIndexForServo(defaults, count, c.joint);
  if (index < 0) return "no joint with that servo id";
  JointSettings s = joints[index];
  const long v0 = lroundf(c.values[0]);
  const long v1 = lroundf(c.values[1]);
  switch (c.key) {
    case SetKey::Enable: s.enabled = c.flag; break;
    case SetKey::Encoder: s.encoderIndex = static_cast<uint8_t>(v0); break;
    case SetKey::Exo:
      s.exoRef = static_cast<uint16_t>(v0);
      s.exoB = static_cast<int16_t>(unwrapCounts(static_cast<uint16_t>(v1), static_cast<uint16_t>(v0)));
      break;
    case SetKey::Servo:
      s.servoA = static_cast<int16_t>(v0);
      s.servoB = static_cast<int16_t>(v1);
      break;
    case SetKey::VMax: s.vmax = static_cast<uint16_t>(v0); break;
    case SetKey::AMax: s.amax = static_cast<uint16_t>(v0); break;
    case SetKey::VSat: s.vsat = static_cast<uint16_t>(v0); break;
    case SetKey::Track: s.trackSteps = static_cast<uint16_t>(v0); break;
    default: return "unknown key";
  }
  const char* error = validateJoint(s, defaults[index], encoderCount);
  if (error == nullptr) joints[index] = s;
  return error;
}

size_t formatDump(const JointSettings* joints, const JointDefaults* defaults, uint8_t count,
                  const GlobalSettings& global, char* out, size_t capacity) {
  Appender a(out, capacity);
  a.add("defaults\n");
  for (uint8_t i = 0; i < count; ++i) {
    const JointSettings& s = joints[i];
    const JointDefaults& d = defaults[i];
    a.add("# %u %s\n", d.servoId, d.name);
    if (s.encoderIndex == kNoEncoder) {
      a.add("set %u enc none\n", d.servoId);
    } else {
      a.add("set %u enc %u\n", d.servoId, s.encoderIndex);
    }
    if (exoCalibrated(s)) a.add("set %u exo %u %u\n", d.servoId, s.exoRef, exoRawB(s));
    if (servoCalibrated(s, d)) a.add("set %u servo %d %d\n", d.servoId, s.servoA, s.servoB);
    a.add("set %u vmax %u\nset %u amax %u\nset %u vsat %u\nset %u trk %u\n", d.servoId, s.vmax,
          d.servoId, s.amax, d.servoId, s.vsat, d.servoId, s.trackSteps);
    a.add("set %u %s\n", d.servoId, s.enabled ? "on" : "off");
  }
  a.add("set lead %g\nset mincut %g\nset beta %g\nset jump %g\n", global.leadMs, global.minCutoffHz,
        global.beta, global.jumpDeg);
  a.add("# end of dump: run `save` to store it\n");
  return a.result();
}

}  // namespace teleop
