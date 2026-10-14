#include "console.h"

#include <ctype.h>
#include <stdlib.h>
#include <string.h>

namespace {

constexpr uint32_t kPollMs = 5;
constexpr uint32_t kPlotPeriodMs = 20;  // 50 Hz Teleplot

struct ParamName {
  const char* key;
  ParamId id;
};

constexpr ParamName kParams[] = {
    {"sa", ParamId::ServoA},       {"sb", ParamId::ServoB},   {"vmax", ParamId::MaxSpeed},
    {"amax", ParamId::Acceleration}, {"lead", ParamId::LeadMs}, {"mincut", ParamId::MinCutoff},
    {"beta", ParamId::Beta},
};

const char* encoderStateShort(AS5048BChannelState state) {
  switch (state) {
    case AS5048BChannelState::Online: return "ok";
    case AS5048BChannelState::NoMagnet: return "nomag";
    case AS5048BChannelState::Offline: return "off";
  }
  return "?";
}

}  // namespace

void Console::startTask() {
  xTaskCreatePinnedToCore(taskEntry, "console", 6144, this, 1, nullptr, config::kConsoleCore);
}

void Console::taskEntry(void* arg) { static_cast<Console*>(arg)->run(); }

void Console::run() {
  char text[160];
  Serial.println();
  Serial.println("Teleoperation M0 ready. Type `help`.");
  for (;;) {
    while (controller_.popMessage(text, sizeof(text))) {
      Serial.print("[ctl] ");
      Serial.println(text);
    }
    while (Serial.available() > 0) {
      const char c = static_cast<char>(Serial.read());
      if (c == '\r') continue;
      if (c == '\n') {
        line_[lineLength_] = '\0';
        handleLine(line_);
        lineLength_ = 0;
      } else if (lineLength_ + 1 < sizeof(line_)) {
        line_[lineLength_++] = c;
      }
    }
    if (plotting_ && millis() - lastPlotMs_ >= kPlotPeriodMs) {
      lastPlotMs_ = millis();
      plot();
    }
    vTaskDelay(pdMS_TO_TICKS(kPollMs));
  }
}

void Console::handleLine(char* line) {
  for (char* p = line; *p; ++p) *p = static_cast<char>(tolower(*p));
  char* save = nullptr;
  const char* cmd = strtok_r(line, " \t", &save);
  if (cmd == nullptr) return;
  const char* arg1 = strtok_r(nullptr, " \t", &save);
  const char* arg2 = strtok_r(nullptr, " \t", &save);

  Command command;
  if (!strcmp(cmd, "help") || !strcmp(cmd, "?")) {
    printHelp();
  } else if (!strcmp(cmd, "status")) {
    printStatus();
  } else if (!strcmp(cmd, "stats")) {
    if (arg1 != nullptr && !strcmp(arg1, "reset")) {
      command.type = CommandType::ResetStats;
      controller_.post(command);
    } else {
      printStats();
    }
  } else if (!strcmp(cmd, "enc")) {
    printEncoders();
  } else if (!strcmp(cmd, "ping")) {
    command.type = CommandType::Ping;
    controller_.post(command);
  } else if (!strcmp(cmd, "arm")) {
    command.type = CommandType::Arm;
    controller_.post(command);
  } else if (!strcmp(cmd, "disarm")) {
    command.type = CommandType::Disarm;
    controller_.post(command);
  } else if (!strcmp(cmd, "x")) {
    command.type = CommandType::Estop;
    controller_.post(command);
  } else if (!strcmp(cmd, "exo") && arg1 != nullptr && (!strcmp(arg1, "a") || !strcmp(arg1, "b"))) {
    command.type = arg1[0] == 'a' ? CommandType::CaptureExoA : CommandType::CaptureExoB;
    controller_.post(command);
  } else if (!strcmp(cmd, "set") && arg1 != nullptr && arg2 != nullptr) {
    handleSet(arg1, arg2);
  } else if (!strcmp(cmd, "plot") && arg1 != nullptr) {
    plotting_ = !strcmp(arg1, "on");
    Serial.printf("plot %s\n", plotting_ ? "on" : "off");
  } else {
    Serial.printf("unknown command: %s (type `help`)\n", cmd);
  }
}

void Console::handleSet(const char* key, const char* value) {
  char* end = nullptr;
  const float v = strtof(value, &end);
  if (end == value || *end != '\0') {
    Serial.printf("not a number: %s\n", value);
    return;
  }
  for (const ParamName& p : kParams) {
    if (!strcmp(p.key, key)) {
      Command command;
      command.type = CommandType::SetParam;
      command.param = p.id;
      command.value = v;
      controller_.post(command);
      return;
    }
  }
  Serial.printf("unknown parameter: %s\n", key);
}

void Console::printHelp() {
  Serial.println(
      "commands:\n"
      "  status            state, joint calibration, servo readings\n"
      "  stats [reset]     loop timing, bus and encoder statistics\n"
      "  enc               all encoder channels\n"
      "  ping              scan servo ids 1-8 (disarmed only)\n"
      "  arm | disarm      torque on (hold, then ALIGN -> FOLLOW) / torque off\n"
      "  x                 E-stop: torque off on every servo (BOOT button does the same)\n"
      "  exo a | exo b     capture the current encoder reading as endpoint A / B\n"
      "  set <key> <val>   sa sb (servo endpoints), vmax amax, lead (ms), mincut beta\n"
      "  plot on|off       Teleplot traces at 50 Hz");
}

void Console::printStatus() {
  controller_.snapshot(snapshot_);
  const Snapshot& s = snapshot_;
  const teleop::JointMap& j = s.joint;
  Serial.printf("state %s  armed %d  last fault: %s  torque-off confirmed %d  echo %d\n",
                teleopStateToString(s.state), s.armed, teleop::faultToString(s.lastFault),
                s.torqueOffConfirmed, s.echo);
  Serial.printf("joint %s: encoder #%u -> servo %u, travel [%d, %d]\n", j.name, j.encoderIndex,
                j.servoId, j.servoMin, j.servoMax);
  Serial.printf("  exo  ref %u%s  A %d  B %d (%.1f deg)  calibrated %d\n", j.exoRef,
                s.exoRefSet ? "" : " (unset)", j.exoA, j.exoB,
                (j.exoB - j.exoA) * teleop::kDegreesPerCount, j.exoCalibrated());
  Serial.printf("  servo A %d  B %d  valid %d  -> ready %d\n", j.servoA, j.servoB,
                j.servoRangeValid(), j.ready());
  Serial.printf("encoder fresh %d raw %u  exo %.0f (%.2f deg) filtered %.1f\n", s.encFresh,
                s.encRaw, s.exoCounts, s.exoCounts * teleop::kDegreesPerCount, s.exoFilteredCounts);
  Serial.printf("target %.1f  goal %.1f  speed reg %u  sent %d\n", s.target, s.goal,
                s.speedRegister, s.goalSent);
  if (s.servoOk) {
    const ServoState& v = s.servo;
    Serial.printf("servo pos %d goal %d torque %d load %.1f%% %.1fV %dC status 0x%02X err 0x%02X\n",
                  v.position, v.goal, v.torqueOn, v.loadPct, v.voltage, v.temperatureC, v.status,
                  v.errorByte);
  } else {
    Serial.println("servo: no reply this cycle");
  }
  const MotionParams& p = s.params;
  Serial.printf("params vmax %.0f amax %.0f lead %.0fms mincut %.3f beta %.4f\n", p.maxSpeed,
                p.acceleration, p.leadMs, p.minCutoffHz, p.beta);
}

void Console::printStats() {
  controller_.snapshot(snapshot_);
  const TimingStats& t = snapshot_.timing;
  const FeetechPortStats& b = snapshot_.port;
  const AS5048BCycleStats& e = snapshot_.encoderCycle;
  Serial.printf("loop %lu cycles, period last %lu us, jitter max %lu us p99 <= %lu us, overruns %lu\n",
                (unsigned long)t.cycles, (unsigned long)t.lastPeriodUs, (unsigned long)t.maxJitterUs,
                (unsigned long)t.p99JitterUs, (unsigned long)t.overruns);
  Serial.printf("exec avg %lu us max %lu us (budget %lu us)\n", (unsigned long)t.averageExecUs(),
                (unsigned long)t.maxExecUs, (unsigned long)config::kControlPeriodUs);
  Serial.printf("servo bus sent %lu ok %lu timeouts %lu parse errors %lu echo errors %lu, reply last %lu us max %lu us\n",
                (unsigned long)b.sent, (unsigned long)b.received, (unsigned long)b.timeouts,
                (unsigned long)b.parseErrors, (unsigned long)b.echoErrors,
                (unsigned long)b.lastReplyUs, (unsigned long)b.maxReplyUs);
  Serial.printf("encoder pass avg %lu us max %lu us, skew max %lu us\n",
                (unsigned long)e.averageCycleUs(), (unsigned long)e.maxCycleUs,
                (unsigned long)e.maxSkewUs);
}

void Console::printEncoders() {
  controller_.snapshot(snapshot_);
  Serial.println("#  addr  state  valid  angle(deg)  agc  magnet  errors");
  for (uint8_t i = 0; i < config::kEncoderCount; ++i) {
    const EncoderSummary& e = snapshot_.encoders[i];
    Serial.printf("%u  0x%02X  %-5s  %d      %7.2f    %3u  %d       %lu\n", i, e.address,
                  encoderStateShort(e.state), e.valid, e.angle * teleop::kDegreesPerCount, e.agc,
                  e.magnetOk, (unsigned long)e.errors);
  }
}

void Console::plot() {
  controller_.snapshot(snapshot_);
  const Snapshot& s = snapshot_;
  Serial.printf(">exo_deg:%.2f\n>exo_filt_deg:%.2f\n>target:%.1f\n>goal:%.1f\n",
                s.exoCounts * teleop::kDegreesPerCount,
                s.exoFilteredCounts * teleop::kDegreesPerCount, s.target, s.goal);
  if (s.servoOk) {
    Serial.printf(">present:%d\n>load:%.1f\n>speed_reg:%u\n", s.servo.position, s.servo.loadPct,
                  s.speedRegister);
  }
}
