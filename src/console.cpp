#include "console.h"

#include <command_parser.h>

using teleop::Verb;

namespace {

constexpr uint32_t kPollMs = 5;
constexpr uint32_t kPlotPeriodMs = 20;  // 50 Hz Teleplot
constexpr TickType_t kPostWait = pdMS_TO_TICKS(200);

char dumpBuffer[3072];

const char* encoderStateShort(AS5048BChannelState state) {
  switch (state) {
    case AS5048BChannelState::Online: return "ok";
    case AS5048BChannelState::NoMagnet: return "nomag";
    case AS5048BChannelState::Offline: return "off";
  }
  return "?";
}

int jointIndex(uint8_t servoId) {
  return teleop::jointIndexForServo(config::kJoints, teleop::kMaxJoints, servoId);
}

}  // namespace

void Console::startTask() {
  xTaskCreatePinnedToCore(taskEntry, "console", 8192, this, 1, nullptr, config::kConsoleCore);
}

void Console::taskEntry(void* arg) { static_cast<Console*>(arg)->run(); }

void Console::drainMessages() {
  char text[160];
  while (controller_.popMessage(text, sizeof(text))) {
    Serial.print("[ctl] ");
    Serial.println(text);
  }
}

void Console::run() {
  Serial.println();
  Serial.println("Teleoperation M1 ready. Type `help`.");
  for (;;) {
    drainMessages();
    while (Serial.available() > 0) {
      const char c = static_cast<char>(Serial.read());
      if (c == '\r') continue;
      if (c == '\n') {
        line_[lineLength_] = '\0';
        handleLine(line_);
        lineLength_ = 0;
        drainMessages();  // keep replies in step with pasted multi-line input
      } else if (lineLength_ + 1 < sizeof(line_)) {
        line_[lineLength_++] = c;
      }
    }
    const uint32_t now = millis();
    if (plotJoint_ != 0 && now - lastPlotMs_ >= kPlotPeriodMs) {
      lastPlotMs_ = now;
      plot();
    }
    if (now - lastMonitorMs_ >= config::kMonitorPeriodMs) {
      lastMonitorMs_ = now;
      controller_.snapshot(snapshot_);
      if (snapshot_.monitor) printMonitorRow();
    }
    vTaskDelay(pdMS_TO_TICKS(kPollMs));
  }
}

void Console::handleLine(const char* line) {
  teleop::ParsedCommand cmd;
  const char* error = teleop::parseCommand(line, cmd);
  if (error != nullptr) {
    Serial.printf("error: %s\n", error);
    return;
  }
  switch (cmd.verb) {
    case Verb::None:
    case Verb::Comment: return;
    case Verb::Help: printHelp(); return;
    case Verb::Status: printStatus(); return;
    case Verb::Stats: printStats(); return;
    case Verb::Enc: printEncoders(); return;
    case Verb::Joints: printJoints(); return;
    case Verb::Dump: printDump(); return;
    case Verb::Plot:
      if (cmd.flag && jointIndex(cmd.joint) < 0) {
        Serial.printf("error: no joint with servo id %u\n", cmd.joint);
        return;
      }
      plotJoint_ = cmd.flag ? cmd.joint : 0;
      Serial.printf("plot %s\n", plotJoint_ ? "on" : "off");
      return;
    default:
      if (!controller_.post(cmd, kPostWait)) Serial.println("error: control task busy, command dropped");
      return;
  }
}

void Console::printHelp() {
  Serial.println(
      "joints are addressed by servo id (1-9)\n"
      "  status | joints | enc | stats [reset]   inspect\n"
      "  monitor on|off       raw encoders + all servo positions at 10 Hz (disarmed)\n"
      "  ping                 scan servo ids (disarmed)\n"
      "  arm [<j>]            torque on: every enabled joint, or one joint while others follow\n"
      "  disarm [<j>] | x     torque off: all / one joint;  x = E-stop (BOOT button too)\n"
      "  set <j> enc <n|none> assign encoder #n      set <j> on|off   enable joint\n"
      "  cal <j> a|b          capture exo endpoint   cal <j> hand a|b servo endpoint (move by hand)\n"
      "  set <j> exo <rawA> <rawB> | servo <a> <b>   enter endpoints directly\n"
      "  set <j> vmax|amax|vsat|trk|grip <n>        per-joint speed / tracking / grip lead (0 = off)\n"
      "  set lead|mincut|beta|jump|gripload <v>     global parameters\n"
      "  set loadtrip on|off                        firmware load trips (off: servo overload decides)\n"
      "  save | load | defaults | dump              NVS store / reload / reset (RAM) / export\n"
      "  limits check | limits write <j|all>        servo angle-limit registers vs travel (disarmed)\n"
      "  plot <j> on | plot off                     Teleplot traces at 50 Hz");
}

void Console::printStatus() {
  controller_.snapshot(snapshot_);
  const Snapshot& s = snapshot_;
  Serial.printf("armed %u joint(s)  monitor %d  echo %d  torque-off confirmed %d\n", s.armedCount,
                s.monitor, s.echo, s.torqueOffConfirmed);
  if (s.lastHandFault != teleop::Fault::None) {
    Serial.printf("last hand-level fault: %s\n", teleop::faultToString(s.lastHandFault));
  }
  for (uint8_t i = 0; i < teleop::kMaxJoints; ++i) {
    const JointSnapshot& j = s.joints[i];
    if (!j.settings.enabled && j.state == JointState::Off) continue;
    const teleop::JointDefaults& d = config::kJoints[i];
    Serial.printf("j%u %-13s %-6s exo %7.2f deg  target %6.1f  goal %6.1f  sent %6.1f  grip %3.0f%s  spd %4u",
                  d.servoId, d.name, jointStateToString(j.state),
                  j.exoFilteredCounts * teleop::kDegreesPerCount, j.target, j.goal, j.sentGoal,
                  j.gripLead, j.contact ? "*" : " ", j.speedRegister);
    if (j.readAgeMs <= config::kServoCommStaleMs) {
      Serial.printf("  pos %4d tq %d load %5.1f%% %.1fV %dC%s\n", j.servo.position, j.servo.torqueOn,
                    j.servo.loadPct, j.servo.voltage, j.servo.temperatureC, j.encFresh ? "" : "  ENC!");
    } else {
      Serial.printf("  servo: no recent reply%s\n", j.encFresh ? "" : "  ENC!");
    }
  }
  const teleop::GlobalSettings& g = s.global;
  Serial.printf("global lead %.0f ms  mincut %.3f Hz  beta %.4f  jump %.0f deg  gripload %.0f %%  loadtrip %s\n",
                g.leadMs, g.minCutoffHz, g.beta, g.jumpDeg, g.gripLoadPct, g.loadTripEnabled ? "on" : "off");
  Serial.println("(grip * = finger in contact, lead shrinking to hold the squeeze near gripload)");
}

void Console::printJoints() {
  controller_.snapshot(snapshot_);
  Serial.println(
      "id name           on enc  exo span   servo A..B   travel     vmax amax  vsat trk  ready state  last fault");
  for (uint8_t i = 0; i < teleop::kMaxJoints; ++i) {
    const teleop::JointDefaults& d = config::kJoints[i];
    const JointSnapshot& j = snapshot_.joints[i];
    const teleop::JointSettings& s = j.settings;
    char enc[6];
    if (s.encoderIndex == teleop::kNoEncoder) {
      strlcpy(enc, "--", sizeof(enc));
    } else {
      snprintf(enc, sizeof(enc), "%u", s.encoderIndex);
    }
    char exo[12];
    if (teleop::exoCalibrated(s)) {
      snprintf(exo, sizeof(exo), "%6.1f", s.exoB * teleop::kDegreesPerCount);
    } else {
      strlcpy(exo, j.refCaptured ? "  A only" : "     --", sizeof(exo));
    }
    char servo[16];
    if (teleop::servoCalibrated(s, d)) {
      snprintf(servo, sizeof(servo), "%4d..%-4d", s.servoA, s.servoB);
    } else {
      strlcpy(servo, "    --    ", sizeof(servo));
    }
    char travel[12];
    if (teleop::travelDefined(d)) {
      snprintf(travel, sizeof(travel), "%4d..%-4d", d.travelMin, d.travelMax);
    } else {
      strlcpy(travel, " undefined", sizeof(travel));
    }
    Serial.printf("%2u %-14s %d  %-4s %-9s  %s  %s %4u %5u %4u %4u  %-5s %-6s %s\n", d.servoId,
                  d.name, s.enabled, enc, exo, servo, travel, s.vmax, s.amax, s.vsat, s.trackSteps,
                  j.ready ? "yes" : "no", jointStateToString(j.state),
                  j.lastFault == teleop::Fault::None ? "-" : teleop::faultToString(j.lastFault));
  }
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
  for (uint8_t i = 0; i < teleop::kMaxJoints; ++i) {
    const JointSnapshot& j = snapshot_.joints[i];
    if (j.settings.enabled && j.jumpRejects > 0) {
      Serial.printf("joint %u: %lu encoder samples rejected by the jump guard\n",
                    config::kJoints[i].servoId, (unsigned long)j.jumpRejects);
    }
  }
}

void Console::printEncoders() {
  controller_.snapshot(snapshot_);
  Serial.println("#  addr  state  valid  angle(deg)  agc  magnet  errors  joint");
  for (uint8_t e = 0; e < config::kEncoderCount; ++e) {
    const EncoderSummary& enc = snapshot_.encoders[e];
    int joint = 0;
    for (uint8_t i = 0; i < teleop::kMaxJoints; ++i) {
      if (snapshot_.joints[i].settings.encoderIndex == e) joint = config::kJoints[i].servoId;
    }
    Serial.printf("%u  0x%02X  %-5s  %d      %7.2f    %3u  %d       %-6lu  %s%d\n", e, enc.address,
                  encoderStateShort(enc.state), enc.valid, enc.angle * teleop::kDegreesPerCount,
                  enc.agc, enc.magnetOk, (unsigned long)enc.errors, joint ? "j" : "-", joint);
  }
}

void Console::printDump() {
  controller_.snapshot(snapshot_);
  teleop::JointSettings joints[teleop::kMaxJoints];
  for (uint8_t i = 0; i < teleop::kMaxJoints; ++i) joints[i] = snapshot_.joints[i].settings;
  const size_t n = teleop::formatDump(joints, config::kJoints, teleop::kMaxJoints, snapshot_.global,
                                      dumpBuffer, sizeof(dumpBuffer));
  if (n == 0) {
    Serial.println("error: dump buffer too small");
    return;
  }
  Serial.print(dumpBuffer);
}

void Console::printMonitorRow() {
  // Assumes snapshot_ is fresh (run() refreshes it right before calling).
  Serial.print("enc");
  for (uint8_t e = 0; e < config::kEncoderCount; ++e) {
    const EncoderSummary& enc = snapshot_.encoders[e];
    if (enc.state == AS5048BChannelState::Offline || !enc.valid) {
      Serial.printf(" %u:  ---  ", e);
    } else {
      Serial.printf(" %u:%6.1f%c", e, enc.angle * teleop::kDegreesPerCount,
                    enc.state == AS5048BChannelState::NoMagnet ? 'M' : ' ');
    }
  }
  Serial.print(" | servo");
  for (uint8_t i = 0; i < teleop::kMaxJoints; ++i) {
    if (snapshot_.monitorOk[i]) {
      Serial.printf(" %u:%4d", config::kJoints[i].servoId, snapshot_.monitorPosition[i]);
    } else {
      Serial.printf(" %u: -- ", config::kJoints[i].servoId);
    }
  }
  Serial.println();
}

void Console::plot() {
  const int i = jointIndex(plotJoint_);
  if (i < 0) return;
  controller_.snapshot(snapshot_);
  const JointSnapshot& j = snapshot_.joints[i];
  Serial.printf(">exo_deg:%.2f\n>exo_filt_deg:%.2f\n>target:%.1f\n>goal:%.1f\n>sent:%.1f\n>ref:%.1f\n"
                ">grip_lead:%.0f\n",
                j.exoCounts * teleop::kDegreesPerCount, j.exoFilteredCounts * teleop::kDegreesPerCount,
                j.target, j.goal, j.sentGoal, j.reference, j.gripLead);
  if (j.readAgeMs <= config::kServoCommStaleMs) {
    Serial.printf(">present:%d\n>load:%.1f\n>speed_reg:%u\n", j.servo.position, j.servo.loadPct,
                  j.speedRegister);
  }
}
