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

// Supply-side current of one servo: the H-bridge draws about phase current x
// drive duty (the load register is the duty), so a servo stalled at a 3 A cap
// may draw only a few hundred mA from the supply.
float supplyEstimateMa(const ServoState& s) { return fabsf(s.currentMa) * fabsf(s.loadPct) / 100.0f; }

int jointIndex(uint8_t servoId) {
  return teleop::jointIndexForServo(config::kJoints, teleop::kMaxJoints, servoId);
}

}  // namespace

void Console::startTask() {
  xTaskCreatePinnedToCore(taskEntry, "console", 8192, this, 1, nullptr, config::kConsoleCore);
}

void Console::taskEntry(void* arg) { static_cast<Console*>(arg)->run(); }

void Console::drainMessages() {
  hostlink::Event event;
  while (controller_.popEvent(event)) {
    Serial.print("[ctl] ");
    Serial.println(event.text);
  }
  // Text-mode commands carry no seq, so there are no completions to report.
  Completion done;
  while (controller_.popCompletion(done)) {
  }
}

void Console::run() {
  Serial.println();
  Serial.println("Teleoperation M2 ready. Type `help`.");
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
    case Verb::Host: Serial.println("error: host link not available in this build"); return;
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
      "  home <j>             multi-turn joint: glove held at pose A, fixes the turn count\n"
      "  set <j> exospan <rawA> <span> | servo <a> <b>  enter endpoints directly (dump format)\n"
      "  set <j> vmax|amax|vsat|trk|grip <n>        per-joint speed / tracking / grip lead (0 = off)\n"
      "  set <j> ilimit <mA>                        HLS goal-current cap (0 = servo protection current)\n"
      "  set <j> itrip <%>                          HLS over-current trip, % of cap (0 = off; not saved)\n"
      "  set lead|mincut|beta|jump|gripload <v>     global parameters\n"
      "  set loadtrip on|off                        firmware load trips (off: servo overload decides)\n"
      "  save | load | defaults | dump              NVS store / reload / reset (RAM) / export\n"
      "  limits check | limits write <j|all>        servo angle-limit registers vs travel (disarmed)\n"
      "  sweep <j> | sweep <j> apply                measure saturation speed, suggest vmax/vsat (disarmed)\n"
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
    Serial.printf("j%u %-13s %-6s exo %7.2f deg  target %6.1f  goal %6.1f  sent %6.1f  grip %3.0f%s  spd %5.0f",
                  d.servoId, d.name, jointStateToString(j.state),
                  j.exoFilteredCounts * teleop::kDegreesPerCount, j.target, j.goal, j.sentGoal,
                  j.gripLead, j.contact ? "*" : " ", j.speed);
    if (j.readAgeMs <= config::kServoCommStaleMs) {
      char current[32] = "";
      if (j.servo.hasCurrent) {
        snprintf(current, sizeof(current), " %5.0fmA(~%.0f in)", j.servo.currentMa, supplyEstimateMa(j.servo));
      }
      Serial.printf("  pos %4d tq %d load %5.1f%%%s %.1fV %dC%s\n", j.servo.position, j.servo.torqueOn,
                    j.servo.loadPct, current, j.servo.voltage, j.servo.temperatureC,
                    !j.homed ? "  HOME!" : (j.encFresh ? "" : "  ENC!"));
    } else {
      Serial.printf("  servo: no recent reply%s\n", !j.homed ? "  HOME!" : (j.encFresh ? "" : "  ENC!"));
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
      "id name           model on enc  exo span   servo A..B   travel     vmax  amax  vsat trk  ilimit/trip "
      "ready home  state  last fault");
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
      snprintf(exo, sizeof(exo), "%7.1f", s.exoB * teleop::kDegreesPerCount);
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
    // auto = single-turn, homes itself; homed / HOME! = multi-turn, needs `home <j>` at pose A
    const char* home = j.singleTurn ? (j.homed ? "auto " : "wait ") : (j.homed ? "homed" : "HOME!");
    // `?` = not identified on the bus yet / differs from config.h (cannot arm)
    char model[8];
    snprintf(model, sizeof(model), "%s%s", d.model == feetech::ServoModel::Hls3606 ? "HLS" : "SCS",
             j.modelOk ? "" : "?");
    // HLS: goal-current cap (`prot` = servo protection current) / over-current trip %
    char ilimit[16];
    if (d.model != feetech::ServoModel::Hls3606) {
      strlcpy(ilimit, "--", sizeof(ilimit));
    } else {
      char cap[8], trip[8];
      if (s.currentLimitMa == 0) {
        strlcpy(cap, "prot", sizeof(cap));
      } else {
        snprintf(cap, sizeof(cap), "%u", s.currentLimitMa);
      }
      if (s.currentTripPct == 0) {
        strlcpy(trip, "off", sizeof(trip));
      } else {
        snprintf(trip, sizeof(trip), "%u", s.currentTripPct);
      }
      snprintf(ilimit, sizeof(ilimit), "%s/%s", cap, trip);
    }
    Serial.printf("%2u %-14s %-5s %d  %-4s %-9s  %s  %s %5u %5u %5u %4u  %-11s %-5s %s %-6s %s\n",
                  d.servoId, d.name, model, s.enabled, enc, exo, servo, travel, s.vmax, s.amax, s.vsat,
                  s.trackSteps, ilimit, j.ready ? "yes" : "no", home, jointStateToString(j.state),
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
    Serial.printf(">present:%d\n>load:%.1f\n>speed_cmd:%.0f\n", j.servo.position, j.servo.loadPct,
                  j.speed);
    Serial.printf(">temp_c:%d\n", j.servo.temperatureC);
    if (j.servo.hasCurrent) {
      Serial.printf(">current_ma:%.0f\n>supply_est_ma:%.0f\n", j.servo.currentMa, supplyEstimateMa(j.servo));
      if (j.currentLimitMa > 0) Serial.printf(">current_pct:%.1f\n", 100.0f * fabsf(j.servo.currentMa) / j.currentLimitMa);
    }
  }
}
