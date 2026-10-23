#include "console.h"

#include <command_parser.h>

#include "host_telemetry.h"

using teleop::Verb;
using hostlink::MsgType;

namespace {

constexpr uint32_t kPollMs = 5;
constexpr uint32_t kPlotPeriodMs = 20;  // 50 Hz Teleplot
constexpr TickType_t kPostWait = pdMS_TO_TICKS(200);
constexpr TickType_t kTelemetryWait = pdMS_TO_TICKS(2);  // host mode loop period at most
constexpr uint16_t kMinHeartbeatTimeoutMs = 200;
constexpr uint16_t kMaxHeartbeatTimeoutMs = 10000;

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
    out_.print("[ctl] ");
    out_.println(event.text);
  }
  // Text-mode commands carry no seq, so there are no completions to report.
  Completion done;
  while (controller_.popCompletion(done)) {
  }
}

void Console::run() {
  out_.println();
  out_.println("Teleoperation M2 ready. Type `help`.");
  for (;;) {
    if (hostMode_) {
      hostStep();
    } else {
      textStep();
    }
  }
}

void Console::textStep() {
  drainMessages();
  while (!hostMode_ && Serial.available() > 0) {
    const char c = static_cast<char>(Serial.read());
    if (c == '\r') continue;
    if (c == '\n') {
      line_[lineLength_] = '\0';
      handleLine(line_);
      lineLength_ = 0;
      if (!hostMode_) drainMessages();  // keep replies in step with pasted multi-line input
    } else if (lineLength_ + 1 < sizeof(line_)) {
      line_[lineLength_++] = c;
    }
  }
  if (hostMode_) return;
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

void Console::handleLine(const char* line) {
  teleop::ParsedCommand cmd;
  const char* error = teleop::parseCommand(line, cmd);
  if (error != nullptr) {
    out_.printf("error: %s\n", error);
    return;
  }
  if (cmd.verb == Verb::Host) {
    if (cmd.flag) {
      enterHostMode();
    } else {
      out_.println("host link is not active");
    }
    return;
  }
  if (runLocal(cmd)) return;
  if (!controller_.post(cmd, kPostWait)) out_.println("error: control task busy, command dropped");
}

bool Console::runLocal(const teleop::ParsedCommand& cmd) {
  switch (cmd.verb) {
    case Verb::None:
    case Verb::Comment: return true;
    case Verb::Help: printHelp(); return true;
    case Verb::Status: printStatus(); return true;
    case Verb::Stats: printStats(); return true;
    case Verb::Enc: printEncoders(); return true;
    case Verb::Joints: printJoints(); return true;
    case Verb::Dump: printDump(); return true;
    case Verb::Plot:
      if (cmd.flag && jointIndex(cmd.joint) < 0) {
        out_.printf("error: no joint with servo id %u\n", cmd.joint);
        return true;
      }
      plotJoint_ = cmd.flag ? cmd.joint : 0;
      out_.printf("plot %s\n", plotJoint_ ? "on" : "off");
      return true;
    default: return false;
  }
}

// ---- host link -----------------------------------------------------------------

void Console::logSink(void* context, const char* line, size_t length) {
  Console& c = *static_cast<Console*>(context);
  const size_t n = hostlink::encodeLog(hostlink::LogSource::Console, line, length, c.logPayload_,
                                       sizeof(c.logPayload_));
  if (n > 0) c.sendFrame(MsgType::Log, c.txSeq_++, c.logPayload_, n);
}

void Console::enterHostMode() {
  // Defaults until HOST_HELLO: no telemetry yet, 1 s heartbeat timeout.
  hostConfig_ = hostlink::HostHello();
  hostConfig_.telemetryDivider = 0;
  hostConfig_.slowDivider = 0;
  reader_.reset();
  plotJoint_ = 0;
  hostMode_ = true;
  lastRxMs_ = millis();
  out_.setLineSink(logSink, this);
  // Ends any text the host has not consumed yet, so it cannot run into HELLO.
  Serial.write(static_cast<uint8_t>(0));
  sendHello();
}

void Console::exitHostMode(hostlink::ByeReason reason) {
  const uint8_t bye = static_cast<uint8_t>(reason);
  sendFrame(MsgType::Bye, txSeq_++, &bye, 1);
  controller_.setTelemetryStream(false);
  out_.setLineSink(nullptr, nullptr);
  hostMode_ = false;
  lineLength_ = 0;
  if (reason == hostlink::ByeReason::HeartbeatTimeout) {
    controller_.snapshot(snapshot_);
    const bool stop = (hostConfig_.flags & hostlink::kHelloEstopOnLinkLoss) && snapshot_.armedCount > 0;
    if (stop) controller_.requestEstop("host link lost");
    out_.printf("[host] link lost%s, text console\n", stop ? ", torque off" : "");
  } else {
    out_.println("[host] link closed, text console");
  }
}

void Console::hostStep() {
  while (hostMode_ && Serial.available() > 0) {
    if (reader_.feed(static_cast<uint8_t>(Serial.read())) == hostlink::FrameReader::Result::Frame) {
      lastRxMs_ = millis();
      handleFrame(reader_.header(), reader_.payload(), reader_.payloadLength());
    }
  }
  if (!hostMode_) return;
  drainHostEvents();

  // Paced by the control loop: wait for the next sample, then send what is queued.
  if (controller_.popTelemetry(sample_, kTelemetryWait)) {
    do {
      sendTelemetry(sample_);
    } while (controller_.popTelemetry(sample_, 0));
  }

  const uint32_t now = millis();
  const uint32_t slowPeriodMs = hostConfig_.slowDivider * (1000 / config::kControlHz);
  if (slowPeriodMs > 0 && now - lastSlowMs_ >= slowPeriodMs) {
    lastSlowMs_ = now;
    sendSlowStatus();
  }
  if (now - lastRxMs_ > hostConfig_.heartbeatTimeoutMs) exitHostMode(hostlink::ByeReason::HeartbeatTimeout);
}

void Console::handleFrame(const hostlink::FrameHeader& header, const uint8_t* payload, size_t length) {
  const bool reply = header.flags & hostlink::kFlagReplyExpected;
  switch (static_cast<MsgType>(header.type)) {
    case MsgType::Heartbeat:
      if (reply) sendAck(header.seq, hostlink::AckStatus::Accepted);
      return;
    case MsgType::Estop:
      // Ahead of anything queued: the control task acts on its next cycle.
      controller_.requestEstop("E-stop (host)");
      sendAck(header.seq, hostlink::AckStatus::Accepted);
      return;
    case MsgType::HostHello: {
      hostlink::HostHello h;
      if (!hostlink::decodeHostHello(payload, length, h) || h.protocol != hostlink::kProtocolVersion) {
        sendAck(header.seq, hostlink::AckStatus::BadFrame, "unsupported protocol version");
        return;
      }
      h.heartbeatTimeoutMs = constrain(h.heartbeatTimeoutMs, kMinHeartbeatTimeoutMs, kMaxHeartbeatTimeoutMs);
      hostConfig_ = h;
      controller_.setTelemetryStream(h.telemetryDivider > 0);
      sendAck(header.seq, hostlink::AckStatus::Accepted);
      return;
    }
    case MsgType::TelemConfig: {
      hostlink::TelemConfig c;
      if (!hostlink::decodeTelemConfig(payload, length, c)) {
        sendAck(header.seq, hostlink::AckStatus::BadFrame, "bad TELEM_CONFIG");
        return;
      }
      hostConfig_.telemetryDivider = c.telemetryDivider;
      hostConfig_.slowDivider = c.slowDivider;
      controller_.setTelemetryStream(c.telemetryDivider > 0);
      sendAck(header.seq, hostlink::AckStatus::Accepted);
      return;
    }
    case MsgType::GetInfo: sendHello(); return;
    case MsgType::Command: handleHostCommand(header.seq, payload, length); return;
    case MsgType::HostExit:
      sendAck(header.seq, hostlink::AckStatus::Accepted);
      exitHostMode(hostlink::ByeReason::HostExit);
      return;
    default: sendAck(header.seq, hostlink::AckStatus::BadFrame, "unknown frame type"); return;
  }
}

// ACK says the line parsed and was queued (or why not); DONE follows once it
// has run, after every event it raised. Console-only verbs answer at once:
// their printed lines arrive as LOG frames, `dump` as one CONFIG frame.
void Console::handleHostCommand(uint16_t seq, const uint8_t* text, size_t length) {
  char line[sizeof(line_)];
  if (length >= sizeof(line)) {
    sendAck(seq, hostlink::AckStatus::ParseError, "command longer than 127 characters");
    return;
  }
  memcpy(line, text, length);
  line[length] = '\0';
  teleop::ParsedCommand cmd;
  const char* error = teleop::parseCommand(line, cmd);
  if (error != nullptr) {
    sendAck(seq, hostlink::AckStatus::ParseError, error);
    return;
  }
  switch (cmd.verb) {
    case Verb::Host:
      sendAck(seq, hostlink::AckStatus::Accepted);
      sendDone(seq, true);
      if (!cmd.flag) exitHostMode(hostlink::ByeReason::HostExit);
      return;
    case Verb::Plot:
      sendAck(seq, hostlink::AckStatus::Accepted);
      out_.println("plot: Teleplot output is not available on the host link; use the GUI plots");
      sendDone(seq, true);
      return;
    case Verb::Dump: {
      sendAck(seq, hostlink::AckStatus::Accepted);
      // Commands that finished just before may not be in the snapshot yet:
      // wait for the next published cycle.
      controller_.snapshot(snapshot_);
      const uint32_t cycle = snapshot_.cycle;
      for (int i = 0; i < 5 && snapshot_.cycle == cycle; ++i) {
        vTaskDelay(pdMS_TO_TICKS(5));
        controller_.snapshot(snapshot_);
      }
      const bool ok = formatDumpText();
      if (ok) {
        const size_t n = hostlink::encodeConfig(seq, hostlink::ConfigFormat::DumpText, dumpBuffer, payload_,
                                                sizeof(payload_));
        sendFrame(MsgType::Config, txSeq_++, payload_, n);
      } else {
        out_.println("error: dump buffer too small");
      }
      sendDone(seq, ok);
      return;
    }
    default: break;
  }
  if (runLocal(cmd)) {
    sendAck(seq, hostlink::AckStatus::Accepted);
    sendDone(seq, true);
    return;
  }
  if (!controller_.post(cmd, kPostWait, seq)) {
    sendAck(seq, hostlink::AckStatus::Busy, "control task busy, command dropped");
    return;
  }
  sendAck(seq, hostlink::AckStatus::Accepted);
}

void Console::drainHostEvents() {
  hostlink::Event event;
  const auto drainEvents = [&]() {
    while (controller_.popEvent(event)) {
      const size_t n = hostlink::encodeEvent(event, payload_, sizeof(payload_));
      sendFrame(MsgType::Event, txSeq_++, payload_, n);
    }
  };
  drainEvents();
  Completion done;
  while (controller_.popCompletion(done)) {
    drainEvents();  // its events were queued before it
    sendDone(done.seq, done.ok);
  }
}

void Console::sendTelemetry(const TelemetrySample& sample) {
  const uint32_t cycle = sample.bytes[0] | (sample.bytes[1] << 8) | (sample.bytes[2] << 16) |
                         (static_cast<uint32_t>(sample.bytes[3]) << 24);
  const uint8_t divider = hostConfig_.telemetryDivider;
  if (divider == 0 || cycle % divider != 0) return;
  sendFrame(MsgType::Telemetry, txSeq_++, sample.bytes, sample.length, true);
}

void Console::sendSlowStatus() {
  controller_.snapshot(snapshot_);
  fillSlowStatus(snapshot_, slow_);
  slow_.droppedTelemetry += droppedTx_;
  slow_.rxCrcErrors = reader_.stats().crcErrors;
  slow_.rxMalformed = reader_.stats().malformed + reader_.stats().overflows;
  const size_t n = hostlink::encodeSlowStatus(slow_, payload_, sizeof(payload_));
  sendFrame(MsgType::SlowStatus, txSeq_++, payload_, n, true);
}

void Console::sendHello() {
  const size_t n = hostlink::encodeHello(deviceInfo(), payload_, sizeof(payload_));
  sendFrame(MsgType::Hello, txSeq_++, payload_, n);
}

void Console::sendAck(uint16_t seq, hostlink::AckStatus status, const char* text) {
  uint8_t buf[160];
  const size_t n = hostlink::encodeAck(seq, status, text, buf, sizeof(buf));
  sendFrame(MsgType::Ack, txSeq_++, buf, n);
}

void Console::sendDone(uint16_t seq, bool ok) {
  uint8_t buf[4];
  const size_t n = hostlink::encodeDone(seq, ok, buf, sizeof(buf));
  sendFrame(MsgType::Done, txSeq_++, buf, n);
}

bool Console::sendFrame(MsgType type, uint16_t seq, const uint8_t* payload, size_t length, bool optional) {
  hostlink::FrameHeader header;
  header.type = static_cast<uint8_t>(type);
  header.seq = seq;
  const size_t n = hostlink::encodeFrame(header, payload, length, wire_, sizeof(wire_));
  if (n == 0) return false;
  if (optional && Serial.availableForWrite() < static_cast<int>(n)) {
    ++droppedTx_;
    return false;
  }
  return Serial.write(wire_, n) == n;
}

void Console::printHelp() {
  out_.println(
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
      "  plot <j> on | plot off                     Teleplot traces at 50 Hz\n"
      "  host on                                    framed link for the PC GUI (it sends this itself)");
}

void Console::printStatus() {
  controller_.snapshot(snapshot_);
  const Snapshot& s = snapshot_;
  out_.printf("armed %u joint(s)  monitor %d  echo %d  torque-off confirmed %d\n", s.armedCount,
                s.monitor, s.echo, s.torqueOffConfirmed);
  if (s.lastHandFault != teleop::Fault::None) {
    out_.printf("last hand-level fault: %s\n", teleop::faultToString(s.lastHandFault));
  }
  for (uint8_t i = 0; i < teleop::kMaxJoints; ++i) {
    const JointSnapshot& j = s.joints[i];
    if (!j.settings.enabled && j.state == JointState::Off) continue;
    const teleop::JointDefaults& d = config::kJoints[i];
    out_.printf("j%u %-13s %-6s exo %7.2f deg  target %6.1f  goal %6.1f  sent %6.1f  grip %3.0f%s  spd %5.0f",
                  d.servoId, d.name, jointStateToString(j.state),
                  j.exoFilteredCounts * teleop::kDegreesPerCount, j.target, j.goal, j.sentGoal,
                  j.gripLead, j.contact ? "*" : " ", j.speed);
    if (j.readAgeMs <= config::kServoCommStaleMs) {
      char current[32] = "";
      if (j.servo.hasCurrent) {
        snprintf(current, sizeof(current), " %5.0fmA(~%.0f in)", j.servo.currentMa, supplyEstimateMa(j.servo));
      }
      out_.printf("  pos %4d tq %d load %5.1f%%%s %.1fV %dC%s\n", j.servo.position, j.servo.torqueOn,
                    j.servo.loadPct, current, j.servo.voltage, j.servo.temperatureC,
                    !j.homed ? "  HOME!" : (j.encFresh ? "" : "  ENC!"));
    } else {
      out_.printf("  servo: no recent reply%s\n", !j.homed ? "  HOME!" : (j.encFresh ? "" : "  ENC!"));
    }
  }
  const teleop::GlobalSettings& g = s.global;
  out_.printf("global lead %.0f ms  mincut %.3f Hz  beta %.4f  jump %.0f deg  gripload %.0f %%  loadtrip %s\n",
                g.leadMs, g.minCutoffHz, g.beta, g.jumpDeg, g.gripLoadPct, g.loadTripEnabled ? "on" : "off");
  out_.println("(grip * = finger in contact, lead shrinking to hold the squeeze near gripload)");
}

void Console::printJoints() {
  controller_.snapshot(snapshot_);
  out_.println(
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
    out_.printf("%2u %-14s %-5s %d  %-4s %-9s  %s  %s %5u %5u %5u %4u  %-11s %-5s %s %-6s %s\n",
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
  out_.printf("loop %lu cycles, period last %lu us, jitter max %lu us p99 <= %lu us, overruns %lu\n",
                (unsigned long)t.cycles, (unsigned long)t.lastPeriodUs, (unsigned long)t.maxJitterUs,
                (unsigned long)t.p99JitterUs, (unsigned long)t.overruns);
  out_.printf("exec avg %lu us max %lu us (budget %lu us)\n", (unsigned long)t.averageExecUs(),
                (unsigned long)t.maxExecUs, (unsigned long)config::kControlPeriodUs);
  out_.printf("servo bus sent %lu ok %lu timeouts %lu parse errors %lu echo errors %lu, reply last %lu us max %lu us\n",
                (unsigned long)b.sent, (unsigned long)b.received, (unsigned long)b.timeouts,
                (unsigned long)b.parseErrors, (unsigned long)b.echoErrors,
                (unsigned long)b.lastReplyUs, (unsigned long)b.maxReplyUs);
  out_.printf("encoder pass avg %lu us max %lu us, skew max %lu us\n",
                (unsigned long)e.averageCycleUs(), (unsigned long)e.maxCycleUs,
                (unsigned long)e.maxSkewUs);
  for (uint8_t i = 0; i < teleop::kMaxJoints; ++i) {
    const JointSnapshot& j = snapshot_.joints[i];
    if (j.settings.enabled && j.jumpRejects > 0) {
      out_.printf("joint %u: %lu encoder samples rejected by the jump guard\n",
                    config::kJoints[i].servoId, (unsigned long)j.jumpRejects);
    }
  }
}

void Console::printEncoders() {
  controller_.snapshot(snapshot_);
  out_.println("#  addr  state  valid  angle(deg)  agc  magnet  errors  joint");
  for (uint8_t e = 0; e < config::kEncoderCount; ++e) {
    const EncoderSummary& enc = snapshot_.encoders[e];
    int joint = 0;
    for (uint8_t i = 0; i < teleop::kMaxJoints; ++i) {
      if (snapshot_.joints[i].settings.encoderIndex == e) joint = config::kJoints[i].servoId;
    }
    out_.printf("%u  0x%02X  %-5s  %d      %7.2f    %3u  %d       %-6lu  %s%d\n", e, enc.address,
                  encoderStateShort(enc.state), enc.valid, enc.angle * teleop::kDegreesPerCount,
                  enc.agc, enc.magnetOk, (unsigned long)enc.errors, joint ? "j" : "-", joint);
  }
}

void Console::printDump() {
  controller_.snapshot(snapshot_);
  if (!formatDumpText()) {
    out_.println("error: dump buffer too small");
    return;
  }
  out_.print(dumpBuffer);
}

bool Console::formatDumpText() {
  // Uses snapshot_ as the caller left it.
  teleop::JointSettings joints[teleop::kMaxJoints];
  for (uint8_t i = 0; i < teleop::kMaxJoints; ++i) joints[i] = snapshot_.joints[i].settings;
  return teleop::formatDump(joints, config::kJoints, teleop::kMaxJoints, snapshot_.global, dumpBuffer,
                            sizeof(dumpBuffer)) > 0;
}

void Console::printMonitorRow() {
  // Assumes snapshot_ is fresh (run() refreshes it right before calling).
  out_.print("enc");
  for (uint8_t e = 0; e < config::kEncoderCount; ++e) {
    const EncoderSummary& enc = snapshot_.encoders[e];
    if (enc.state == AS5048BChannelState::Offline || !enc.valid) {
      out_.printf(" %u:  ---  ", e);
    } else {
      out_.printf(" %u:%6.1f%c", e, enc.angle * teleop::kDegreesPerCount,
                    enc.state == AS5048BChannelState::NoMagnet ? 'M' : ' ');
    }
  }
  out_.print(" | servo");
  for (uint8_t i = 0; i < teleop::kMaxJoints; ++i) {
    if (snapshot_.monitorOk[i]) {
      out_.printf(" %u:%4d", config::kJoints[i].servoId, snapshot_.monitorPosition[i]);
    } else {
      out_.printf(" %u: -- ", config::kJoints[i].servoId);
    }
  }
  out_.println();
}

void Console::plot() {
  const int i = jointIndex(plotJoint_);
  if (i < 0) return;
  controller_.snapshot(snapshot_);
  const JointSnapshot& j = snapshot_.joints[i];
  out_.printf(">exo_deg:%.2f\n>exo_filt_deg:%.2f\n>target:%.1f\n>goal:%.1f\n>sent:%.1f\n>ref:%.1f\n"
                ">grip_lead:%.0f\n",
                j.exoCounts * teleop::kDegreesPerCount, j.exoFilteredCounts * teleop::kDegreesPerCount,
                j.target, j.goal, j.sentGoal, j.reference, j.gripLead);
  if (j.readAgeMs <= config::kServoCommStaleMs) {
    out_.printf(">present:%d\n>load:%.1f\n>speed_cmd:%.0f\n", j.servo.position, j.servo.loadPct,
                  j.speed);
    out_.printf(">temp_c:%d\n", j.servo.temperatureC);
    if (j.servo.hasCurrent) {
      out_.printf(">current_ma:%.0f\n>supply_est_ma:%.0f\n", j.servo.currentMa, supplyEstimateMa(j.servo));
      if (j.currentLimitMa > 0) out_.printf(">current_pct:%.1f\n", 100.0f * fabsf(j.servo.currentMa) / j.currentLimitMa);
    }
  }
}
