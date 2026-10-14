#include "control_task.h"

#include <config_blob.h>
#include <esp_task_wdt.h>
#include <stdarg.h>

#include "config_store.h"

using teleop::Fault;
using teleop::FaultScope;
using teleop::JumpGuard;
using teleop::Verb;

namespace {

// The servo jumps to speed without a ramp (servo_bringup single-write check G),
// so the capability model only needs a token acceleration limit.
constexpr float kModelAcceleration = 20000.0f;

const char* notReadyReason(const teleop::JointSettings& s, const teleop::JointDefaults& d) {
  if (!teleop::travelDefined(d)) return "travel not defined in config.h";
  if (!s.enabled) return "disabled";
  if (s.encoderIndex >= config::kEncoderCount) return "no encoder assigned (set <j> enc <n>)";
  if (!teleop::exoCalibrated(s)) return "exo not calibrated (cal <j> a / b)";
  if (!teleop::servoCalibrated(s, d)) return "servo endpoints not set (cal <j> hand a / b)";
  return "ready";
}

}  // namespace

TeleopController* TeleopController::instance_ = nullptr;

const char* jointStateToString(JointState state) {
  switch (state) {
    case JointState::Off: return "OFF";
    case JointState::Align: return "ALIGN";
    case JointState::Follow: return "FOLLOW";
    case JointState::Hold: return "HOLD";
  }
  return "?";
}

TeleopController::TeleopController(ServoBus& bus, FeetechPort& port, AS5048BGroup& encoders)
    : bus_(bus), port_(port), encoders_(encoders) {
  global_ = config::kGlobalDefaults;
  for (uint8_t i = 0; i < teleop::kMaxJoints; ++i) {
    JointChannel& ch = channels_[i];
    ch.def = &config::kJoints[i];
    ch.settings = teleop::defaultSettings(*ch.def);
    ch.limiter = teleop::RateLimiter(ch.def->travelMin, ch.settings.vmax, ch.settings.amax,
                                     ch.def->travelMin, ch.def->travelMax);
    ch.model = ch.limiter;
    resetChannelRuntime(ch);
    applyRuntimeParams(ch);
  }
}

bool TeleopController::begin() {
  instance_ = this;
  commands_ = xQueueCreate(32, sizeof(Command));
  messages_ = xQueueCreate(32, sizeof(Message));
  if (commands_ == nullptr || messages_ == nullptr) return false;

  pinMode(config::kEstopPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(config::kEstopPin), estopIsr, FALLING);

  message("servo bus echo: %s", port_.echo() ? "detected, filtering enabled" : "none");
  loadConfig(true);
  torqueOffAll("boot");
  pingAll();
  publish();
  return true;
}

void TeleopController::startTask() {
  xTaskCreatePinnedToCore(taskEntry, "control", 8192, this, configMAX_PRIORITIES - 2, &task_,
                          config::kControlCore);
}

void TeleopController::taskEntry(void* arg) { static_cast<TeleopController*>(arg)->run(); }

void IRAM_ATTR TeleopController::estopIsr() {
  if (instance_ != nullptr) instance_->estopRequested_ = true;
}

bool TeleopController::post(const Command& command, TickType_t wait) {
  return xQueueSend(commands_, &command, wait) == pdTRUE;
}

void TeleopController::snapshot(Snapshot& out) {
  portENTER_CRITICAL(&mux_);
  out = shared_;
  portEXIT_CRITICAL(&mux_);
}

bool TeleopController::popMessage(char* buffer, size_t length) {
  Message msg;
  if (xQueueReceive(messages_, &msg, 0) != pdTRUE) return false;
  strlcpy(buffer, msg.text, length);
  return true;
}

void TeleopController::message(const char* format, ...) {
  if (messages_ == nullptr) return;
  Message msg;
  va_list args;
  va_start(args, format);
  vsnprintf(msg.text, sizeof(msg.text), format, args);
  va_end(args);
  xQueueSend(messages_, &msg, 0);  // drop rather than block the control loop
}

TeleopController::JointChannel* TeleopController::channelForServo(uint8_t servoId) {
  const int index = teleop::jointIndexForServo(config::kJoints, teleop::kMaxJoints, servoId);
  return index < 0 ? nullptr : &channels_[index];
}

uint8_t TeleopController::armedCount() const {
  uint8_t n = 0;
  for (const JointChannel& ch : channels_) n += ch.armed() ? 1 : 0;
  return n;
}

void TeleopController::resetChannelRuntime(JointChannel& ch) {
  ch.filter.reset();
  ch.guard.reset();
  ch.supervisor.reset();
  ch.refCaptured = teleop::exoCalibrated(ch.settings);
  ch.state = JointState::Off;
  ch.encBad = 0;
  ch.encFresh = false;
  ch.reportedFault = Fault::None;
}

void TeleopController::applyRuntimeParams(JointChannel& ch) {
  ch.filter.setParams(global_.minCutoffHz, global_.beta, config::kFilterDCutoffHz);
  ch.guard.setMaxStep(global_.jumpDeg);
  teleop::SupervisorConfig sc;
  sc.trackingErrorSteps = ch.settings.trackSteps;
  ch.supervisor.setConfig(sc);
  ch.limiter.setTravel(ch.def->travelMin, ch.def->travelMax);
  const float speed = ch.state == JointState::Align ? config::kArmSpeed : ch.settings.vmax;
  ch.limiter.setLimits(speed, ch.settings.amax);
  ch.model.setTravel(ch.def->travelMin, ch.def->travelMax);
  ch.model.setLimits(ch.settings.vsat, kModelAcceleration);
}

// ---- loop -------------------------------------------------------------------

void TeleopController::run() {
  esp_task_wdt_add(nullptr);
  const TickType_t period = pdMS_TO_TICKS(1000 / config::kControlHz);
  TickType_t lastWake = xTaskGetTickCount();
  for (;;) {
    vTaskDelayUntil(&lastWake, period);
    const uint32_t wakeUs = micros();
    updateTiming(wakeUs);
    cycle();
    const uint32_t execUs = micros() - wakeUs;
    timing_.lastExecUs = execUs;
    timing_.totalExecUs += execUs;
    if (execUs > timing_.maxExecUs) timing_.maxExecUs = execUs;
    if (execUs > config::kControlPeriodUs) ++timing_.overruns;
    publish();
    esp_task_wdt_reset();
  }
}

void TeleopController::updateTiming(uint32_t wakeUs) {
  if (lastWakeUs_ != 0) {
    const uint32_t periodUs = wakeUs - lastWakeUs_;
    const uint32_t jitter = periodUs > config::kControlPeriodUs ? periodUs - config::kControlPeriodUs
                                                                : config::kControlPeriodUs - periodUs;
    timing_.lastPeriodUs = periodUs;
    if (jitter > timing_.maxJitterUs) timing_.maxJitterUs = jitter;
    uint32_t bin = jitter / TimingStats::kBinUs;
    if (bin >= TimingStats::kBins) bin = TimingStats::kBins - 1;
    ++jitterHistogram_[bin];
    ++timing_.cycles;
  }
  lastWakeUs_ = wakeUs;
}

void TeleopController::cycle() {
  const float dt = 1.0f / config::kControlHz;
  const uint32_t nowMs = millis();
  ++cycle_;

  if (estopRequested_) {
    estopRequested_ = false;
    torqueOffAll("E-stop");
  }
  Command command;
  while (xQueueReceive(commands_, &command, 0) == pdTRUE) handleCommand(command);

  encoders_.sample(cycle_ % config::kFullEncoderReadEvery == 0);
  for (JointChannel& ch : channels_) {
    if (ch.settings.enabled && ch.settings.encoderIndex < config::kEncoderCount) {
      processEncoder(ch, dt);
    } else {
      ch.encFresh = false;
      ch.encBad = 0;
    }
  }

  readServos(nowMs);
  for (uint8_t i = 0; i < teleop::kMaxJoints; ++i) stepJoint(i, dt, nowMs);
  writeGoals(nowMs);
}

bool TeleopController::encoderSample(uint8_t index, uint16_t& raw) const {
  if (index >= encoders_.count()) return false;
  const AS5048BChannel& ch = encoders_.channel(index);
  if (ch.state != AS5048BChannelState::Online || ch.lastStatus != AS5048BStatus::Ok) return false;
  raw = ch.reading.angle;
  return true;
}

void TeleopController::processEncoder(JointChannel& ch, float dt) {
  uint16_t raw = 0;
  if (!encoderSample(ch.settings.encoderIndex, raw)) {
    ch.encFresh = false;
    if (ch.encBad < 0xFF) ++ch.encBad;
    return;
  }
  const float deg = teleop::unwrapCounts(raw, ch.settings.exoRef) * teleop::kDegreesPerCount;
  const JumpGuard::Result guard = ch.guard.check(deg);
  if (guard != JumpGuard::Result::Accepted) {
    ++ch.jumpRejects;
    ch.encFresh = false;
    if (ch.encBad < 0xFF) ++ch.encBad;
    if (guard == JumpGuard::Result::Resynced) {
      ch.filter.reset();
      message("joint %u: encoder jumped > %.0f deg per cycle 3 times, re-synced", ch.def->servoId,
              global_.jumpDeg);
    }
    return;
  }

  ch.encFresh = true;
  ch.encBad = 0;
  ch.encRaw = raw;
  ch.exoCounts = deg / teleop::kDegreesPerCount;
  ch.exoFiltered = ch.filter.filter(deg, dt) / teleop::kDegreesPerCount;
  if (!teleop::exoCalibrated(ch.settings)) return;

  const teleop::JointMap map = teleop::toJointMap(ch.settings, *ch.def);
  ch.target = teleop::mapToServo(map, ch.exoFiltered);
  if (global_.leadMs > 0.0f) {
    const float slope = static_cast<float>(map.servoB - map.servoA) / static_cast<float>(map.exoB);
    const float servoVelocity = ch.filter.derivative() / teleop::kDegreesPerCount * slope;
    const float lo = min(map.servoA, map.servoB);
    const float hi = max(map.servoA, map.servoB);
    ch.target = constrain(ch.target + servoVelocity * global_.leadMs * 1e-3f, lo, hi);
  }
}

void TeleopController::readMonitor() {
  // One servo per cycle so a full sweep of the hand costs ~0.6 ms per cycle.
  const uint8_t i = monitorIndex_;
  monitorIndex_ = static_cast<uint8_t>((monitorIndex_ + 1) % teleop::kMaxJoints);
  JointChannel& ch = channels_[i];
  ServoState st;
  monitorOk_[i] = bus_.readState(ch.def->servoId, st);
  if (monitorOk_[i]) {
    monitorPosition_[i] = st.position;
    ch.servo = st;
  }
  ch.servoOk = monitorOk_[i];
}

void TeleopController::readServos(uint32_t nowMs) {
  for (JointChannel& ch : channels_) ch.readThisCycle = false;
  if (monitor_ && armedCount() == 0) {
    readMonitor();
    return;
  }

  // Round-robin over enabled (or armed) joints, at most kServoReadsPerCycle per cycle.
  uint8_t candidates[teleop::kMaxJoints];
  uint8_t n = 0;
  for (uint8_t i = 0; i < teleop::kMaxJoints; ++i) {
    if (channels_[i].settings.enabled || channels_[i].armed()) candidates[n++] = i;
  }
  const uint8_t reads = n < config::kServoReadsPerCycle ? n : config::kServoReadsPerCycle;

  Fault handFault = Fault::None;
  int handJoint = -1;
  Fault jointFaults[teleop::kMaxJoints];
  for (Fault& f : jointFaults) f = Fault::None;

  for (uint8_t k = 0; k < reads; ++k) {
    const uint8_t i = candidates[(readCursor_ + k) % n];
    JointChannel& ch = channels_[i];
    ServoState st;
    ch.readThisCycle = true;
    ch.servoOk = bus_.readState(ch.def->servoId, st);
    if (ch.servoOk) {
      ch.servo = st;
      ch.lastGoodReadMs = nowMs;
    }
    teleop::ServoSample sample;
    sample.readOk = ch.servoOk;
    sample.torqueOn = st.torqueOn;
    sample.position = st.position;
    sample.loadPct = st.loadPct;
    sample.voltage = st.voltage;
    sample.temperatureC = st.temperatureC;
    sample.errorByte = st.errorByte;
    sample.status = st.status;
    const Fault fault = ch.supervisor.update(sample, ch.armed(), ch.goal, ch.reference, nowMs);

    if (fault != Fault::None && !ch.armed()) {
      // Torque on while the firmware has the joint off: switch it off again.
      if (fault == Fault::TorqueMismatch) bus_.writeTorque(ch.def->servoId, false);
      if (fault != ch.reportedFault) {
        message("joint %u idle fault: %s", ch.def->servoId, teleop::faultToString(fault));
      }
    }
    ch.reportedFault = fault;
    if (fault == Fault::None) continue;
    if (teleop::faultScope(fault) == FaultScope::Hand) {
      if (handFault == Fault::None) {
        handFault = fault;
        handJoint = i;
      }
    } else if (ch.armed()) {
      jointFaults[i] = fault;
    }
  }
  if (n > 0) readCursor_ = static_cast<uint8_t>((readCursor_ + reads) % n);

  // Bus silence on an armed joint; with round-robin a joint is read every ~30 ms.
  float voltages[teleop::kMaxJoints];
  uint8_t voltageCount = 0;
  for (uint8_t i = 0; i < teleop::kMaxJoints; ++i) {
    const JointChannel& ch = channels_[i];
    const uint32_t age = nowMs - ch.lastGoodReadMs;
    if (ch.armed() && age > config::kServoCommStaleMs && handFault == Fault::None) {
      handFault = Fault::CommLost;
      handJoint = i;
    }
    if (ch.settings.enabled && age <= config::kServoCommStaleMs) voltages[voltageCount++] = ch.servo.voltage;
  }
  const Fault supply = supplySupervisor_.update(voltages, voltageCount, nowMs);
  if (supply != Fault::None && handFault == Fault::None) {
    handFault = supply;
    handJoint = -1;
  }
  if (supply != reportedSupplyFault_ && supply != Fault::None && armedCount() == 0) {
    message("idle fault: %s", teleop::faultToString(supply));
  }
  reportedSupplyFault_ = supply;

  if (handFault != Fault::None && armedCount() > 0) {
    disarmAll(handFault, handJoint, teleop::faultToString(handFault));
    return;
  }
  for (uint8_t i = 0; i < teleop::kMaxJoints; ++i) {
    if (jointFaults[i] != Fault::None) disarmJoint(i, jointFaults[i], teleop::faultToString(jointFaults[i]));
  }
}

void TeleopController::stepJoint(uint8_t index, float dt, uint32_t nowMs) {
  JointChannel& ch = channels_[index];
  const uint8_t id = ch.def->servoId;
  const bool encFault = ch.encBad >= config::kEncoderFailCycles;

  switch (ch.state) {
    case JointState::Off:
      if (ch.readThisCycle && ch.servoOk) {
        ch.limiter.reset(ch.servo.position);
        ch.model.reset(ch.servo.position);
        ch.goal = ch.reference = ch.servo.position;
      }
      return;

    case JointState::Align:
      if (encFault) {
        disarmJoint(index, Fault::None, "encoder lost during ALIGN");
        return;
      }
      ch.limiter.setTarget(ch.target);
      ch.goal = ch.limiter.step(dt);
      if (fabsf(ch.goal - ch.target) <= config::kAlignTolerance) {
        ch.state = JointState::Follow;
        ch.limiter.setLimits(ch.settings.vmax, ch.settings.amax);
        message("joint %u aligned -> FOLLOW", id);
      }
      break;

    case JointState::Follow:
      if (encFault) {
        ch.state = JointState::Hold;
        ch.holdSinceMs = nowMs;
        ch.encGoodTiming = false;
        ch.limiter.setTarget(ch.limiter.position());
        message("joint %u encoder fault -> HOLD", id);
      } else {
        ch.limiter.setTarget(ch.target);
      }
      ch.goal = ch.limiter.step(dt);
      break;

    case JointState::Hold:
      ch.goal = ch.limiter.step(dt);
      if (ch.encFresh) {
        if (!ch.encGoodTiming) {
          ch.encGoodTiming = true;
          ch.encGoodSinceMs = nowMs;
        } else if (nowMs - ch.encGoodSinceMs >= config::kHoldRecoverMs) {
          ch.state = JointState::Align;
          ch.limiter.setLimits(config::kArmSpeed, ch.settings.amax);
          message("joint %u encoder back -> ALIGN", id);
          break;
        }
      } else {
        ch.encGoodTiming = false;
      }
      if (nowMs - ch.holdSinceMs > config::kHoldTimeoutMs) {
        disarmJoint(index, Fault::None, "encoder lost > 3 s");
        return;
      }
      break;
  }

  // Capability model for the tracking check: chases the goal at vsat.
  ch.model.setTarget(ch.goal);
  ch.reference = ch.model.step(dt);
}

void TeleopController::writeGoals(uint32_t nowMs) {
  uint8_t ids[teleop::kMaxJoints];
  int16_t positions[teleop::kMaxJoints];
  uint16_t speeds[teleop::kMaxJoints];
  uint8_t n = 0;
  for (JointChannel& ch : channels_) {
    ch.goalSent = false;
    // Only to armed joints whose torque a recent read showed on: a goal write
    // to a disarmed SCS0009 would switch its torque on by itself.
    if (!ch.armed() || !ch.servo.torqueOn || nowMs - ch.lastGoodReadMs > config::kServoStaleMs) continue;
    ch.speedRegister = streamSpeed(ch.limiter.velocity());
    ids[n] = ch.def->servoId;
    positions[n] = static_cast<int16_t>(lroundf(ch.goal));
    speeds[n] = ch.speedRegister;
    ch.goalSent = true;
    ++n;
  }
  if (n > 0) bus_.syncWriteGoal(ids, positions, speeds, n);
}

uint16_t TeleopController::streamSpeed(float velocity) const {
  const float wanted = ceilf(config::kStreamSpeedGain * fabsf(velocity));
  float speed = max(static_cast<float>(config::kStreamSpeedFloor), wanted);
  speed = min(speed, static_cast<float>(config::kStreamSpeedCap));
  return static_cast<uint16_t>(speed);
}

// ---- commands -----------------------------------------------------------------

void TeleopController::handleCommand(const Command& c) {
  if (armedCount() > 0 && teleop::requiresDisarmed(c)) {
    message("refused while joints are armed: disarm first");
    return;
  }
  switch (c.verb) {
    case Verb::Arm:
      if (c.joint == 0) {
        armAll();
      } else {
        armOne(c.joint);
      }
      break;
    case Verb::Disarm:
      if (c.joint == 0) {
        disarmAll(Fault::None, -1, "operator");
      } else {
        const int i = teleop::jointIndexForServo(config::kJoints, teleop::kMaxJoints, c.joint);
        if (i < 0) {
          message("no joint with servo id %u", c.joint);
        } else if (!channels_[i].armed()) {
          message("joint %u is not armed", c.joint);
        } else {
          disarmJoint(i, Fault::None, "operator");
        }
      }
      break;
    case Verb::Estop: torqueOffAll("E-stop (console)"); break;
    case Verb::Ping: pingAll(); break;
    case Verb::StatsReset:
      timing_ = TimingStats();
      memset(jitterHistogram_, 0, sizeof(jitterHistogram_));
      lastWakeUs_ = 0;
      port_.clearStats();
      encoders_.clearStats();
      for (JointChannel& ch : channels_) ch.jumpRejects = 0;
      message("statistics cleared");
      break;
    case Verb::Monitor:
      monitor_ = c.flag;
      message("monitor %s", monitor_ ? "on" : "off");
      break;
    case Verb::Cal: handleCal(c); break;
    case Verb::CalHand: handleCalHand(c); break;
    case Verb::Set: handleSet(c); break;
    case Verb::Save: saveConfig(); break;
    case Verb::Load: loadConfig(false); break;
    case Verb::Defaults: loadDefaults(); break;
    default: break;  // console-only verbs never reach the controller
  }
}

void TeleopController::handleSet(const Command& c) {
  teleop::JointSettings joints[teleop::kMaxJoints];
  for (uint8_t i = 0; i < teleop::kMaxJoints; ++i) joints[i] = channels_[i].settings;
  teleop::GlobalSettings global = global_;
  const char* error =
      teleop::applySetting(c, joints, config::kJoints, teleop::kMaxJoints, global, config::kEncoderCount);
  if (error != nullptr) {
    message("set rejected: %s", error);
    return;
  }
  global_ = global;
  for (uint8_t i = 0; i < teleop::kMaxJoints; ++i) {
    JointChannel& ch = channels_[i];
    const teleop::JointSettings& next = joints[i];
    const bool mappingChanged = next.encoderIndex != ch.settings.encoderIndex ||
                                next.exoRef != ch.settings.exoRef || next.exoB != ch.settings.exoB;
    ch.settings = next;
    if (mappingChanged) {
      ch.filter.reset();
      ch.guard.reset();
      ch.refCaptured = teleop::exoCalibrated(ch.settings);
    }
    applyRuntimeParams(ch);
  }
  if (teleop::isGlobalKey(c.key)) {
    message("global: lead %.0f ms, mincut %.3f Hz, beta %.4f, jump %.0f deg", global_.leadMs,
            global_.minCutoffHz, global_.beta, global_.jumpDeg);
  } else if (JointChannel* ch = channelForServo(c.joint)) {
    const teleop::JointSettings& s = ch->settings;
    message("joint %u: %s enc %d exo %d servo %d..%d vmax %u amax %u vsat %u trk %u", c.joint,
            s.enabled ? "on" : "off", s.encoderIndex == teleop::kNoEncoder ? -1 : s.encoderIndex,
            s.exoB, s.servoA, s.servoB, s.vmax, s.amax, s.vsat, s.trackSteps);
  }
}

void TeleopController::handleCal(const Command& c) {
  JointChannel* ch = channelForServo(c.joint);
  if (ch == nullptr) {
    message("no joint with servo id %u", c.joint);
    return;
  }
  if (ch->settings.encoderIndex >= config::kEncoderCount) {
    message("joint %u: assign an encoder first (set %u enc <n>)", c.joint, c.joint);
    return;
  }
  uint16_t raw = 0;
  if (!encoderSample(ch->settings.encoderIndex, raw)) {
    message("joint %u: encoder %u not healthy, capture refused", c.joint, ch->settings.encoderIndex);
    return;
  }
  if (!c.flag) {
    ch->settings.exoRef = raw;
    ch->settings.exoB = 0;  // B is relative to the new reference: capture it again
    ch->refCaptured = true;
    ch->filter.reset();
    ch->guard.reset();
    message("joint %u: exo A captured (raw %u). Move to the other end and run `cal %u b`.", c.joint,
            raw, c.joint);
    return;
  }
  if (!ch->refCaptured) {
    message("joint %u: run `cal %u a` first", c.joint, c.joint);
    return;
  }
  const int32_t span = teleop::unwrapCounts(raw, ch->settings.exoRef);
  if (abs(span) < teleop::kMinExoSpanCounts) {
    message("joint %u: span %.1f deg is too small, rejected", c.joint, span * teleop::kDegreesPerCount);
    return;
  }
  ch->settings.exoB = static_cast<int16_t>(span);
  ch->filter.reset();
  ch->guard.reset();
  message("joint %u: exo B captured, span %.1f deg", c.joint, span * teleop::kDegreesPerCount);
}

void TeleopController::handleCalHand(const Command& c) {
  JointChannel* ch = channelForServo(c.joint);
  if (ch == nullptr) {
    message("no joint with servo id %u", c.joint);
    return;
  }
  if (!teleop::travelDefined(*ch->def)) {
    message("joint %u: travel not defined in config.h; measure it with `monitor on` first", c.joint);
    return;
  }
  ServoState st;
  if (!bus_.readState(c.joint, st)) {
    message("joint %u: no reply from servo", c.joint);
    return;
  }
  if (st.torqueOn) {
    bus_.writeTorque(c.joint, false);
    message("joint %u: servo had torque on, switched off; move the finger by hand and retry", c.joint);
    return;
  }
  if (st.position < ch->def->travelMin || st.position > ch->def->travelMax) {
    message("joint %u: position %d outside travel [%d, %d], rejected", c.joint, st.position,
            ch->def->travelMin, ch->def->travelMax);
    return;
  }
  (c.flag ? ch->settings.servoB : ch->settings.servoA) = st.position;
  message("joint %u: servo %c = %d%s", c.joint, c.flag ? 'B' : 'A', st.position,
          ch->settings.servoA == ch->settings.servoB ? " (A = B: capture the other end)" : "");
}

void TeleopController::saveConfig() {
  teleop::JointSettings joints[teleop::kMaxJoints];
  for (uint8_t i = 0; i < teleop::kMaxJoints; ++i) joints[i] = channels_[i].settings;
  uint8_t blob[teleop::kBlobSize];
  const size_t n =
      teleop::packConfig(joints, teleop::kMaxJoints, global_, config::kServoBackend, blob, sizeof(blob));
  const bool ok = n > 0 && config_store::save(blob, n);
  message(ok ? "configuration saved to NVS" : "SAVE FAILED");
}

bool TeleopController::loadConfig(bool atBoot) {
  uint8_t blob[teleop::kBlobSize + 16];
  const size_t length = config_store::load(blob, sizeof(blob));
  if (length == 0) {
    message("no stored configuration%s", atBoot ? ", using defaults" : "");
    return false;
  }
  teleop::JointSettings joints[teleop::kMaxJoints];
  teleop::GlobalSettings global;
  uint8_t badJoint = 0xFF;
  const teleop::BlobStatus status = teleop::unpackConfig(
      blob, length, config::kServoBackend, config::kJoints, teleop::kMaxJoints, config::kEncoderCount,
      config::kGlobalDefaults, joints, global, &badJoint);
  if (!teleop::blobApplied(status)) {
    message("stored configuration rejected (%s), %s", teleop::blobStatusToString(status),
            atBoot ? "using defaults" : "keeping current settings");
    return false;
  }
  global_ = global;
  for (uint8_t i = 0; i < teleop::kMaxJoints; ++i) {
    channels_[i].settings = joints[i];
    resetChannelRuntime(channels_[i]);
    applyRuntimeParams(channels_[i]);
  }
  if (status == teleop::BlobStatus::Ok) {
    message("configuration loaded from NVS");
  } else if (badJoint < teleop::kMaxJoints) {
    message("configuration loaded with corrections: %s (first: joint %u)",
            teleop::blobStatusToString(status), config::kJoints[badJoint].servoId);
  } else {
    message("configuration loaded: %s; run `save` to store it in the new format",
            teleop::blobStatusToString(status));
  }
  return true;
}

void TeleopController::loadDefaults() {
  global_ = config::kGlobalDefaults;
  for (JointChannel& ch : channels_) {
    ch.settings = teleop::defaultSettings(*ch.def);
    resetChannelRuntime(ch);
    applyRuntimeParams(ch);
  }
  message("defaults loaded (not saved; run `save` to keep them)");
}

void TeleopController::pingAll() {
  for (const JointChannel& ch : channels_) {
    const uint8_t id = ch.def->servoId;
    uint16_t model = 0;
    if (bus_.ping(id, &model)) {
      message("servo %u (%s): model %u%s", id, ch.def->name, model,
              model == bus_.expectedModel() ? "" : " (unexpected)");
    } else {
      message("servo %u (%s): no reply", id, ch.def->name);
    }
  }
}

// ---- arming -------------------------------------------------------------------

bool TeleopController::preflight(uint8_t index, ServoState& st) {
  JointChannel& ch = channels_[index];
  const uint8_t id = ch.def->servoId;
  if (!teleop::jointReady(ch.settings, *ch.def, config::kEncoderCount)) {
    message("arm refused: joint %u %s", id, notReadyReason(ch.settings, *ch.def));
    return false;
  }
  if (!ch.encFresh) {
    message("arm refused: joint %u encoder not healthy", id);
    return false;
  }
  if (!bus_.readState(id, st)) {
    message("arm refused: no status reply from servo %u", id);
    return false;
  }
  if (st.errorByte != 0 || st.status != 0) {
    message("arm refused: servo %u error byte 0x%02X status 0x%02X", id, st.errorByte, st.status);
    return false;
  }
  if (st.position < ch.def->travelMin || st.position > ch.def->travelMax) {
    message("arm refused: servo %u at %d outside travel [%d, %d]; move it back by hand", id,
            st.position, ch.def->travelMin, ch.def->travelMax);
    return false;
  }
  return true;
}

const char* TeleopController::armJoint(uint8_t index, const ServoState& st, uint32_t nowMs) {
  JointChannel& ch = channels_[index];
  const uint8_t id = ch.def->servoId;
  // Goal before torque: a goal write switches torque on by itself on SCS0009,
  // so the joint may only ever be told to hold where it already is.
  const int16_t hold = st.position;
  ServoState check;
  if (!bus_.writeGoal(id, hold, config::kArmSpeed)) return "goal write failed";
  if (!bus_.writeTorque(id, true)) return "torque write failed";
  if (!bus_.readState(id, check)) return "no read-back after arming";
  if (!check.torqueOn) return "torque did not switch on";
  if (check.goal != hold) return "goal register does not match the hold position";
  if (abs(check.position - hold) > config::kArmHoldTolerance) return "joint moved while arming";

  ch.state = JointState::Align;
  ch.lastFault = Fault::None;
  ch.servo = check;
  ch.servoOk = true;
  ch.lastGoodReadMs = nowMs;
  ch.limiter.reset(hold);
  ch.limiter.setLimits(config::kArmSpeed, ch.settings.amax);
  ch.model.reset(hold);
  ch.goal = ch.reference = hold;
  ch.supervisor.reset();
  return nullptr;
}

void TeleopController::armAll() {
  ServoState states[teleop::kMaxJoints];
  uint8_t toArm = 0;
  for (uint8_t i = 0; i < teleop::kMaxJoints; ++i) {
    JointChannel& ch = channels_[i];
    if (!ch.settings.enabled || ch.armed()) continue;
    if (!preflight(i, states[i])) return;
    ++toArm;
  }
  if (toArm == 0) {
    message(armedCount() > 0 ? "every enabled joint is already armed"
                             : "arm refused: no joint enabled (set <j> on)");
    return;
  }

  monitor_ = false;
  const uint32_t nowMs = millis();
  for (uint8_t i = 0; i < teleop::kMaxJoints; ++i) {
    JointChannel& ch = channels_[i];
    if (!ch.settings.enabled || ch.armed()) continue;
    const char* problem = armJoint(i, states[i], nowMs);
    if (problem != nullptr) {
      disarmAll(Fault::None, i, problem);
      return;
    }
  }
  if (armedCount() == toArm) supplySupervisor_.reset();
  lastHandFault_ = Fault::None;
  message("armed %u joint(s), holding -> ALIGN (keep the glove still)", toArm);
}

void TeleopController::armOne(uint8_t servoId) {
  const int i = teleop::jointIndexForServo(config::kJoints, teleop::kMaxJoints, servoId);
  if (i < 0) {
    message("no joint with servo id %u", servoId);
    return;
  }
  JointChannel& ch = channels_[i];
  if (ch.armed()) {
    message("joint %u is already armed", servoId);
    return;
  }
  if (!ch.settings.enabled) {
    message("arm refused: joint %u disabled (set %u on)", servoId, servoId);
    return;
  }
  ServoState st;
  if (!preflight(i, st)) return;
  monitor_ = false;
  const char* problem = armJoint(i, st, millis());
  if (problem != nullptr) {
    disarmJoint(i, Fault::None, problem);
    return;
  }
  message("joint %u armed, holding -> ALIGN", servoId);
}

bool TeleopController::switchOff(const uint8_t* ids, uint8_t count) {
  if (count == 0) return true;
  bus_.syncTorqueOff(ids, count);
  bool confirmed = true;
  for (uint8_t i = 0; i < count; ++i) {
    bus_.writeTorque(ids[i], false);
    ServoState check;
    confirmed &= bus_.readState(ids[i], check) && !check.torqueOn;
  }
  return confirmed;
}

void TeleopController::disarmJoint(uint8_t index, Fault fault, const char* reason) {
  JointChannel& ch = channels_[index];
  ch.state = JointState::Off;  // first, so no goal can follow
  const uint8_t id = ch.def->servoId;
  const bool confirmed = switchOff(&id, 1);
  ch.supervisor.reset();
  ch.lastFault = fault;
  message("joint %u (%s) DISARM: %s; torque off %s; other joints unaffected", id, ch.def->name, reason,
          confirmed ? "confirmed" : "NOT CONFIRMED");
}

void TeleopController::disarmAll(Fault fault, int jointIndex, const char* reason) {
  uint8_t ids[teleop::kMaxJoints];
  uint8_t n = 0;
  for (JointChannel& ch : channels_) {
    if (ch.armed() || ch.settings.enabled) ids[n++] = ch.def->servoId;
    ch.state = JointState::Off;  // first, so no goal can follow
  }
  const bool confirmed = switchOff(ids, n);
  for (JointChannel& ch : channels_) ch.supervisor.reset();
  torqueOffConfirmed_ = confirmed;
  lastHandFault_ = fault;
  if (jointIndex >= 0) {
    channels_[jointIndex].lastFault = fault;
    message("DISARM ALL: %s (joint %u %s); torque off %s", reason, channels_[jointIndex].def->servoId,
            channels_[jointIndex].def->name, confirmed ? "confirmed" : "NOT CONFIRMED");
  } else {
    message("DISARM ALL: %s; torque off %s", reason, confirmed ? "confirmed" : "NOT CONFIRMED");
  }
}

void TeleopController::torqueOffAll(const char* reason) {
  for (JointChannel& ch : channels_) ch.state = JointState::Off;
  uint8_t ids[teleop::kMaxJoints];
  for (uint8_t i = 0; i < teleop::kMaxJoints; ++i) ids[i] = config::kJoints[i].servoId;
  bus_.syncTorqueOff(ids, teleop::kMaxJoints);
  bool allConfirmed = true;
  for (const uint8_t id : ids) {
    ServoState check;
    if (!bus_.readState(id, check)) continue;  // absent servo: nothing to switch off
    if (check.torqueOn) {
      bus_.writeTorque(id, false);
      allConfirmed &= bus_.readState(id, check) && !check.torqueOn;
    }
  }
  for (JointChannel& ch : channels_) ch.supervisor.reset();
  torqueOffConfirmed_ = allConfirmed;
  message("%s: torque off on all servos%s", reason, allConfirmed ? "" : " (NOT CONFIRMED)");
}

// ---- snapshot -----------------------------------------------------------------

void TeleopController::publish() {
  if (timing_.cycles > 0 && cycle_ % 50 == 0) {
    const uint32_t threshold = timing_.cycles - timing_.cycles / 100;
    uint32_t count = 0;
    for (uint16_t i = 0; i < TimingStats::kBins; ++i) {
      count += jitterHistogram_[i];
      if (count >= threshold) {
        timing_.p99JitterUs = (i + 1u) * TimingStats::kBinUs;
        break;
      }
    }
  }

  const uint32_t nowMs = millis();
  Snapshot& s = staging_;
  s.cycle = cycle_;
  s.uptimeMs = nowMs;
  s.armedCount = armedCount();
  s.lastHandFault = lastHandFault_;
  s.torqueOffConfirmed = torqueOffConfirmed_;
  s.echo = port_.echo();
  s.monitor = monitor_;
  s.global = global_;
  for (uint8_t i = 0; i < teleop::kMaxJoints; ++i) {
    const JointChannel& ch = channels_[i];
    JointSnapshot& j = s.joints[i];
    j.settings = ch.settings;
    j.state = ch.state;
    j.lastFault = ch.lastFault;
    j.ready = teleop::jointReady(ch.settings, *ch.def, config::kEncoderCount);
    j.refCaptured = ch.refCaptured;
    j.encFresh = ch.encFresh;
    j.encRaw = ch.encRaw;
    j.exoCounts = ch.exoCounts;
    j.exoFilteredCounts = ch.exoFiltered;
    j.target = ch.target;
    j.goal = ch.goal;
    j.reference = ch.reference;
    j.speedRegister = ch.speedRegister;
    j.goalSent = ch.goalSent;
    j.servoOk = ch.servoOk;
    j.readAgeMs = nowMs - ch.lastGoodReadMs;
    j.servo = ch.servo;
    j.jumpRejects = ch.jumpRejects;
    s.monitorPosition[i] = monitorPosition_[i];
    s.monitorOk[i] = monitorOk_[i];
  }
  s.timing = timing_;
  s.port = port_.stats();
  s.encoderCycle = encoders_.stats();
  for (uint8_t i = 0; i < config::kEncoderCount && i < encoders_.count(); ++i) {
    const AS5048BChannel& ch = encoders_.channel(i);
    EncoderSummary& e = s.encoders[i];
    e.address = ch.encoder->address();
    e.state = ch.state;
    e.valid = ch.valid;
    e.angle = ch.reading.angle;
    e.agc = ch.reading.agc;
    e.magnetOk = ch.reading.diag.magnetOk();
    e.errors = ch.errorCount;
  }

  portENTER_CRITICAL(&mux_);
  shared_ = s;
  portEXIT_CRITICAL(&mux_);
}
