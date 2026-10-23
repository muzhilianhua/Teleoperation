#include "control_task.h"

#include <config_blob.h>
#include <esp_task_wdt.h>
#include <stdarg.h>

#include "config_store.h"
#include "host_telemetry.h"

using teleop::Fault;
using teleop::FaultScope;
using teleop::Verb;
using EV = hostlink::EventCode;
using SV = hostlink::Severity;

namespace {

// The servo jumps to speed without a ramp (servo_bringup single-write check G),
// so the capability model only needs a token acceleration limit.
constexpr float kModelAcceleration = 20000.0f;

// Why a servo reported an error, for the disarm message. The overload bit is
// the servo's own protection (load > 80 % for 1 s) giving up on a squeeze.
void describeServoError(const ServoState& st, char* out, size_t length) {
  const uint8_t e = st.errorByte;
  snprintf(out, length, "servo reports%s%s%s%s%s", (e & 0x20) ? " overload (own protection)" : "",
           (e & 0x08) ? " over-current" : "", (e & 0x04) ? " overheat" : "",
           (e & 0x02) ? " angle-sensor error" : "", (e == 0 && st.status) ? " a status flag" : "");
}

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
    case JointState::Sweep: return "SWEEP";
  }
  return "?";
}

TeleopController::TeleopController(ServoBus& bus, FeetechPort& port, AS5048BGroup& encoders)
    : bus_(bus), port_(port), encoders_(encoders) {
  global_ = config::kGlobalDefaults;
  for (uint8_t i = 0; i < teleop::kMaxJoints; ++i) {
    JointChannel& ch = channels_[i];
    ch.def = &config::kJoints[i];
    ch.scale = teleop::stepScale(*ch.def);
    bus_.setModel(ch.def->servoId, ch.def->model);  // until boot identifies the bus
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
  commands_ = xQueueCreate(32, sizeof(QueuedCommand));
  messages_ = xQueueCreate(64, sizeof(hostlink::Event));
  completions_ = xQueueCreate(32, sizeof(Completion));
  telemetry_ = xQueueCreate(16, sizeof(TelemetrySample));
  if (commands_ == nullptr || messages_ == nullptr || completions_ == nullptr || telemetry_ == nullptr) {
    return false;
  }

  pinMode(config::kEstopPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(config::kEstopPin), estopIsr, FALLING);

  message("servo bus echo: %s", port_.echo() ? "detected, filtering enabled" : "none");
  loadConfig(true);
  // Identify first (~10 ms): reads decode per model, so the torque-off
  // read-back below could not check a servo whose model is still unknown.
  pingAll();
  torqueOffAll("boot");
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

bool TeleopController::post(const Command& command, TickType_t wait, uint16_t seq) {
  const QueuedCommand queued{command, seq};
  return xQueueSend(commands_, &queued, wait) == pdTRUE;
}

void TeleopController::requestEstop() { estopRequested_ = true; }

void TeleopController::snapshot(Snapshot& out) {
  portENTER_CRITICAL(&mux_);
  out = shared_;
  portEXIT_CRITICAL(&mux_);
}

bool TeleopController::popEvent(hostlink::Event& event) {
  return xQueueReceive(messages_, &event, 0) == pdTRUE;
}

bool TeleopController::popCompletion(Completion& completion) {
  return xQueueReceive(completions_, &completion, 0) == pdTRUE;
}

void TeleopController::setTelemetryStream(bool on) {
  telemetryOn_ = on;
  if (!on) xQueueReset(telemetry_);
}

bool TeleopController::popTelemetry(TelemetrySample& sample, TickType_t wait) {
  return xQueueReceive(telemetry_, &sample, wait) == pdTRUE;
}

void TeleopController::postEvent(const EventTag& tag, const char* format, va_list args) {
  if (messages_ == nullptr) return;
  hostlink::Event e;
  e.code = tag.code;
  e.severity = tag.severity;
  e.joint = tag.joint;
  e.cmdSeq = currentSeq_;
  memcpy(e.args, tag.args, sizeof(e.args));
  vsnprintf(e.text, sizeof(e.text), format, args);
  // Drop rather than block the control loop; the host sees the count.
  if (xQueueSend(messages_, &e, 0) != pdTRUE) ++droppedEvents_;
}

void TeleopController::message(const char* format, ...) {
  va_list args;
  va_start(args, format);
  postEvent(EventTag{}, format, args);
  va_end(args);
}

void TeleopController::event(const EventTag& tag, const char* format, ...) {
  va_list args;
  va_start(args, format);
  postEvent(tag, format, args);
  va_end(args);
}

void TeleopController::refuse(const char* format, ...) {
  cmdFailed_ = true;
  va_list args;
  va_start(args, format);
  postEvent(EventTag{EV::Refused, 0, SV::Error}, format, args);
  va_end(args);
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
  ch.tracker.configure(ch.settings.exoRef, ch.settings.exoB, config::kExoDeadZoneCounts);
  ch.wasHomed = false;
  ch.supervisor.reset();
  ch.state = JointState::Off;
  ch.encBad = 0;
  ch.encFresh = false;
  ch.reportedFault = Fault::None;
}

void TeleopController::applyRuntimeParams(JointChannel& ch) {
  ch.filter.setParams(global_.minCutoffHz, global_.beta, config::kFilterDCutoffHz);
  ch.tracker.setMaxStep(static_cast<int32_t>(global_.jumpDeg / teleop::kDegreesPerCount));
  ch.tracker.setLossMs(config::kTrackingLossMs);
  teleop::SupervisorConfig sc;
  sc.trackingErrorSteps = ch.settings.trackSteps;
  sc.loadTripEnabled = global_.loadTripEnabled;
  // DEX_hand_software: trip at 0.9 x the effective current cap for 0.2 s
  // (`itrip` changes the fraction; 0 = off).
  sc.currentTripMa = 0.01f * ch.settings.currentTripPct * effectiveCurrentMa(ch);
  ch.supervisor.setConfig(sc);
  applyGoalCurrent(ch);
  teleop::GripLimiter::Config gc;
  gc.maxLead = ch.settings.gripSteps;
  gc.loadPct = global_.gripLoadPct;
  // Step-based tuning is in SCS0009 steps; keep the same angles on an HLS joint.
  gc.shrinkStep *= ch.scale;
  gc.growStep *= ch.scale;
  gc.minLead *= ch.scale;
  gc.stallSpeed *= ch.scale;
  ch.grip.setConfig(gc);
  ch.limiter.setTravel(ch.def->travelMin, ch.def->travelMax);
  const float speed = ch.state == JointState::Align ? config::kArmSpeed * ch.scale : ch.settings.vmax;
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
    cycleWakeUs_ = wakeUs;
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
  ++cycle_;

  if (estopRequested_) {
    estopRequested_ = false;
    torqueOffAll("E-stop");
  }
  QueuedCommand queued;
  while (xQueueReceive(commands_, &queued, 0) == pdTRUE) {
    // Events raised while handling it carry its seq; DONE follows them.
    currentSeq_ = queued.seq;
    cmdFailed_ = false;
    handleCommand(queued.command);
    if (queued.seq != 0) {
      const Completion done{queued.seq, !cmdFailed_};
      xQueueSend(completions_, &done, 0);
    }
    currentSeq_ = 0;
  }
  // Sampled after the commands: arming takes tens of ms and stamps
  // lastGoodReadMs with millis(), so an earlier nowMs would make
  // nowMs - lastGoodReadMs underflow and read as a stale (lost) bus.
  const uint32_t nowMs = millis();

  encoders_.sample(cycle_ % config::kFullEncoderReadEvery == 0);
  for (JointChannel& ch : channels_) {
    // Tracked whether enabled or not: calibration and homing need an
    // unbroken position history, and multi-turn joints lose it on a gap.
    if (ch.settings.encoderIndex < config::kEncoderCount) {
      processEncoder(ch, dt, nowMs);
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

void TeleopController::processEncoder(JointChannel& ch, float dt, uint32_t nowMs) {
  uint16_t raw = 0;
  if (!encoderSample(ch.settings.encoderIndex, raw)) {
    ch.tracker.noSample(nowMs);
    ch.encFresh = false;
    if (ch.encBad < 0xFF) ++ch.encBad;
    return;
  }
  ch.encRaw = raw;
  const teleop::ExoTracker::Result r = ch.tracker.update(raw, nowMs);
  if (r != teleop::ExoTracker::Result::Accepted) {
    ch.encFresh = false;
    if (ch.encBad < 0xFF) ++ch.encBad;
    if (r == teleop::ExoTracker::Result::Rejected || r == teleop::ExoTracker::Result::Lost) ++ch.jumpRejects;
    if (r == teleop::ExoTracker::Result::Lost) {
      event({EV::Generic, ch.def->servoId, SV::Warning}, "joint %u: encoder jumped > %.0f deg per cycle 3 times, tracking lost%s", ch.def->servoId,
              global_.jumpDeg, ch.tracker.singleTurn() ? " (re-resolving)" : "; `home` it at pose A");
    }
    if (ch.wasHomed && !ch.tracker.homed() && !ch.tracker.singleTurn() && ch.tracker.calibrated()) {
      event({EV::Generic, ch.def->servoId, SV::Warning}, "joint %u: multi-turn position lost; hold the glove at pose A and run `home %u`",
              ch.def->servoId, ch.def->servoId);
    }
    ch.wasHomed = ch.tracker.homed();
    return;
  }
  if (!ch.wasHomed) ch.filter.reset();  // position (re)acquired: do not glide from a stale value
  ch.wasHomed = true;

  ch.encFresh = true;
  ch.encBad = 0;
  const float deg = ch.tracker.position() * teleop::kDegreesPerCount;
  ch.exoCounts = static_cast<float>(ch.tracker.position());
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
  // A sweep reads only its joint, every cycle, for timing resolution.
  uint8_t candidates[teleop::kMaxJoints];
  uint8_t n = 0;
  if (sweepJoint_ >= 0) {
    candidates[n++] = static_cast<uint8_t>(sweepJoint_);
  } else {
    for (uint8_t i = 0; i < teleop::kMaxJoints; ++i) {
      if (channels_[i].settings.enabled || channels_[i].armed()) candidates[n++] = i;
    }
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
      const float effort =
          teleop::gripEffortPct(st.loadPct, st.hasCurrent, st.currentMa, effectiveCurrentMa(ch));
      ch.contact = ch.armed() && ch.grip.update(effort, st.speed);
    }
    teleop::ServoSample sample;
    // Contact and sweeps both make lagging behind the goal expected, so the
    // tracking check is suspended (a sweep drives the joint at full speed).
    sample.inContact = ch.contact || ch.state == JointState::Sweep;
    sample.readOk = ch.servoOk;
    sample.torqueOn = st.torqueOn;
    sample.position = st.position;
    sample.loadPct = st.loadPct;
    sample.voltage = st.voltage;
    sample.temperatureC = st.temperatureC;
    sample.errorByte = st.errorByte;
    sample.status = st.status;
    sample.hasCurrent = st.hasCurrent;
    sample.currentMa = st.currentMa;
    const Fault fault = ch.supervisor.update(sample, ch.armed(), ch.sentGoal, ch.reference, nowMs);

    if (fault != Fault::None && !ch.armed()) {
      // Torque on while the firmware has the joint off: switch it off again.
      if (fault == Fault::TorqueMismatch) bus_.writeTorque(ch.def->servoId, false);
      if (fault != ch.reportedFault) {
        event({EV::Fault, ch.def->servoId, SV::Warning, {static_cast<int32_t>(fault)}}, "joint %u idle fault: %s", ch.def->servoId, teleop::faultToString(fault));
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
    event({EV::Fault, 0, SV::Warning, {static_cast<int32_t>(supply)}}, "idle fault: %s", teleop::faultToString(supply));
  }
  reportedSupplyFault_ = supply;

  if (handFault != Fault::None && armedCount() > 0) {
    disarmAll(handFault, handJoint, teleop::faultToString(handFault));
    return;
  }
  for (uint8_t i = 0; i < teleop::kMaxJoints; ++i) {
    if (jointFaults[i] == Fault::None) continue;
    char reason[96];
    if (jointFaults[i] == Fault::ServoError) {
      describeServoError(channels_[i].servo, reason, sizeof(reason));
    } else {
      strlcpy(reason, teleop::faultToString(jointFaults[i]), sizeof(reason));
    }
    disarmJoint(i, jointFaults[i], reason);
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
        ch.goal = ch.sentGoal = ch.reference = ch.servo.position;
      }
      ch.contact = false;
      return;

    case JointState::Sweep:
      stepSweep(index, nowMs);
      return;

    case JointState::Align:
      if (encFault) {
        disarmJoint(index, Fault::None, "encoder lost during ALIGN");
        return;
      }
      ch.limiter.setTarget(ch.target);
      ch.goal = ch.limiter.step(dt);
      if (fabsf(ch.goal - ch.target) <= config::kAlignTolerance * ch.scale) {
        ch.state = JointState::Follow;
        ch.limiter.setLimits(ch.settings.vmax, ch.settings.amax);
        event({EV::JointState, id, SV::Info, {static_cast<int32_t>(JointState::Follow)}}, "joint %u aligned -> FOLLOW", id);
      }
      break;

    case JointState::Follow:
      if (encFault) {
        ch.state = JointState::Hold;
        ch.holdSinceMs = nowMs;
        ch.encGoodTiming = false;
        ch.limiter.setTarget(ch.limiter.position());
        event({EV::JointState, id, SV::Warning, {static_cast<int32_t>(JointState::Hold)}}, "joint %u encoder fault -> HOLD", id);
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
          ch.limiter.setLimits(config::kArmSpeed * ch.scale, ch.settings.amax);
          event({EV::JointState, id, SV::Info, {static_cast<int32_t>(JointState::Align)}}, "joint %u encoder back -> ALIGN", id);
          break;
        }
      } else {
        ch.encGoodTiming = false;
      }
      if (nowMs - ch.holdSinceMs > config::kHoldTimeoutMs) {
        disarmJoint(index, Fault::None,
                    ch.tracker.homed() ? "encoder lost > 3 s"
                                       : "exo position lost > 3 s (multi-turn: `home` it at pose A)");
        return;
      }
      break;
  }

  // Grip limiter: the servo never gets a goal further than the current lead
  // from where it was last measured (bounds the squeeze on an object).
  ch.sentGoal = ch.grip.clamp(ch.goal, ch.servo.position);
  // Capability model for the tracking check: chases the sent goal at vsat.
  ch.model.setTarget(ch.sentGoal);
  ch.reference = ch.model.step(dt);
}

void TeleopController::writeGoals(uint32_t nowMs) {
  uint8_t ids[teleop::kMaxJoints];
  int16_t positions[teleop::kMaxJoints];
  float speeds[teleop::kMaxJoints];
  uint8_t n = 0;
  for (JointChannel& ch : channels_) {
    ch.goalSent = false;
    // Only to armed joints whose torque a recent read showed on: a goal write
    // to a disarmed SCS0009 would switch its torque on by itself.
    if (!ch.armed() || !ch.servo.torqueOn || nowMs - ch.lastGoodReadMs > config::kServoStaleMs) continue;
    if (ch.state == JointState::Sweep) continue;  // the sweep writes its own single goals
    ch.speed = streamSpeed(ch);
    ids[n] = ch.def->servoId;
    positions[n] = static_cast<int16_t>(lroundf(ch.sentGoal));
    speeds[n] = ch.speed;
    ch.goalSent = true;
    ++n;
  }
  if (n > 0) bus_.syncWriteGoal(ids, positions, speeds, n);
}

// Speed sent with a streamed goal, in the joint's own steps/s ([bringup]
// dynamic mode: gain x |setpoint velocity|, with a floor so the servo glides
// between updates). The bus converts it to the model's speed register.
float TeleopController::streamSpeed(const JointChannel& ch) const {
  const float floorSpeed = config::kStreamSpeedFloor * ch.scale;
  const float cap = max(config::kStreamSpeedGain * ch.settings.vmax, floorSpeed);
  const float wanted = config::kStreamSpeedGain * fabsf(ch.limiter.velocity());
  return min(max(wanted, floorSpeed), cap);
}

// HLS goal current: the servo's protection current, or the operator's
// `ilimit` if lower. 0 when unknown (not armed yet) or for an SCS0009.
float TeleopController::effectiveCurrentMa(const JointChannel& ch) const {
  if (ch.def->model != feetech::ServoModel::Hls3606 || ch.protectionRaw == 0) return 0.0f;
  const float protection = ch.protectionRaw * feetech::modelInfo(ch.def->model).currentMaPerUnit;
  return ch.settings.currentLimitMa > 0 ? min(protection, static_cast<float>(ch.settings.currentLimitMa))
                                        : protection;
}

void TeleopController::applyGoalCurrent(JointChannel& ch) {
  if (ch.def->model != feetech::ServoModel::Hls3606 || ch.protectionRaw == 0) return;
  const float units = effectiveCurrentMa(ch) / feetech::modelInfo(ch.def->model).currentMaPerUnit;
  bus_.setGoalCurrent(ch.def->servoId, static_cast<uint16_t>(max(1.0f, floorf(units))));
}

// ---- commands -----------------------------------------------------------------

void TeleopController::handleCommand(const Command& c) {
  if (armedCount() > 0 && teleop::requiresDisarmed(c)) {
    refuse("refused while joints are armed: disarm first");
    return;
  }
  if (sweepJoint_ >= 0 && c.verb == Verb::Arm) {
    refuse("refused while a sweep is running (`disarm` or `x` stops it)");
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
          refuse("no joint with servo id %u", c.joint);
        } else if (!channels_[i].armed()) {
          refuse("joint %u is not armed", c.joint);
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
      event({EV::Monitor, 0, SV::Info, {monitor_ ? 1 : 0}}, "monitor %s", monitor_ ? "on" : "off");
      break;
    case Verb::Cal: handleCal(c); break;
    case Verb::CalHand: handleCalHand(c); break;
    case Verb::Home: handleHome(c.joint); break;
    case Verb::Set: handleSet(c); break;
    case Verb::Save: saveConfig(); break;
    case Verb::Load: loadConfig(false); break;
    case Verb::Defaults: loadDefaults(); break;
    case Verb::LimitsCheck: handleLimitsCheck(); break;
    case Verb::LimitsWrite: handleLimitsWrite(c.joint); break;
    case Verb::Sweep: handleSweep(c.joint); break;
    case Verb::SweepApply: handleSweepApply(c.joint); break;
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
    refuse("set rejected: %s", error);
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
      ch.tracker.configure(ch.settings.exoRef, ch.settings.exoB, config::kExoDeadZoneCounts);
      ch.wasHomed = false;
    }
    applyRuntimeParams(ch);
  }
  if (teleop::isGlobalKey(c.key)) {
    event({EV::SettingChanged, 0}, "global: lead %.0f ms, mincut %.3f Hz, beta %.4f, jump %.0f deg, gripload %.0f %%, loadtrip %s",
            global_.leadMs, global_.minCutoffHz, global_.beta, global_.jumpDeg, global_.gripLoadPct,
            global_.loadTripEnabled ? "on" : "off");
  } else if (JointChannel* ch = channelForServo(c.joint)) {
    const teleop::JointSettings& s = ch->settings;
    event({EV::SettingChanged, c.joint}, "joint %u: %s enc %d exo %ld servo %d..%d vmax %u amax %u vsat %u trk %u grip %u", c.joint,
            s.enabled ? "on" : "off", s.encoderIndex == teleop::kNoEncoder ? -1 : s.encoderIndex,
            static_cast<long>(s.exoB), s.servoA, s.servoB, s.vmax, s.amax, s.vsat, s.trackSteps,
            s.gripSteps);
    if (ch->def->model == feetech::ServoModel::Hls3606) {
      event({EV::SettingChanged, c.joint}, "joint %u: ilimit %u mA (cap now %.0f mA), itrip %u %%%s", c.joint, s.currentLimitMa,
              effectiveCurrentMa(*ch), s.currentTripPct, s.currentTripPct == 0 ? " (off)" : "");
      // The grip limiter holds the squeeze near gripload % of the cap.
      if (s.currentTripPct != 0 && s.currentTripPct < global_.gripLoadPct + 10.0f) {
        event({EV::Generic, c.joint, SV::Warning}, "warning: itrip %u %% is within 10 %% of gripload %.0f %%: a held grasp may trip",
                s.currentTripPct, global_.gripLoadPct);
      }
    }
  }
}

void TeleopController::handleCal(const Command& c) {
  JointChannel* ch = channelForServo(c.joint);
  if (ch == nullptr) {
    refuse("no joint with servo id %u", c.joint);
    return;
  }
  if (ch->settings.encoderIndex >= config::kEncoderCount) {
    refuse("joint %u: assign an encoder first (set %u enc <n>)", c.joint, c.joint);
    return;
  }
  uint16_t raw = 0;
  if (!encoderSample(ch->settings.encoderIndex, raw)) {
    refuse("joint %u: encoder %u not healthy, capture refused", c.joint, ch->settings.encoderIndex);
    return;
  }
  const uint32_t nowMs = millis();
  if (!c.flag) {
    ch->settings.exoRef = raw;
    ch->settings.exoB = 0;  // B is relative to the new reference: capture it again
    ch->tracker.configure(raw, 0, config::kExoDeadZoneCounts);
    ch->tracker.homeAtA(raw, nowMs);
    ch->wasHomed = true;
    ch->filter.reset();
    event({EV::CalCaptured, c.joint, SV::Info, {0, raw}}, "joint %u: exo A captured (raw %u). Move to the other end, through as many turns as it "
            "takes, and run `cal %u b`.", c.joint, raw, c.joint);
    return;
  }
  // B is the position tracked continuously since A, so multi-turn spans are exact.
  if (!ch->tracker.continuous()) {
    refuse("joint %u: tracking was interrupted since `cal %u a` (or it was not run): redo `cal %u a`",
            c.joint, c.joint, c.joint);
    return;
  }
  const int32_t span = ch->tracker.position();
  if (labs(span) < teleop::kMinExoSpanCounts || labs(span) > teleop::kMaxExoSpanCounts) {
    refuse("joint %u: span %.1f deg rejected (4 deg .. 3 turns)", c.joint, span * teleop::kDegreesPerCount);
    return;
  }
  ch->settings.exoB = span;
  ch->tracker.setSpan(span);
  event({EV::CalCaptured, c.joint, SV::Info, {1, span}}, "joint %u: exo B captured, span %.1f deg (%.2f encoder turns, %s)", c.joint,
          span * teleop::kDegreesPerCount, span / static_cast<float>(teleop::kEncoderCounts),
          ch->tracker.singleTurn() ? "single-turn: homes itself at boot"
                                   : "multi-turn: run `home` at pose A after every power-up");
}

void TeleopController::handleHome(uint8_t servoId) {
  JointChannel* ch = channelForServo(servoId);
  if (ch == nullptr) {
    refuse("no joint with servo id %u", servoId);
    return;
  }
  if (ch->armed()) {
    refuse("joint %u is armed: `disarm %u` before homing", servoId, servoId);
    return;
  }
  uint16_t raw = 0;
  if (ch->settings.encoderIndex >= config::kEncoderCount || !encoderSample(ch->settings.encoderIndex, raw)) {
    refuse("joint %u: no healthy encoder sample, home refused", servoId);
    return;
  }
  const char* error = ch->tracker.home(raw, millis());
  if (error != nullptr) {
    refuse("joint %u: home refused: %s", servoId, error);
    return;
  }
  ch->wasHomed = true;
  ch->filter.reset();
  const float pct = 100.0f * ch->tracker.position() / static_cast<float>(ch->settings.exoB);
  event({EV::Homed, servoId, SV::Info, {static_cast<int32_t>(ch->tracker.position())}}, "joint %u homed: %.1f deg from A (%.0f %% of the span)%s", servoId,
          ch->tracker.position() * teleop::kDegreesPerCount, pct,
          ch->tracker.singleTurn() ? " (single-turn joints also home themselves)" : "");
}

void TeleopController::handleCalHand(const Command& c) {
  JointChannel* ch = channelForServo(c.joint);
  if (ch == nullptr) {
    refuse("no joint with servo id %u", c.joint);
    return;
  }
  if (!teleop::travelDefined(*ch->def)) {
    refuse("joint %u: travel not defined in config.h; measure it with `monitor on` first", c.joint);
    return;
  }
  ServoState st;
  if (!bus_.readState(c.joint, st)) {
    refuse("joint %u: no reply from servo", c.joint);
    return;
  }
  if (st.torqueOn) {
    bus_.writeTorque(c.joint, false);
    refuse("joint %u: servo had torque on, switched off; move the finger by hand and retry", c.joint);
    return;
  }
  if (st.position < ch->def->travelMin || st.position > ch->def->travelMax) {
    refuse("joint %u: position %d outside travel [%d, %d], rejected", c.joint, st.position,
            ch->def->travelMin, ch->def->travelMax);
    return;
  }
  (c.flag ? ch->settings.servoB : ch->settings.servoA) = st.position;
  event({EV::CalCaptured, c.joint, SV::Info, {c.flag ? 3 : 2, st.position}}, "joint %u: servo %c = %d%s", c.joint, c.flag ? 'B' : 'A', st.position,
          ch->settings.servoA == ch->settings.servoB ? " (A = B: capture the other end)" : "");
}

void TeleopController::handleLimitsCheck() {
  uint8_t mismatches = 0;
  for (const JointChannel& ch : channels_) {
    const uint8_t id = ch.def->servoId;
    int16_t lo = 0, hi = 0;
    if (!bus_.readAngleLimits(id, lo, hi)) {
      event({EV::LimitsRow, id, SV::Warning, {0, 0, -1}}, "limits j%u %-13s no reply", id, ch.def->name);
      continue;
    }
    if (!teleop::travelDefined(*ch.def)) {
      event({EV::LimitsRow, id, SV::Info, {lo, hi, -2}}, "limits j%u %-13s servo %4d..%-4d  travel undefined", id, ch.def->name, lo, hi);
      continue;
    }
    const bool match = lo == ch.def->travelMin && hi == ch.def->travelMax;
    mismatches += match ? 0 : 1;
    const bool multiTurn = lo == 0 && hi == 0 && bus_.model(id) == feetech::ServoModel::Hls3606;
    event({EV::LimitsRow, id, SV::Info, {lo, hi, match ? 1 : 0}}, "limits j%u %-13s servo %4d..%-4d  travel %4d..%-4d  %s", id, ch.def->name, lo, hi,
            ch.def->travelMin, ch.def->travelMax,
            match ? "ok" : (multiTurn ? "MISMATCH (HLS factory multi-turn)" : "MISMATCH"));
  }
  message(mismatches ? "%u mismatch(es): `limits write all` syncs them" : "all defined joints match",
          mismatches);
}

void TeleopController::handleLimitsWrite(uint8_t servoId) {
  for (const JointChannel& ch : channels_) {
    const uint8_t id = ch.def->servoId;
    if (servoId != 0 && id != servoId) continue;
    if (!teleop::travelDefined(*ch.def)) {
      event({EV::Generic, id, SV::Warning}, "limits j%u: travel undefined in config.h, skipped", id);
      continue;
    }
    int16_t lo = 0, hi = 0;
    if (!bus_.readAngleLimits(id, lo, hi)) {
      event({EV::Generic, id, SV::Warning}, "limits j%u: no reply, skipped", id);
      continue;
    }
    if (lo == ch.def->travelMin && hi == ch.def->travelMax) {
      if (servoId != 0) message("limits j%u: already %d..%d, nothing written", id, lo, hi);
      continue;  // EEPROM is only written when it differs
    }
    const bool ok = bus_.writeAngleLimits(id, ch.def->travelMin, ch.def->travelMax);
    const bool hls = bus_.model(id) == feetech::ServoModel::Hls3606;
    event({EV::Generic, id, ok ? SV::Info : SV::Error}, "limits j%u: %d..%d -> %d..%d %s%s", id, lo, hi, ch.def->travelMin, ch.def->travelMax,
            ok ? "written and verified" : "FAILED (torque must be off; check the bus)",
            ok && hls ? "; power-cycle the servos before arming" : "");
    if (!ok) cmdFailed_ = true;
  }
  if (servoId != 0 && channelForServo(servoId) == nullptr) refuse("no joint with servo id %u", servoId);
}

// ---- speed sweep --------------------------------------------------------------

void TeleopController::handleSweep(uint8_t servoId) {
  const int index = teleop::jointIndexForServo(config::kJoints, teleop::kMaxJoints, servoId);
  if (index < 0) {
    refuse("no joint with servo id %u", servoId);
    return;
  }
  JointChannel& ch = channels_[index];
  const teleop::JointDefaults& d = *ch.def;
  if (!teleop::travelDefined(d)) {
    refuse("sweep refused: joint %u travel not defined in config.h", servoId);
    return;
  }
  // Calibrated servo endpoints are the range the operator has checked by hand;
  // otherwise the configured travel. 10 SCS steps of margin on each side.
  int16_t lo = d.travelMin, hi = d.travelMax;
  if (teleop::servoCalibrated(ch.settings, d)) {
    lo = min(ch.settings.servoA, ch.settings.servoB);
    hi = max(ch.settings.servoA, ch.settings.servoB);
  }
  lo += static_cast<int16_t>(lroundf(10.0f * ch.scale));
  hi -= static_cast<int16_t>(lroundf(10.0f * ch.scale));
  if (hi - lo < 60.0f * ch.scale) {
    refuse("sweep refused: joint %u range %d..%d is too short to time", servoId, lo, hi);
    return;
  }
  ServoState st;
  if (!bus_.readState(servoId, st)) {
    refuse("sweep refused: no reply from servo %u", servoId);
    return;
  }
  if (st.errorByte != 0 || st.status != 0 || st.position < d.travelMin || st.position > d.travelMax) {
    refuse("sweep refused: servo %u error 0x%02X or position %d outside travel", servoId, st.errorByte,
            st.position);
    return;
  }

  monitor_ = false;
  const uint32_t nowMs = millis();
  const char* problem = armJoint(index, st, nowMs);
  if (problem != nullptr) {
    disarmJoint(index, Fault::None, problem);
    return;
  }
  ch.state = JointState::Sweep;
  teleop::SweepConfig cfg;
  cfg.lo = lo;
  cfg.hi = hi;
  // Tiers, tolerances and suggestion bounds are SCS values scaled to this model.
  for (uint8_t i = 0; i < cfg.speedCount; ++i) cfg.speeds[i] *= ch.scale;
  cfg.approachSpeed *= ch.scale;
  cfg.reachTolerance *= ch.scale;
  cfg.minSuggestedVmax *= ch.scale;
  const feetech::ModelInfo& info = feetech::modelInfo(d.model);
  cfg.maxSuggestedSpeed = info.maxSpeedRegister * info.stepsPerSpeedUnit;
  sweep_.start(cfg, st.position, nowMs);
  sweepJoint_ = static_cast<int8_t>(index);
  event({EV::Generic, servoId}, "sweep j%u (%s, %s): range %d..%d, %.0f/%.0f/%.0f/%.0f steps/s, both directions; `x`, `disarm` "
          "or BOOT aborts",
          servoId, d.name, info.name, lo, hi, cfg.speeds[0], cfg.speeds[1], cfg.speeds[2], cfg.speeds[3]);
}

void TeleopController::stepSweep(uint8_t index, uint32_t nowMs) {
  JointChannel& ch = channels_[index];
  const bool valid = ch.readThisCycle && ch.servoOk;
  const teleop::SweepStep step = sweep_.update(nowMs, ch.servo.position, ch.servo.loadPct, valid);
  if (step.write) {
    bus_.writeGoal(ch.def->servoId, step.goal, step.speed);
    ch.goal = ch.sentGoal = ch.reference = step.goal;
  }
  if (step.runFinished) {
    const teleop::SweepResult& r = sweep_.result();
    const teleop::SweepRun& run = r.runs[r.runCount - 1];
    if (!run.valid) {
      event({EV::SweepRun, ch.def->servoId, SV::Info, {static_cast<int32_t>(run.commandSpeed), -1, 0, 0}}, "sweep j%u %5.0f %c: skipped (previous move did not get back far enough)", ch.def->servoId,
              run.commandSpeed, run.direction > 0 ? '+' : '-');
    } else {
      event({EV::SweepRun, ch.def->servoId, SV::Info, {static_cast<int32_t>(run.commandSpeed), static_cast<int32_t>(run.speed * 10.0f), run.reached ? 1 : 0, static_cast<int32_t>(run.maxLoadPct * 10.0f)}}, "sweep j%u %5.0f %c: %5.0f steps/s %s, max load %.0f %%", ch.def->servoId,
              run.commandSpeed, run.direction > 0 ? '+' : '-', run.speed,
              run.reached ? "arrived  " : "TIMED OUT", run.maxLoadPct);
    }
  }
  if (step.done) {
    endSweep(nullptr);
    disarmJoint(index, Fault::None, "sweep finished");
  }
}

void TeleopController::endSweep(const char* abortReason) {
  if (sweepJoint_ < 0) return;
  const uint8_t index = static_cast<uint8_t>(sweepJoint_);
  sweepJoint_ = -1;  // before anything that could disarm and re-enter
  if (sweep_.active()) sweep_.abort(abortReason != nullptr ? abortReason : "stopped");
  lastSweep_[index] = sweep_.result();
  haveSweep_[index] = true;
  reportSweep(index);
}

void TeleopController::reportSweep(uint8_t index) {
  const teleop::SweepResult& r = lastSweep_[index];
  const JointChannel& ch = channels_[index];
  const uint8_t id = ch.def->servoId;
  if (r.aborted) {
    event({EV::SweepDone, id, SV::Warning, {0, 0, 0, 1}}, "sweep j%u aborted: %s (%u move(s) recorded, no suggestion)", id, r.abortReason, r.runCount);
    return;
  }
  char plus[24], minus[24];
  if (r.limitedPlus) {
    strlcpy(plus, "FORCE-LIMITED", sizeof(plus));
  } else {
    snprintf(plus, sizeof(plus), "%.0f steps/s", r.saturationPlus);
  }
  if (r.limitedMinus) {
    strlcpy(minus, "FORCE-LIMITED", sizeof(minus));
  } else {
    snprintf(minus, sizeof(minus), "%.0f steps/s", r.saturationMinus);
  }
  event({EV::Generic, id}, "sweep j%u done: saturation + %s, - %s", id, plus, minus);
  if (r.applicable()) {
    event({EV::SweepDone, id, SV::Info, {r.suggestedVmax, r.suggestedVsat, 1, 0}}, "sweep j%u suggests vmax %u vsat %u (now %u / %u): `sweep %u apply`, then `save`", id,
            r.suggestedVmax, r.suggestedVsat, ch.settings.vmax, ch.settings.vsat, id);
  } else {
    event({EV::SweepDone, id, SV::Warning, {0, 0, 0, 0}}, "sweep j%u: no suggestion; a direction never arrived at register >= 300 "
            "(friction / tendon preload: fix the mechanics, then sweep again)", id);
  }
}

void TeleopController::handleSweepApply(uint8_t servoId) {
  const int index = teleop::jointIndexForServo(config::kJoints, teleop::kMaxJoints, servoId);
  if (index < 0) {
    refuse("no joint with servo id %u", servoId);
    return;
  }
  if (!haveSweep_[index]) {
    refuse("joint %u has no sweep result yet: run `sweep %u` first", servoId, servoId);
    return;
  }
  const teleop::SweepResult& r = lastSweep_[index];
  if (!r.applicable()) {
    refuse("apply refused: the last sweep of joint %u was %s", servoId,
            r.aborted ? "aborted" : "force-limited");
    return;
  }
  Command c;
  c.verb = Verb::Set;
  c.joint = servoId;
  c.valueCount = 1;
  c.key = teleop::SetKey::VMax;
  c.values[0] = r.suggestedVmax;
  handleSet(c);
  c.key = teleop::SetKey::VSat;
  c.values[0] = r.suggestedVsat;
  handleSet(c);
}

void TeleopController::saveConfig() {
  teleop::JointSettings joints[teleop::kMaxJoints];
  for (uint8_t i = 0; i < teleop::kMaxJoints; ++i) joints[i] = channels_[i].settings;
  uint8_t blob[teleop::kBlobSize];
  const size_t n =
      teleop::packConfig(joints, config::kJoints, teleop::kMaxJoints, global_, blob, sizeof(blob));
  const bool ok = n > 0 && config_store::save(blob, n);
  event({EV::ConfigSaved, 0, ok ? SV::Info : SV::Error, {ok ? 1 : 0}}, ok ? "configuration saved to NVS" : "SAVE FAILED");
  if (!ok) cmdFailed_ = true;
}

bool TeleopController::loadConfig(bool atBoot) {
  uint8_t blob[teleop::kBlobSize + 16];
  const size_t length = config_store::load(blob, sizeof(blob));
  if (length == 0) {
    event({EV::ConfigLoaded, 0, SV::Warning, {-1}}, "no stored configuration%s", atBoot ? ", using defaults" : "");
    if (!atBoot) cmdFailed_ = true;
    return false;
  }
  teleop::JointSettings joints[teleop::kMaxJoints];
  teleop::GlobalSettings global;
  uint8_t badJoint = 0xFF;
  const teleop::BlobStatus status = teleop::unpackConfig(
      blob, length, config::kJoints, teleop::kMaxJoints, config::kEncoderCount,
      config::kGlobalDefaults, joints, global, &badJoint);
  if (!teleop::blobApplied(status)) {
    event({EV::ConfigLoaded, 0, SV::Error, {static_cast<int32_t>(status)}}, "stored configuration rejected (%s), %s", teleop::blobStatusToString(status),
            atBoot ? "using defaults" : "keeping current settings");
    if (!atBoot) cmdFailed_ = true;
    return false;
  }
  global_ = global;
  for (uint8_t i = 0; i < teleop::kMaxJoints; ++i) {
    channels_[i].settings = joints[i];
    resetChannelRuntime(channels_[i]);
    applyRuntimeParams(channels_[i]);
  }
  if (status == teleop::BlobStatus::Ok) {
    event({EV::ConfigLoaded, 0, SV::Info, {static_cast<int32_t>(status)}}, "configuration loaded from NVS");
  } else if (badJoint < teleop::kMaxJoints) {
    event({EV::ConfigLoaded, 0, SV::Info, {static_cast<int32_t>(status)}}, "configuration loaded with corrections: %s (first: joint %u)",
            teleop::blobStatusToString(status), config::kJoints[badJoint].servoId);
  } else {
    event({EV::ConfigLoaded, 0, SV::Info, {static_cast<int32_t>(status)}}, "configuration loaded: %s; run `save` to store it in the new format",
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
  event({EV::Defaults, 0}, "defaults loaded (not saved; run `save` to keep them)");
}

void TeleopController::pingAll() {
  for (JointChannel& ch : channels_) {
    const uint8_t id = ch.def->servoId;
    feetech::ServoModel model = feetech::ServoModel::Unknown;
    uint8_t fwMajor = 0, fwMinor = 0;
    if (!bus_.identify(id, model, fwMajor, fwMinor)) {
      ch.detectedModel = feetech::ServoModel::Unknown;
      ch.modelOk = false;
      event({EV::PingResult, id, SV::Warning, {0, 0, 0, 0}}, "servo %u (%s): no reply", id, ch.def->name);
      continue;
    }
    ch.detectedModel = model;
    ch.modelOk = model == ch.def->model;
    // Decode this id as what is actually on the bus, so monitor / limits work
    // even when config.h is wrong; arming stays refused until it matches.
    if (model != feetech::ServoModel::Unknown) bus_.setModel(id, model);
    if (ch.modelOk) {
      event({EV::PingResult, id, SV::Info, {static_cast<int32_t>(model), fwMajor, fwMinor, 1}}, "servo %u (%s): %s, firmware %u.%u", id, ch.def->name, feetech::modelName(model), fwMajor,
              fwMinor);
    } else {
      event({EV::PingResult, id, SV::Error, {static_cast<int32_t>(model), fwMajor, fwMinor, 0}}, "servo %u (%s): %s on the bus but config.h says %s: joint cannot be armed", id, ch.def->name,
              feetech::modelName(model), feetech::modelName(ch.def->model));
    }
  }
}

// ---- arming -------------------------------------------------------------------

bool TeleopController::preflight(uint8_t index, ServoState& st) {
  JointChannel& ch = channels_[index];
  const uint8_t id = ch.def->servoId;
  if (!ch.modelOk) {
    refuse("arm refused: servo %u is %s, config.h expects %s (run `ping` after fixing)", id,
            feetech::modelName(ch.detectedModel), feetech::modelName(ch.def->model));
    return false;
  }
  if (!teleop::jointReady(ch.settings, *ch.def, config::kEncoderCount)) {
    refuse("arm refused: joint %u %s", id, notReadyReason(ch.settings, *ch.def));
    return false;
  }
  if (!ch.tracker.homed()) {
    refuse("arm refused: joint %u multi-turn position unknown: hold the glove at pose A and run `home %u`",
            id, id);
    return false;
  }
  if (!ch.encFresh) {
    refuse("arm refused: joint %u encoder not healthy", id);
    return false;
  }
  if (!bus_.readState(id, st)) {
    refuse("arm refused: no status reply from servo %u", id);
    return false;
  }
  if (st.errorByte != 0 || st.status != 0) {
    refuse("arm refused: servo %u error byte 0x%02X status 0x%02X", id, st.errorByte, st.status);
    return false;
  }
  if (st.position < ch.def->travelMin || st.position > ch.def->travelMax) {
    refuse("arm refused: servo %u at %d outside travel [%d, %d]; move it back by hand", id,
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
  if (!ch.modelOk) return "servo model not identified or not as in config.h (run `ping`)";
  if (ch.def->model == feetech::ServoModel::Hls3606) {
    // DEX_hand_software arming checks: single-turn limits covering the
    // travel, resolution 1, position mode, a usable protection current.
    feetech::ArmInfo info;
    if (!bus_.readArmInfo(id, info)) return "no reply reading the HLS setup registers";
    if (info.minLimit == 0 && info.maxLimit == 0) {
      return "HLS angle limits are 0/0 (factory multi-turn): run `limits write <j>`, then power-cycle";
    }
    if (info.minLimit > ch.def->travelMin || info.maxLimit < ch.def->travelMax) {
      return "HLS angle limits do not cover the travel: run `limits write <j>`, then power-cycle";
    }
    if (info.resolution != 1) return "HLS angle resolution (register 30) is not 1";
    if (info.mode != 0) return "HLS not in position mode (register 33)";
    if (info.protectionCurrent < 1 || info.protectionCurrent > feetech::kHlsMaxGoalCurrent) {
      return "HLS protection current (register 28) out of range";
    }
    if (hold < 0) return "HLS position is negative (multi-turn): set the limits and power-cycle";
    ch.protectionRaw = info.protectionCurrent;
    applyGoalCurrent(ch);  // before the first goal: it carries the current cap
  }
  ServoState check;
  if (!bus_.writeGoal(id, hold, config::kArmSpeed * ch.scale)) return "goal write failed";
  if (!bus_.writeTorque(id, true)) return "torque write failed";
  if (!bus_.readState(id, check)) return "no read-back after arming";
  if (!check.torqueOn) return "torque did not switch on";
  if (check.goal != hold) return "goal register does not match the hold position";
  if (abs(check.position - hold) > config::kArmHoldTolerance * ch.scale) return "joint moved while arming";

  ch.state = JointState::Align;
  ch.lastFault = Fault::None;
  ch.servo = check;
  ch.servoOk = true;
  ch.lastGoodReadMs = nowMs;
  ch.limiter.reset(hold);
  ch.limiter.setLimits(config::kArmSpeed * ch.scale, ch.settings.amax);
  ch.model.reset(hold);
  applyRuntimeParams(ch);  // current trip now that the protection current is known
  ch.goal = ch.sentGoal = ch.reference = hold;
  ch.supervisor.reset();
  ch.grip.reset();
  ch.contact = false;
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
    if (armedCount() > 0) {
      message("every enabled joint is already armed");
    } else {
      refuse("arm refused: no joint enabled (set <j> on)");
    }
    return;
  }

  monitor_ = false;
  const uint32_t nowMs = millis();
  for (uint8_t i = 0; i < teleop::kMaxJoints; ++i) {
    JointChannel& ch = channels_[i];
    if (!ch.settings.enabled || ch.armed()) continue;
    const char* problem = armJoint(i, states[i], nowMs);
    if (problem != nullptr) {
      cmdFailed_ = true;
      disarmAll(Fault::None, i, problem);
      return;
    }
  }
  if (armedCount() == toArm) supplySupervisor_.reset();
  lastHandFault_ = Fault::None;
  event({EV::Armed, 0, SV::Info, {toArm}}, "armed %u joint(s), holding -> ALIGN (keep the glove still)", toArm);
}

void TeleopController::armOne(uint8_t servoId) {
  const int i = teleop::jointIndexForServo(config::kJoints, teleop::kMaxJoints, servoId);
  if (i < 0) {
    refuse("no joint with servo id %u", servoId);
    return;
  }
  JointChannel& ch = channels_[i];
  if (ch.armed()) {
    refuse("joint %u is already armed", servoId);
    return;
  }
  if (!ch.settings.enabled) {
    refuse("arm refused: joint %u disabled (set %u on)", servoId, servoId);
    return;
  }
  ServoState st;
  if (!preflight(i, st)) return;
  monitor_ = false;
  const char* problem = armJoint(i, st, millis());
  if (problem != nullptr) {
    cmdFailed_ = true;
    disarmJoint(i, Fault::None, problem);
    return;
  }
  event({EV::Armed, servoId, SV::Info, {1}}, "joint %u armed, holding -> ALIGN", servoId);
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
  if (sweepJoint_ == static_cast<int8_t>(index)) endSweep(reason);
  JointChannel& ch = channels_[index];
  ch.state = JointState::Off;  // first, so no goal can follow
  const uint8_t id = ch.def->servoId;
  const bool confirmed = switchOff(&id, 1);
  ch.supervisor.reset();
  ch.lastFault = fault;
  event({EV::Disarmed, id, fault == Fault::None ? SV::Info : SV::Error, {static_cast<int32_t>(fault), confirmed ? 1 : 0}}, "joint %u (%s) DISARM: %s; torque off %s; other joints unaffected", id, ch.def->name, reason,
          confirmed ? "confirmed" : "NOT CONFIRMED");
}

void TeleopController::disarmAll(Fault fault, int jointIndex, const char* reason) {
  endSweep(reason);
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
    event({EV::Disarmed, 0, fault == Fault::None ? SV::Info : SV::Error, {static_cast<int32_t>(fault), confirmed ? 1 : 0}}, "DISARM ALL: %s (joint %u %s); torque off %s", reason, channels_[jointIndex].def->servoId,
            channels_[jointIndex].def->name, confirmed ? "confirmed" : "NOT CONFIRMED");
  } else {
    event({EV::Disarmed, 0, fault == Fault::None ? SV::Info : SV::Error, {static_cast<int32_t>(fault), confirmed ? 1 : 0}}, "DISARM ALL: %s; torque off %s", reason, confirmed ? "confirmed" : "NOT CONFIRMED");
  }
}

void TeleopController::torqueOffAll(const char* reason) {
  endSweep(reason);
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
  event({EV::TorqueOff, 0, SV::Warning, {allConfirmed ? 1 : 0}}, "%s: torque off on all servos%s", reason, allConfirmed ? "" : " (NOT CONFIRMED)");
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
  s.tUs = cycleWakeUs_;
  s.uptimeMs = nowMs;
  s.armedCount = armedCount();
  s.lastHandFault = lastHandFault_;
  s.torqueOffConfirmed = torqueOffConfirmed_;
  s.droppedEvents = droppedEvents_;
  s.droppedTelemetry = droppedTelemetry_;
  s.sweepActive = sweepJoint_ >= 0;
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
    j.refCaptured = !teleop::exoCalibrated(ch.settings) && ch.tracker.continuous();
    j.homed = ch.tracker.homed();
    j.singleTurn = !teleop::exoCalibrated(ch.settings) || ch.tracker.singleTurn();
    j.encFresh = ch.encFresh;
    j.encRaw = ch.encRaw;
    j.exoCounts = ch.exoCounts;
    j.exoFilteredCounts = ch.exoFiltered;
    j.target = ch.target;
    j.goal = ch.goal;
    j.sentGoal = ch.sentGoal;
    j.gripLead = ch.grip.lead();
    j.contact = ch.contact;
    j.reference = ch.reference;
    j.speed = ch.speed;
    j.detectedModel = ch.detectedModel;
    j.modelOk = ch.modelOk;
    j.currentLimitMa = effectiveCurrentMa(ch);
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

  if (telemetryOn_) {
    fillTelemetry(s, telemetryScratch_);
    if (telemetryGap_) telemetryScratch_.handFlags |= hostlink::kHandTelemetryDropped;
    sampleScratch_.length = static_cast<uint16_t>(
        hostlink::encodeTelemetry(telemetryScratch_, sampleScratch_.bytes, sizeof(sampleScratch_.bytes)));
    // Never wait: a slow host loses samples (flagged in the next one), not cycles.
    telemetryGap_ = xQueueSend(telemetry_, &sampleScratch_, 0) != pdTRUE;
    if (telemetryGap_) ++droppedTelemetry_;
  }
}
