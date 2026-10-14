#include "control_task.h"

#include <esp_task_wdt.h>
#include <stdarg.h>

using teleop::Fault;

TeleopController* TeleopController::instance_ = nullptr;

const char* teleopStateToString(TeleopState state) {
  switch (state) {
    case TeleopState::Idle: return "IDLE";
    case TeleopState::Align: return "ALIGN";
    case TeleopState::Follow: return "FOLLOW";
    case TeleopState::Hold: return "HOLD";
  }
  return "?";
}

TeleopController::TeleopController(ServoBus& bus, FeetechPort& port, AS5048BGroup& encoders)
    : bus_(bus), port_(port), encoders_(encoders) {
  joint_.name = config::kJointName;
  joint_.encoderIndex = config::kJointEncoderIndex;
  joint_.servoId = config::kJointServoId;
  joint_.servoMin = config::kJointServoMin;
  joint_.servoMax = config::kJointServoMax;
  // Servo endpoints default to the travel limits; refine with `set sa/sb`.
  joint_.servoA = config::kJointServoMin;
  joint_.servoB = config::kJointServoMax;
  filter_.setParams(params_.minCutoffHz, params_.beta, config::kFilterDCutoffHz);
  limiter_ = teleop::RateLimiter(joint_.servoMin, params_.maxSpeed, params_.acceleration,
                                 joint_.servoMin, joint_.servoMax);
}

bool TeleopController::begin() {
  instance_ = this;
  commands_ = xQueueCreate(16, sizeof(Command));
  messages_ = xQueueCreate(24, sizeof(Message));
  if (commands_ == nullptr || messages_ == nullptr) return false;

  pinMode(config::kEstopPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(config::kEstopPin), estopIsr, FALLING);

  message("servo bus echo: %s", port_.echo() ? "detected, filtering enabled" : "none");
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

bool TeleopController::post(const Command& command) {
  return xQueueSend(commands_, &command, 0) == pdTRUE;
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

// ---- loop -------------------------------------------------------------------

void TeleopController::run() {
  esp_task_wdt_add(nullptr);
  const TickType_t period = pdMS_TO_TICKS(1000 / config::kControlHz);
  TickType_t lastWake = xTaskGetTickCount();
  for (;;) {
    vTaskDelayUntil(&lastWake, period);
    const uint32_t wakeUs = micros();
    updateTiming(wakeUs);
    cycle(wakeUs);
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

void TeleopController::cycle(uint32_t /*wakeUs*/) {
  const float dt = 1.0f / config::kControlHz;
  const uint32_t nowMs = millis();
  ++cycle_;

  if (estopRequested_) {
    estopRequested_ = false;
    torqueOffAll("E-stop");
  }
  Command command;
  while (xQueueReceive(commands_, &command, 0) == pdTRUE) handleCommand(command);

  // 1. Encoder -> filtered exo position -> mapped servo target
  encoders_.sample(cycle_ % config::kFullEncoderReadEvery == 0);
  const AS5048BChannel& ch = encoders_.channel(joint_.encoderIndex);
  encFresh_ = ch.state == AS5048BChannelState::Online && ch.lastStatus == AS5048BStatus::Ok;
  if (encFresh_) {
    encBadCycles_ = 0;
    encRaw_ = ch.reading.angle;
    exoCounts_ = static_cast<float>(teleop::unwrapCounts(encRaw_, joint_.exoRef));
    const float deg = filter_.filter(exoCounts_ * teleop::kDegreesPerCount, dt);
    exoFiltered_ = deg / teleop::kDegreesPerCount;
    target_ = teleop::mapToServo(joint_, exoFiltered_);
    if (params_.leadMs > 0.0f && joint_.exoCalibrated()) {
      const float slope =
          static_cast<float>(joint_.servoB - joint_.servoA) / static_cast<float>(joint_.exoB - joint_.exoA);
      const float servoVelocity = filter_.derivative() / teleop::kDegreesPerCount * slope;
      const float lo = min(joint_.servoA, joint_.servoB);
      const float hi = max(joint_.servoA, joint_.servoB);
      target_ = constrain(target_ + servoVelocity * params_.leadMs * 1e-3f, lo, hi);
    }
  } else if (encBadCycles_ < 0xFF) {
    ++encBadCycles_;
  }
  const bool encFault = encBadCycles_ >= config::kEncoderFailCycles;

  // 2. Servo state + supervision
  ServoState st;
  servoOk_ = bus_.readState(joint_.servoId, st);
  if (servoOk_) servo_ = st;
  teleop::ServoSample sample;
  sample.readOk = servoOk_;
  sample.torqueOn = st.torqueOn;
  sample.position = st.position;
  sample.loadPct = st.loadPct;
  sample.voltage = st.voltage;
  sample.temperatureC = st.temperatureC;
  sample.errorByte = st.errorByte;
  sample.status = st.status;
  Fault fault = jointSupervisor_.update(sample, armed_, goal_, nowMs);
  if (fault == Fault::None && servoOk_) fault = supplySupervisor_.update(&st.voltage, 1, nowMs);
  if (fault != Fault::None) {
    if (armed_) {
      disarm(fault, teleop::faultToString(fault));
    } else {
      if (fault == Fault::TorqueMismatch) {
        // Torque on while the firmware has the joint disarmed: switch it off again.
        bus_.writeTorque(joint_.servoId, false);
      }
      if (fault != lastReportedFault_) message("idle fault: %s", teleop::faultToString(fault));
    }
  }
  lastReportedFault_ = fault;

  // 3. State machine -> goal
  switch (state_) {
    case TeleopState::Idle:
      if (servoOk_) limiter_.reset(st.position);
      goal_ = limiter_.position();
      break;

    case TeleopState::Align:
      if (encFault) {
        disarm(Fault::None, "encoder lost during ALIGN");
        break;
      }
      limiter_.setLimits(params_.armSpeed, params_.acceleration);
      limiter_.setTarget(target_);
      goal_ = limiter_.step(dt);
      if (fabsf(goal_ - target_) <= config::kAlignTolerance) {
        limiter_.setLimits(params_.maxSpeed, params_.acceleration);
        state_ = TeleopState::Follow;
        message("ALIGN done -> FOLLOW");
      }
      break;

    case TeleopState::Follow:
      if (encFault) {
        state_ = TeleopState::Hold;
        holdSinceMs_ = nowMs;
        encGoodTiming_ = false;
        limiter_.setTarget(limiter_.position());
        message("encoder fault (%s) -> HOLD", AS5048B::statusToString(ch.lastStatus));
      } else {
        limiter_.setTarget(target_);
      }
      goal_ = limiter_.step(dt);
      break;

    case TeleopState::Hold:
      goal_ = limiter_.step(dt);
      if (encFresh_) {
        if (!encGoodTiming_) {
          encGoodTiming_ = true;
          encGoodSinceMs_ = nowMs;
        } else if (nowMs - encGoodSinceMs_ >= config::kHoldRecoverMs) {
          state_ = TeleopState::Align;
          message("encoder back -> ALIGN");
          break;
        }
      } else {
        encGoodTiming_ = false;
      }
      if (nowMs - holdSinceMs_ > config::kHoldTimeoutMs) disarm(Fault::None, "encoder lost > 3 s");
      break;
  }

  // 4. Goal write: only to an armed joint whose torque we just read back as on
  goalSent_ = false;
  if (armed_ && servoOk_ && st.torqueOn && state_ != TeleopState::Idle) {
    const uint8_t id = joint_.servoId;
    const int16_t position = static_cast<int16_t>(lroundf(goal_));
    speedRegister_ = streamSpeed(limiter_.velocity());
    bus_.syncWriteGoal(&id, &position, &speedRegister_, 1);
    goalSent_ = true;
  }
}

uint16_t TeleopController::streamSpeed(float velocity) const {
  const float wanted = ceilf(config::kStreamSpeedGain * fabsf(velocity));
  float speed = max(static_cast<float>(config::kStreamSpeedFloor), wanted);
  speed = min(speed, static_cast<float>(config::kStreamSpeedCap));
  return static_cast<uint16_t>(speed);
}

// ---- commands -----------------------------------------------------------------

void TeleopController::handleCommand(const Command& command) {
  switch (command.type) {
    case CommandType::Arm: arm(); break;
    case CommandType::Disarm: disarm(Fault::None, "operator"); break;
    case CommandType::Estop: torqueOffAll("E-stop (console)"); break;
    case CommandType::CaptureExoA: captureExo(true); break;
    case CommandType::CaptureExoB: captureExo(false); break;
    case CommandType::SetParam: setParam(command.param, command.value); break;
    case CommandType::Ping:
      if (armed_) {
        message("ping refused while armed");
      } else {
        pingAll();
      }
      break;
    case CommandType::ResetStats:
      timing_ = TimingStats();
      memset(jitterHistogram_, 0, sizeof(jitterHistogram_));
      lastWakeUs_ = 0;
      port_.clearStats();
      encoders_.clearStats();
      message("statistics cleared");
      break;
  }
}

void TeleopController::setParam(ParamId param, float value) {
  switch (param) {
    case ParamId::ServoA:
    case ParamId::ServoB: {
      if (armed_) {
        message("disarm before changing servo endpoints");
        return;
      }
      const int16_t v = static_cast<int16_t>(lroundf(value));
      if (v < joint_.servoMin || v > joint_.servoMax) {
        message("rejected: %d outside travel [%d, %d]", v, joint_.servoMin, joint_.servoMax);
        return;
      }
      (param == ParamId::ServoA ? joint_.servoA : joint_.servoB) = v;
      message("servo %c = %d", param == ParamId::ServoA ? 'A' : 'B', v);
      return;
    }
    case ParamId::MaxSpeed:
      if (value < 1.0f || value > 1000.0f) break;
      params_.maxSpeed = value;
      if (state_ != TeleopState::Align) limiter_.setLimits(params_.maxSpeed, params_.acceleration);
      message("vmax = %.0f steps/s", value);
      return;
    case ParamId::Acceleration:
      if (value < 10.0f || value > 20000.0f) break;
      params_.acceleration = value;
      limiter_.setLimits(state_ == TeleopState::Align ? params_.armSpeed : params_.maxSpeed,
                         params_.acceleration);
      message("amax = %.0f steps/s^2", value);
      return;
    case ParamId::LeadMs:
      if (value < 0.0f || value > 300.0f) break;
      params_.leadMs = value;
      message("lead = %.0f ms", value);
      return;
    case ParamId::MinCutoff:
      if (value < 0.01f || value > 30.0f) break;
      params_.minCutoffHz = value;
      filter_.setParams(params_.minCutoffHz, params_.beta, config::kFilterDCutoffHz);
      message("mincut = %.3f Hz", value);
      return;
    case ParamId::Beta:
      if (value < 0.0f || value > 10.0f) break;
      params_.beta = value;
      filter_.setParams(params_.minCutoffHz, params_.beta, config::kFilterDCutoffHz);
      message("beta = %.4f", value);
      return;
  }
  message("rejected: value %.3f out of range", value);
}

void TeleopController::captureExo(bool endpointA) {
  if (armed_) {
    message("disarm before capturing exo endpoints");
    return;
  }
  if (!encFresh_) {
    message("encoder %u not healthy, capture refused", joint_.encoderIndex);
    return;
  }
  if (endpointA) {
    joint_.exoRef = encRaw_;
    joint_.exoA = 0;
    joint_.exoB = 0;  // B is relative to the new reference: capture it again
    exoRefSet_ = true;
    filter_.reset();
    message("exo A captured (raw %u). Move to the other end and run `exo b`.", encRaw_);
    return;
  }
  if (!exoRefSet_) {
    message("capture `exo a` first");
    return;
  }
  const int32_t span = teleop::unwrapCounts(encRaw_, joint_.exoRef);
  if (abs(span) < config::kMinExoSpanCounts) {
    message("rejected: span %.1f deg is too small", span * teleop::kDegreesPerCount);
    return;
  }
  joint_.exoB = static_cast<int16_t>(span);
  message("exo B captured: span %.1f deg (%ld counts)", span * teleop::kDegreesPerCount,
          static_cast<long>(span));
}

void TeleopController::pingAll() {
  for (uint8_t i = 0; i < config::kHandServoCount; ++i) {
    const uint8_t id = config::kHandServoIds[i];
    uint16_t model = 0;
    if (bus_.ping(id, &model)) {
      message("servo %u: model %u%s", id, model, model == bus_.expectedModel() ? "" : " (unexpected)");
    } else {
      message("servo %u: no reply", id);
    }
  }
}

// ---- arming -------------------------------------------------------------------

bool TeleopController::arm() {
  if (armed_) {
    message("already armed");
    return false;
  }
  if (!joint_.ready()) {
    message("arm refused: joint not calibrated (exo a / exo b, servo sa / sb)");
    return false;
  }
  if (encBadCycles_ != 0) {
    message("arm refused: encoder not healthy");
    return false;
  }
  const uint8_t id = joint_.servoId;
  ServoState st;
  if (!bus_.readState(id, st)) {
    message("arm refused: no status reply from servo %u", id);
    return false;
  }
  if (st.errorByte != 0 || st.status != 0) {
    message("arm refused: servo error byte 0x%02X status 0x%02X", st.errorByte, st.status);
    return false;
  }
  if (st.position < joint_.servoMin || st.position > joint_.servoMax) {
    message("arm refused: position %d outside travel [%d, %d]; move it back by hand", st.position,
            joint_.servoMin, joint_.servoMax);
    return false;
  }

  // Goal before torque: a goal write switches torque on by itself on SCS0009,
  // so the joint may only ever be told to hold where it already is.
  const int16_t hold = st.position;
  const char* problem = nullptr;
  ServoState check;
  if (!bus_.writeGoal(id, hold, params_.armSpeed)) {
    problem = "goal write failed";
  } else if (!bus_.writeTorque(id, true)) {
    problem = "torque write failed";
  } else if (!bus_.readState(id, check)) {
    problem = "no read-back after arming";
  } else if (!check.torqueOn) {
    problem = "torque did not switch on";
  } else if (check.goal != hold) {
    problem = "goal register does not match the hold position";
  } else if (abs(check.position - hold) > config::kArmHoldTolerance) {
    problem = "joint moved while arming";
  }
  if (problem != nullptr) {
    disarm(Fault::None, problem);
    return false;
  }

  armed_ = true;
  state_ = TeleopState::Align;
  limiter_.reset(hold);
  goal_ = hold;
  jointSupervisor_.reset();
  supplySupervisor_.reset();
  lastFault_ = Fault::None;
  message("armed, holding %d -> ALIGN", hold);
  return true;
}

void TeleopController::disarm(Fault fault, const char* reason) {
  armed_ = false;  // first, so no goal can follow
  state_ = TeleopState::Idle;
  const uint8_t id = joint_.servoId;
  bus_.syncTorqueOff(&id, 1);
  bus_.writeTorque(id, false);
  ServoState check;
  torqueOffConfirmed_ = bus_.readState(id, check) && !check.torqueOn;
  lastFault_ = fault;
  jointSupervisor_.reset();
  message("DISARM: %s; torque off %s", reason, torqueOffConfirmed_ ? "confirmed" : "NOT CONFIRMED");
}

void TeleopController::torqueOffAll(const char* reason) {
  armed_ = false;
  state_ = TeleopState::Idle;
  bus_.syncTorqueOff(config::kHandServoIds, config::kHandServoCount);
  bool allConfirmed = true;
  for (uint8_t i = 0; i < config::kHandServoCount; ++i) {
    const uint8_t id = config::kHandServoIds[i];
    ServoState check;
    if (!bus_.readState(id, check)) continue;  // absent servo: nothing to switch off
    if (check.torqueOn) {
      bus_.writeTorque(id, false);
      allConfirmed &= bus_.readState(id, check) && !check.torqueOn;
    }
  }
  torqueOffConfirmed_ = allConfirmed;
  jointSupervisor_.reset();
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

  Snapshot s;
  s.cycle = cycle_;
  s.uptimeMs = millis();
  s.state = state_;
  s.armed = armed_;
  s.lastFault = lastFault_;
  s.torqueOffConfirmed = torqueOffConfirmed_;
  s.echo = port_.echo();
  s.encFresh = encFresh_;
  s.encRaw = encRaw_;
  s.exoCounts = exoCounts_;
  s.exoFilteredCounts = exoFiltered_;
  s.target = target_;
  s.goal = goal_;
  s.speedRegister = speedRegister_;
  s.goalSent = goalSent_;
  s.servoOk = servoOk_;
  s.servo = servo_;
  s.joint = joint_;
  s.exoRefSet = exoRefSet_;
  s.params = params_;
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
