// 100 Hz control loop over the joint table: encoder -> jump guard -> filter ->
// map -> rate limit -> one SYNC_WRITE per cycle. Every joint has its own state
// machine (OFF/ALIGN/FOLLOW/HOLD) so a joint-level fault only switches that
// joint off; hand-level faults switch everything off. Also owns arming, safety
// supervision and the settings (calibration) store.
//
// Only this task touches the servo bus, the encoder bus and NVS. The console
// talks to it through a command queue (in) and a message queue plus snapshot
// (out), so nothing on the control core ever blocks on USB.

#pragma once

#include <Arduino.h>
#include <AS5048BGroup.h>
#include <command_parser.h>
#include <feetech_port.h>
#include <grip_limiter.h>
#include <joint_config.h>
#include <jump_guard.h>
#include <one_euro.h>
#include <rate_limiter.h>
#include <servo_bus.h>
#include <supervisor.h>

#include "config.h"

enum class JointState : uint8_t { Off, Align, Follow, Hold };
const char* jointStateToString(JointState state);

using Command = teleop::ParsedCommand;

struct TimingStats {
  static constexpr uint16_t kBinUs = 10;
  static constexpr uint16_t kBins = 200;  // 0..2 ms of period jitter

  uint32_t cycles = 0;
  uint32_t overruns = 0;     // execution longer than one period
  uint32_t lastPeriodUs = 0;
  uint32_t maxJitterUs = 0;  // |period - nominal|
  uint32_t p99JitterUs = 0;
  uint32_t lastExecUs = 0;
  uint32_t maxExecUs = 0;
  uint64_t totalExecUs = 0;

  uint32_t averageExecUs() const { return cycles ? static_cast<uint32_t>(totalExecUs / cycles) : 0; }
};

struct EncoderSummary {
  uint8_t address = 0;
  AS5048BChannelState state = AS5048BChannelState::Offline;
  bool valid = false;
  uint16_t angle = 0;
  uint8_t agc = 0;
  bool magnetOk = false;
  uint32_t errors = 0;
};

struct JointSnapshot {
  teleop::JointSettings settings;
  JointState state = JointState::Off;
  teleop::Fault lastFault = teleop::Fault::None;  // what last switched this joint off
  bool ready = false;
  bool refCaptured = false;  // `cal a` done (or exo calibrated)
  bool encFresh = false;
  uint16_t encRaw = 0;
  float exoCounts = 0.0f;          // unwrapped, relative to exoRef
  float exoFilteredCounts = 0.0f;
  float target = 0.0f;             // mapped servo position
  float goal = 0.0f;               // rate-limited target
  float sentGoal = 0.0f;           // goal after the grip limiter: what the servo receives
  float gripLead = 0.0f;           // current grip limiter lead (steps)
  bool contact = false;            // grip limiter sees the finger pressing on something
  float reference = 0.0f;          // servo-capability model used by the tracking check
  uint16_t speedRegister = 0;
  bool goalSent = false;
  bool servoOk = false;            // last read attempt succeeded
  uint32_t readAgeMs = 0;          // since the last good read
  ServoState servo;
  uint32_t jumpRejects = 0;
};

struct Snapshot {
  uint32_t cycle = 0;
  uint32_t uptimeMs = 0;
  uint8_t armedCount = 0;
  teleop::Fault lastHandFault = teleop::Fault::None;
  bool torqueOffConfirmed = true;
  bool echo = false;
  bool monitor = false;
  teleop::GlobalSettings global;
  JointSnapshot joints[teleop::kMaxJoints];
  int16_t monitorPosition[teleop::kMaxJoints] = {0};
  bool monitorOk[teleop::kMaxJoints] = {false};
  TimingStats timing;
  FeetechPortStats port;
  AS5048BCycleStats encoderCycle;
  EncoderSummary encoders[config::kEncoderCount];
};

class TeleopController {
 public:
  TeleopController(ServoBus& bus, FeetechPort& port, AS5048BGroup& encoders);

  // Call from setup(): creates queues, arms the E-stop interrupt, loads the
  // stored settings, switches every hand servo to torque-off and pings them.
  bool begin();
  void startTask();

  // Blocks up to `wait` when the queue is full (pasted dumps arrive in bursts).
  bool post(const Command& command, TickType_t wait);
  void snapshot(Snapshot& out);
  // Event text for the console; returns false when the queue is empty.
  bool popMessage(char* buffer, size_t length);

 private:
  struct Message {
    char text[160];
  };

  struct JointChannel {
    const teleop::JointDefaults* def = nullptr;
    teleop::JointSettings settings;
    teleop::OneEuroFilter filter;
    teleop::RateLimiter limiter;
    teleop::RateLimiter model;
    teleop::JumpGuard guard;
    teleop::JointSupervisor supervisor;
    teleop::GripLimiter grip;
    JointState state = JointState::Off;
    teleop::Fault lastFault = teleop::Fault::None;
    bool refCaptured = false;
    // Encoder path
    uint8_t encBad = 0;
    bool encFresh = false;
    uint16_t encRaw = 0;
    float exoCounts = 0.0f;
    float exoFiltered = 0.0f;
    float target = 0.0f;
    // Output path
    float goal = 0.0f;
    float sentGoal = 0.0f;
    bool contact = false;
    float reference = 0.0f;
    uint16_t speedRegister = 0;
    bool goalSent = false;
    // Servo readings (round-robin)
    bool servoOk = false;
    bool readThisCycle = false;
    uint32_t lastGoodReadMs = 0;
    ServoState servo;
    // Hold bookkeeping
    uint32_t holdSinceMs = 0;
    uint32_t encGoodSinceMs = 0;
    bool encGoodTiming = false;
    teleop::Fault reportedFault = teleop::Fault::None;
    uint32_t jumpRejects = 0;

    bool armed() const { return state != JointState::Off; }
  };

  static void taskEntry(void* arg);
  static void IRAM_ATTR estopIsr();

  void run();
  void cycle();
  void updateTiming(uint32_t wakeUs);
  void processEncoder(JointChannel& ch, float dt);
  bool encoderSample(uint8_t index, uint16_t& raw) const;
  void readServos(uint32_t nowMs);
  void readMonitor();
  void stepJoint(uint8_t index, float dt, uint32_t nowMs);
  void writeGoals(uint32_t nowMs);

  void handleCommand(const Command& command);
  void handleSet(const Command& command);
  void handleCal(const Command& command);
  void handleCalHand(const Command& command);
  void handleLimitsCheck();
  void handleLimitsWrite(uint8_t servoId);
  void saveConfig();
  bool loadConfig(bool atBoot);
  void loadDefaults();
  void resetChannelRuntime(JointChannel& ch);
  void applyRuntimeParams(JointChannel& ch);
  JointChannel* channelForServo(uint8_t servoId);
  uint8_t armedCount() const;

  void pingAll();
  bool preflight(uint8_t index, ServoState& state);
  const char* armJoint(uint8_t index, const ServoState& state, uint32_t nowMs);
  void armAll();
  void armOne(uint8_t servoId);
  void disarmJoint(uint8_t index, teleop::Fault fault, const char* reason);
  void disarmAll(teleop::Fault fault, int jointIndex, const char* reason);
  void torqueOffAll(const char* reason);
  bool switchOff(const uint8_t* ids, uint8_t count);
  uint16_t streamSpeed(float velocity) const;
  void publish();
  void message(const char* format, ...) __attribute__((format(printf, 2, 3)));

  static TeleopController* instance_;

  ServoBus& bus_;
  FeetechPort& port_;
  AS5048BGroup& encoders_;

  QueueHandle_t commands_ = nullptr;
  QueueHandle_t messages_ = nullptr;
  TaskHandle_t task_ = nullptr;
  portMUX_TYPE mux_ = portMUX_INITIALIZER_UNLOCKED;
  Snapshot staging_;
  Snapshot shared_;
  volatile bool estopRequested_ = false;

  JointChannel channels_[teleop::kMaxJoints];
  teleop::GlobalSettings global_;
  teleop::SupplySupervisor supplySupervisor_;

  bool monitor_ = false;
  uint8_t monitorIndex_ = 0;
  uint8_t readCursor_ = 0;
  int16_t monitorPosition_[teleop::kMaxJoints] = {0};
  bool monitorOk_[teleop::kMaxJoints] = {false};
  teleop::Fault lastHandFault_ = teleop::Fault::None;
  bool torqueOffConfirmed_ = true;
  teleop::Fault reportedSupplyFault_ = teleop::Fault::None;

  uint32_t cycle_ = 0;
  TimingStats timing_;
  uint32_t jitterHistogram_[TimingStats::kBins] = {0};
  uint32_t lastWakeUs_ = 0;
};
