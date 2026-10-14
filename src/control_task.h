// 100 Hz control loop: encoder -> filter -> map -> rate limit -> servo goal,
// with arming, the IDLE/ALIGN/FOLLOW/HOLD state machine and safety supervision.
//
// Only this task touches the servo bus and the encoder bus. Other tasks talk to
// it through a command queue (in) and a message queue plus snapshot (out), so
// nothing on the control core ever blocks on USB.

#pragma once

#include <Arduino.h>
#include <AS5048BGroup.h>
#include <feetech_port.h>
#include <joint_map.h>
#include <one_euro.h>
#include <rate_limiter.h>
#include <servo_bus.h>
#include <supervisor.h>

#include "config.h"

enum class TeleopState : uint8_t { Idle, Align, Follow, Hold };
const char* teleopStateToString(TeleopState state);

enum class CommandType : uint8_t {
  Arm,
  Disarm,
  Estop,
  CaptureExoA,
  CaptureExoB,
  SetParam,
  Ping,
  ResetStats,
};

enum class ParamId : uint8_t { ServoA, ServoB, MaxSpeed, Acceleration, LeadMs, MinCutoff, Beta };

struct Command {
  CommandType type = CommandType::Disarm;
  ParamId param = ParamId::MaxSpeed;
  float value = 0.0f;
};

struct MotionParams {
  float maxSpeed = config::kMaxSpeed;
  float acceleration = config::kAcceleration;
  float leadMs = config::kLeadMs;
  float minCutoffHz = config::kFilterMinCutoffHz;
  float beta = config::kFilterBeta;
  uint16_t armSpeed = config::kArmSpeed;
};

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

struct Snapshot {
  uint32_t cycle = 0;
  uint32_t uptimeMs = 0;
  TeleopState state = TeleopState::Idle;
  bool armed = false;
  teleop::Fault lastFault = teleop::Fault::None;
  bool torqueOffConfirmed = true;
  bool echo = false;

  bool encFresh = false;
  uint16_t encRaw = 0;
  float exoCounts = 0.0f;          // unwrapped, relative to exoRef
  float exoFilteredCounts = 0.0f;
  float target = 0.0f;             // mapped servo position
  float goal = 0.0f;               // rate-limited value sent to the servo
  uint16_t speedRegister = 0;
  bool goalSent = false;

  bool servoOk = false;
  ServoState servo;

  teleop::JointMap joint;
  bool exoRefSet = false;
  MotionParams params;

  TimingStats timing;
  FeetechPortStats port;
  AS5048BCycleStats encoderCycle;
  EncoderSummary encoders[config::kEncoderCount];
};

class TeleopController {
 public:
  TeleopController(ServoBus& bus, FeetechPort& port, AS5048BGroup& encoders);

  // Call from setup(): creates queues, arms the E-stop interrupt, switches
  // every hand servo to torque-off and reports which ones answer.
  bool begin();
  void startTask();

  bool post(const Command& command);
  void snapshot(Snapshot& out);
  // Event text for the console; returns false when the queue is empty.
  bool popMessage(char* buffer, size_t length);

 private:
  struct Message {
    char text[160];
  };

  static void taskEntry(void* arg);
  static void IRAM_ATTR estopIsr();

  void run();
  void cycle(uint32_t wakeUs);
  void updateTiming(uint32_t wakeUs);
  void handleCommand(const Command& command);
  void setParam(ParamId param, float value);
  void captureExo(bool endpointA);
  void pingAll();
  bool arm();
  void disarm(teleop::Fault fault, const char* reason);
  void torqueOffAll(const char* reason);
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
  Snapshot shared_;
  volatile bool estopRequested_ = false;

  teleop::JointMap joint_;
  bool exoRefSet_ = false;
  MotionParams params_;
  teleop::OneEuroFilter filter_;
  teleop::RateLimiter limiter_;
  teleop::JointSupervisor jointSupervisor_;
  teleop::SupplySupervisor supplySupervisor_;

  TeleopState state_ = TeleopState::Idle;
  bool armed_ = false;
  teleop::Fault lastFault_ = teleop::Fault::None;
  teleop::Fault lastReportedFault_ = teleop::Fault::None;
  bool torqueOffConfirmed_ = true;

  uint32_t cycle_ = 0;
  uint8_t encBadCycles_ = 0;
  bool encFresh_ = false;
  uint16_t encRaw_ = 0;
  float exoCounts_ = 0.0f;
  float exoFiltered_ = 0.0f;
  float target_ = 0.0f;
  float goal_ = 0.0f;
  uint16_t speedRegister_ = 0;
  bool goalSent_ = false;
  bool servoOk_ = false;
  ServoState servo_;
  uint32_t holdSinceMs_ = 0;
  uint32_t encGoodSinceMs_ = 0;
  bool encGoodTiming_ = false;

  TimingStats timing_;
  uint32_t jitterHistogram_[TimingStats::kBins] = {0};
  uint32_t lastWakeUs_ = 0;
};
