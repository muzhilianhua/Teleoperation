// Build-time configuration for the teleoperation firmware.
// Values marked [bringup] come from servo_bringup (measured on this hand) and
// should stay in sync with it. Operator calibration is not here: it is entered
// on the console and stored in NVS.

#pragma once

#include <config_blob.h>
#include <joint_config.h>
#include <stdint.h>

namespace config {

// ---- Encoders: AS5048B on one I2C bus, addresses burned into OTP ---------
constexpr int kI2cSda = 8;
constexpr int kI2cScl = 9;
constexpr uint32_t kI2cHz = 400000;
constexpr uint16_t kI2cTimeoutMs = 2;  // a hung bus must not stall the control loop
constexpr uint8_t kEncoderAddresses[] = {0x40, 0x44, 0x48, 0x4C, 0x50, 0x54, 0x58, 0x5C, 0x60};
constexpr uint8_t kEncoderCount = sizeof(kEncoderAddresses);

// ---- Servo bus: FE-URT-2 TTL port on UART1 --------------------------------
constexpr int kServoRxPin = 18;
constexpr int kServoTxPin = 17;
constexpr uint32_t kServoBaud = 1000000;  // [bringup] SERVO_BAUDRATE
constexpr uint32_t kServoLatencyUs = 2000;
constexpr uint8_t kEchoProbeId = 253;     // [bringup] diagnose S2 probe id, must be unused

// ---- E-stop: BOOT button on the DevKit, active low ------------------------
constexpr int kEstopPin = 0;

// ---- Control loop ---------------------------------------------------------
constexpr uint32_t kControlHz = 100;
constexpr uint32_t kControlPeriodUs = 1000000UL / kControlHz;
constexpr uint8_t kFullEncoderReadEvery = 10;  // AGC / diagnostics refresh
constexpr uint32_t kWatchdogTimeoutS = 2;
// Full status reads cost ~0.55 ms each; reading all 9 servos every cycle would
// use half the period. Up to this many are read per cycle, round-robin.
constexpr uint8_t kServoReadsPerCycle = 3;
// A goal is written only if the joint's last good read (showing torque on) is
// at most this old.
constexpr uint32_t kServoStaleMs = 50;
// An armed joint without a good read for this long is a lost bus ([bringup] stale_s).
constexpr uint32_t kServoCommStaleMs = 100;
constexpr uint8_t kControlCore = 1;
constexpr uint8_t kConsoleCore = 0;

// ---- Joint table: one entry per hand servo --------------------------------
// Travel limits: [bringup] JOINTS (config.py:61-71). They can only be changed
// here, never from the console.
// vmax ~ 1.2 x the faster-direction saturation speed and vsat ~ 0.9 x the
// slower-direction one, from [bringup] reports/verify_motion_*.json speed
// sweeps. Joints 3/4/5/7 are force-limited (friction / tendon preload, 2-10
// steps/s): their values are placeholders until the mechanics are fixed, and
// their tracking check will trip until then.
// Joint 9 was added after servo_bringup; travel 0/0 means "not defined": the
// joint can be monitored but not enabled, calibrated, swept or armed until
// its travel is measured (monitor, move by hand, ~15 steps margin) and
// entered here. Its name is a placeholder too.
// Encoder assignment and enable flags below are only boot defaults; the
// operator sets them with `set <j> enc/on` and stores them in NVS.
// All servos on the bus get torque-off at boot and on E-stop.
constexpr uint16_t kAcceleration = 8000;  // steps/s^2
inline constexpr teleop::JointDefaults kJoints[teleop::kMaxJoints] = {
    // id  name             min  max  vmax vsat amax           encoder            enabled
    {1, "triple_pip",      350, 730, 660, 245, kAcceleration, 5,                 true},
    {2, "triple_dip",      600, 890, 660, 370, kAcceleration, teleop::kNoEncoder, false},
    {3, "index_pip",       230, 630, 400, 50,  kAcceleration, teleop::kNoEncoder, false},
    {4, "index_dip",       250, 590, 400, 50,  kAcceleration, teleop::kNoEncoder, false},
    {5, "index_mcp",       420, 600, 400, 50,  kAcceleration, teleop::kNoEncoder, false},
    {6, "thumb_mcp_rot",   300, 700, 690, 365, kAcceleration, teleop::kNoEncoder, false},
    {7, "thumb_mcp",       650, 900, 400, 50,  kAcceleration, teleop::kNoEncoder, false},
    {8, "thumb_ip",         20, 280, 380, 230, kAcceleration, teleop::kNoEncoder, false},
    {9, "joint9",          480, 950, 400, 50,  kAcceleration, teleop::kNoEncoder, false},
};
constexpr teleop::ServoBackend kServoBackend = teleop::ServoBackend::Scs;

// ---- Global defaults (runtime-adjustable with `set`, stored in NVS) -------
inline constexpr teleop::GlobalSettings kGlobalDefaults = {
    0.0f,   // leadMs: look-ahead against stick-slip, tuned in M4
    1.0f,   // minCutoffHz
    0.01f,  // beta, 1/deg
    25.0f,  // jumpDeg: 2500 deg/s at 100 Hz, beyond any human finger
};
constexpr float kFilterDCutoffHz = 1.0f;
constexpr uint16_t kArmSpeed = 50;        // [bringup] arm_speed, also the ALIGN speed limit
constexpr int16_t kArmHoldTolerance = 5;  // [bringup] arm_hold_tolerance
constexpr float kAlignTolerance = 3.0f;   // steps, ALIGN -> FOLLOW
// Streamed speed register = clamp(gain * |v|, floor, cap)  [bringup] dynamic mode
constexpr float kStreamSpeedGain = 1.3f;
constexpr uint16_t kStreamSpeedFloor = 50;
constexpr uint16_t kStreamSpeedCap = 1000;

// ---- Encoder fault handling -----------------------------------------------
constexpr uint8_t kEncoderFailCycles = 3;    // consecutive bad reads -> HOLD
constexpr uint32_t kHoldTimeoutMs = 3000;    // HOLD longer than this -> torque off
constexpr uint32_t kHoldRecoverMs = 200;     // encoder healthy this long -> re-ALIGN
// Exo tracking (multi-turn capable). A gap longer than this may hide more
// than half an encoder turn (finger speed x transmission ratio), so a
// multi-turn joint must be homed again; single-turn joints re-resolve.
constexpr uint32_t kTrackingLossMs = 100;
// Dead zone beyond each calibrated end (10 deg) used to resolve single-turn
// joints at boot; range + 2 x dead zone >= one turn makes a joint multi-turn.
constexpr int32_t kExoDeadZoneCounts = 455;

// ---- Monitor mode -----------------------------------------------------------
constexpr uint32_t kMonitorPeriodMs = 100;   // console table rate

}  // namespace config
