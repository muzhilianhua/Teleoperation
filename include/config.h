// Build-time configuration for the M0 single-joint teleoperation firmware.
// Values marked [bringup] come from servo_bringup/dexhand_servo/config.py
// (measured on this hand) and should stay in sync with it.

#pragma once

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
// Every servo on the hand; all of them get torque-off at boot and on E-stop.
constexpr uint8_t kHandServoIds[] = {1, 2, 3, 4, 5, 6, 7, 8};
constexpr uint8_t kHandServoCount = sizeof(kHandServoIds);

// ---- E-stop: BOOT button on the DevKit, active low ------------------------
constexpr int kEstopPin = 0;

// ---- Control loop ---------------------------------------------------------
constexpr uint32_t kControlHz = 100;
constexpr uint32_t kControlPeriodUs = 1000000UL / kControlHz;
constexpr uint8_t kFullEncoderReadEvery = 10;  // AGC / diagnostics refresh
constexpr uint32_t kWatchdogTimeoutS = 2;
constexpr uint8_t kControlCore = 1;
constexpr uint8_t kConsoleCore = 0;

// ---- M0 joint: ID 1 triple_pip, the freest-moving joint [bringup] --------
constexpr const char* kJointName = "triple_pip";
constexpr uint8_t kJointEncoderIndex = 0;  // 0x40
constexpr uint8_t kJointServoId = 1;
constexpr int16_t kJointServoMin = 350;    // [bringup] JOINTS id 1 min
constexpr int16_t kJointServoMax = 730;    // [bringup] JOINTS id 1 max

// ---- Motion defaults (runtime-adjustable with `set`) ----------------------
constexpr float kMaxSpeed = 200.0f;       // steps/s, [bringup] speed sweep suggests 190-266 for id 1
constexpr float kAcceleration = 2000.0f;  // steps/s^2
constexpr float kLeadMs = 0.0f;           // look-ahead against stick-slip, tuned in M4
constexpr float kFilterMinCutoffHz = 1.0f;
constexpr float kFilterBeta = 0.01f;      // 1/deg
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
constexpr int32_t kMinExoSpanCounts = 182;   // 4 deg: smaller A-B spans are rejected

}  // namespace config
