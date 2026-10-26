// Exoskeleton -> dexterous hand teleoperation, milestone M2:
// 9-joint hand with per-joint state machines and fault isolation, console
// calibration stored in NVS, on-device speed sweep, grip force limiting.
// One board per hand: the GPIO4 strap selects the right or left hand.

#include <Arduino.h>
#include <AS5048B.h>
#include <AS5048BGroup.h>
#include <Wire.h>
#include <esp_task_wdt.h>
#include <feetech_port.h>
#include <feetech_mixed_bus.h>

#include "config.h"
#include "console.h"
#include "control_task.h"
#include "hand.h"

namespace {

// Both gloves' encoders, by address; setup() adds only this hand's set.
AS5048B rightEncoders[] = {
    AS5048B(config::kRightEncoderAddresses[0]), AS5048B(config::kRightEncoderAddresses[1]),
    AS5048B(config::kRightEncoderAddresses[2]), AS5048B(config::kRightEncoderAddresses[3]),
    AS5048B(config::kRightEncoderAddresses[4]), AS5048B(config::kRightEncoderAddresses[5]),
    AS5048B(config::kRightEncoderAddresses[6]), AS5048B(config::kRightEncoderAddresses[7]),
    AS5048B(config::kRightEncoderAddresses[8]),
};
AS5048B leftEncoders[] = {
    AS5048B(config::kLeftEncoderAddresses[0]), AS5048B(config::kLeftEncoderAddresses[1]),
    AS5048B(config::kLeftEncoderAddresses[2]), AS5048B(config::kLeftEncoderAddresses[3]),
    AS5048B(config::kLeftEncoderAddresses[4]), AS5048B(config::kLeftEncoderAddresses[5]),
    AS5048B(config::kLeftEncoderAddresses[6]), AS5048B(config::kLeftEncoderAddresses[7]),
    AS5048B(config::kLeftEncoderAddresses[8]),
};
static_assert(sizeof(rightEncoders) / sizeof(rightEncoders[0]) == config::kEncoderCount &&
                  sizeof(leftEncoders) / sizeof(leftEncoders[0]) == config::kEncoderCount,
              "one AS5048B object per configured address");
char encoderNames[config::kEncoderCount][4];  // "E08": the AS5048BGroup keeps the pointers

AS5048BGroup encoderGroup;
FeetechPort servoPort(Serial1);
FeetechMixedBus servoBus(servoPort);
TeleopController controller(servoBus, servoPort, encoderGroup);
Console console(controller);

}  // namespace

void setup() {
  hand::select();  // first: the joint table, encoders and NVS key depend on it
  Serial.setRxBufferSize(4096);  // a pasted `dump` (~1.5 KB) must not overflow
  // Host link: ~50 KB/s of frames. A short timeout bounds how long a write
  // can stall the console when nobody reads the port.
  Serial.setTxBufferSize(4096);
  Serial.setTxTimeoutMs(20);
  Serial.begin(115200);
  const uint32_t waitStart = millis();
  while (!Serial && millis() - waitStart < 1500) delay(10);

  Wire.begin(config::kI2cSda, config::kI2cScl, config::kI2cHz);
  Wire.setTimeOut(config::kI2cTimeoutMs);
  Serial.printf("[boot] hand: %s (GPIO%d %s)\n", hand::name(), config::kHandStrapPin,
                hand::which() == config::Hand::Left ? "to GND" : "open");
  if (hand::usedBeforeSelect()) {
    Serial.println("[boot] BUG: the hand's tables were read before the GPIO4 strap; settings may be the right hand's");
  }
  AS5048B* encoders = hand::which() == config::Hand::Left ? leftEncoders : rightEncoders;
  for (uint8_t i = 0; i < config::kEncoderCount; ++i) {
    snprintf(encoderNames[i], sizeof(encoderNames[i]), "E%02X", hand::encoderAddresses()[i]);
    encoderGroup.add(encoders[i], encoderNames[i]);
  }
  const uint8_t online = encoderGroup.beginAll();
  Serial.printf("[boot] encoders responding: %u / %u\n", online, config::kEncoderCount);

  servoPort.begin(config::kServoBaud, config::kServoRxPin, config::kServoTxPin,
                  config::kServoLatencyUs);
  servoPort.detectEcho(config::kEchoProbeId);

  const esp_err_t wdt = esp_task_wdt_init(config::kWatchdogTimeoutS, true);
  Serial.printf("[boot] task watchdog %lu s: %s\n", (unsigned long)config::kWatchdogTimeoutS,
                esp_err_to_name(wdt));

  if (!controller.begin()) {
    Serial.println("[boot] FATAL: could not create control queues");
    for (;;) delay(1000);
  }
  controller.startTask();
  console.startTask();
}

void loop() {
  // All work runs in the pinned control and console tasks.
  vTaskDelete(nullptr);
}
