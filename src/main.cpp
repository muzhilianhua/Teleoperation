// Exoskeleton -> dexterous hand teleoperation, milestone M2:
// 9-joint hand with per-joint state machines and fault isolation, console
// calibration stored in NVS, on-device speed sweep, grip force limiting.

#include <Arduino.h>
#include <AS5048B.h>
#include <AS5048BGroup.h>
#include <Wire.h>
#include <esp_task_wdt.h>
#include <feetech_port.h>
#include <scs_bus.h>

#include "config.h"
#include "console.h"
#include "control_task.h"

namespace {

AS5048B encoders[] = {
    AS5048B(config::kEncoderAddresses[0]), AS5048B(config::kEncoderAddresses[1]),
    AS5048B(config::kEncoderAddresses[2]), AS5048B(config::kEncoderAddresses[3]),
    AS5048B(config::kEncoderAddresses[4]), AS5048B(config::kEncoderAddresses[5]),
    AS5048B(config::kEncoderAddresses[6]), AS5048B(config::kEncoderAddresses[7]),
    AS5048B(config::kEncoderAddresses[8]),
};
static_assert(sizeof(encoders) / sizeof(encoders[0]) == config::kEncoderCount,
              "one AS5048B object per configured address");
const char* const kEncoderNames[] = {"E40", "E44", "E48", "E4C", "E50", "E54", "E58", "E5C", "E60"};

AS5048BGroup encoderGroup;
FeetechPort servoPort(Serial1);
ScsBus servoBus(servoPort);
TeleopController controller(servoBus, servoPort, encoderGroup);
Console console(controller);

}  // namespace

void setup() {
  Serial.setRxBufferSize(4096);  // a pasted `dump` (~1.5 KB) must not overflow
  Serial.begin(115200);
  const uint32_t waitStart = millis();
  while (!Serial && millis() - waitStart < 1500) delay(10);

  Wire.begin(config::kI2cSda, config::kI2cScl, config::kI2cHz);
  Wire.setTimeOut(config::kI2cTimeoutMs);
  for (uint8_t i = 0; i < config::kEncoderCount; ++i) encoderGroup.add(encoders[i], kEncoderNames[i]);
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
