// Encoder sampling on its own task (core 0).
//
// With a hung bus state machine the I2C driver (arduino-esp32 2.0.14 / IDF
// 4.4) blocks for ~1 s regardless of Wire.setTimeOut. On the control task
// that tripped the 2 s task watchdog (e.g. a board with no glove: weak
// internal pull-ups, nine offline encoders probed every second). Here a hang
// only delays samples: the control task triggers a pass, waits a bounded time
// for it and treats a missing pass as "no sample" (encoder HOLD logic).
//
// After boot (AS5048BGroup::beginAll in setup) only this task touches Wire.

#pragma once

#include <Arduino.h>
#include <AS5048BGroup.h>

#include "config.h"

struct EncoderSummary {
  uint8_t address = 0;
  AS5048BChannelState state = AS5048BChannelState::Offline;
  bool valid = false;
  bool readOk = false;  // the channel's read in this pass succeeded
  uint16_t angle = 0;
  uint8_t agc = 0;
  bool magnetOk = false;
  uint32_t errors = 0;
};

struct EncoderPass {
  uint32_t request = 0;  // trigger this pass answered (0: boot state from beginAll)
  uint32_t endUs = 0;    // micros() at the end of the pass
  AS5048BCycleStats cycle;
  EncoderSummary channels[config::kEncoderCount];
};

class EncoderSampler {
 public:
  explicit EncoderSampler(AS5048BGroup& group) : group_(group) {}

  // After AS5048BGroup::beginAll; publishes the boot state as the first pass.
  bool begin();
  void startTask();

  // Control task only. Starts a pass (full: AGC/diagnostics too); while the
  // previous pass still runs, the next pass answers this trigger instead.
  void trigger(bool full);
  // Control task only. Waits up to `wait` ticks for the pass answering the
  // last trigger; `out` always receives the latest pass. False: it is late.
  bool take(EncoderPass& out, TickType_t wait);
  // Any task: the encoder task clears its statistics before the next pass.
  void requestClearStats() { clearRequested_ = true; }

 private:
  static void taskEntry(void* arg);
  void run();
  void publish(uint32_t request);
  void copyLatest(EncoderPass& out);

  AS5048BGroup& group_;
  TaskHandle_t task_ = nullptr;
  SemaphoreHandle_t done_ = nullptr;
  portMUX_TYPE mux_ = portMUX_INITIALIZER_UNLOCKED;
  volatile uint32_t requested_ = 0;
  volatile bool full_ = false;
  volatile bool clearRequested_ = false;
  EncoderPass scratch_;  // encoder task only
  EncoderPass latest_;   // under mux_
};
