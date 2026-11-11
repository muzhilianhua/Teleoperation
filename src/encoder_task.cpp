#include "encoder_task.h"

bool EncoderSampler::begin() {
  done_ = xSemaphoreCreateBinary();
  if (done_ == nullptr) return false;
  publish(0);
  return true;
}

void EncoderSampler::startTask() {
  xTaskCreatePinnedToCore(taskEntry, "encoders", 4096, this, config::kEncoderPriority, &task_,
                          config::kEncoderCore);
}

void EncoderSampler::taskEntry(void* arg) { static_cast<EncoderSampler*>(arg)->run(); }

void EncoderSampler::run() {
  // Not on the task watchdog: a hung bus legitimately blocks here for ~1 s,
  // and the control task already sees it as late passes.
  for (;;) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    if (clearRequested_) {
      clearRequested_ = false;
      group_.clearStats();
    }
    const uint32_t request = requested_;
    group_.sample(full_);
    publish(request);
    xSemaphoreGive(done_);
  }
}

void EncoderSampler::trigger(bool full) {
  full_ = full;
  requested_ = requested_ + 1;
  if (task_ != nullptr) xTaskNotifyGive(task_);
}

bool EncoderSampler::take(EncoderPass& out, TickType_t wait) {
  const TickType_t start = xTaskGetTickCount();
  for (;;) {
    copyLatest(out);
    if (out.request == requested_) return true;
    const TickType_t elapsed = xTaskGetTickCount() - start;
    if (elapsed >= wait) return false;
    // Gives from earlier (late) passes may still be pending: re-check the request.
    xSemaphoreTake(done_, wait - elapsed);
  }
}

void EncoderSampler::publish(uint32_t request) {
  EncoderPass& p = scratch_;
  p.request = request;
  p.endUs = micros();
  p.cycle = group_.stats();
  for (uint8_t i = 0; i < config::kEncoderCount; ++i) {
    EncoderSummary& e = p.channels[i];
    if (i >= group_.count()) {
      e = EncoderSummary();
      continue;
    }
    const AS5048BChannel& ch = group_.channel(i);
    e.address = ch.encoder->address();
    e.state = ch.state;
    e.valid = ch.valid;
    e.readOk = ch.lastStatus == AS5048BStatus::Ok;
    e.angle = ch.reading.angle;
    e.agc = ch.reading.agc;
    e.magnetOk = ch.reading.diag.magnetOk();
    e.errors = ch.errorCount;
  }
  portENTER_CRITICAL(&mux_);
  latest_ = p;
  portEXIT_CRITICAL(&mux_);
}

void EncoderSampler::copyLatest(EncoderPass& out) {
  portENTER_CRITICAL(&mux_);
  out = latest_;
  portEXIT_CRITICAL(&mux_);
}
