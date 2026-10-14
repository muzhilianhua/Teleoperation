// USB-CDC text console (M0): line commands in, event messages, status tables
// and Teleplot traces out. Runs on the core that does not host the control loop.

#pragma once

#include <Arduino.h>

#include "control_task.h"

class Console {
 public:
  explicit Console(TeleopController& controller) : controller_(controller) {}
  void startTask();

 private:
  static void taskEntry(void* arg);
  void run();
  void handleLine(char* line);
  void handleSet(const char* key, const char* value);
  void printHelp();
  void printStatus();
  void printStats();
  void printEncoders();
  void plot();

  TeleopController& controller_;
  Snapshot snapshot_;
  char line_[96];
  size_t lineLength_ = 0;
  bool plotting_ = false;
  uint32_t lastPlotMs_ = 0;
};
