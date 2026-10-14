// USB-CDC text console: line commands in (parsed by teleop::parseCommand),
// event messages, tables, monitor rows and Teleplot traces out. Runs on the
// core that does not host the control loop.

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
  void drainMessages();
  void handleLine(const char* line);
  void printHelp();
  void printStatus();
  void printStats();
  void printEncoders();
  void printJoints();
  void printDump();
  void printMonitorRow();
  void plot();

  TeleopController& controller_;
  Snapshot snapshot_;
  char line_[128];
  size_t lineLength_ = 0;
  uint8_t plotJoint_ = 0;  // servo id, 0 = off
  uint32_t lastPlotMs_ = 0;
  uint32_t lastMonitorMs_ = 0;
};
