// USB-CDC console: line commands in (parsed by teleop::parseCommand), event
// messages, tables, monitor rows and Teleplot traces out. Runs on the core
// that does not host the control loop.
//
// `host on` switches it to the framed host link for the PC GUI
// (docs/host-protocol.md): commands arrive as COMMAND frames in the same
// grammar, and telemetry, events, replies and every printed line go out as
// frames. A missing heartbeat or HOST_EXIT returns to the text console.

#pragma once

#include <Arduino.h>
#include <frame.h>
#include <host_messages.h>

#include "console_out.h"
#include "control_task.h"

class Console {
 public:
  explicit Console(TeleopController& controller) : controller_(controller), out_(Serial) {}
  void startTask();

 private:
  static void taskEntry(void* arg);
  void run();
  void textStep();
  void drainMessages();
  void handleLine(const char* line);
  // Console-only verbs; false if the command belongs to the controller.
  bool runLocal(const teleop::ParsedCommand& cmd);
  bool formatDumpText();  // into dumpBuffer
  void printHelp();
  void printStatus();
  void printStats();
  void printEncoders();
  void printJoints();
  void printDump();
  void printMonitorRow();
  void plot();

  // ---- host link ----
  void enterHostMode();
  void exitHostMode(hostlink::ByeReason reason);
  void hostStep();
  void handleFrame(const hostlink::FrameHeader& header, const uint8_t* payload, size_t length);
  void handleHostCommand(uint16_t seq, const uint8_t* text, size_t length);
  void drainHostEvents();
  void sendTelemetry(const TelemetrySample& sample);
  void sendSlowStatus();
  void sendHello();
  void sendAck(uint16_t seq, hostlink::AckStatus status, const char* text = "");
  void sendDone(uint16_t seq, bool ok);
  // Returns false if the frame was not (completely) written. `optional`
  // frames are dropped rather than waiting for room in the USB buffer.
  bool sendFrame(hostlink::MsgType type, uint16_t seq, const uint8_t* payload, size_t length,
                 bool optional = false);
  static void logSink(void* context, const char* line, size_t length);

  TeleopController& controller_;
  ConsoleOut out_;
  Snapshot snapshot_;
  char line_[128];
  size_t lineLength_ = 0;
  uint8_t plotJoint_ = 0;  // servo id, 0 = off
  uint32_t lastPlotMs_ = 0;
  uint32_t lastMonitorMs_ = 0;

  bool hostMode_ = false;
  hostlink::FrameReader reader_;
  hostlink::HostHello hostConfig_;
  uint16_t txSeq_ = 0;
  uint32_t lastRxMs_ = 0;
  uint32_t lastSlowMs_ = 0;
  uint32_t droppedTx_ = 0;  // optional frames dropped for lack of USB buffer
  TelemetrySample sample_;
  hostlink::SlowStatus slow_;
  uint8_t payload_[hostlink::kMaxPayload];
  uint8_t logPayload_[256];
  uint8_t wire_[hostlink::maxWireSize(hostlink::kMaxPayload)];
};
