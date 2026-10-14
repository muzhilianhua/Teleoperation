// Half-duplex Feetech bus transport over an ESP32 HardwareSerial.
//
// The reply timeout follows servo_bringup's SafePortHandler:
//   byte_time * (expected_length + 3) + latency
// so one dead servo costs a bounded slice of the control cycle instead of the
// 50 ms an SDK default would.

#pragma once

#include <Arduino.h>
#include <feetech_packet.h>

struct FeetechPortStats {
  uint32_t sent = 0;
  uint32_t received = 0;
  uint32_t timeouts = 0;
  uint32_t parseErrors = 0;
  uint32_t echoErrors = 0;     // echo enabled but the echoed bytes did not match
  uint32_t lastReplyUs = 0;    // send start -> reply parsed
  uint32_t maxReplyUs = 0;
};

class FeetechPort {
 public:
  explicit FeetechPort(HardwareSerial& serial) : serial_(serial) {}

  void begin(uint32_t baud, int8_t rxPin, int8_t txPin, uint32_t latencyUs);

  // Pings an id that must not exist on the bus. If the adapter loops our own
  // bytes back to RX, echo filtering is enabled. Returns the detected state.
  bool detectEcho(uint8_t probeId);
  bool echo() const { return echo_; }

  // Drops stale input, transmits, waits for the last bit, consumes the echo.
  bool send(const uint8_t* packet, size_t length);
  // Waits for one status packet; out.params points into an internal buffer
  // that stays valid until the next call.
  feetech::ParseResult receive(uint8_t expectedId, int expectedParams, feetech::StatusPacket& out);

  const FeetechPortStats& stats() const { return stats_; }
  void clearStats() { stats_ = FeetechPortStats(); }

 private:
  bool readByte(uint8_t& value, uint32_t deadlineUs);
  void dropInput();

  HardwareSerial& serial_;
  uint32_t byteUs_ = 10;
  uint32_t latencyUs_ = 2000;
  bool echo_ = false;
  uint32_t sendStartUs_ = 0;
  uint8_t rx_[feetech::kMaxPacket];
  FeetechPortStats stats_;
};
