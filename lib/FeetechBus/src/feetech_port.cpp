#include "feetech_port.h"

using feetech::ParseResult;
using feetech::StatusPacket;

namespace {

bool reached(uint32_t nowUs, uint32_t deadlineUs) {
  return static_cast<int32_t>(nowUs - deadlineUs) >= 0;
}

}  // namespace

void FeetechPort::begin(uint32_t baud, int8_t rxPin, int8_t txPin, uint32_t latencyUs) {
  byteUs_ = (10000000UL + baud - 1) / baud;  // 8N1 = 10 bits per byte
  latencyUs_ = latencyUs;
  serial_.setRxBufferSize(512);
  serial_.begin(baud, SERIAL_8N1, rxPin, txPin);
  // Hand bytes to the ring buffer after 2 idle symbols instead of waiting
  // for the default FIFO threshold; replies are short.
  serial_.setRxTimeout(2);
  dropInput();
}

void FeetechPort::dropInput() {
  while (serial_.available() > 0) serial_.read();
}

bool FeetechPort::readByte(uint8_t& value, uint32_t deadlineUs) {
  while (serial_.available() <= 0) {
    if (reached(micros(), deadlineUs)) return false;
  }
  value = static_cast<uint8_t>(serial_.read());
  return true;
}

bool FeetechPort::detectEcho(uint8_t probeId) {
  uint8_t packet[8];
  const size_t n = feetech::buildPing(packet, sizeof(packet), probeId);
  dropInput();
  serial_.write(packet, n);
  serial_.flush();
  const uint32_t deadline = micros() + byteUs_ * (n + 3) + latencyUs_;
  size_t got = 0;
  uint8_t value = 0;
  while (got < n && readByte(value, deadline)) {
    if (value != packet[got]) break;
    ++got;
  }
  echo_ = got == n;
  dropInput();
  return echo_;
}

bool FeetechPort::send(const uint8_t* packet, size_t length) {
  if (packet == nullptr || length == 0) return false;
  dropInput();
  sendStartUs_ = micros();
  serial_.write(packet, length);
  serial_.flush();  // returns after the last stop bit left the UART
  ++stats_.sent;
  if (!echo_) return true;
  const uint32_t deadline = micros() + byteUs_ * (length + 3) + latencyUs_;
  for (size_t i = 0; i < length; ++i) {
    uint8_t value = 0;
    if (!readByte(value, deadline) || value != packet[i]) {
      ++stats_.echoErrors;
      return false;
    }
  }
  return true;
}

ParseResult FeetechPort::receive(uint8_t expectedId, int expectedParams, StatusPacket& out) {
  const size_t expectedLength =
      expectedParams >= 0 ? feetech::statusPacketSize(static_cast<size_t>(expectedParams)) : 16;
  const uint32_t deadline = micros() + byteUs_ * (expectedLength + 3) + latencyUs_;

  // Resynchronise on FF FF: a glitch byte before the header must not cost the reply.
  uint8_t value = 0;
  uint8_t previous = 0;
  bool header = false;
  while (!header) {
    if (!readByte(value, deadline)) {
      ++stats_.timeouts;
      return ParseResult::Incomplete;
    }
    header = previous == 0xFF && value == 0xFF;
    previous = value;
  }
  rx_[0] = rx_[1] = 0xFF;
  size_t got = 2;
  // A third FF can only be padding before the id (0xFF is not a valid id).
  do {
    if (!readByte(value, deadline)) {
      ++stats_.timeouts;
      return ParseResult::Incomplete;
    }
  } while (value == 0xFF);
  rx_[got++] = value;  // id
  if (!readByte(value, deadline)) {
    ++stats_.timeouts;
    return ParseResult::Incomplete;
  }
  rx_[got++] = value;  // length
  const size_t total = static_cast<size_t>(value) + 4;
  if (value < 2 || total > sizeof(rx_)) {
    ++stats_.parseErrors;
    return ParseResult::BadLength;
  }
  while (got < total) {
    if (!readByte(rx_[got], deadline)) {
      ++stats_.timeouts;
      return ParseResult::Incomplete;
    }
    ++got;
  }

  const ParseResult result = feetech::parseStatus(rx_, got, expectedId, expectedParams, out);
  if (result != ParseResult::Ok) {
    ++stats_.parseErrors;
    return result;
  }
  ++stats_.received;
  stats_.lastReplyUs = micros() - sendStartUs_;
  if (stats_.lastReplyUs > stats_.maxReplyUs) stats_.maxReplyUs = stats_.lastReplyUs;
  return result;
}
