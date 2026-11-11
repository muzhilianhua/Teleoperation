// Every byte the console prints goes through here. In text mode it is passed
// straight to the USB serial port, exactly as before. With a line sink set
// (host link mode), output is collected per line and each complete line is
// handed to the sink, which wraps it in a LOG frame, so tables and messages
// never interleave with binary frames.

#pragma once

#include <Arduino.h>

class ConsoleOut : public Print {
 public:
  using LineSink = void (*)(void* context, const char* line, size_t length);

  explicit ConsoleOut(Print& text) : text_(text) {}

  size_t write(uint8_t c) override;
  size_t write(const uint8_t* buffer, size_t size) override;

  // nullptr returns to text mode. Any partial line is flushed first.
  void setLineSink(LineSink sink, void* context);
  void flushLine();

 private:
  static constexpr size_t kLineMax = 200;  // longer lines are split

  Print& text_;
  LineSink sink_ = nullptr;
  void* context_ = nullptr;
  char line_[kLineMax];
  size_t length_ = 0;
};
