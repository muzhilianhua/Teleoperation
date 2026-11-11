#include "console_out.h"

size_t ConsoleOut::write(uint8_t c) {
  if (sink_ == nullptr) return text_.write(c);
  if (c == '\n') {
    flushLine();
  } else if (c != '\r') {
    line_[length_++] = static_cast<char>(c);
    if (length_ == kLineMax) flushLine();
  }
  return 1;
}

size_t ConsoleOut::write(const uint8_t* buffer, size_t size) {
  if (sink_ == nullptr) return text_.write(buffer, size);
  for (size_t i = 0; i < size; ++i) write(buffer[i]);
  return size;
}

void ConsoleOut::flushLine() {
  if (sink_ == nullptr || length_ == 0) return;
  sink_(context_, line_, length_);
  length_ = 0;
}

void ConsoleOut::setLineSink(LineSink sink, void* context) {
  flushLine();
  sink_ = sink;
  context_ = context;
  length_ = 0;
}
