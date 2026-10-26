#include "hand.h"

#include <Arduino.h>

namespace hand {

namespace {
config::Hand selected = config::Hand::Right;
bool isSelected = false;
bool earlyUse = false;

config::Hand current() {
  if (!isSelected) earlyUse = true;
  return selected;
}
}  // namespace

void select() {
  pinMode(config::kHandStrapPin, INPUT_PULLUP);
  delayMicroseconds(50);  // let the pull-up settle
  selected = digitalRead(config::kHandStrapPin) == LOW ? config::Hand::Left : config::Hand::Right;
  isSelected = true;
}

bool usedBeforeSelect() { return earlyUse; }

config::Hand which() { return current(); }

const char* name() { return current() == config::Hand::Left ? "left" : "right"; }

const teleop::JointDefaults* joints() {
  return current() == config::Hand::Left ? config::kLeftJoints : config::kRightJoints;
}

const uint8_t* encoderAddresses() {
  return current() == config::Hand::Left ? config::kLeftEncoderAddresses : config::kRightEncoderAddresses;
}

const char* storeKey() { return current() == config::Hand::Left ? "cfg_l" : "cfg"; }

}  // namespace hand
