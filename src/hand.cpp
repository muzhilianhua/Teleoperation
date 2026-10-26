#include "hand.h"

#include <Arduino.h>

namespace hand {

namespace {
config::Hand selected = config::Hand::Right;
}  // namespace

void select() {
  pinMode(config::kHandStrapPin, INPUT_PULLUP);
  delayMicroseconds(50);  // let the pull-up settle
  selected = digitalRead(config::kHandStrapPin) == LOW ? config::Hand::Left : config::Hand::Right;
}

config::Hand which() { return selected; }

const char* name() { return selected == config::Hand::Left ? "left" : "right"; }

const teleop::JointDefaults* joints() {
  return selected == config::Hand::Left ? config::kLeftJoints : config::kRightJoints;
}

const uint8_t* encoderAddresses() {
  return selected == config::Hand::Left ? config::kLeftEncoderAddresses : config::kRightEncoderAddresses;
}

const char* storeKey() { return selected == config::Hand::Left ? "cfg_l" : "cfg"; }

}  // namespace hand
