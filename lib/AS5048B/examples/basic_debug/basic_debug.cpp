// AS5048B bring-up sketch. Not part of the main firmware; build it with:
//   pio ci lib/AS5048B/examples/basic_debug --lib lib/AS5048B --project-conf platformio.ini
// or copy it over src/main.cpp temporarily.
//
// Serial commands (115200 baud):
//   z  set zero at current position (volatile, lost on power cycle)
//   c  clear zero register
//   r  toggle software direction
//   t  reset multi-turn counter
//   s  rescan I2C bus

#include <Arduino.h>
#include <AS5048B.h>

AS5048B encoder(AS5048B::addressFromPins(false, false));  // A1=GND, A2=GND -> 0x40

static void scanBus() {
  uint8_t found[4];
  const uint8_t count = AS5048B::scan(Wire, found, sizeof(found));
  Serial.printf("[scan] %u device(s) in 0x40..0x43:", count);
  for (uint8_t i = 0; i < count && i < sizeof(found); ++i) Serial.printf(" 0x%02X", found[i]);
  Serial.println();
}

void setup() {
  Serial.begin(115200);
  delay(1500);
  Serial.println("\nAS5048B debug");

  if (!encoder.begin()) {
    Serial.printf("[init] failed at 0x%02X: %s\n", encoder.address(),
                  AS5048B::statusToString(encoder.lastStatus()));
    scanBus();
  } else {
    uint16_t zero = 0;
    uint8_t addrReg = 0;
    encoder.readZeroPosition(zero);
    encoder.readAddressRegister(addrReg);
    Serial.printf("[init] OK addr=0x%02X addrReg=0x%02X zero=%u\n", encoder.address(), addrReg,
                  zero);
  }
}

static void handleCommand(char cmd) {
  uint16_t zero = 0;
  AS5048BStatus st;
  switch (cmd) {
    case 'z':
      st = encoder.setZeroHere(&zero);
      Serial.printf("[zero] %s, zero=%u\n", AS5048B::statusToString(st), zero);
      break;
    case 'c':
      st = encoder.writeZeroPosition(0);
      Serial.printf("[zero] clear: %s\n", AS5048B::statusToString(st));
      break;
    case 'r':
      encoder.setReversed(!encoder.isReversed());
      Serial.printf("[dir] reversed=%d\n", encoder.isReversed());
      break;
    case 't':
      st = encoder.resetTurns();
      Serial.printf("[turns] reset: %s\n", AS5048B::statusToString(st));
      break;
    case 's':
      scanBus();
      break;
    default:
      break;
  }
}

void loop() {
  while (Serial.available()) handleCommand(static_cast<char>(Serial.read()));

  encoder.update();

  static uint32_t lastPrint = 0;
  if (millis() - lastPrint < 50) return;
  lastPrint = millis();

  AS5048BReading r;
  const AS5048BStatus st = encoder.readAll(r);
  if (st != AS5048BStatus::Ok) {
    Serial.printf("[read] %s\n", AS5048B::statusToString(st));
    return;
  }
  Serial.printf(
      "angle=%5u (%7.2f deg) raw=%5u mag=%5u agc=%3u diag=0x%02X [OCF=%d COF=%d weak=%d "
      "strong=%d] turns=%ld cum=%9.2f deg\n",
      r.angle, r.angleDegrees(), r.angleRaw, r.magnitude, r.agc, r.diag.raw, r.diag.ocf, r.diag.cof,
      r.diag.compHigh, r.diag.compLow, static_cast<long>(encoder.getTurns()),
      encoder.getCumulativeDegrees());
}
