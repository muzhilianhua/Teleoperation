// AS5048B I2C address OTP burning jig. IRREVERSIBLE - read lib/AS5048B/README.md first.
//
// Build / upload without touching src/:
//   pio ci lib/AS5048B/examples/otp_address_jig --lib lib/AS5048B --project-conf platformio.ini
// or temporarily copy it over src/main.cpp.
//
// Rules:
//   - Exactly ONE encoder on the bus (fresh chips all share the factory address).
//   - Sensor freshly powered (zero register must be 0) with its magnet in place:
//     without a field OCF stays 0 and begin() refuses the chip.
//   - Supply: 5 V mode, or 3.3 V mode at >= 3.3 V with 10 uF on VDD3V. Room temperature.
//
// Flow: scan -> identify -> enter target -> volatile dry run -> type "BURN <addr>"
//       -> burn + verify -> power-cycle the sensor -> jig re-identifies it.

#include <Arduino.h>
#include <AS5048B.h>

namespace {

constexpr uint32_t kJigI2cFreq = 100000;  // conservative while programming

String readLine() {
  String line;
  while (true) {
    while (!Serial.available()) delay(5);
    const char c = static_cast<char>(Serial.read());
    if (c == '\n' || c == '\r') {
      if (line.length() > 0) break;
      continue;
    }
    line += c;
  }
  line.trim();
  return line;
}

bool parseAddress(String text, uint8_t& out) {
  text.trim();
  text.toLowerCase();
  if (text.startsWith("0x")) text = text.substring(2);
  if (text.length() == 0 || text.length() > 2) return false;
  char* end = nullptr;
  const long value = strtol(text.c_str(), &end, 16);
  if (*end != '\0' || value < 0x08 || value > 0x77) return false;
  out = static_cast<uint8_t>(value);
  return true;
}

void halt(const char* reason) {
  Serial.printf("\n[STOP] %s\n", reason);
  Serial.println("Power-cycle the SENSOR (not just reset the ESP32) and press RESET to start over.");
  while (true) delay(1000);
}

bool acks(uint8_t address) { return AS5048B::scan(Wire, nullptr, 0, address, address) == 1; }

// A burn interrupted by a flaky contact is the worst case, so demand a clean
// run of probes + register reads before identifying and again before burning.
bool linkStable(AS5048B& encoder, const char* stage) {
  constexpr uint8_t kChecks = 50;
  uint8_t ok = 0;
  uint8_t firstReg = 0xFF;
  for (uint8_t i = 0; i < kChecks; ++i) {
    uint8_t reg = 0;
    if (encoder.isConnected() && encoder.readAddressRegister(reg) == AS5048BStatus::Ok &&
        (firstReg == 0xFF || reg == firstReg)) {
      firstReg = reg;
      ++ok;
    }
    delay(4);
  }
  Serial.printf("[link] %s: %u/%u good transfers at 0x%02X\n", stage, ok, kChecks,
                encoder.address());
  return ok == kChecks;
}

void rescanReport(uint8_t rounds) {
  for (uint8_t r = 0; r < rounds; ++r) {
    uint8_t found[4];
    const uint8_t count = AS5048B::scan(Wire, found, sizeof(found), 0x08, 0x77);
    Serial.printf("[rescan %u] %u device(s):", r + 1, count);
    for (uint8_t i = 0; i < count && i < sizeof(found); ++i) Serial.printf(" 0x%02X", found[i]);
    Serial.println();
    delay(50);
  }
}

}  // namespace

void setup() {
  Serial.begin(115200);
  Serial.println("\nAS5048B address OTP jig - press Enter to start");
  readLine();

  if (!Wire.begin(AS5048B_DEFAULT_SDA, AS5048B_DEFAULT_SCL, kJigI2cFreq)) halt("Wire.begin failed");

  // 1. Scan the full usable range: a chip may already sit at a burned address.
  uint8_t found[4];
  const uint8_t count = AS5048B::scan(Wire, found, sizeof(found), 0x08, 0x77);
  Serial.printf("[scan] %u device(s):", count);
  for (uint8_t i = 0; i < count && i < sizeof(found); ++i) Serial.printf(" 0x%02X", found[i]);
  Serial.println();
  if (count != 1) halt("connect exactly ONE encoder to the jig");

  // 2. Identify.
  const uint8_t current = found[0];
  AS5048B encoder(current);
  if (!encoder.begin(AS5048B_DEFAULT_SDA, AS5048B_DEFAULT_SCL, kJigI2cFreq, false)) {
    Serial.printf("[chip] begin at 0x%02X failed: %s\n", current,
                  AS5048B::statusToString(encoder.lastStatus()));
    if (encoder.lastStatus() == AS5048BStatus::MagnetError) {
      halt("chip answers but OCF=0 (no magnetic field): mount the magnet over the chip and retry");
    }
    rescanReport(5);
    halt("unstable or ghost device: check sensor power, GND, SDA/SCL contact, pull-ups");
  }
  if (!linkStable(encoder, "identify")) {
    rescanReport(5);
    halt("I2C link unstable, nothing was written: fix contact/power before burning");
  }
  uint8_t reg15 = 0;
  uint16_t zero = 0;
  AS5048BDiagnostics diag;
  if (encoder.readAddressRegister(reg15) != AS5048BStatus::Ok ||
      encoder.readZeroPosition(zero) != AS5048BStatus::Ok ||
      encoder.readDiagnostics(diag) != AS5048BStatus::Ok) {
    halt("register read failed");
  }
  const uint8_t pins = current & 0x03;
  Serial.printf("[chip] addr=0x%02X reg15=0x%02X zero=%u diag=0x%02X  A2=%u A1=%u (fixed by PCB)\n",
                current, reg15, zero, diag.raw, (pins >> 1) & 1, pins & 1);

  if ((reg15 & 0x1F) != 0) {
    Serial.printf("[done] this chip is already programmed: address 0x%02X. Label it.\n", current);
    halt("nothing to do");
  }
  if (zero != 0) halt("zero register is not 0; power-cycle the sensor first");

  // 3. Target address. The two LSBs come from A1/A2 and cannot change.
  uint8_t target = 0;
  while (true) {
    Serial.printf("\nEnter target address (hex). Must satisfy (addr & 3) == %u, e.g. 0x%02X:\n",
                  pins, static_cast<unsigned>(0x44 | pins));
    if (!parseAddress(readLine(), target)) {
      Serial.println("  invalid (0x08..0x77 hex)");
      continue;
    }
    if ((target & 0x03) != pins) {
      Serial.printf("  0x%02X needs A2A1=%u%u, this PCB has %u%u\n", target, (target >> 1) & 1,
                    target & 1, (pins >> 1) & 1, pins & 1);
      continue;
    }
    if (AS5048B::addressRegisterValue(target) == 0) {
      Serial.println("  that is the factory address, no burn needed");
      continue;
    }
    break;
  }
  Serial.printf("[plan] 0x%02X -> 0x%02X, reg15 value 0x%02X\n", current, target,
                AS5048B::addressRegisterValue(target));

  // 4. Volatile dry run: proves the address math before anything is burned.
  const AS5048BStatus dry = encoder.changeAddress(target);
  const bool newAcks = acks(target);
  const bool oldAcks = acks(current);
  Serial.printf("[dry-run] changeAddress: %s, new 0x%02X ack=%d, old 0x%02X ack=%d\n",
                AS5048B::statusToString(dry), target, newAcks, current, oldAcks);
  if (dry != AS5048BStatus::Ok || !newAcks || oldAcks) halt("dry run failed, nothing was burned");

  // 5. Explicit confirmation.
  char expected[16];
  snprintf(expected, sizeof(expected), "BURN %02X", target);
  Serial.printf("\nDry run OK. This is PERMANENT. Type exactly \"%s\" to burn, anything else aborts:\n",
                expected);
  String answer = readLine();
  answer.toUpperCase();
  if (answer != expected) halt("aborted, nothing was burned (volatile change reverts on power-cycle)");

  // 6. Burn + verify.
  if (!linkStable(encoder, "pre-burn")) {
    halt("I2C link unstable, burn skipped (volatile change reverts on power-cycle)");
  }
  const AS5048BStatus burn = encoder.burnAddressOTP(AS5048B::kOtpConfirmKey);
  uint8_t after = 0;
  encoder.readAddressRegister(after);
  Serial.printf("[burn] %s, reg15=0x%02X (expected 0x%02X)\n", AS5048B::statusToString(burn), after,
                AS5048B::addressRegisterValue(target));
  Serial.printf("[log] chip burned to 0x%02X, result=%s\n", target, AS5048B::statusToString(burn));

  // 7. Final check needs a real power cycle: the jig re-identifies the chip on next boot.
  halt(burn == AS5048BStatus::Ok
           ? "burn OK. Unplug USB (sensor power-cycle), reconnect, run the jig again to confirm"
           : "burn NOT confirmed. Power-cycle, run the jig again: the scan shows where the chip is");
}

void loop() {}
