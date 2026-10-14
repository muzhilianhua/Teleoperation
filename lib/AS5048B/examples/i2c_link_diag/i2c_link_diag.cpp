// AS5048B I2C link diagnostic. Read-only: never writes a register.
// Copy over src/main.cpp temporarily, upload, open the monitor, press Enter.
//
// Probes one address with different idle gaps between transactions and runs
// register reads. An ACK rate that depends on the gap points to a supply problem
// (missing/poor VDD, chip parasitically powered through the SDA/SCL pull-ups),
// a gap-independent failure points to wiring or the chip itself.

#include <Arduino.h>
#include <AS5048B.h>

namespace {

constexpr uint32_t kFreq = 100000;
constexpr uint8_t kRepeats = 40;

void waitEnter() {
  while (true) {
    while (!Serial.available()) delay(5);
    const char c = static_cast<char>(Serial.read());
    if (c == '\n' || c == '\r') return;
  }
}

uint8_t probeAddress() {
  uint8_t found[4];
  const uint8_t count = AS5048B::scan(Wire, found, sizeof(found), 0x08, 0x77);
  Serial.printf("[scan] %u device(s):", count);
  for (uint8_t i = 0; i < count && i < sizeof(found); ++i) Serial.printf(" 0x%02X", found[i]);
  Serial.println();
  return count >= 1 ? found[0] : 0;
}

void gapTest(AS5048B& enc, uint32_t gapUs) {
  delay(100);  // identical idle before every series
  uint8_t ok = 0;
  String pattern;
  for (uint8_t i = 0; i < kRepeats; ++i) {
    const bool ack = enc.isConnected();
    ok += ack;
    pattern += ack ? '+' : '.';
    if (gapUs >= 1000) delay(gapUs / 1000);
    else delayMicroseconds(gapUs);
  }
  Serial.printf("[probe gap %6lu us] %2u/%u ACK  %s\n", static_cast<unsigned long>(gapUs), ok,
                kRepeats, pattern.c_str());
}

void readTest(AS5048B& enc) {
  delay(100);
  uint8_t ok = 0;
  AS5048BReading r;
  AS5048BStatus last = AS5048BStatus::Ok;
  for (uint8_t i = 0; i < kRepeats; ++i) {
    last = enc.readAll(r);
    if (last == AS5048BStatus::Ok) ++ok;
    delay(5);
  }
  Serial.printf("[readAll x%u] %u OK, last=%s\n", kRepeats, ok, AS5048B::statusToString(last));
  if (ok > 0) {
    uint8_t reg15 = 0;
    uint16_t zero = 0;
    enc.readAddressRegister(reg15);
    enc.readZeroPosition(zero);
    Serial.printf("  angle=%u mag=%u agc=%u diag=0x%02X reg15=0x%02X zero=%u\n", r.angleRaw,
                  r.magnitude, r.agc, r.diag.raw, reg15, zero);
  }
}

}  // namespace

void setup() {
  Serial.begin(115200);
  Serial.println("\nAS5048B I2C link diagnostic (read-only) - press Enter");
  waitEnter();
  Wire.begin(AS5048B_DEFAULT_SDA, AS5048B_DEFAULT_SCL, kFreq);

  const uint8_t address = probeAddress();
  if (address == 0) {
    Serial.println("no device found");
    return;
  }
  AS5048B enc(address);
  for (uint32_t gap : {0UL, 200UL, 1000UL, 5000UL, 20000UL, 100000UL}) gapTest(enc, gap);
  readTest(enc);
  Serial.println("[done] press RESET to run again");
}

void loop() {}
