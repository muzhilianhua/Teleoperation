// Multi-encoder validation: five AS5048B (0x50..0x60) on one I2C bus.
// Copy over src/main.cpp temporarily to upload; build-check with:
//   pio ci lib/AS5048B/examples/multi_read --lib lib/AS5048B --project-conf platformio.ini
//
// Serial commands:
//   p  toggle output: table <-> Teleplot (">E50:123.45")
//   r  cycle sample rate 100 -> 200 -> 500 -> 1000 Hz
//   b  benchmark: back-to-back cycles, 3 s fast + 3 s full (clears stats)
//   z  zero all (volatile)      t  reset turn counters
//   c  clear statistics         s  rescan bus
//
// Table cell: "<name> <angle><state><field>a<AGC> <turns>"
//   state: ' ' ok, 'M' no magnet (OCF=0/COF), '!' offline/invalid
//   field: ' ' in range, 'w' too weak (Comp High: move magnet closer),
//          's' too strong (Comp Low: move magnet away)
//   AGC 0..255: 0 = strong field, 255 = weak field; aim for the middle.
// Field flags and AGC refresh with full reads (rate / 10).

#include <Arduino.h>
#include <AS5048B.h>
#include <AS5048BGroup.h>

namespace {

constexpr uint32_t kI2cFreq = 400000;
constexpr uint32_t kRates[] = {100, 200, 500, 1000};
constexpr uint8_t kFullReadEvery = 10;  // diagnostics refresh = rate / 10
constexpr uint32_t kTablePeriodMs = 100;
constexpr uint32_t kPlotPeriodMs = 20;
constexpr uint32_t kStatsPeriodMs = 5000;
constexpr uint32_t kBenchmarkMs = 3000;

AS5048B encoders[] = {AS5048B(0x40), AS5048B(0x44), AS5048B(0x48), AS5048B(0x4c), AS5048B(0x50), AS5048B(0x54), AS5048B(0x58), AS5048B(0x5C), AS5048B(0x60)};
const char* const kNames[] = {"E40", "E44", "E48", "E4C", "E50", "E54", "E58", "E5C", "E60"};
constexpr uint8_t kCount = sizeof(encoders) / sizeof(encoders[0]);
static_assert(kCount == sizeof(kNames) / sizeof(kNames[0]), "one name per encoder");

AS5048BGroup group;

uint8_t rateIndex = 1;
uint32_t periodUs = 1000000UL / kRates[1];
uint32_t nextDueUs = 0;
uint32_t cycleIndex = 0;
uint32_t missedDeadlines = 0;
bool plotMode = false;

uint32_t lastOutputMs = 0;
uint32_t lastStatsMs = 0;
uint32_t statsWindowCycles = 0;

char flagFor(const AS5048BChannel& ch) {
  if (ch.state == AS5048BChannelState::Offline || !ch.valid) return '!';
  if (ch.state == AS5048BChannelState::NoMagnet) return 'M';
  return ' ';
}

char fieldFlagFor(const AS5048BChannel& ch) {
  if (ch.reading.diag.compHigh) return 'w';
  if (ch.reading.diag.compLow) return 's';
  return ' ';
}

void scanAndCompare() {
  uint8_t found[16];
  const uint8_t n = AS5048B::scan(Wire, found, sizeof(found), 0x08, 0x77);
  Serial.printf("[scan] %u device(s):", n);
  for (uint8_t i = 0; i < n && i < sizeof(found); ++i) Serial.printf(" 0x%02X", found[i]);
  Serial.println();

  for (uint8_t e = 0; e < kCount; ++e) {
    bool present = false;
    for (uint8_t i = 0; i < n && i < sizeof(found); ++i) present |= found[i] == encoders[e].address();
    if (!present) Serial.printf("[scan] MISSING %s @0x%02X\n", kNames[e], encoders[e].address());
  }
  for (uint8_t i = 0; i < n && i < sizeof(found); ++i) {
    bool expected = false;
    for (uint8_t e = 0; e < kCount; ++e) expected |= found[i] == encoders[e].address();
    if (!expected) Serial.printf("[scan] UNEXPECTED device @0x%02X\n", found[i]);
  }
}

void printChannels(const char* tag) {
  for (uint8_t i = 0; i < group.count(); ++i) {
    const AS5048BChannel& ch = group.channel(i);
    const uint32_t total = ch.okCount + ch.errorCount;
    Serial.printf(
        "[%s] %s @0x%02X %-8s ok=%lu err=%lu (%.3f%%) maxRead=%luus rec=%lu agc=%u mag=%u "
        "diag=0x%02X last=%s\n",
        tag, ch.name, ch.encoder->address(), AS5048BGroup::stateToString(ch.state),
        static_cast<unsigned long>(ch.okCount), static_cast<unsigned long>(ch.errorCount),
        total ? 100.0 * ch.errorCount / total : 0.0, static_cast<unsigned long>(ch.maxReadUs),
        static_cast<unsigned long>(ch.recoveries), ch.reading.agc, ch.reading.magnitude,
        ch.reading.diag.raw, AS5048B::statusToString(ch.lastStatus));
  }
}

void printStats(uint32_t nowMs) {
  const AS5048BCycleStats& s = group.stats();
  const float windowS = (nowMs - lastStatsMs) / 1000.0f;
  const uint32_t cycles = s.cycles - statsWindowCycles;
  Serial.printf(
      "[stats] rate=%.1f Hz (target %lu) cycle avg/max=%lu/%lu us skew last/max=%lu/%lu us "
      "missed=%lu\n",
      windowS > 0 ? cycles / windowS : 0.0f, static_cast<unsigned long>(kRates[rateIndex]),
      static_cast<unsigned long>(s.averageCycleUs()), static_cast<unsigned long>(s.maxCycleUs),
      static_cast<unsigned long>(s.lastSkewUs), static_cast<unsigned long>(s.maxSkewUs),
      static_cast<unsigned long>(missedDeadlines));
  printChannels("stats");
  statsWindowCycles = s.cycles;
  lastStatsMs = nowMs;
}

void printTable() {
  Serial.printf("t=%8.3f |", millis() / 1000.0f);
  for (uint8_t i = 0; i < group.count(); ++i) {
    const AS5048BChannel& ch = group.channel(i);
    Serial.printf(" %s %7.2f%c%ca%3u %+ld |", ch.name, ch.reading.angleDegrees(), flagFor(ch),
                  fieldFlagFor(ch), ch.reading.agc, static_cast<long>(ch.encoder->getTurns()));
  }
  Serial.println();
}

void printPlot() {
  for (uint8_t i = 0; i < group.count(); ++i) {
    const AS5048BChannel& ch = group.channel(i);
    if (!ch.valid) continue;
    Serial.printf(">%s:%.2f\n", ch.name, ch.reading.angleDegrees());
    Serial.printf(">%s_agc:%u\n", ch.name, ch.reading.agc);
  }
  Serial.printf(">skew_us:%lu\n", static_cast<unsigned long>(group.stats().lastSkewUs));
}

void runBenchmark(bool full) {
  group.clearStats();
  const uint32_t start = millis();
  while (millis() - start < kBenchmarkMs) group.sample(full);
  const AS5048BCycleStats& s = group.stats();
  const float seconds = (millis() - start) / 1000.0f;
  uint8_t responding = 0;
  for (uint8_t i = 0; i < group.count(); ++i) {
    responding += group.channel(i).state != AS5048BChannelState::Offline;
  }
  Serial.printf(
      "[bench %s] %.0f cycles/s, cycle avg/max=%lu/%lu us, ~%lu us/encoder (%u responding), "
      "skew max=%lu us\n",
      full ? "full" : "fast", s.cycles / seconds, static_cast<unsigned long>(s.averageCycleUs()),
      static_cast<unsigned long>(s.maxCycleUs),
      static_cast<unsigned long>(responding ? s.averageCycleUs() / responding : 0), responding,
      static_cast<unsigned long>(s.maxSkewUs));
  printChannels(full ? "bench full" : "bench fast");
}

void resetSchedule() {
  periodUs = 1000000UL / kRates[rateIndex];
  nextDueUs = micros();
  missedDeadlines = 0;
  statsWindowCycles = group.stats().cycles;
  lastStatsMs = millis();
}

void handleCommand(char cmd) {
  switch (cmd) {
    case 'p':
      plotMode = !plotMode;
      Serial.printf("[mode] %s\n", plotMode ? "teleplot" : "table");
      break;
    case 'r':
      rateIndex = (rateIndex + 1) % (sizeof(kRates) / sizeof(kRates[0]));
      resetSchedule();
      Serial.printf("[rate] %lu Hz\n", static_cast<unsigned long>(kRates[rateIndex]));
      break;
    case 'b':
      Serial.println("[bench] running 3 s fast + 3 s full ...");
      runBenchmark(false);
      runBenchmark(true);
      group.clearStats();
      resetSchedule();
      break;
    case 'z':
      Serial.printf("[zero] %u/%u set (volatile)\n", group.zeroAll(), group.count());
      break;
    case 't':
      group.resetTurnsAll();
      Serial.println("[turns] reset");
      break;
    case 'c':
      group.clearStats();
      resetSchedule();
      Serial.println("[stats] cleared");
      break;
    case 's':
      scanAndCompare();
      break;
    default:
      break;
  }
}

}  // namespace

void setup() {
  // Larger TX buffer so periodic prints do not stall the sampling schedule.
  Serial.setTxBufferSize(4096);
  Serial.begin(115200);
  delay(1500);
  Serial.println("\nAS5048B multi-encoder test (0x50..0x60)");

  Wire.begin(AS5048B_DEFAULT_SDA, AS5048B_DEFAULT_SCL, kI2cFreq);
  scanAndCompare();

  for (uint8_t i = 0; i < kCount; ++i) group.add(encoders[i], kNames[i]);
  const uint8_t responding = group.beginAll();
  Serial.printf("[init] %u/%u responding\n", responding, kCount);
  printChannels("init");

  resetSchedule();
}

void loop() {
  while (Serial.available()) handleCommand(static_cast<char>(Serial.read()));

  const uint32_t nowUs = micros();
  if (static_cast<int32_t>(nowUs - nextDueUs) >= 0) {
    if (nowUs - nextDueUs >= periodUs) {
      ++missedDeadlines;  // more than a full period late: skip ahead instead of bursting
      nextDueUs = nowUs;
    }
    group.sample(cycleIndex++ % kFullReadEvery == 0);
    nextDueUs += periodUs;
  }

  const uint32_t nowMs = millis();
  const uint32_t outputPeriod = plotMode ? kPlotPeriodMs : kTablePeriodMs;
  if (nowMs - lastOutputMs >= outputPeriod) {
    lastOutputMs = nowMs;
    plotMode ? printPlot() : printTable();
  }
  if (nowMs - lastStatsMs >= kStatsPeriodMs) printStats(nowMs);
}
