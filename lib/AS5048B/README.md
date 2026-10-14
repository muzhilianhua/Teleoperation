# AS5048B driver

I2C driver for the ams AS5048B 14-bit magnetic rotary encoder (0.0219°/LSB), targeting
Arduino-ESP32 on ESP32-S3. Reference: ams datasheet v1-11 and
[sosandroid/AMS_AS5048B](https://github.com/sosandroid/AMS_AS5048B).

## Wiring (current project)

| AS5048B | ESP32-S3 | Note |
|---|---|---|
| SDA (1) | GPIO8 | 4.7 kΩ pull-up to 3V3 if the module has none |
| SCL (2) | GPIO9 | 4.7 kΩ pull-up to 3V3 |
| A1 (4), A2 (3) | GND | address 0x40 |
| VDD5V (11) + VDD3V (12) | 3V3 | 3.3 V mode: short VDD3V to VDD5V |
| TEST (5) | GND | |

Address = `0x40 | A2<<1 | A1` (factory OTP).

## Quick start

```cpp
#include <AS5048B.h>

AS5048B encoder(0x40);              // or AS5048B::addressFromPins(a1High, a2High)

void setup() {
  Serial.begin(115200);
  if (!encoder.begin()) {           // Wire.begin(8, 9, 400 kHz) + wait for OCF
    Serial.println(AS5048B::statusToString(encoder.lastStatus()));
  }
}

void loop() {
  encoder.update();                 // multi-turn tracking
  Serial.println(encoder.getAngleDegrees());
}
```

Every `AS5048BStatus`-returning call also stores the result in `lastStatus()`.

## API

| Group | Call | Notes |
|---|---|---|
| Setup | `begin(sda, scl, freq, initBus)` | `initBus=false` when `Wire` is already started / shared |
| | `isConnected()` | address ACK probe |
| | `AS5048B::scan(Wire, buf, n, first, last)` | find chips on the bus |
| Read | `readAngleRaw(u16&)` | register 0xFE/0xFF, 0..16383, zero-corrected by the chip |
| | `readAngle(u16&)` | raw + software direction |
| | `getAngleDegrees()` / `getAngleRadians()` | `NAN` on error |
| | `readMagnitude(u16&)`, `readAgc(u8&)`, `readDiagnostics(diag&)` | |
| | `readAll(AS5048BReading&)` | one 6-byte burst 0xFA..0xFF (consistent snapshot) |
| Direction | `setReversed(bool)` | restarts multi-turn tracking |
| Multi-turn | `update()`, `trackAngle(angle)`, `resetTurns()`, `getTurns()`, `getCumulativeCounts()`, `getCumulativeDegrees()` | call `update()` faster than half a turn per period |
| Zero (volatile) | `setZeroHere(&z)`, `writeZeroPosition(z)`, `readZeroPosition(z&)` | lost on power cycle |
| Address (volatile) | `changeAddress(newAddr)`, `readAddressRegister(u8&)` | low 2 bits fixed by A1/A2 |
| OTP (irreversible) | `burnZeroPositionOTP(AS5048B::kOtpConfirmKey)` | datasheet steps 4–8 + verify |
| | `burnAddressOTP(AS5048B::kOtpConfirmKey)` | ams AN000386 sequence with Burn-bit polling + Verify; needs `changeAddress()` first and zero register = 0 |
| Low level | `readRegisters(reg, buf, len)`, `writeRegister(reg, val)` | |

### Diagnostics (register 0xFB)

| Flag | Meaning |
|---|---|
| `ocf` | offset compensation finished; must be 1 |
| `cof` | CORDIC overflow; angle invalid |
| `compHigh` | field too weak (magnet too far) |
| `compLow` | field too strong (magnet too close) |
| `magnetOk()` | `ocf && !cof && !compHigh && !compLow` |

AGC: 0 = strong field, 255 = weak field; aim for the middle of the range.

## OTP warnings

- Either burn programs the whole customer OTP block (zero and address) from the
  current register values, and it can be done only once.
- Programming needs VDD5V operation, or 3.3 V mode at >= 3.3 V with 10 µF on VDD3V.
- Keep the magnet still during `burnZeroPositionOTP()`: it checks that the angle reads ~0 afterwards.

## Multiple encoders: `AS5048BGroup`

```cpp
#include <AS5048BGroup.h>

AS5048B e50(0x50), e54(0x54);       // must outlive the group
AS5048BGroup group;

void setup() {
  Wire.begin(8, 9, 400000);          // the group never starts the bus itself
  group.add(e50, "E50");
  group.add(e54, "E54");
  group.beginAll();
}

void loop() {
  group.sample(false);               // angle only (2 bytes per encoder)
  // group.sample(true);             // + AGC/diag/magnitude, refreshes channel state
  const AS5048BChannel& ch = group.channel(0);
  if (ch.valid) Serial.println(ch.reading.angleDegrees());
}
```

| State | Meaning | Sampling |
|---|---|---|
| `Online` | responding, OCF=1, no COF | read every cycle; multi-turn tracked |
| `NoMagnet` | responding, OCF=0 or COF | read every cycle; no multi-turn tracking |
| `Offline` | 3 consecutive errors | skipped; retried every 1 s; multi-turn restarts on recovery |

Per channel: ok/error counts, max read time, recoveries. Per cycle (`stats()`): average/max
cycle time and skew (end of first to end of last read in one cycle).

Bus rules: one pull-up set per bus (total >= 1.1 kOhm, the AS5048B sinks 3 mA), bus capacitance
< 400 pF, every encoder in 3.3 V mode with 100 nF, magnets mounted.

`examples/multi_read/multi_read.cpp` validates 0x50..0x60 on one bus: scan vs. expected list,
scheduled sampling (100–1000 Hz), table / Teleplot output, 5 s statistics, `b` benchmark.

## Address OTP jig

`examples/otp_address_jig/otp_address_jig.cpp` burns one chip at a time: full-bus scan,
identify, enter the target, volatile dry run (`changeAddress`), type `BURN <addr>`, burn and verify,
then power-cycle the sensor and rerun the jig to confirm. A1/A2 fix the two address LSBs, so with
both pins grounded the usable targets are 0x44, 0x48, ... (0x40 needs no burn). `reg15 = ((addr >> 2) ^ 0x10) & 0x1F`.

## Example

`examples/basic_debug/basic_debug.cpp` is an interactive serial monitor. Build it without touching `src/`:

```sh
pio ci lib/AS5048B/examples/basic_debug --lib lib/AS5048B --project-conf platformio.ini
```
