# Host link protocol v1

The PC GUI (`host/`) talks to the firmware over the same native USB port as the text console. The text console stays the default; `host on` switches it to framed binary messages until the host leaves or stops sending heartbeats.

- **Firmware side**: `lib/HostLink` (codec, unit-tested), `src/console.cpp` (session), `src/host_telemetry.cpp` (snapshot → wire structs).
- **Host side**: `host/src/teleop_host/protocol/`.
- **Golden byte vectors**: `test/test_hostlink/golden_vectors.h`, which both test suites check.

## Framing

```
wire frame = COBS(header | payload | crc16) 0x00
header     = version u8 (=1) | type u8 | seq u16 | flags u8          5 B
crc16      = CRC16-CCITT-FALSE (poly 0x1021, init 0xFFFF) over header and payload, little-endian
```

- COBS removes every 0x00 from the body, so 0x00 always marks a frame boundary.
- A reader that joins mid-stream, or after text, resynchronises at the next 0x00.
- A frame that fails COBS decoding, is too short, has the wrong version or fails the CRC is dropped and counted.
- **Size**: a decoded frame is at most 4096 B, so a payload is at most 4089 B.
- **seq**: each direction keeps its own counter, which lets the receiver spot lost frames.
  - Host to device: `seq` identifies the frame. ACK and DONE refer back to it.
  - Device to host: `seq` counts frames sent.
- **flags**: bit 0 means "reply expected". On a HEARTBEAT it asks for an ACK; every other host frame is answered anyway.
- **Field encoding**: all integers are little-endian. `f32` is IEEE 754. `str8` is a u8 length followed by that many bytes. `text` is the rest of the payload, not NUL-terminated.

## Session

1. The host opens the port with DTR and RTS de-asserted (de-asserting them avoids resetting the ESP32-S3), waits briefly, and sends the text line `host on\n`.
2. The device replies with **HELLO**. From now on, everything the console prints goes out as **LOG** frames.
3. The host sends **HOST_HELLO**: protocol version, heartbeat timeout, telemetry dividers, and the link-loss policy. The device answers with an ACK and starts streaming.
4. The host sends **HEARTBEAT** every 200 ms. Any valid frame also feeds the watchdog.
5. The session ends in one of three ways:
   - **HOST_EXIT** (or the command `host off`): the device sends `BYE(0)` and prints `[host] link closed, text console`.
   - **No valid frame within the heartbeat timeout** (default 1000 ms, clamped to 200–10000 ms): the device sends `BYE(1)`. If HOST_HELLO bit 0 is set (the default) and any joint is armed, it switches every servo off (`host link lost`). It then prints `[host] link lost[, torque off], text console`.

## Host → device

| Type | Name | Payload |
|---|---|---|
| 0x01 | HOST_HELLO | `protocol u8`, `flags u8` (bit 0 = torque off on link loss), `heartbeatTimeoutMs u16`, `telemetryDivider u8` (control cycles per TELEMETRY; 0 = off), `slowDivider u8` (control cycles per SLOW_STATUS; 0 = off) |
| 0x02 | HEARTBEAT | empty |
| 0x03 | COMMAND | `text`: one console command line, at most 127 characters. Same grammar as the text console |
| 0x04 | ESTOP | empty. Skips the command queue and switches every servo off on the next control cycle |
| 0x05 | TELEM_CONFIG | `telemetryDivider u8`, `slowDivider u8` |
| 0x06 | HOST_EXIT | empty |
| 0x07 | GET_INFO | empty. The device answers with HELLO |

**COMMAND replies**

- The device always answers with **ACK** `{ackSeq, status, text}`:
  - `0` accepted;
  - `1` parse error (`text` gives the reason);
  - `2` control task busy;
  - `3` bad frame.
- Only an accepted command is followed by **DONE** `{ackSeq, result}`, where `0` = ok and `1` = refused or failed. DONE always comes after every EVENT that command raised; those events carry `cmdSeq = ackSeq`.
- Console-only verbs (`help`, `status`, `stats`, `enc`, `joints`) print through LOG frames, then send DONE.
- `dump` returns one **CONFIG** frame with the dump text, then DONE.
- `plot` is a no-op on the link.

## Device → host

| Type | Name | Payload |
|---|---|---|
| 0x81 | HELLO | Device info (see below) |
| 0x82 | TELEMETRY | Hand block plus one record per joint, every control cycle ÷ divider |
| 0x83 | SLOW_STATUS | Settings, statistics, monitor positions and encoders |
| 0x84 | EVENT | `code u8`, `severity u8` (0 info, 1 warning, 2 error), `joint u8` (servo id, 0 = none), `cmdSeq u16`, `args i32[4]`, `text` (exactly the text console's `[ctl]` message) |
| 0x85 | ACK | `ackSeq u16`, `status u8`, `text` |
| 0x86 | DONE | `ackSeq u16`, `result u8` |
| 0x87 | CONFIG | `ackSeq u16`, `format u8` (0 = dump text), `text` |
| 0x88 | LOG | `source u8` (0 console, 1 ESP log, 2 boot), `text`: one printed line |
| 0x89 | BYE | `reason u8` (0 host exit, 1 heartbeat timeout) |

### HELLO

```
protocol u8 | firmware str8 | controlHz u16 | degreesPerCount f32 | encoderCount u8
minIlimitMa u16 | maxIlimitMa u16 | minItripPct u8 | maxItripPct u8 | defaultItripPct u8
minExoSpanCounts i32 | maxExoSpanCounts i32
jointCount u8, then per joint:
  servoId u8 | model u8 | travelMin i16 | travelMax i16 | vmax u16 | vsat u16 | amax u16
  maxSpeed u16 | encoderDefault u8 | enabledDefault u8 | stepScale f32 | degreesPerStep f32 | name str8
name tables, each count u8 + str8 per entry: faults, joint states, event codes, servo models
```

The joint table comes from `config.h`, so the host needs no copy of it. `model` is `0` unknown, `1` SCS0009 or `2` HLS3606.

### TELEMETRY (18 B + 48 B per joint, 450 B for 9 joints)

```
cycle u32 | tUs u32 (micros() at the cycle's wake-up) | uptimeMs u32 | armedCount u8 | handFault u8
handFlags u8 | execUs u16 | jointCount u8
per joint: state u8 | fault u8 | flags u16 | encRaw u16
           exoCounts f32 | exoFilteredCounts f32 | target f32 | goal f32 | sentGoal f32 | reference f32
           speedCommand u16 | gripLead i16 | position i16 | speed i16 | loadDeciPct i16 | currentMa i16
           voltageDeciV u8 | temperatureC i8 | status u8 | errorByte u8 | readAgeMs u16
```

**`handFlags` bits**

| Bit | Meaning |
|---|---|
| 0 | Torque off confirmed |
| 1 | Monitor on |
| 2 | Sweep running |
| 3 | Samples were lost before this one |

**Joint `flags` bits**

| Bit | Meaning |
|---|---|
| 0 | Enabled |
| 1 | Ready |
| 2 | Exo A captured |
| 3 | Homed |
| 4 | Single-turn |
| 5 | Encoder fresh |
| 6 | Grip contact |
| 7 | Goal sent |
| 8 | Model OK |
| 9 | Servo read OK |
| 10 | Torque on |
| 11 | Moving |
| 12 | Has current |
| 13 | Servo reading fresh (100 ms or less) |

**Field units**

- `state`: index into the joint-state name table (OFF, ALIGN, FOLLOW, HOLD, SWEEP).
- `fault` and `handFault`: index into the fault name table.
- `exo*`: encoder counts relative to the joint's `exoRef`. Multiply by `degreesPerCount` to get degrees.
- Servo values (`target`, `goal`, `sentGoal`, `reference`, `position`) are in the servo's own steps.

The control task encodes every cycle into a 16-deep queue without waiting. A slow host loses samples, which sets `handFlags` bit 3; the control loop never slows down.

### SLOW_STATUS (520 B for 9 joints and 9 encoders)

```
global:   leadMs f32 | minCutoffHz f32 | beta f32 | jumpDeg f32 | gripLoadPct f32 | loadTrip u8
timing:   cycles u32 | overruns u32 | lastPeriodUs u32 | maxJitterUs u32 | p99JitterUs u32
          lastExecUs u32 | maxExecUs u32 | averageExecUs u32
bus:      sent u32 | received u32 | timeouts u32 | parseErrors u32 | echoErrors u32
          lastReplyUs u32 | maxReplyUs u32 | echo u8
encoders: averageUs u32 | maxUs u32 | maxSkewUs u32
link:     droppedEvents u32 | droppedTelemetry u32 | rxCrcErrors u32 | rxMalformed u32
monitor:  on u8 | okMask u16 | jointCount u8 | position i16 x jointCount
joints:   enabled u8 | encoderIndex u8 (0xFF none) | exoRef u16 | exoB i32 | servoA i16 | servoB i16
          vmax u16 | amax u16 | vsat u16 | trk u16 | grip u16 | ilimit u16 | itrip u8
          detectedModel u8 | effectiveCurrentMa u16 | jumpRejects u32            (32 B each)
encoderCount u8, then per encoder:
          address u8 | state u8 (0 online, 1 no magnet, 2 offline) | valid u8 | angle u16
          agc u8 | magnetOk u8 | errors u32                                       (11 B each)
```

### EVENT codes

| Code | Name | `joint` | `args` |
|---|---|---|---|
| 0 | generic | optional | — |
| 1 | refused | — | — (the command fails) |
| 2 | armed | 0 = all, or id | armed count |
| 3 | disarmed | 0 = all, or id | fault, torque-off confirmed |
| 4 | torque_off | — | confirmed |
| 5 | fault | id, or 0 = hand | fault |
| 6 | joint_state | id | new state |
| 7 | cal_captured | id | which (0 exo A, 1 exo B, 2 servo A, 3 servo B), value |
| 8 | homed | id | position (counts from A) |
| 9 | ping_result | id | model, firmware major, firmware minor, ok (1 = matches `config.h`) |
| 10 | limits_row | id | servo lo, servo hi, match (1 ok, 0 mismatch, −1 no reply, −2 travel undefined) |
| 11 | sweep_run | id | command speed, measured speed × 10 (−1 = skipped), reached, max load × 10 |
| 12 | sweep_done | id | suggested vmax, suggested vsat, applicable, aborted |
| 13 | config_saved | — | ok |
| 14 | config_loaded | — | blob status (−1 = nothing stored) |
| 15 | defaults | — | — |
| 16 | setting_changed | id, or 0 = global | — |
| 17 | monitor | — | on |

## Budget

| Stream | Size | Bandwidth |
|---|---|---|
| TELEMETRY at 100 Hz | about 465 B framed | about 47 KB/s |
| SLOW_STATUS at 10 Hz | — | about 5 KB/s |
| Events and log | — | negligible |

That is well within USB-Serial/JTAG. If not, raise `telemetryDivider`.

The firmware drops a TELEMETRY or SLOW_STATUS frame when the USB TX buffer (4 KB) has no room. These drops are counted in `droppedTelemetry`. All other frames wait up to 20 ms.

## Known limits

- Output from ROM or panic code paths can still enter the byte stream. The host shows bytes outside frames as text, and the frames around them are lost (counted). Text from the boot sequence before `host on` arrives the same way.
- Errors from the Arduino core (`log_e`) are not wrapped, so they behave the same way.
