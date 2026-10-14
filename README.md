# Teleoperation firmware (ESP32-S3)

Exoskeleton encoders (AS5048B) drive the dexterous hand's Feetech servos. The control loop runs entirely on the ESP32-S3; a PC is optional and only listens.

**Current milestone: M0.** One encoder drives one servo through the full path:

> encoder → One-Euro filter → two-point mapping → rate limiter → servo goal

It runs with arming, a safety supervisor and a USB console. Design document: <https://claude.ai/artifact/81sxWeamYJnzXBLv281o2v>

## Hardware

| Function | Connection |
|---|---|
| MCU | ESP32-S3-WROOM-2-N32R16V DevKit |
| Encoders | AS5048B × 9 on one I2C bus: SDA GPIO8, SCL GPIO9, 400 kHz. Addresses `0x40 0x44 … 0x60` are burned into OTP |
| Servo bus | FE-URT-2 TTL port, 1 Mbps: ESP32 GPIO17 (TX) → URT-2 RX, GPIO18 (RX) ← URT-2 TX, common GND |
| Servo power | ~6 V. The supervisor trips outside 5.5–6.5 V |
| E-stop | DevKit BOOT button (GPIO0, active low) |
| Console | native USB port (`USB` connector), 115200 |

**M0 joint:**

| Item | Value |
|---|---|
| Servo | ID 1 `triple_pip` |
| Travel | 350–730 |
| Encoder | #0 (`0x40`) |

These are set in `include/config.h`. Values marked `[bringup]` come from `../servo_bringup/dexhand_servo/config.py`.

> Only one master may drive the servo bus. When the ESP32 is connected, close the servo_bringup tools.

## Build, test, flash

```bash
cd Nimo_Dexhand/EE/Teleoperation
~/.platformio/penv/bin/pio test -e native                    # host unit tests (packet codec, mapping, limiter, supervisor)
~/.platformio/penv/bin/pio run -e esp32-s3-n32r16            # build
~/.platformio/penv/bin/pio run -e esp32-s3-n32r16 -t upload  # flash over the USB connector
~/.platformio/penv/bin/pio device monitor                    # console
```

For live traces, use the Teleplot VS Code extension on the same port and run `plot on`.

## Console

| Command | Effect |
|---|---|
| `help` | List commands |
| `status` | State machine, joint calibration, encoder/target/goal, servo readings, parameters |
| `stats [reset]` | Loop period jitter (max, p99), execution time, servo bus and encoder statistics |
| `enc` | All encoder channels: state, angle, AGC, magnet flags, errors |
| `ping` | Scan servo IDs 1–8 and check the model number (SCS0009 = 1284). Only while disarmed |
| `arm` / `disarm` | Torque on (hold → ALIGN → FOLLOW) / torque off |
| `x` | E-stop: torque off on every servo, read back. The BOOT button does the same |
| `exo a` / `exo b` | Capture the current encoder reading as endpoint A / B (RAM only until M1) |
| `set sa <n>` / `set sb <n>` | Servo counts that correspond to exo A / B. Defaults are the travel limits |
| `set vmax <n>` / `set amax <n>` | Rate-limiter speed (steps/s) / acceleration (steps/s²) |
| `set lead <ms>` | Look-ahead on the goal, against SCS stick-slip. 0 = off |
| `set mincut <hz>` / `set beta <f>` | One-Euro filter |
| `plot on\|off` | Teleplot traces at 50 Hz: `exo_deg exo_filt_deg target goal present load speed_reg` |

## Behaviour

**States.** `IDLE → ALIGN → FOLLOW ⇄ HOLD → IDLE`:

| State | Behaviour |
|---|---|
| IDLE | Torque off. Goals are never written |
| ALIGN | Moves from the held position to the mapped target at `arm_speed` (50 steps/s). Hold the exoskeleton still until `FOLLOW` is reported |
| FOLLOW | Goal = rate-limited mapped target, written with one SYNC_WRITE per cycle |
| HOLD | Encoder lost for 3 cycles. The goal decelerates and freezes. If the encoder is healthy again for 200 ms, the joint re-ALIGNs; after 3 s it disarms |

**Arming sequence.** These rules are measured SCS0009 behaviour, taken from servo_bringup:

1. Read the servo state. It must show no error and a position inside travel.
2. Write goal = present position at speed 50. A goal write turns torque on by itself, so the joint only ever holds where it is.
3. Write torque on.
4. Read back and check: torque on, goal register = hold position, and the joint has moved no more than 5 steps.

**Safety supervisor.** Thresholds come from servo_bringup `MotionConfig`. Any of these disarms the joint:

| Trip | Condition |
|---|---|
| Load | ≥ 75 % for 0.3 s, or ≥ 80 % for 0.1 s |
| Temperature | ≥ 65 °C for 1 s |
| Supply | Median voltage outside 5.5–6.5 V for 0.2 s |
| Servo reads | 3 consecutive failures |
| Torque state | Disagrees with the firmware |
| Servo error | Non-zero error byte or status register |
| Tracking error | > 40 steps for 0.4 s |

**Other servo rules:**
- The speed register is never 0 (0 means maximum speed).
- The goal time is always 0.
- SCS0009 has no communication watchdog, so torque-off is broadcast at boot and on E-stop.

## M0 bench test

Check each step in order. Keep a hand on the BOOT button while the servo has torque.

1. **Boot log.** Expect:
   - encoders responding
   - servo bus echo result
   - `torque off on all servos`
   - `servo 1: model 1284`
2. **Encoder.** Run `enc` and confirm channel 0 is `ok` with a mid-range AGC. Run `plot on` and turn the exoskeleton joint: `exo_deg` should follow.
3. **No torque while disarmed.** Run `exo a` at one end, move to the other end and run `exo b`. `status` should then show `ready 1`. Moving the exoskeleton changes `target`, but the servo must not move.
4. **Follow.**
   - Run `arm`. Expect `armed, holding …`, then `ALIGN done -> FOLLOW`.
   - Move the exoskeleton and check the direction. If it is reversed, swap the values with `set sa` / `set sb`.
   - Fast moves must be limited to `vmax`: `goal` slope ≤ 200 steps/s.
5. **Faults.**

   | Action | Expected result |
   |---|---|
   | Unplug the encoder SDA | `HOLD`, then `DISARM: encoder lost > 3 s` |
   | Press BOOT | Torque off within one cycle |
   | Block the finger by hand | Load trip disarm |

6. **Timing.** Run `stats reset`, let it follow for 10 min, then run `stats`.
   - Acceptance: jitter max < 500 µs.
   - Record the average and maximum execution time as the budget baseline for M2.

## Layout

```
include/config.h          pins, bus, joint and motion defaults, thresholds
lib/AS5048B/              encoder driver (vendored, see below)
lib/FeetechProtocol/      pure C++ packet codec (host-tested)
lib/FeetechBus/           UART transport, ServoBus interface, ScsBus backend (SCS0009)
lib/TeleopCore/           pure C++ mapping, One-Euro, rate limiter, supervisor (host-tested)
src/control_task.*        100 Hz loop on core 1: state machine, arming, supervision
src/console.*             USB console on core 0
src/main.cpp              setup and task start
test/                     Unity tests for the native environment
```

`ServoBus` hides register addresses, byte order and resolution. M5 (HLS3606) adds an `HlsBus` backend without touching the control code.

## Vendored code

`lib/AS5048B` is copied unmodified from `~/Documents/PlatformIO/Projects/Dexhand_Encoded`, commit `cd89c14` ("feat: otp_address_jig and multi_read examples"), on 2026-10-14. Make fixes there first, then re-copy.
