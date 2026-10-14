# Teleoperation firmware (ESP32-S3)

Exoskeleton encoders (AS5048B) drive the dexterous hand's Feetech servos. The control loop runs entirely on the ESP32-S3; a PC is optional and only listens.

**Current milestone: M1.** The joint table covers all 8 hand servos:
- per-joint calibration is entered on the console and stored in NVS;
- speed limits are set per joint from measured servo capability;
- an encoder jump guard and a capability-based tracking check are added.

M1 is validated with 1–2 enabled joints. Full-hand operation is M2.

Design document: <https://claude.ai/artifact/81sxWeamYJnzXBLv281o2v>

## Hardware

| Function | Connection |
|---|---|
| MCU | ESP32-S3-WROOM-2-N32R16V DevKit |
| Encoders | AS5048B × 9 on one I2C bus: SDA GPIO8, SCL GPIO9, 400 kHz. Addresses `0x40 0x44 … 0x60` (encoder #0–#8) burned into OTP |
| Servo bus | FE-URT-2 TTL port, 1 Mbps: GPIO17 (TX) → URT-2 RX, GPIO18 (RX) ← URT-2 TX, common GND |
| Servo power | ~6 V. The supervisor trips outside 5.5–6.5 V |
| E-stop | DevKit BOOT button (GPIO0) |
| Console | Native USB port (`USB` connector), 115200 |

> Only one master may drive the servo bus. Close the servo_bringup tools while the ESP32 is connected.

## Build, test, flash

```bash
cd Nimo_Dexhand/EE/Teleoperation
~/.platformio/penv/bin/pio test -e native                    # host unit tests
~/.platformio/penv/bin/pio run -e esp32-s3-n32r16 -t upload  # build + flash
~/.platformio/penv/bin/pio device monitor                    # console
```

The firmware environment builds with `-std=gnu++17`; the core's default is gnu++11.

## Joints and speed limits

Joints are addressed by **servo id**. Travel limits come from servo_bringup and can only be changed in `include/config.h`, never from the console.

| ID | Joint | Travel | vmax | vsat | Note |
|---|---|---|---|---|---|
| 1 | triple_pip | 350–730 | 660 | 245 | Boot default: enabled, encoder #5 |
| 2 | triple_dip | 600–890 | 660 | 370 | |
| 3 | index_pip | 250–600 | 400 | 50 | Force-limited, placeholder values |
| 4 | index_dip | 250–590 | 400 | 50 | Force-limited, placeholder values |
| 5 | index_mcp | 435–600 | 400 | 50 | Force-limited, placeholder values |
| 6 | thumb_mcp_rot | 300–700 | 690 | 365 | |
| 7 | thumb_mcp | 650–900 | 400 | 50 | Force-limited, placeholder values |
| 8 | thumb_ip | 20–280 | 380 | 230 | |

Units are steps/s; `amax` defaults to 8000 steps/s² for every joint.

**`vmax`: rate-limiter ceiling.** About 1.2 × the servo's saturation speed in its faster direction, taken from the servo_bringup speed sweeps. Normal motion is never slowed by firmware; only physically impossible jumps are.

**Why not `vmax` = 1000?** The SCS0009 saturates at 270–570 steps/s (80–170°/s). A higher ceiling cannot make the hand faster. It only removes protection against bad encoder data, since the hand has no mechanical end stops.

**`vsat`: tracking check.** About 0.9 × the saturation speed in the slower direction. A model limiter chases the goal at `vsat`, and the joint trips only when the servo falls more than `trk` steps (default 40) behind that model for 0.4 s. A goal that outruns the servo's physical speed is therefore not a fault.

**Force-limited joints (3/4/5/7).** Friction and tendon preload limit them to 2–10 steps/s, so their tracking check trips until the mechanics are fixed. Validate M1 with joints 1/2/6/8.

**Jump guard.** An exo change above `jump` degrees per cycle (default 25° = 2500°/s) is discarded. Three in a row count as an encoder fault: HOLD, re-sync, then a slow re-ALIGN.

## Calibration workflow

Run every step disarmed. Settings live in RAM until `save`.

1. Run `monitor on`. Move one exoskeleton joint and note which encoder column changes. Then `monitor off`.
2. Assign and enable: `set 2 enc 3`, `set 2 on`.
3. Servo endpoints. Torque is off, so move the finger by hand:
   - open → `cal 2 hand a`
   - closed → `cal 2 hand b`
4. Exo endpoints, with the glove in the matching poses:
   - open → `cal 2 a`
   - closed → `cal 2 b`

   A ↔ A and B ↔ B sets the direction.
5. Check that `joints` shows `ready yes`. Then `save`.
6. Back up with `dump`: it prints the configuration as commands starting with `defaults`. Pasting them back restores it exactly; then run `save`.

## Console

| Command | Effect |
|---|---|
| `help` | List commands |
| `status` | State, last fault, one line per enabled joint (exo, target, goal, model ref, servo readings), global parameters |
| `joints` | All 8 joints: enabled, encoder, exo span, servo endpoints, travel, `vmax`/`amax`/`vsat`/`trk`, ready |
| `enc` | All encoder channels with state, AGC, magnet flags, errors and the joint each one drives |
| `stats [reset]` | Loop jitter (max, p99), execution time, servo bus and encoder statistics, jump-guard rejections |
| `monitor on\|off` | 10 Hz rows: 9 encoder angles and all 8 servo positions, polling one servo per cycle. Disarmed only |
| `ping` | Scan the hand servos and check the model number. Disarmed only |
| `arm` | Torque on for every enabled joint. All must be ready |
| `disarm` / `x` | Torque off / E-stop on all servos. BOOT does the same as `x` |
| `set <j> enc <n\|none>` / `set <j> on\|off` | Assign encoder / enable joint. Disarmed only |
| `cal <j> a\|b` | Capture exo endpoint A / B from the live encoder |
| `cal <j> hand a\|b` | Capture servo endpoint A / B from the servo position (moved by hand) |
| `set <j> exo <rawA> <rawB>` / `set <j> servo <a> <b>` | Enter endpoints directly (this is the `dump` format) |
| `set <j> vmax\|amax\|vsat\|trk <n>` | Per-joint motion / tracking parameters. Allowed while armed |
| `set lead\|mincut\|beta\|jump <v>` | Global parameters. Allowed while armed |
| `save` / `load` / `defaults` / `dump` | Write NVS / reload NVS / reset to defaults in RAM / print as commands. The first three are disarmed only |
| `plot <j> on` / `plot off` | Teleplot traces at 50 Hz: `exo_deg exo_filt_deg target goal ref present load speed_reg` |

**When NVS is loaded at boot:**

| Stored data | Result |
|---|---|
| Missing, wrong size, bad CRC, or unknown version | Rejected; defaults are used |
| Saved for another servo series (SCS ↔ HLS) | Exo calibration is kept; servo endpoints are cleared |
| A joint fails validation | That joint is reset to defaults |

## Behaviour

**States.** `IDLE → ALIGN → FOLLOW ⇄ HOLD → IDLE`. There is one state machine for the hand; per-joint fault isolation is M2.

| State | Behaviour |
|---|---|
| IDLE | Torque off. Goals are never written |
| ALIGN | Every armed joint moves to its mapped target at 50 steps/s. Keep the glove still until `FOLLOW` is reported |
| FOLLOW | Goals are rate-limited per joint and all go out in one SYNC_WRITE per cycle |
| HOLD | Any armed joint's encoder lost for 3 cycles. All goals decelerate and freeze. If the encoders are healthy for 200 ms, the hand re-ALIGNs; after 3 s it disarms |

**Arming sequence, per joint.** These rules are measured SCS0009 behaviour, from servo_bringup:
1. Read the servo state: no error, position inside travel.
2. Write goal = present position at speed 50. A goal write turns torque on by itself.
3. Write torque on.
4. Read back: torque on, goal = hold position, movement ≤ 5 steps.

Any failure disarms every joint.

**Supervisor.** Any of these disarms the hand:

| Trip | Condition |
|---|---|
| Load | ≥ 75 % for 0.3 s, or ≥ 80 % for 0.1 s |
| Temperature | ≥ 65 °C for 1 s |
| Supply | Median voltage outside 5.5–6.5 V for 0.2 s |
| Servo reads | 3 consecutive failures |
| Torque state | Disagrees with the firmware |
| Servo error | Non-zero error byte or status register |
| Tracking | Lag behind the capability model, as above |

**Other servo rules:**
- The speed register is never 0 (0 means maximum speed).
- The goal time is always 0.
- Torque-off is broadcast at boot and on E-stop, because SCS0009 has no communication watchdog.
- `save` is refused while armed: writing flash stalls both cores' caches.

## M1 bench test

1. **Fresh NVS.** Expect `no stored configuration, using defaults`. `joints` shows joint 1 enabled on encoder #5, not ready.
2. **Monitor.** Run `monitor on` and move each exo joint to identify its encoder. Enable a second joint (2, 6 or 8): `set <j> enc <n>`, `set <j> on`.
3. **Calibrate and persist.** Calibrate both joints (`cal <j> hand a/b`, `cal <j> a/b`), then `save` and reboot.
   - Expect `configuration loaded from NVS`.
   - `dump` must match the output before the reboot.
   - Paste the dump back: no change.
4. **Follow.** Run `arm`; both joints follow.
   - With `plot <j> on`, the `goal` slope reaches `vmax` on fast moves and `present` reaches the servo's saturation speed.
   - No tracking trips on fast moves of joints 1/2/6/8.
5. **Jump guard.** Run `set jump 2`. Fast moves should cause `HOLD` and a re-sync. Restore with `set jump 25`.
6. **Timing.** Run `stats reset`, follow for 10 min, then run `stats`.
   - Acceptance: jitter max < 500 µs.
   - Record the execution time with 2 joints as the M2 budget input.
7. **Save refused.** `save` while armed must be refused.

## Layout

```
include/config.h          pins, bus, joint table (travel, vmax/vsat/amax), global defaults, thresholds
lib/AS5048B/              encoder driver (vendored, see below)
lib/FeetechProtocol/      pure C++ packet codec (host-tested)
lib/FeetechBus/           UART transport, ServoBus interface, ScsBus backend (SCS0009)
lib/TeleopCore/           pure C++, host-tested:
  joint_map / joint_config  mapping, per-joint settings, validation
  config_blob               NVS serialization (CRC32, version, servo series)
  command_parser            console grammar, settings edits, dump format
  jump_guard / one_euro / rate_limiter / supervisor
src/control_task.*        100 Hz loop on core 1: joint channels, state machine, arming, settings
src/config_store.*        Preferences (NVS) read/write
src/console.*             USB console on core 0
src/main.cpp              setup and task start
test/                     Unity tests (native): packet, core, config
```

## Vendored code

`lib/AS5048B` is copied unmodified from `~/Documents/PlatformIO/Projects/Dexhand_Encoded`, commit `cd89c14` ("feat: otp_address_jig and multi_read examples"), on 2026-10-14. Make fixes there first, then re-copy.
