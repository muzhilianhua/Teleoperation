# Teleoperation firmware (ESP32-S3)

Exoskeleton encoders (AS5048B) drive the dexterous hand's Feetech servos. The control loop runs entirely on the ESP32-S3; a PC is optional and only listens.

**Current milestone: M2, full hand with 9 joints.** It adds:
- per-joint state machines with fault isolation;
- round-robin servo reads;
- an on-device speed sweep that measures each joint's speed limits;
- commands to sync the servos' own angle-limit registers;
- grasping support: torque is released only when the servo's own overload protection fires, and a grip force limiter holds the squeeze.

Design document: <https://claude.ai/artifact/81sxWeamYJnzXBLv281o2v>

## Hardware

| Function | Connection |
|---|---|
| MCU | ESP32-S3-WROOM-2-N32R16V DevKit |
| Encoders | AS5048B × 9 on one I2C bus: SDA GPIO8, SCL GPIO9, 400 kHz. Addresses `0x40 0x44 … 0x60` (encoder #0–#8) burned into OTP |
| Servo bus | 9 × SCS0009, IDs 1–9, via the FE-URT-2 TTL port at 1 Mbps: GPIO17 (TX) → URT-2 RX, GPIO18 (RX) ← URT-2 TX, common GND |
| Servo power | ~6 V. The supervisor switches the hand off outside 5.5–6.5 V. **Use a supply with ample current before full-hand or grasp tests**: servo_bringup recorded 6 V sag tripping every servo at once |
| E-stop | DevKit BOOT button (GPIO0) |
| Console | Native USB port (`USB` connector), 115200 |

> Only one master may drive the servo bus. Close the servo_bringup tools while the ESP32 is connected.

## Build, test, flash

```bash
cd Nimo_Dexhand/EE/Teleoperation
~/.platformio/penv/bin/pio test -e native                    # host unit tests (85)
~/.platformio/penv/bin/pio run -e esp32-s3-n32r16 -t upload  # build + flash
~/.platformio/penv/bin/pio device monitor                    # console
```

## Joints

Joints are addressed by **servo id**. Travel limits come from servo_bringup and can only be changed in `include/config.h`, never from the console.

| ID | Joint | Travel | vmax | vsat | Note |
|---|---|---|---|---|---|
| 1 | triple_pip | 350–730 | 660 | 245 | Boot default: enabled, encoder #5 |
| 2 | triple_dip | 600–890 | 660 | 370 | |
| 3 | index_pip | 250–600 | 400 | 50 | Placeholder speeds: re-measure with `sweep 3` |
| 4 | index_dip | 250–590 | 400 | 50 | Placeholder speeds: re-measure with `sweep 4` |
| 5 | index_mcp | 435–600 | 400 | 50 | Placeholder speeds: re-measure with `sweep 5` |
| 6 | thumb_mcp_rot | 300–700 | 690 | 365 | |
| 7 | thumb_mcp | 650–900 | 400 | 50 | Placeholder speeds: re-measure with `sweep 7` |
| 8 | thumb_ip | 20–280 | 380 | 230 | |
| 9 | joint9 | 480–950 | 400 | 50 | Encoder turns 1–2× over the travel (transmission): **multi-turn, needs `home 9`**, see below |

Units are steps/s; `amax` defaults to 8000 steps/s².

- **`vmax`** is the rate-limiter ceiling, about 1.2 × the faster-direction saturation speed.
- **`vsat`** feeds the tracking check, about 0.9 × the slower-direction saturation speed.

**Adding joint 9's travel.** A joint whose travel is `0, 0` can be monitored but not enabled, calibrated, swept or armed.
1. Disarm, then run `monitor on`.
2. Move the finger by hand to both mechanical ends and read the servo 9 column.
3. Enter `min + 15` and `max − 15` (and a real name) in `kJoints` in `include/config.h`, then reflash.
4. Run `limits write 9`.

## Bring-up order after flashing M2

1. **Boot with the M1 settings in NVS.** Expect `migrated from version 1`. Joints 1–8 keep their calibration; run `save` to store the new format.
2. **Joint 9 travel.** See above.
3. **Servo angle limits.** Run `limits check`, then `limits write all`, then `limits check` again. Every defined joint should then read `ok`.
4. **Speeds.**
   - Run `sweep 1` first as a sanity check. It should come out close to servo_bringup's numbers: 411–548 steps/s in +, ~272 in −.
   - Then sweep `3 4 5 7 9`.
   - Run `sweep <j> apply` where a suggestion is printed, then `save`.
5. **Calibrate.** Calibrate the remaining joints (see *Calibration*), enable them, then `save`.
6. **Arm.** `arm`.

## Calibration

Run every step disarmed. Settings live in RAM until `save`.

1. Run `monitor on`. Move one exo joint and note which encoder column changes. Then `monitor off`.
2. Assign and enable: `set 2 enc 3`, `set 2 on`.
3. Servo endpoints. Torque is off, so move the finger by hand:
   - open → `cal 2 hand a`
   - closed → `cal 2 hand b`
4. Exo endpoints, with the glove in the matching poses:
   - open → `cal 2 a`
   - closed → `cal 2 b`

   A ↔ A and B ↔ B sets the direction. Between `cal a` and `cal b`, move the joint continuously; the encoder may turn any number of times (up to 3). If tracking is interrupted (encoder dropout), `cal b` is refused; redo `cal a`.
5. Check `joints` shows `ready yes`. Then `save`. Back up with `dump`: paste its output to restore.

## Multi-turn joints and homing (`home <j>`)

Each joint's exo position is tracked continuously from endpoint A, by summing the shortest step between samples. This stays exact while the encoder turns less than half a turn per cycle, i.e. below 18000°/s at 100 Hz. So the calibrated span may cover several encoder turns. This is the case for joint 9, whose transmission turns the encoder 1–2 times over its travel.

**What happens at power-up.** The turn count is unknown; `joints` shows it in the `home` column.

| `home` column | Joint type | Behaviour |
|---|---|---|
| `auto` | Single-turn: span + 2 × 10° dead zone < 360° | Exactly one position matches the raw reading, so the joint homes itself. This also covers spans between 180° and 360° that crossed the old fold limit |
| `wait` | Single-turn | The glove is in the unused arc; move it into range |
| `HOME!` / `homed` | Multi-turn | Several positions match. Hold the glove at **pose A** (the `cal a` pose) and run `home <j>`; only that joint needs to be disarmed. The turn nearest A is taken. A result outside the calibrated range is refused, e.g. when the glove is not at A. `arm` refuses an unhomed joint |

**Losing the home.**
- Causes: no valid sample for 100 ms (encoder dropout), or 3 rejected jumps in a row.
- A single-turn joint re-resolves at once.
- A multi-turn joint goes to HOLD. After 3 s it is switched off with a message to `home` it again.
- Moving a multi-turn joint while the ESP32 is off is harmless: you home it at the next power-up anyway.

**Planned mechanical fix.** Change joint 9's transmission so its full travel stays within one encoder turn, ideally ≤ 300°. Then redo `cal 9 a` / `cal 9 b`: the joint becomes single-turn and homes itself. No firmware change is needed.

## Speed sweep (`sweep <j>`)

This ports servo_bringup's verify-motion speed sweep to the firmware.

**What it does.**
- Arms the joint alone; every other joint must be off.
- Moves the joint to the low end of its range: the calibrated servo endpoints, or the travel, minus 10 steps each side.
- Makes single-goal moves low → high → low at speed registers 100, 300, 600 and 1000.
- Reports each move's speed, whether it arrived, and its peak load.

**Results.**
- Saturation per direction is the fastest move that arrived.
- A direction that never arrives at register ≥ 300 is **force-limited**: friction or tendon preload. It gets no suggestion; fix the mechanics and sweep again.
- A move is shown as `skipped` when the previous move never came back far enough to start a full move. This happens with a stuck direction.

**Suggestions.** `vmax = 1.2 × faster`, `vsat = 0.9 × slower`. `sweep <j> apply` writes them through the normal `set` checks; they stay in RAM until `save`.

**Stopping.** `disarm`, `x`, BOOT or any fault stops it. Load ≥ 90 % for 0.3 s aborts it, to protect a stuck joint.

**Duration.** About 10–20 s for a free joint, and up to about a minute for a stuck one.

## Grasping

The SCS0009's torque ceiling (register 16) is already at its factory maximum of 100 %. What used to stop a grasp were the firmware's own checks.

**`loadtrip` (default off).** When off, the firmware load trips (75 % / 0.3 s, 80 % / 0.1 s) never fire. A joint is released only when the servo itself reports overload or over-current, i.e. its own protection (80 % for 1 s) gave up; the disarm message says so. That joint alone goes off. `set loadtrip on` restores the M1 behaviour.

**Grip limiter, per joint `grip` (default 60 steps, 0 = off).** It caps how far the sent goal may lead the measured position.
- On contact (load ≥ `gripload`, speed < 20 steps/s), the cap shrinks by 2 steps per read, down to 3.
- When the load falls below `gripload − 10`, it grows back by 4 per read.
- The held squeeze therefore settles near `gripload` (global, default 60 %), whatever the joint's P gain. This is below the servo's 80 % protection, so a grasp can be held instead of dropped after 1 s.
- In free motion the load is low and the cap stays at `grip`.
- Raise `grip` if a joint feels slowed (watch `sent` vs `goal` in `plot`). Raise `gripload` for a firmer hold.

**Tracking check.** It is suspended while a finger is in contact. Temperature (65 °C for 1 s) and voltage protection always stay on.

**Risks.**
- Sustained grasps heat the servos.
- Several fingers squeezing at once draw a large current; see the power note above.
- Check tendon and gear strength with the mechanical side.

## Console

| Command | Effect |
|---|---|
| `help` | List commands |
| `status` | Armed count, last hand-level fault, one line per enabled or armed joint, global parameters. Each joint line shows state, exo, target, goal, sent goal, grip lead (`*` = contact), and servo position, torque, load, voltage and temperature |
| `joints` | All 9 joints: enabled, encoder, exo span, servo endpoints, travel, `vmax`/`amax`/`vsat`/`trk`, ready, state, last fault |
| `enc` | All encoder channels with state, AGC, magnet flags, errors and the joint each one drives |
| `stats [reset]` | Loop jitter (max, p99), execution time, servo bus and encoder statistics, jump-guard rejections |
| `monitor on\|off` | 10 Hz rows: 9 encoder angles and 9 servo positions, polling one servo per cycle. Disarmed only |
| `ping` | Scan the hand servos and check the model number. Disarmed only |
| `arm` | Torque on for every enabled joint not yet armed. All must be ready |
| `arm <j>` | Torque on for one joint while others follow, e.g. after it was isolated by a fault |
| `disarm` / `disarm <j>` / `x` | Torque off: all joints / one joint / E-stop on every servo. BOOT does the same as `x` |
| `set <j> enc <n\|none>` / `set <j> on\|off` | Assign encoder / enable joint. Disarmed only |
| `cal <j> a\|b` / `cal <j> hand a\|b` | Capture the exo endpoint / the servo endpoint (move the finger by hand). Disarmed only |
| `home <j>` | Multi-turn joint: the glove is at pose A, fix the turn count. Only that joint must be disarmed |
| `set <j> exospan <rawA> <span>` / `set <j> servo <a> <b>` | Enter endpoints directly (the `dump` format; span in counts, multi-turn allowed). The old `set <j> exo <rawA> <rawB>` from earlier dumps still works for spans below half a turn |
| `set <j> vmax\|amax\|vsat\|trk\|grip <n>` | Per-joint motion, tracking and grip parameters. Allowed while armed |
| `set lead\|mincut\|beta\|jump\|gripload <v>` | Global parameters. Allowed while armed |
| `set loadtrip on\|off` | Firmware load trips (off = only the servo's overload protection decides) |
| `sweep <j>` / `sweep <j> apply` | Measure speed and suggest `vmax`/`vsat` / apply the suggestion. `sweep <j>` is disarmed only |
| `limits check` / `limits write <j\|all>` | Compare / sync the servos' angle-limit registers with the travel. Writes only differing values. Disarmed only |
| `save` / `load` / `defaults` / `dump` | Write NVS / reload / reset to defaults in RAM / print as commands |
| `plot <j> on` / `plot off` | Teleplot at 50 Hz: `exo_deg exo_filt_deg target goal sent ref grip_lead present load speed_reg` |

## Behaviour

**Per-joint states.** `OFF → ALIGN → FOLLOW ⇄ HOLD`, plus `SWEEP`. Each joint moves through these on its own:

| State | Behaviour |
|---|---|
| ALIGN | Moves to the mapped target at 50 steps/s, then reports `aligned -> FOLLOW` |
| FOLLOW | Follows the glove |
| HOLD | Entered on encoder loss (3 cycles); freezes the joint. If the encoder is healthy again for 200 ms, the joint re-aligns; after 3 s it is switched off |

**Fault scope.**

| Scope | Faults | Action |
|---|---|---|
| Joint | Load trip (only when `loadtrip on`), overheat, tracking lag, servo error (overload, over-current, angle sensor), encoder lost > 3 s | Only that joint goes off. `joints` shows why; `arm <j>` brings it back |
| Hand | Supply voltage, servo voltage error, torque mismatch, comm lost (3 failed reads or no good read for 100 ms), E-stop | Every servo goes off |

**Servo reads.**
- Full status reads are round-robin, 3 servos per cycle, so each servo is read about every 30 ms with 9 enabled.
- A goal is written only if that joint's last good read is at most 50 ms old and showed torque on: a goal write to a disarmed SCS0009 would switch its torque on by itself.
- All goals go out in one SYNC_WRITE.
- Arming 9 joints at once takes about 22 ms, which counts one overrun in `stats`.

**Arming sequence, per joint.** These rules are measured SCS0009 behaviour, from servo_bringup:
1. Read the servo state: no error, position inside travel.
2. Write goal = present position at speed 50.
3. Write torque on.
4. Read back: torque on, goal = hold position, movement ≤ 5 steps.

**Other rules:**
- The speed register is never 0 (0 means maximum speed).
- The goal time is always 0.
- Torque-off is broadcast to all 9 servos at boot and on E-stop: SCS0009 has no communication watchdog.
- `save`, `limits` and `sweep` are refused while any joint is armed.

**NVS.** Format version 3 is 231 bytes and holds 9 joints, the grip fields and a 32-bit exo span.
- Versions 2 (M2) and 1 (M1, 8 joints) migrate on load; run `save` afterwards.
- Missing, corrupt or unknown-version data is rejected, and defaults are used.
- Data saved for another servo series keeps the exo calibration but clears the servo endpoints.

## M2 bench test

1. **Migration.** Covered by bring-up steps 1–3 above.
2. **Sweeps.**
   - `sweep 1` should be close to the servo_bringup numbers.
   - Sweep `3 4 5 7 9`, then `apply` and `save` for any that are not force-limited.
   - Hand the force-limited ones, with their load data, to the mechanical side.
3. **Full hand.**
   - Calibrate and enable all 9 joints, then `arm`. All of them follow.
   - Run `stats reset`, follow for 10 min, then `stats`.
   - Acceptance: jitter max < 500 µs and average execution time ≤ 5 ms. No overruns apart from the arming cycle.
4. **Grasp.** Use the upgraded supply.
   - Run `plot <j> on`. Grasp a sponge: the load settles near 60 %, `grip_lead` shrinks, and the grasp holds ≥ 10 s.
   - Run `set gripload 75` and squeeze firmer.
   - With `set <j> grip 300` and the finger blocked, the servo's own protection releases that joint after about 1 s; the others keep following.
5. **Isolation.**
   - With `set loadtrip on`, block one finger: only that joint goes off, and `arm <j>` re-arms it. Then `set loadtrip off`.
   - Unplug one encoder: that joint goes to HOLD, then off after 3 s.
   - Press BOOT: everything goes off.
6. **Multi-turn joint 9.**
   - Run `cal 9 hand a/b`, then `cal 9 a` at the open pose. Turn slowly to the other end and run `cal 9 b`: `joints` shows a span of 360–720° and `HOME!`/`homed`. Then `save` and reboot.
   - `arm 9` must be refused and ask for `home 9`. Run `home 9` at pose A, then `arm 9`: it follows the full travel with no jump when the encoder crosses 0°.
   - `home 9` with the glove near B must be refused.
   - Unplug and replug encoder 0x40: joint 9 goes to HOLD, then off after 3 s with a request to `home` it.

## Mapping measurement (`tools/measure_mapping.py`)

Measures how well the two-point mapping makes the robot follow the glove, and which change the data points to (session zero, a 3-point mapping, angle mapping, or none). It needs only the standard library; plots need matplotlib, which `uv run` installs.

**1. Record.** Log the serial output:

```bash
~/.platformio/penv/bin/pio device monitor | tee measure.log
```

Per round, hold each glove pose still for 2 s and type `status`:
- closing: A, 25 %, 50 %, 75 %, B;
- then opening: B, 75 %, 50 %, 25 %, A.

Take the glove off and on between rounds; 2–3 rounds is enough.

**2. Extract.** This writes one row per snapshot and joint, with round, direction and pose pre-filled for that protocol. Fix them if you typed an extra `status`.

```bash
uv run tools/measure_mapping.py extract measure.log --joints 1 6 9 -o poses.csv
```

**3. Fill in** `theta_h` (human joint angle from the jig) and `theta_r` (measured robot joint angle), in degrees.
- With `theta_h` empty, the exoskeleton encoder angle is used instead.
- With `theta_r` empty and `--theta-r-from-pos 0.293`, the servo position is used. This is valid for direct-drive joints only.

**4. Analyze.**

```bash
uv run tools/measure_mapping.py analyze poses.csv --plot plots/
```

| Reported | Meaning | Points to |
|---|---|---|
| `\|pos-sent\|` > 10 steps | The servo did not follow its own goal | Fix the servo side first; the rest is polluted |
| Offset spread > 3° | Re-donning shifts the mapping between rounds | Session zero |
| Non-linearity > 5° | Largest pose-mean deviation from each round's A–B line, i.e. what the firmware's linear mapping gets wrong mid-range | 3-point / piecewise mapping |
| Slope vs `--expected-gain` | Proportional, not equal-angle | Angle mapping |
| Hysteresis > 3° | Closing vs opening at the same pose | Backlash: not fixed by any calibration |

Thresholds are options (`--offset-deg`, `--nonlinear-deg`, …).

## Layout

```
include/config.h          pins, bus, joint table (travel, vmax/vsat/amax), global defaults, timing
lib/AS5048B/              encoder driver (vendored, see below)
lib/FeetechProtocol/      pure C++ packet codec (host-tested)
lib/FeetechBus/           UART transport, ServoBus interface (incl. angle limits), ScsBus backend
lib/TeleopCore/           pure C++, host-tested:
  joint_map / joint_config  mapping, per-joint settings, validation
  config_blob               NVS serialization (CRC32, version 3, v1/v2 migration, servo series)
  exo_tracker               continuous multi-turn exo position, auto-resolve / homing
  command_parser            console grammar, settings edits, dump format
  supervisor                per-joint checks, fault scope
  grip_limiter              load-adaptive goal lead cap
  speed_sweep               saturation speed measurement
  jump_guard / one_euro / rate_limiter
src/control_task.*        100 Hz loop on core 1: per-joint states, reads, arming, sweep, settings
src/config_store.*        Preferences (NVS) read/write
src/console.*             USB console on core 0
src/main.cpp              setup and task start
test/                     Unity tests (native): packet, core, config, sweep, tracker
tools/measure_mapping.py  mapping consistency measurement (host Python)
```

## Vendored code

`lib/AS5048B` is copied unmodified from `~/Documents/PlatformIO/Projects/Dexhand_Encoded`, commit `cd89c14` ("feat: otp_address_jig and multi_read examples"), on 2026-10-14. Make fixes there first, then re-copy.
