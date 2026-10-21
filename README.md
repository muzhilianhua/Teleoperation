# Teleoperation firmware (ESP32-S3)

Exoskeleton encoders (AS5048B) drive the dexterous hand's Feetech servos. The control loop runs entirely on the ESP32-S3; a PC is optional and only listens.

**Current target: the rebuilt hand (SCS0009 + HLS3606 on one bus), same glove and wiring.** It adds, on top of M2:
- per-servo models on one bus: each id is identified at boot and encoded/decoded as SCS0009 or HLS3606;
- HLS3606 arming checks, a goal current (`ilimit`) and an over-current trip;
- speeds in steps/s end to end, with step-based defaults and tolerances scaled per model;
- a new joint table (placeholders until measured) and NVS format v4: earlier calibrations are discarded.

M2 (full hand with 9 joints) added:
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
| Servo bus | IDs 1–9, mixed models (see *Joints*), via the FE-URT-2 TTL port at 1 Mbps: GPIO17 (TX) → URT-2 RX, GPIO18 (RX) ← URT-2 TX, common GND |
| Servo power | ~6 V. The supervisor switches the hand off outside 5.5–6.5 V. **Use a supply with ample current before full-hand or grasp tests**: servo_bringup recorded 6 V sag tripping every servo at once |
| E-stop | DevKit BOOT button (GPIO0) |
| Console | Native USB port (`USB` connector), 115200 |

> Only one master may drive the servo bus. Close the servo_bringup tools while the ESP32 is connected.

## Build, test, flash

```bash
cd Nimo_Dexhand/EE/Teleoperation
~/.platformio/penv/bin/pio test -e native                    # host unit tests (94)
~/.platformio/penv/bin/pio run -e esp32-s3-n32r16 -t upload  # build + flash
~/.platformio/penv/bin/pio device monitor                    # console
```

## Joints

Joints are addressed by **servo id**. Model, name and travel live in `kJoints` in `include/config.h` and can only be changed there, never from the console.

| ID | Model | Name | Travel | Default vmax / vsat (own steps/s) |
|---|---|---|---|---|
| 1, 3, 5 | SCS0009 | `id1`, `id3`, `id5` | undefined (0, 0) | 400 / 50 |
| 2, 4, 6, 7, 8, 9 | HLS3606 | `id2` … `id9` | undefined (0, 0) | 1333 / 167 |

Names and travel are **placeholders** until the rebuilt hand is measured on the bench (see *Bring-up*); every joint boots disabled. The models come from the 2026-10-21 bus scan.

**Two models, one bus.**

| | SCS0009 | HLS3606 |
|---|---|---|
| Resolution | 1024 steps ≈ 300°, 0.293°/step | 4096 steps/turn, 0.088°/step (1 SCS step ≈ 3.333 HLS steps) |
| Speed register | ≈ steps/s, 0 = full speed | ≈ 49.97 steps/s per unit, 0 = stop, max 250 |
| Goal block (reg 42) | position, time = 0, speed | position, **goal current** (6.5 mA/unit), speed |
| Lock register | 48 | 55 |
| Factory angle limits | single-turn | 0/0 = multi-turn: **must be written before arming** |

Both models share one SYNC_WRITE (address 42, 6 bytes per servo); each entry is encoded in its own model's byte order.

**Units.** Every per-joint value is in **that servo's own steps**: `servo A/B`, travel, `vmax`/`vsat` (steps/s), `amax` (steps/s²), `trk`, `grip`. The firmware's fixed step values (arm speed 50, align tolerance 3, arm hold tolerance 5, stream speed floor 50, sweep tiers and margins, grip-limiter steps) are SCS0009 steps and are multiplied by 3.333 on an HLS3606 joint, so both models move the same angle. Defaults scale the same way: `trk` 40 → 133, `grip` 60 → 200.

- **`vmax`** is the rate-limiter ceiling, about 1.2 × the faster-direction saturation speed.
- **`vsat`** feeds the tracking check, about 0.9 × the slower-direction saturation speed.
- **`ilimit`** (HLS only, mA): the goal current is the servo's own protection current (register 28, factory 800 = 5.2 A), or `ilimit` if lower. `0` = protection current. Range 7–13306.

**Model check.** At boot (and on `ping`) every id is identified. A servo whose model differs from `config.h` is reported, still decoded as what it really is (so `monitor` and `limits` work), and its joint cannot be armed until `config.h` is fixed. `joints` marks it `SCS?` / `HLS?`.

## Bring-up of the rebuilt hand

Old calibrations are discarded at the first boot (`saved for the hand before the rebuild`). Run every step disarmed.

1. **Models.** `ping`: ids 1, 3, 5 report SCS0009, the others HLS3606, no mismatch.
2. **Travel.** `monitor on`, move each finger by hand to both mechanical ends and note the servo column. An HLS3606 still in factory multi-turn mode may show negative or > 4095 readings; note them anyway.
3. **`config.h`.** For each joint enter a name and the travel: `min + 15` / `max − 15` for SCS0009, `min + 50` / `max − 50` for HLS3606. Reflash.
4. **Angle limits.** `limits check`, then `limits write all`. HLS3606 servos are unlocked at register 55, written and locked again. **Power-cycle the servos**, then `limits check`: every joint `ok`.
   - An HLS joint whose travel crosses the encoder zero cannot be expressed by limits: set a position offset with DEX_hand_software's `hls-setup` first (not implemented here).
5. **Speeds.** `sweep <j>` for each joint, `sweep <j> apply` where a suggestion is printed.
6. **Calibrate** (see *Calibration*); joint 9 needs `home 9` if it stays multi-turn. Optionally `set <j> ilimit <mA>` for HLS joints.
7. **`save`**, then `arm` one joint at a time, then the whole hand.

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
- Moves the joint to the low end of its range: the calibrated servo endpoints, or the travel, minus 10 SCS steps (33 HLS steps) each side.
- Makes single-goal moves low → high → low at four speed tiers: 100, 300, 600, 1000 steps/s on an SCS0009, 333, 1000, 2000, 3333 on an HLS3606.
- Reports each move's speed, whether it arrived, and its peak load.

**Results.**
- Saturation per direction is the fastest move that arrived.
- A direction that never arrives from the second tier on is **force-limited**: friction or tendon preload. It gets no suggestion; fix the mechanics and sweep again.
- A move is shown as `skipped` when the previous move never came back far enough to start a full move. This happens with a stuck direction.

**Suggestions.** `vmax = 1.2 × faster`, `vsat = 0.9 × slower`. `sweep <j> apply` writes them through the normal `set` checks; they stay in RAM until `save`.

**Stopping.** `disarm`, `x`, BOOT or any fault stops it. Load ≥ 90 % for 0.3 s aborts it, to protect a stuck joint.

**Duration.** About 10–20 s for a free joint, and up to about a minute for a stuck one.

## Grasping

The SCS0009's torque ceiling (register 16) is already at its factory maximum of 100 %. What used to stop a grasp were the firmware's own checks.

**HLS3606 current.** Every goal carries the joint's goal current (`min(protection, ilimit)`), so the squeeze is limited by current, not by the load percentage alone. The firmware trips `over-current` on that joint alone when |current| ≥ 0.9 × the cap for 0.2 s (as DEX_hand_software does). Lower `ilimit` for a gentler grasp; `status` shows the present current.

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
| `status` | Armed count, last hand-level fault, one line per enabled or armed joint, global parameters. Each joint line shows state, exo, target, goal, sent goal, grip lead (`*` = contact), commanded speed (steps/s), and servo position, torque, load, current (HLS), voltage and temperature |
| `joints` | All 9 joints: model (`?` = not identified / mismatch), enabled, encoder, exo span, servo endpoints, travel, `vmax`/`amax`/`vsat`/`trk`, `ilimit` (`prot` = protection current), ready, state, last fault |
| `enc` | All encoder channels with state, AGC, magnet flags, errors and the joint each one drives |
| `stats [reset]` | Loop jitter (max, p99), execution time, servo bus and encoder statistics, jump-guard rejections |
| `monitor on\|off` | 10 Hz rows: 9 encoder angles and 9 servo positions, polling one servo per cycle. Disarmed only |
| `ping` | Identify every hand servo (model, firmware) and compare with `config.h`. Disarmed only |
| `arm` | Torque on for every enabled joint not yet armed. All must be ready |
| `arm <j>` | Torque on for one joint while others follow, e.g. after it was isolated by a fault |
| `disarm` / `disarm <j>` / `x` | Torque off: all joints / one joint / E-stop on every servo. BOOT does the same as `x` |
| `set <j> enc <n\|none>` / `set <j> on\|off` | Assign encoder / enable joint. Disarmed only |
| `cal <j> a\|b` / `cal <j> hand a\|b` | Capture the exo endpoint / the servo endpoint (move the finger by hand). Disarmed only |
| `home <j>` | Multi-turn joint: the glove is at pose A, fix the turn count. Only that joint must be disarmed |
| `set <j> exospan <rawA> <span>` / `set <j> servo <a> <b>` | Enter endpoints directly (the `dump` format; span in counts, multi-turn allowed). The old `set <j> exo <rawA> <rawB>` from earlier dumps still works for spans below half a turn |
| `set <j> vmax\|amax\|vsat\|trk\|grip <n>` | Per-joint motion, tracking and grip parameters, in the servo's own steps. Allowed while armed |
| `set <j> ilimit <mA>` | HLS goal-current cap, 0 = protection current. Allowed while armed (applied at once) |
| `set lead\|mincut\|beta\|jump\|gripload <v>` | Global parameters. Allowed while armed |
| `set loadtrip on\|off` | Firmware load trips (off = only the servo's overload protection decides) |
| `sweep <j>` / `sweep <j> apply` | Measure speed and suggest `vmax`/`vsat` / apply the suggestion. `sweep <j>` is disarmed only |
| `limits check` / `limits write <j\|all>` | Compare / sync the servos' angle-limit registers with the travel. Writes only differing values; power-cycle HLS servos afterwards. Disarmed only |
| `save` / `load` / `defaults` / `dump` | Write NVS / reload / reset to defaults in RAM / print as commands |
| `plot <j> on` / `plot off` | Teleplot at 50 Hz: `exo_deg exo_filt_deg target goal sent ref grip_lead present load speed_cmd`, plus `current_ma` on HLS joints |

## Behaviour

**Per-joint states.** `OFF → ALIGN → FOLLOW ⇄ HOLD`, plus `SWEEP`. Each joint moves through these on its own:

| State | Behaviour |
|---|---|
| ALIGN | Moves to the mapped target at 50 SCS steps/s (167 HLS), then reports `aligned -> FOLLOW` |
| FOLLOW | Follows the glove |
| HOLD | Entered on encoder loss (3 cycles); freezes the joint. If the encoder is healthy again for 200 ms, the joint re-aligns; after 3 s it is switched off |

**Fault scope.**

| Scope | Faults | Action |
|---|---|---|
| Joint | Load trip (only when `loadtrip on`), overheat, over-current (HLS), tracking lag, servo error (overload, over-current, angle sensor), encoder lost > 3 s | Only that joint goes off. `joints` shows why; `arm <j>` brings it back |
| Hand | Supply voltage, servo voltage error, torque mismatch, comm lost (3 failed reads or no good read for 100 ms), E-stop | Every servo goes off |

**Servo reads.**
- Full status reads are round-robin, 3 servos per cycle, so each servo is read about every 30 ms with 9 enabled.
- A goal is written only if that joint's last good read is at most 50 ms old and showed torque on: a goal write to a disarmed SCS0009 would switch its torque on by itself.
- All goals go out in one SYNC_WRITE.
- Arming 9 joints at once takes about 22 ms, which counts one overrun in `stats`.

**Arming sequence, per joint.** Measured SCS0009 behaviour from servo_bringup, plus DEX_hand_software's HLS3606 checks:
1. The servo was identified as the model in `config.h`.
2. Read the servo state: no error, position inside travel.
3. HLS3606 only: angle limits not 0/0 and covering the travel, resolution (register 30) = 1, position mode (register 33) = 0, protection current (register 28) 1–2047, position not negative. Then set the goal current.
4. Write goal = present position at the arm speed.
5. Write torque on.
6. Read back: torque on, goal = hold position, movement ≤ 5 SCS steps (17 HLS).

The goal is written before torque on, which is safe whether or not an HLS3606 goal write switches torque on by itself (DEX_hand_software's measurements disagree).

**Other rules:**
- Speeds are steps/s at every interface; the bus converts them to each model's speed register. An SCS0009 register is never 0 (full speed); an HLS3606 register is at least 1 (0 stops it) and at most 250.
- The SCS0009 goal time is always 0.
- Torque-off is broadcast to all 9 servos at boot and on E-stop: the servos have no communication watchdog.
- `save`, `limits` and `sweep` are refused while any joint is armed.

**NVS.** Format version 4 is 258 bytes and holds, per joint, the servo model and `ilimit` besides the M2 fields.
- Versions 1–3 were saved for the hand before the rebuild (other ids and servos) and are discarded.
- Missing, corrupt or unknown-version data is rejected, and defaults are used.
- A joint whose stored model differs from `config.h` is reset to its defaults; the other joints keep their settings.

## Rebuilt-hand bench test

1. **Models.** `ping` matches `config.h`. Change one id's model in `config.h` on purpose: boot reports the mismatch and `arm <j>` is refused.
2. **HLS limits.** `arm <j>` on an HLS joint with 0/0 limits is refused with the reason. After `limits write <j>` and a power-cycle, `limits check` shows `ok`.
3. **Follow.** After bring-up, arm each joint alone, then the whole hand. `stats`: jitter max < 0.5 ms; `|pos − sent|` normal.
4. **Current.** `set <j> ilimit 500` on an HLS joint and squeeze a sponge: `status` shows the current settling near 500 mA. Block the finger with a higher `ilimit`: `over-current` switches off that joint only.
5. **SCS regression.** Joints 1, 3, 5 follow, grasp and stop on E-stop as before.

## M2 bench test (previous hand)

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
include/config.h          pins, bus, joint table (model, travel, vmax/vsat/amax), global defaults, timing
lib/AS5048B/              encoder driver (vendored, see below)
lib/FeetechProtocol/      pure C++ packet codec and SCS0009 / HLS3606 model codecs (host-tested)
lib/FeetechBus/           UART transport, ServoBus interface, FeetechMixedBus (per-id models)
lib/TeleopCore/           pure C++, host-tested:
  joint_map / joint_config  mapping, per-joint settings, validation
  config_blob               NVS serialization (CRC32, version 4, per-joint model)
  exo_tracker               continuous multi-turn exo position, auto-resolve / homing
  command_parser            console grammar, settings edits, dump format
  supervisor                per-joint checks (incl. HLS over-current), fault scope
  grip_limiter              load-adaptive goal lead cap
  speed_sweep               saturation speed measurement
  jump_guard / one_euro / rate_limiter
src/control_task.*        100 Hz loop on core 1: per-joint states, reads, arming, sweep, settings
src/config_store.*        Preferences (NVS) read/write
src/console.*             USB console on core 0
src/main.cpp              setup and task start
test/                     Unity tests (native): packet, models, core, config, sweep, tracker
tools/measure_mapping.py  mapping consistency measurement (host Python)
```

## Vendored code

`lib/AS5048B` is copied unmodified from `~/Documents/PlatformIO/Projects/Dexhand_Encoded`, commit `cd89c14` ("feat: otp_address_jig and multi_read examples"), on 2026-10-14. Make fixes there first, then re-copy.
