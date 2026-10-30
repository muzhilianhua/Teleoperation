# teleop-host: PC GUI for the teleoperation firmware

A local Python service that talks to the ESP32-S3 over the host link ([`../docs/host-protocol.md`](../docs/host-protocol.md)) and serves a browser UI. Everything the serial console does is available in the GUI, and the text console keeps working whenever the GUI is not connected.

## Run

```bash
cd host
uv sync
uv run teleop-host                                # connects every ESP32 found (one per hand), opens http://127.0.0.1:8790
uv run teleop-host --port /dev/cu.usbmodem11101 --port /dev/cu.usbmodem11201
uv run teleop-host --sim                          # two simulated hands, no hardware
uv run teleop-host --recordings ~/teach           # where Teach keeps recordings (default ../recordings, git-ignored)
```

- **Before you start**: close `pio device monitor`, because only one program can hold the port.
- **Two hands**: each board reports its hand (GPIO4 strap) when it connects, and the GUI binds it to that hand. Swapping USB ports or cables never swaps hands. A second board claiming an occupied hand is disconnected with a message to check its strap. Boards found later are added from the top bar (**Connect board**).
- **Which hand a page acts on**: the Right / Left switch in the top bar. The Dashboard also has a *Both hands* view. **E-STOP** (and `Esc`) and **Disarm all** act on both hands; everything else on the selected one.
- **On connect**: the GUI switches the board to the host link. Disconnecting, or quitting `teleop-host`, returns it to the text console.
- **If the link drops while joints are armed** (USB unplugged, the GUI crashed): that board switches every servo off within about 1 s. The other hand keeps following.
- **Closing the browser tab does not** do this. The board keeps following as long as `teleop-host` runs.

## Pages

| Page | Replaces |
|---|---|
| Dashboard | `status`, `arm [j]`, `disarm [j]`, `home j`. **E-STOP** is always in the top bar (also the `Esc` key) |
| Plots | `plot`/Teleplot. Every control cycle on the device clock, with CSV export |
| Teach | No console equivalent. Records teleoperation of one or both hands, with markers; lists, previews, renames and deletes recordings; replays one on one or both hands at 0.25 to 1× (see *Teach API*) |
| Calibration | `monitor`, `set j enc`, `cal j hand a/b`, `cal j a/b`, `home j`, `set j on/off`, `set j exospan/servo`, `save` |
| Servo setup | `ping`, `set j travel` (this robot's travel, captured from the live position), `limits check`, `limits write`, `sweep j`, `sweep j apply`, `ilimit`/`itrip` with live current |
| Parameters | `joints`, every `set` key (per joint and global), `save` |
| Config store | `save`, `load`, `defaults`, `dump`, and backups / diff / restore. Replaces `tools/calib.py`; uses the same `calib/*.txt` files |
| Encoders | `enc`, `monitor` |
| Diagnostics | `stats`, `stats reset`, telemetry rate, link counters |
| Power | Both hands' servo voltage and estimated supply current against the bench supply's limit, and the voltage / current of the second before every fault that switched joints off |
| Console | Any text command, with history, as on the serial port |
| Plugins | Reserved pages: Dataset export (M3), Filter tuning, Parameter tuning (M4), Camera |

The firmware still decides everything:

- A command refused while joints are armed is refused the same way it would be in the text console. The GUI only greys such controls out.
- Every refusal or failure shows as a red toast and is listed under Diagnostics.

## Layout

```
src/teleop_host/
  protocol/   COBS, CRC16, frames, message codecs (tested against ../test/test_hostlink/golden_vectors.h)
  link/       serial transport, port discovery, LinkManager (handshake, heartbeat, E-stop priority, reconnect)
  device.py   asyncio device model: latest state, command() -> ACK / events / DONE, dump / diff / restore
  config_files.py   calib/ backups
  power.py    supply voltage / estimated current per hand, trip records (Power page)
  teach/      recorder, recordings store, host/device clock alignment, replay player
  server.py   FastAPI: REST + one WebSocket per tab
  sim/        byte-level fake ESP32 with a simulated hand (tests, --sim)
  plugins/    extension point (see below)
  static/     built web UI (committed)
web/          Svelte 5 + TypeScript + Vite source of the UI
```

## Develop

```bash
uv run pytest && uv run ruff check                   # host tests (fake device, no hardware)
cd web && bun install
bun run check && bun run test                        # svelte-check, vitest
bun run dev                                          # Vite on :5173, proxies /api and /ws to :8790
uv run teleop-host --sim --dev                       # in another shell: API on :8790, opens :5173
bun run build                                        # writes ../src/teleop_host/static; commit it
```

A protocol change touches four places, and both test suites must pass:

1. `lib/HostLink` (firmware);
2. `golden_vectors.h`;
3. `protocol/messages.py`;
4. `web/src/lib/protocol.ts`.

## Teach API

The Teach page uses these routes. Another program (for example, a robot arm controller) can use them too, to start recording or replay at an agreed time.

| Route | Does |
|---|---|
| `GET /api/time` | `{time}`: the host clock (epoch seconds) that every `at` refers to |
| `GET /api/teach/status` | `{record, replay}`: recorder and player state. The WebSocket pushes the same as `teach` at 5 Hz while either is active, and `teach_event` on start, save, marker, ready, done, stop and error |
| `POST /api/teach/record/start` | `{hands: ["right", "left"], name, at?}`: record from now, or from `at` |
| `POST /api/teach/record/marker` | `{label}`: a marker at the current recording time |
| `POST /api/teach/record/stop` / `cancel` | save (returns the recording's metadata) / discard |
| `GET /api/teach/recordings` | every recording's metadata, newest first |
| `GET /api/teach/recordings/{id}/preview`, `/{hand}.csv` | decimated goal / position / exo traces; the full CSV |
| `PATCH` / `DELETE /api/teach/recordings/{id}` | rename (`{name}`) / delete |
| `POST /api/teach/replay/prepare` | `{id, hands?, speed, wait?}`: check, load, arm, approach the first frame. With `wait`, answers once ready or failed |
| `POST /api/teach/replay/go` | `{at?, wait?}`: play every prepared hand together, now or at `at`. With `wait`, answers once it has ended |
| `POST /api/teach/replay/stop` | stop; the joints hold where they are |

```bash
curl -s localhost:8790/api/teach/replay/prepare -H 'Content-Type: application/json' \
     -d '{"id": "20261028-150341_pick-cup", "speed": 0.5, "wait": true}'
curl -s localhost:8790/api/teach/replay/go -H 'Content-Type: application/json' \
     -d "{\"at\": $(python3 -c 'import time; print(time.time() + 2)'), \"wait\": true}"
```

Errors: `404` for no such recording, `400` when it does not fit (another hand or joint table, nothing to replay, too short, bad speed), and `409` when busy or a hand is not connected.

Each recording is `meta.json` plus one CSV per hand. The CSV has one row per received control cycle:

- `t`: seconds from the start, on the host clock;
- `cycle` and `t_us`: from the board;
- per servo id `k`: `jk_goal`, `jk_sent`, `jk_pos`, `jk_exo_deg`, `jk_load`, `jk_current`, `jk_state`.

`t` comes from fitting `host = a · device + b` to the lower envelope of the arrival delays, separately for each hand. That puts both hands on one time axis without the USB jitter.

## Plugins

A feature such as recording or tuning lives in two halves:

- **Backend**: a `teleop_host.plugins.Plugin` subclass.
  - `on_message()` sees every frame at full rate (100 Hz telemetry with device timestamps).
  - `router()` adds REST routes under `/api/plugins/<id>/`.
  - `start(device)` gives it `device.command()`.
- **Frontend**: a page registered in `web/src/plugins/registry.ts` (`pluginPages`) under the same id.

Register the plugin in `plugins/__init__.py`. The placeholders in `plugins/placeholders.py` describe what is planned.

## Bench checklist

1. **Text console unchanged.** Without the GUI, `help`, `status`, `joints` and `dump` print as before; `help` adds one line, `host on`.
2. **Connect.**
   - The top bar shows both hands connected, with the firmware build (`git describe`), and every page fills in.
   - Swap the two USB cables and restart `teleop-host`: the hands stay the same.
   - Strap both boards as right: the second one is rejected with a notice.
3. **Telemetry.** Run for 10 min:
   - Diagnostics shows `dropped telemetry` 0;
   - jitter and overruns are the same with telemetry at 100 Hz as at 10 Hz;
   - Plots show continuous traces.
4. **E-stop.** E-STOP and `Esc` switch both hands off at once; BOOT stops its own hand.
5. **Link loss.** With both hands armed, unplug one board's USB: that hand's torque goes off within 1 s, and its text console prints `[host] link lost, torque off`. The other hand keeps following.
6. **Reconnect.** Restart `teleop-host`: it takes the board over again, even within the 1 s heartbeat window.
7. **Calibration.** Calibrate one joint end to end in the Calibration page, arm it on the Dashboard, and watch it in Plots.
8. **Backups.**
   - Back up, change a setting, Diff (one change), Restore, Diff (no changes).
   - Import a left-hand dump into the right hand's store and try to restore it: refused.
   - Backups are stored in `calib/right/` and `calib/left/`.
9. **Refusals.** While armed, Save / Load / Ping are refused with a toast. Live parameters (vmax, ilimit, …) still apply.
10. **Teach, one hand.**
    - Record about 10 s of a grasp under teleoperation, then disarm.
    - Replay it at 1×: Prepare, then Play.
    - In Plots, compare `goal` and `present` with the recording.
11. **Teach, both hands.** Record and replay both hands together. The two hands start within one control cycle of each other.
12. **Link loss during a replay.** Unplug one board's USB: that hand's torque goes off within 1 s, and the other hand's replay continues.
13. **End of a replay.** After the last frame the grasp holds until Disarm all.
