# teleop-host: PC GUI for the teleoperation firmware

A local Python service that talks to the ESP32-S3 over the host link ([`../docs/host-protocol.md`](../docs/host-protocol.md)) and serves a browser UI. Everything the serial console does is available in the GUI, and the text console keeps working whenever the GUI is not connected.

## Run

```bash
cd host
uv sync
uv run teleop-host                                # connects to the only ESP32 found and opens http://127.0.0.1:8790
uv run teleop-host --port /dev/cu.usbmodem11101   # pick the port explicitly
uv run teleop-host --sim                          # simulated hand, no hardware
```

- **Before you start**: close `pio device monitor`, because only one program can hold the port.
- **Port choice**: with several boards connected, or none, pick the port in the top bar.
- **On connect**: the GUI switches the board to the host link. Disconnecting, or quitting `teleop-host`, returns it to the text console.
- **If the link drops while joints are armed** (USB unplugged, the GUI crashed): the board switches every servo off within about 1 s.
- **Closing the browser tab does not** do this. The board keeps following as long as `teleop-host` runs.

## Pages

| Page | Replaces |
|---|---|
| Dashboard | `status`, `arm [j]`, `disarm [j]`, `home j`. **E-STOP** is always in the top bar (also the `Esc` key) |
| Plots | `plot`/Teleplot. Every control cycle on the device clock, with CSV export |
| Calibration | `monitor`, `set j enc`, `cal j hand a/b`, `cal j a/b`, `home j`, `set j on/off`, `set j exospan/servo`, `save` |
| Servo setup | `ping`, `limits check`, `limits write`, `sweep j`, `sweep j apply`, `ilimit`/`itrip` with live current |
| Parameters | `joints`, every `set` key (per joint and global), `save` |
| Config store | `save`, `load`, `defaults`, `dump`, and backups / diff / restore. Replaces `tools/calib.py`; uses the same `calib/*.txt` files |
| Encoders | `enc`, `monitor` |
| Diagnostics | `stats`, `stats reset`, telemetry rate, link counters |
| Console | Any text command, with history, as on the serial port |
| Plugins | Reserved pages: Recording (M3), Filter tuning, Parameter tuning (M4), Camera |

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
2. **Connect.** The top bar shows `connected` with the firmware build (`git describe`). Every page fills in.
3. **Telemetry.** Run for 10 min:
   - Diagnostics shows `dropped telemetry` 0;
   - jitter and overruns are the same with telemetry at 100 Hz as at 10 Hz;
   - Plots show continuous traces.
4. **E-stop.** E-STOP and `Esc` switch everything off at once; BOOT still works.
5. **Link loss.** With a joint armed, unplug USB or kill `teleop-host`: torque goes off within 1 s, and the text console prints `[host] link lost, torque off`.
6. **Reconnect.** Restart `teleop-host`: it takes the board over again, even within the 1 s heartbeat window.
7. **Calibration.** Calibrate one joint end to end in the Calibration page, arm it on the Dashboard, and watch it in Plots.
8. **Backups.** Back up, change a setting, Diff (one change), Restore, Diff (no changes).
9. **Refusals.** While armed, Save / Load / Ping are refused with a toast. Live parameters (vmax, ilimit, …) still apply.
