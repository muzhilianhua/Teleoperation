# teleop-host

PC GUI for the teleoperation firmware: a local Python service (FastAPI) that
talks to the ESP32-S3 over the host link (`../docs/host-protocol.md`) and
serves a browser UI.

```bash
cd host
uv sync
uv run teleop-host --help
uv run pytest
```
