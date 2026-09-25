# Notes for AI agents

To talk to the robot, use the bridge command language in [`docs/COMMANDS.md`](docs/COMMANDS.md):

- MCP: server `dreame-bridge` (`python tools/bridge_tool.py mcp`, declared in `.mcp.json`).
- CLI: `python tools/bridge_tool.py <command>` → one JSON object, exit 0 when `"ok": true`.

Start with `status`, then `uart.xfer <wait_ms> "<cmd>\r\n"` (robot MCU) or
`soc.xfer <wait_ms> "<cmd>\n"` (robot SoC Linux shell) for request/response.
Do not send `swd HALT`/`STEP`/`RUNUNTIL` or `wifi.*` changes unless the user asked.

Firmware source: `firmware/esp_uart_bridge/` (ESP-IDF 6.x, targets esp32s2 and esp32s3).
The command table lives in `main/cmd.c`; the web app in `main/web/index.html`.
