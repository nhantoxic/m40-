# Bridge command language

One text line in, one line of JSON out. The **same commands** work everywhere:

| Client | How to send `<line>` |
|---|---|
| Web app | tab *Lệnh / API* (the other tabs use the same commands underneath) |
| PC CLI | `python tools/bridge_tool.py <line>` (arguments are quoted for you) |
| AI agent | MCP server `python tools/bridge_tool.py mcp` (tool `bridge_command`), or the CLI |
| HTTP | `POST http://<bridge>/api/cmd`, header `X-Bridge: 1`, body = `<line>` |
| USB | write `@CMD <line>\n` to the USB serial port, read the `@CMD <json>` line |

Every reply has `"ok": true|false`; a failed reply also has `"error"`. The CLI exits
with 0 when `ok` is true and 1 otherwise.

## Commands

| Command | Reply fields | Notes |
|---|---|---|
| `help` | `commands[]{cmd,usage,help}` | lists the commands below |
| `status` | `fw, uptime_s, free_heap, min_free_heap, wifi{…}, uart{baud,mode,tcp_port,tcp_client,rx_bytes,tx_bytes,frame_err,parity_err,break,fifo_ovf,buf_full,buffered}, soc{…same…}, swd{…}` | cheap; good first call. `uart` = MCU, `soc` = SoC shell (absent if disabled) |
| `wifi.scan` | `networks[]{ssid,rssi,channel,auth,supported}` | ~2–3 s |
| `wifi.set <ssid> <password>` | `ssid, ip` | saved **only if** an IP is obtained (≤ 20 s), else the old network stays. Quote values with spaces |
| `wifi.forget` | | erases the network and starts the setup AP |
| `wifi.ap on\|off` | `setup_ap` | setup AP `DreameBridge-XXXX` on for 10 min / off (off only while connected) |
| `uart.baud [rate]` | `baud` | shows or sets and saves the robot UART baud (default 115200) |
| `uart.send <data>` | `sent` | no line ending is added |
| `uart.sendhex <hex>` | `sent` | `3C 00 0x01,3E` style, max 1024 bytes |
| `uart.read [wait_ms]` | `len, text, hex, overflow` | bytes received since the last `uart.read`/`uart.xfer`; waits up to `wait_ms` (≤ 30000) for data, returns after 60 ms of silence |
| `uart.xfer <wait_ms> <data>` | `sent, len, text, hex, overflow` | clears the buffer, sends, collects the reply; returns after 100 ms of silence or `wait_ms` |
| `soc.baud` / `soc.send` / `soc.sendhex` / `soc.read` / `soc.xfer` | same as `uart.*` | the same commands for the **robot SoC Linux shell** UART (tcp/2323); `uart.*` is the **robot MCU** |
| `swd <cmd>` | `response`, `data` (hex, for `READ`) | `PING ID DPID PID CTRL RAW HALT RESUME STEP REGREAD n REGWRITE n v RUNUNTIL … READ addr len`. `WRITE`/`MWRITE`/`DUMP` need the raw port tcp/2325 |
| `crash` | `boot_mode, reset_reason, early_crashes, crash` | why the last run ended; `crash` = stored panic summary `task=… pc=… bt=…` (decode with the firmware ELF). `boot_mode` is `normal`, `safe` (after 2 early crashes: no UART/SWD/mDNS) or `usb_only` (after 4) |
| `crash.clear` | | erase the stored crash and the crash-loop counter |
| `crash.test` | | deliberately crash, to test the crash report / safe mode |
| `reboot` | | restarts after the reply |

### Data escapes (`uart.send`, `uart.xfer`)

`\r \n \t \0 \\ \" \xHH`. The data is the rest of the line; one pair of surrounding
double quotes is removed. Examples:

```text
uart.xfer 800 "info -a\r\n"
uart.send ver -t\r\n
uart.send "\x3C\x00\x01\x3E"
```

### Received data

`text` is the reply as a JSON string: valid UTF-8 is kept, and control characters or
stray bytes appear as `\u00XX` (the byte value). `hex` is the exact bytes. The buffer
keeps the newest 4 KB (ESP32-S2) / 16 KB (ESP32-S3); `overflow: true` means older
bytes were dropped. The buffer is shared: two clients calling `uart.read` split the
data between them.

## Tips for AI agents

- Start with `status`. `uart.baud` tells you the rate; `frame_err`/`parity_err`
  going up usually means the baud rate is wrong.
- Two UARTs: `uart.*` = robot MCU CLI (usually `\r\n`), `soc.*` = robot SoC Linux
  shell (usually `\n`, e.g. `soc.xfer 1000 "uname -a\n"`; `\x03` is Ctrl+C).
- Prefer `uart.xfer <wait_ms> "<cmd>\r\n"` for request/response. Use `uart.read`
  to collect spontaneous output (logs).
- The robot console usually needs `\r\n`; nothing is appended for you.
- `wifi.set` changes the network the bridge is on. If you reach the bridge over
  that network you may lose it; the reply still tells you the new IP.
- `swd` commands can halt the robot MCU (`HALT`, `STEP`, `RUNUNTIL`); send `RESUME`
  afterwards.
