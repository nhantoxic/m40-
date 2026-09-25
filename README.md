# m40 – cầu nối Wi-Fi ↔ UART/SWD cho robot (ESP32-S2 / ESP32-S3)

| Thư mục | Nội dung |
|---|---|
| `firmware/esp_uart_bridge/` | Firmware ESP-IDF, có web app nhúng. Build cho **ESP32-S2** (LOLIN S2 mini) hoặc **ESP32-S3**. Xem [README firmware](firmware/esp_uart_bridge/README.md). |
| `tools/bridge_tool.py` | CLI + MCP server trên PC. Chỉ cần Python 3.8+ (thêm `pyserial` nếu dùng `--usb`). |
| `docs/COMMANDS.md` | Bộ lệnh chung cho web app, CLI, AI agent, HTTP và USB. |

## Bắt đầu nhanh

1. Nạp firmware: `pio run -e lolin_s2_mini -t upload` (hoặc `-e esp32s3`).
2. Kết nối điện thoại vào Wi-Fi **DreameBridge-XXXX** (mật khẩu `dreame-setup`), mở
   **http://192.168.4.1/**, chọn Wi-Fi nhà, nhập mật khẩu → *Lưu & kết nối*.
3. Từ đó dùng **http://dreame-bridge.local/** trên điện thoại/PC, hoặc CLI bên dưới.

## CLI trên PC (cùng lệnh với app)

Không truyền `--host` thì tool lần lượt thử `$BRIDGE_HOST`, bridge dùng lần trước,
`dreame-bridge.local` (mDNS), UDP discovery `2326`, rồi AP cài đặt `192.168.4.1`.

```powershell
python tools\bridge_tool.py status --pretty
python tools\bridge_tool.py wifi.scan
python tools\bridge_tool.py wifi.set "Tên Wi-Fi" "mật khẩu"
python tools\bridge_tool.py --usb COM8 wifi.set "Tên Wi-Fi" "mật khẩu"   # qua cáp USB
python tools\bridge_tool.py uart.baud 115200
python tools\bridge_tool.py uart.xfer 800 "info -a\r\n"                 # gửi và lấy phản hồi (JSON)
python tools\bridge_tool.py --text uart.xfer 800 "info -a\r\n"          # chỉ in text phản hồi
python tools\bridge_tool.py uart.read 500
python tools\bridge_tool.py uart.sendhex "3C 00 01 3E"
python tools\bridge_tool.py swd ID
python tools\bridge_tool.py swd READ 0x08000000 64
python tools\bridge_tool.py help                                         # danh sách lệnh từ firmware
python tools\bridge_tool.py discover                                     # các bridge trong LAN
python tools\bridge_tool.py term                                         # terminal raw tcp/2324
python tools\bridge_tool.py app                                          # mở web app
```

Mỗi lệnh in ra một dòng JSON (`--pretty` để xuống dòng thụt lề). Mã thoát là 0 khi `"ok": true`.

## AI agent

- **MCP**: `python tools/bridge_tool.py mcp` là một MCP server (stdio) với các tool
  `bridge_command`, `uart_xfer`, `uart_read`, `bridge_status`, `bridge_discover`.
  File `.mcp.json` ở gốc repo đã khai báo sẵn cho Claude Code.
- **Không dùng MCP**: agent có thể gọi thẳng CLI ở trên, hoặc
  `curl -H "X-Bridge: 1" --data 'uart.xfer 800 "info -a\r\n"' http://dreame-bridge.local/api/cmd`.
- Mô tả lệnh và mẹo dùng cho agent: [`docs/COMMANDS.md`](docs/COMMANDS.md).

## Không commit

`sdkconfig` và `sdkconfig.<env>` có thể chứa mật khẩu Wi-Fi nên đã nằm trong `.gitignore`.
Đổi mặc định thì sửa `sdkconfig.defaults*`.
