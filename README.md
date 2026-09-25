# m40 – cầu nối Wi-Fi ↔ UART/SWD cho robot (ESP32-S2 mini)

| Thư mục | Nội dung |
|---|---|
| `firmware/esp_uart_bridge/` | Firmware ESP-IDF (PlatformIO). Xem [README firmware](firmware/esp_uart_bridge/README.md). |
| `tools/bridge_tool.py` | CLI + GUI trên PC (chỉ cần Python 3.8+, GUI dùng tkinter có sẵn trong bản Python Windows). |

## Công cụ PC

Không truyền `--host` thì tool thử `dreame-bridge.local` (mDNS) rồi đến UDP discovery `2326`.

```powershell
python tools\bridge_tool.py gui                        # giao diện: terminal, macro, SWD, trạng thái
python tools\bridge_tool.py discover                   # liệt kê bridge trong LAN
python tools\bridge_tool.py status                     # JSON discovery + bộ đếm lỗi UART
python tools\bridge_tool.py term --eol crlf --log uart.log   # terminal raw; ":hex 3C 00 3E" gửi hex, ":q" thoát
python tools\bridge_tool.py send "info -a" --wait 1    # gửi một lệnh, in phản hồi
python tools\bridge_tool.py send "3C 00 01 01 0E 00 01 06 00 0E 08 3E" --hex --hex-out
python tools\bridge_tool.py swd PING ID "READ 0x08000000 256"
python tools\bridge_tool.py swd "DUMP 0x08000000 0x80000" --out mcu.bin
```

GUI:

- **Terminal**: xem dạng text hoặc hex, chọn kết thúc dòng (none/LF/CR/CRLF), gửi hex, lịch sử
  lệnh bằng ↑/↓, timestamp, lưu log.
- **Macro**: mỗi dòng `tên = lệnh` thành một nút; hỗ trợ `\r \n \xHH`.
- **SWD**: nút PING/ID/DPID/HALT/RESUME/STEP và ô lệnh tuỳ ý (READ hiện hexdump). Lệnh ghi
  (`WRITE`/`MWRITE`) cố ý không có trong tool.
- **Trạng thái**: hỏi UDP discovery mỗi 2 s, hiện tốc độ ↓/↑ và các bộ đếm
  `frame_err`/`parity_err`/`fifo_ovf`/`buf_full`.

## Không commit

`sdkconfig` và `sdkconfig.lolin_s2_mini` chứa SSID/mật khẩu Wi-Fi nên đã nằm trong
`.gitignore`. Đổi mặc định thì sửa `sdkconfig.defaults`.
