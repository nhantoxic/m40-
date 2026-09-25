# esp_uart_bridge

Cầu **Wi-Fi ↔ UART/SWD** cho robot (cổng debug MCU), chạy trên **ESP32-S2** (LOLIN S2 mini)
hoặc **ESP32-S3** (DevKitC-1 / module S3). Có sẵn:

- **Web app** nhúng trong firmware (`http://dreame-bridge.local/`): cài Wi-Fi, terminal UART,
  macro, chạy lệnh, SWD. Mở được trên điện thoại hoặc PC, không cần cài gì.
- **Một bộ lệnh chung** cho web app, CLI trên PC, AI agent (MCP) và USB — xem
  [`docs/COMMANDS.md`](../../docs/COMMANDS.md).
- Cổng TCP raw cũ (`2324` UART, `2325` SWD text, `3335` OpenOCD remote-bitbang) và UDP
  discovery `2326`, vẫn tương thích với RobotMonitor.

## Chọn chip

| | ESP32-S2 (LOLIN S2 mini) | ESP32-S3 (DevKitC-1) |
|---|---|---|
| Env PlatformIO | `lolin_s2_mini` | `esp32s3` |
| CPU | 1 nhân, 240 MHz | 2 nhân, 240 MHz: Wi-Fi/lwIP ở core 0, bơm UART + SWD bit-bang ở core 1 |
| Heap lúc chạy | ~150 KB | ~240 KB |
| Bộ đệm | UART ring 8 KB, `uart.read` 4 KB, TCP window 5,7 KB | UART ring 16 KB, `uart.read` 16 KB, TCP window 11,5 KB, `swd READ` tối đa 16 KB |
| USB | ROM USB-CDC (DTR=0, RTS=1 mới có dữ liệu) | USB-Serial-JTAG |
| UART MCU (tcp 2324) | UART1: TX **GPIO35**, RX **GPIO33** | UART1: TX **GPIO17**, RX **GPIO21** |
| UART SoC shell (tcp 2323) | UART0: TX **GPIO37**, RX **GPIO39** | UART2: TX **GPIO38**, RX **GPIO39** |
| SWD | SWDIO **GPIO16**, SWCLK **GPIO18** | SWDIO **GPIO16**, SWCLK **GPIO18** |
| LED trạng thái | GPIO15 | không (LED RGB địa chỉ) |
| Nút cài đặt | BOOT (GPIO0) | BOOT (GPIO0) |

Chân nằm trong `sdkconfig.defaults.esp32s2` / `sdkconfig.defaults.esp32s3`. Trên module S3
có PSRAM octal (N8R8/N16R8) thì **không** dùng GPIO35–37. Mắc điện trở 1 kΩ nối tiếp trên
TX/RX và SWD sau khi đã xác nhận chân phía robot.

Cả hai UART mặc định **115200 8N1**. Đổi lúc chạy bằng `uart.baud <rate>` (MCU) hoặc
`soc.baud <rate>` (SoC); giá trị mới được lưu lại. Tắt cổng SoC bằng menuconfig
`BRIDGE_SOC_UART_ENABLE`.

## Build và nạp

```powershell
cd firmware\esp_uart_bridge
pio run -e lolin_s2_mini -t upload --upload-port COM5     # S2: giữ BOOT, nhấn RESET nếu không thấy cổng
pio run -e esp32s3 -t upload                              # S3
```

Hoặc dùng ESP-IDF trực tiếp (cần **v6.x**; đã build thử với v6.1):

```bash
idf.py set-target esp32s2      # hoặc esp32s3
idf.py build flash monitor
```

Lần build đầu sẽ tải component `espressif/mdns` (cần Internet). ESP-IDF đọc
`sdkconfig.defaults` rồi `sdkconfig.defaults.<chip>`. **Các file này chỉ có tác dụng khi
`sdkconfig.<env>` được tạo mới.** Nếu bạn đang có sẵn `sdkconfig.lolin_s2_mini` từ firmware cũ,
hãy xoá nó (Wi-Fi giờ được cài qua app nên file này không còn cần giữ mật khẩu) hoặc sửa bằng
`pio run -e lolin_s2_mini -t menuconfig`.

## Lần đầu: cài Wi-Fi bằng app

1. Cấp nguồn. Khi chưa có Wi-Fi (hoặc mất Wi-Fi quá 30 s, hoặc giữ nút **BOOT 3 s**), bridge
   phát AP **`DreameBridge-XXXX`**, mật khẩu **`dreame-setup`** (đổi trong menuconfig:
   *Setup access point password*). LED nháy đôi.
2. Điện thoại/PC kết nối vào AP đó, mở **http://192.168.4.1/** → tab **Wi-Fi** → *Quét mạng*
   → chọn mạng nhà → nhập mật khẩu → **Lưu & kết nối**.
3. Bridge thử kết nối. Chỉ khi **nhận được IP** thì mạng mới được lưu. Sai mật khẩu hay không
   thấy mạng thì app báo lý do, mạng cũ vẫn giữ nguyên. Nhận IP xong, điện thoại có thể bị
   ngắt khỏi AP cài đặt (AP chuyển sang kênh của router): cứ quay về Wi-Fi nhà rồi mở
   **http://dreame-bridge.local/** (hoặc IP được báo).
4. AP cài đặt tự tắt 30 s sau khi đã vào mạng nhà và không còn ai kết nối vào nó.

Cách khác: cắm USB và chạy `python tools/bridge_tool.py --usb COM8 wifi.set "Tên Wi-Fi" "mật khẩu"`.

Chỉ nhận mạng WPA2/WPA3 (mật khẩu 8–63 ký tự); mạng mở bị từ chối.

## Dùng

- **Web app**: `http://dreame-bridge.local/` (hoặc IP). Các tab: Wi-Fi, Terminal (WebSocket,
  xem text/hex, CR/LF/CRLF, gửi hex, lịch sử ↑/↓, macro, đổi baud, lưu log), Lệnh / API, SWD.
- **PC / AI agent**: `tools/bridge_tool.py` — cùng lệnh với app. Xem
  [README gốc](../../README.md) và [`docs/COMMANDS.md`](../../docs/COMMANDS.md).
- **Raw TCP** (RobotMonitor, YMODEM, PuTTY chế độ *Raw*, **không phải Telnet**): `2324` = MCU,
  `2323` = SoC shell. Client mới sẽ chiếm quyền client cũ trên cùng cổng.
- **Terminal trong app**: ô *Cổng* chuyển giữa MCU và SoC shell (WebSocket `/ws` và `/ws/soc`).
  Macro và kiểu kết thúc dòng được nhớ riêng cho từng cổng; shell Linux thường dùng `LF`.
- **SWD text** `2325`: nhiều lệnh trên một kết nối, đóng khi rỗi 60 s.
- **OpenOCD** `3335` (remote_bitbang, không có NRST → `reset_config none`):

  ```text
  adapter driver remote_bitbang
  remote_bitbang host dreame-bridge.local
  remote_bitbang port 3335
  transport select swd
  reset_config none
  ```

  Khi OpenOCD đang kết nối, lệnh `swd ...` từ app/CLI báo *busy*.

### LED (S2)

| Trạng thái | Ý nghĩa |
|---|---|
| Nháy nhanh (100 ms) | đang vào Wi-Fi / vừa mất kết nối |
| Nháy đôi | AP cài đặt đang bật |
| Nháy chậm (1 s) | đã vào mạng, chưa có client TCP |
| Sáng liên tục | có client trên cổng UART 2324 |

### Log qua USB

- **S2** (ROM USB-CDC): chỉ phát dữ liệu khi **DTR = 0, RTS = 1**. RTS đi từ 1 xuống 0 (kể cả
  lúc đóng cổng) sẽ **reboot** board; cài đặt thì vẫn giữ nguyên.
- **S3** (USB-Serial-JTAG): để DTR = RTS = 0; RTS = 1 sẽ giữ chip ở trạng thái reset.

`bridge_tool.py --usb` tự nhận ra loại chip qua USB VID:PID.

### LAN discovery (UDP 2326)

Gửi `DREAME_BRIDGE_DISCOVER` → JSON gồm `name`, `ip`, `mac`, `uart_port`/`mcu_port` (MCU),
`soc_port`/`soc_baud` (SoC shell, `0` nếu tắt), `uart_baud`,
`uart_mode`, `swd_port`, `bitbang_port`, `http_port`, `setup_ap`, `uart_tx_level`,
`uart_rx_level` và `uart_stats`. Cách đọc bộ đếm (cũng có trong `status`):

| Quan sát | Kết luận |
|---|---|
| `rx_bytes` tăng | ESP đã giải mã được byte ở cặp chân/baud hiện tại (chưa chứng minh byte đến từ MCU). |
| `frame_err`/`parity_err` tăng, `rx_bytes` gần 0 | Có tín hiệu nhưng sai baud/parity, nhiễu, hoặc sai nguồn. |
| mọi bộ đếm = 0 | Không có byte hợp lệ: dây, đối tác im/ngủ, hoặc nghe nhầm UART. |
| `tx_bytes` tăng khi gửi | Driver UART của ESP đã nhận byte (chưa chứng minh MCU đã nhận). |
| `fifo_ovf`/`buf_full` tăng | Robot gửi nhanh hơn tốc độ bridge chuyển đi. |

## Thiết kế và tối ưu

- **Wi-Fi không chặn lúc boot.** Mọi dịch vụ lắng nghe trên cả mạng nhà lẫn AP cài đặt. Wi-Fi
  tự kết nối lại (1 s → 5 s → 15 s), `WIFI_PS_NONE` (tắt modem-sleep), tự chọn AP mạnh nhất
  khi nhà có mesh/extender, driver không ghi flash mỗi lần đổi cấu hình (`WIFI_STORAGE_RAM`).
- **Không polling.** Mỗi kênh UART có hai task chặn: `uart_rx` là task duy nhất đọc UART, thức
  theo event của driver với RX timeout 4 ký tự (~0,35 ms ở 115200) rồi phát tới client TCP,
  WebSocket và bộ đệm `uart.read`; `uart_tcp` dùng `select()` không timeout.
- **Client mới chiếm quyền** cổng 2324; keepalive 5 s + 3 × 2 s phát hiện client chết trong
  ~11 s (mặc định lwIP là 2 giờ).
- **Lệnh HTTP chạy trên task riêng**, nên một `uart.xfer`/`wifi.set` dài không làm treo terminal.
- **SWD**: lệnh text đọc theo lô (không còn 1 `recv()` mỗi byte), nhiều lệnh/kết nối;
  remote-bitbang gom `recv`/`send` và ghi thẳng thanh ghi GPIO. Trên S3 chạy ở core 1.
- **Build**: `-O2`, CPU 240 MHz, lwIP trong IRAM, ISR UART trong IRAM (không tràn FIFO khi đang
  ghi NVS), phân vùng app 1,5 MB (offset NVS giữ nguyên nên cài đặt không mất).
- **`send()` được lặp tới hết**, không đổi CR/LF, không Telnet: byte vào sao thì ra vậy.

## Bảo mật

- AP cài đặt có WPA2 và chỉ bật khi cần. Nhưng ai biết mật khẩu AP thì truy cập được UART/SWD,
  nên hãy **đổi `BRIDGE_SETUP_AP_PASSWORD`**.
- Trong LAN, HTTP/TCP **không có mật khẩu ứng dụng**. Bất kỳ ai trong cùng mạng đều gửi lệnh
  được, kể cả lệnh ghi SRAM/Flash qua cổng SWD 2325. Chỉ dùng trong LAN tin cậy hoặc VLAN riêng.
- Riêng trình duyệt: `/api/cmd` bắt buộc header `X-Bridge` và WebSocket kiểm tra `Origin`, để
  một trang web lạ đang mở trên máy bạn không điều khiển được robot.
