# esp_uart_bridge

Cầu **UART ↔ TCP** cho cổng debug của robot Dreame, kèm một cổng
**OpenOCD remote-bitbang ↔ SWD**. Có hai kênh UART độc lập: console SoC và UART
tham số của MCU ARM.

Profile UART-only đang chạy: UART1 trên GPIO35/33, TCP `2324`, `230400 8N1`;
UART0 phụ bị tắt. Đây là cấu hình đầu ESP; đường vật lý tới UART ứng dụng MCU
chưa được xác nhận. App bridge mới đã nạp qua COM5 vào offset `0x10000` và xác
minh hash thành công. `info -a` và `ver -t` đã được gửi qua TCP nhưng không có
byte trả lời; xem báo cáo live ở
[`BRIDGE_RECHECK_20260923.md`](../../BRIDGE_RECHECK_20260923.md).
IP là DHCP-assigned; các tool SWD/UART tự dò qua UDP `2326` khi không truyền
`--host`.

Biên dịch được cho cả `esp32s2` (LOLIN S2 mini) và `esp32s3` (DevKitC-1, và sau này là
module trên bo Rev A).

## Xây dựng

```bash
cd firmware/esp_uart_bridge
idf.py set-target esp32s2          # hoặc esp32s3
idf.py menuconfig                  # → "Dreame bridge configuration"
idf.py build
idf.py -p COM5 flash monitor       # Linux: -p /dev/ttyACM0
```

Yêu cầu ESP-IDF **v5.0 trở lên** (dùng `UART_SCLK_DEFAULT` và
`esp_event_handler_instance_register`).

## Cấu hình

Toàn bộ nằm trong `menuconfig` → *Dreame bridge configuration*:

| Mục | Mặc định | Ghi chú |
|---|---|---|
| Wi-Fi SSID / password | `changeme` | **Bắt buộc đặt mật khẩu WPA2 tối thiểu 8 ký tự.** Lưu trong sdkconfig, không commit. |
| Wi-Fi SSID dự phòng | rỗng | Tùy chọn. Sau `MAX_RETRY` lần fail ở SSID chính, bridge tự chuyển sang SSID này với **cùng mật khẩu**. Board nằm trong robot nên không sửa SSID bằng tay được: một lỗi gõ tên mạng hoặc một AP chỉ phủ một phần khu vực sẽ làm mất kết nối vĩnh viễn nếu không có đường lui. |
| Hostname | `dreame-bridge` | Tra IP trên router bằng tên này |
| LAN discovery UDP | `2326` | RobotMonitor gửi broadcast để tự tìm ESP trong cùng mạng |
| Primary TCP port / UART | `2324` / UART1 | Raw UART tại GPIO35 ESP TX / GPIO33 ESP RX. Điểm cuối robot chưa xác nhận là MCU UART; trang X40 page 18 ghi SoC RX/TX. |
| Primary UART baud | `230400 8N1` | Cấu hình ESP hiện tại; chưa xác minh baud của đường nối MCU CLI trên máy đang thử. |
| UART0 phụ | tắt | Chỉ bật trong profile dual-UART; GPIO16/18 không được đồng thời dùng cho UART SoC và SWD |
| OpenOCD remote-bitbang TCP | `3335` | SWDIO GPIO16, SWCLK GPIO18; không có NRST |
| SWD text TCP | `2325` | Có ID/READ/DUMP, core debug, ghi SRAM và ghi Flash; không phải API read-only |
| Software flow control (`ixoff`) | off | Chỉ bật sau khi xác nhận robot hiểu XON/XOFF; nó gửi XOFF tại 96 byte FIFO và XON tại 32 byte. |
| LED GPIO | `15` | S2 mini. Đặt `-1` trên S3 DevKitC (LED ở đó là RGB addressable). |

## Dùng

```powershell
# Tự dò IP qua UDP 2326 rồi đọc ID SWD:
python tools\swd_probe.py id

# Tự dò IP rồi hỏi CLI MCU (lệnh chỉ đọc thông tin):
python tools\mcu_identity_probe.py
```

Discovery live gần nhất trả `192.168.1.36`, MAC `48:F6:EE:6B:05:26`.
Địa chỉ do DHCP cấp nên có thể đổi; dùng nút **Quét ESP** trong RobotMonitor
hoặc để các tool tự dò thay vì sao chép một IP cũ.

### Chọn mạng Wi-Fi

Log khởi động in ra mọi AP trong tầm ngay trước khi kết nối, nên không cần
đoán xem board có nhìn thấy mạng hay không:

```text
I (2842) wifi: scan: 18 AP(s) visible
I (2842) wifi:   ssid="Ongtrumnoitro.com VT" rssi=-59 ch=3 auth=4
I (2844) wifi:   ssid="Ongtrumnoitro.com VN" rssi=-83 ch=10 auth=4
I (2850) wifi: joining "Ongtrumnoitro.com VT"
I (3000) wifi:connected with Ongtrumnoitro.com VT, aid = 13, channel 3, 40U, bssid = 30:42:40:eb:9d:68
```

Máy tính quét bằng `netsh wlan show networks` có thể chỉ thấy một nửa số SSID mà
bridge thấy, vì adapter PC ở xa AP hơn hoặc kết quả bị cache. Log trên board là
nguồn đúng khi hai bên không khớp.

Cả hai cổng đều là raw TCP, **không phải Telnet**: không thương lượng IAC, không đổi CR/LF.
Byte vào sao thì byte ra vậy — cần thiết cho shell SoC và giao thức RobotMonitor.
PuTTY phải chọn chế độ **Raw**.

### Đọc log của bridge qua USB CDC

Console ESP-IDF nằm trên USB CDC của S2 mini, nhưng driver CDC của ROM chỉ phát TX
khi đường điều khiển ở đúng mức: **DTR = 0, RTS = 1**. Để nguyên mặc định của
pyserial/`pio device monitor` sẽ mở được cổng mà không thấy byte nào.

```python
import serial
s = serial.Serial("COM5", 115200, timeout=0.2)
s.dtr = False
s.rts = True          # bắt đầu thấy log
print(s.read(8192))
```

Đừng hạ RTS từ 1 xuống 0 khi DTR = 0 ngoài ý muốn: theo `usb_console.c` của
ESP-IDF, cạnh xuống đó là lệnh **reboot bình thường** (và nếu DTR = 1 thì là
reboot vào bootloader). Đóng cổng bằng pyserial cũng tạo cạnh xuống này, nên
board sẽ khởi động lại sau mỗi lần đóng cổng và mất vài giây để vào lại Wi-Fi.

### Dùng xPack OpenOCD qua SWD

Cổng TCP `3335` không phải UART raw. Nó dùng giao thức `remote_bitbang` của OpenOCD:
OpenOCD gửi từng mức SWCLK/SWDIO qua Wi-Fi, ESP32-S2 phát chúng trên GPIO18/GPIO16.
Do robot không đưa NRST ra đầu nối, cấu hình probe dùng `reset_config none`.

Ví dụ với xPack OpenOCD Windows:

```powershell
$ocd = 'tmp\openocd-xpack\xpack-openocd-0.12.0-7'
& "$ocd\bin\openocd.exe" -s "$ocd\openocd\scripts" `
  -f tools\openocd_remote_swd_probe.cfg
```

Cấu hình OpenOCD mẫu chỉ đọc DAP-ID; nó chưa khai báo lệnh ghi. Riêng TCP
`2325` còn có lệnh điều khiển core, ghi SRAM và ghi Flash; coi các cổng debug
này là có quyền sửa MCU. `ID` báo clock cấu hình của engine nội bộ (bản live
đã đọc `1000 kHz`). Remote-bitbang `3335` có timing do OpenOCD và độ trễ mạng
điều khiển; baud UART không áp dụng cho SWD.

### Chế độ tạm thời: UART1 trên GPIO35/33

Theo hàng chân thực tế của S2 mini, GPIO16/GPIO18 nối vào SWDIO/SWCLK và không được dùng làm
UART. Firmware tạm dùng UART1 trên GPIO35/33, nhưng đầu robot của cặp dây này chưa được
định danh. UART0 phụ bị tắt. Cấu hình tạm hiện tại là:

```text
GPIO16             → SWDIO (không drive bằng UART)
GPIO18             → SWCLK (không drive bằng UART)
UART1, GPIO35/33   → UART thô (ESP TX/RX; target robot chưa xác nhận), 230400 8N1
TCP 2324             → RobotMonitor/raw client / YMODEM sender
UART0 phụ            → tắt
```

Các phép thử live gần nhất trên máy này nhận 0 byte cho `info -a` và `ver -t`.
Lần chạy trước có 176 byte RX nhưng bridge đã bỏ chúng khi chưa có client nên
không thể xem nội dung. Firmware đang nằm trong Flash có mục lệnh `info` và
nhánh xử lý đối số `-a`, vì vậy sự im lặng là lỗi đường giao tiếp hoặc trạng
thái MCU, không phải bằng chứng rằng lệnh không tồn tại. Hình X40 page 18 chỉ
ghi SoC RX/TX; hãy xác nhận pin MCU UART trước khi xem GPIO35/33 là đường MCU.
Để bắt thông báo khi target khởi động, mở capture trước rồi mới bật robot:

```powershell
python tools\capture_mcu_uart.py --seconds 120
python tools\mcu_identity_probe.py
```

Để nghe trên cổng COM trực tiếp thay vì bridge:

```powershell
python tools\mcu_identity_probe.py --port COM8 --baud 230400
```

Nếu cần quay lại dual-UART, trả UART1 về SoC GPIO16/18, bật `BRIDGE_MCU_UART_ENABLE`,
giữ UART0 ở GPIO35/33 và dùng TCP2323 cho SoC, TCP2324 cho MCU.

Đèn LED:

| Trạng thái | Ý nghĩa |
|---|---|
| Nháy nhanh (100 ms) | đang vào Wi-Fi, hoặc vừa mất kết nối |
| Nháy chậm (1 s) | đã có IP, đang chờ client |
| Sáng liên tục | đang có client |

## Thiết kế

- **Một client tại một thời điểm trên mỗi kênh.** Hai kênh có thể hoạt động đồng thời nhưng
  không được trộn dữ liệu vào nhau.
- **Một task, một vòng lặp cho mỗi kênh.** `select()` trên socket với timeout 5 ms, rồi vét
  UART không chặn.
- **`send()` được lặp cho tới hết.** `send()` trả về thiếu là chuyện bình thường; mất đuôi
  một dòng giữa phiên shell là loại lỗi sẽ bị đổ cho robot suốt mấy tuần.
- **Xả bộ đệm UART khi có client mới** (tắt được). Nếu không, thứ đầu tiên người dùng thấy
  là một mẩu log cụt của mười phút trước — trông y hệt một cú crash đang diễn ra.
- **`ixoff` là tuỳ chọn, mặc định tắt.** Nó có thể giảm nguy cơ tràn khi robot phát log quá
  nhanh, nhưng chỉ an toàn nếu robot thực sự dừng phát khi nhận XOFF; nếu không, hai byte
  điều khiển có thể đi vào console như dữ liệu thường.
- **Không có AP fallback, không có portal cấu hình.** Một AP mở dẫn thẳng vào console root
  còn tệ hơn là gõ sai SSID.

## LAN discovery cho RobotMonitor

ESP chạy station mode và chỉ gia nhập AP khi có mật khẩu WPA2 hợp lệ; không có
AP mở hoặc portal cấu hình. Sau khi nhận IP, firmware lắng nghe UDP `2326`.
RobotMonitor gửi chuỗi `DREAME_BRIDGE_DISCOVER` và ESP trả JSON chứa hostname,
IP, MAC, `uart_port` (kênh primary), `mcu_port` (kênh được cấu hình làm MCU), `uart_mode` và các
cổng SWD. Ở chế độ dual-UART, `uart_port`=`2323` (SoC) còn `mcu_port`=`2324`;
RobotMonitor ưu tiên `mcu_port`. Mật khẩu Wi-Fi không bao giờ được gửi trong
bản tin discovery.

Bản tin còn kèm chẩn đoán vật lý, dùng để tách lỗi dây khỏi lỗi giao thức:

```json
{
  "uart_baud": 230400,
  "uart_tx_level": 1, "uart_rx_level": 1,
  "uart_stats": {
    "rx_bytes": 0, "tx_bytes": 28, "frame_err": 0,
    "parity_err": 0, "break": 0, "fifo_ovf": 0, "buf_full": 0
  }
}
```

`uart_tx_level` / `uart_rx_level` là mức GPIO tại thời điểm trả lời; UART idle
bình thường là `1`. `uart_stats` là bộ đếm từ lúc boot và đọc được cả khi không
có client TCP nào. Cách đọc:

| Quan sát | Kết luận |
|---|---|
| `rx_bytes` tăng | ESP UART đã giải mã byte ở cặp GPIO/tốc độ hiện tại. Chưa xác nhận byte đến từ MCU; có thể là SoC, loopback hoặc nguồn khác. |
| `frame_err` hoặc `parity_err` tăng trong khi `rx_bytes` gần 0 | ESP RX gặp tín hiệu không giải mã được; có thể sai baud/parity, nhiễu hoặc sai nguồn tín hiệu. |
| `rx_bytes` = 0 và mọi bộ đếm lỗi = 0 | Không có byte hợp lệ được giải mã; có thể do đường dây, đối tác im lặng/ngủ, hoặc đang nghe nhầm UART. Chưa kết luận được nguyên nhân. |
| `tx_bytes` tăng khi gửi qua TCP | TCP → ESP UART driver đã nhận byte. Chưa chứng minh mức tín hiệu trên chân hoặc MCU đã nhận. |

## Chưa có

- ADB / USB host. SWD remote-bitbang đã được triển khai trên TCP `3335`; sơ đồ dây hiện
  tại dùng GPIO16 = SWDIO, GPIO18 = SWCLK. GPIO37 có thể dành cho NRST tùy chọn; GPIO39
  để VTREF cảm nhận hoặc để hở. Không dùng GPIO19/20 vì đó là USB native của S2 mini.
- Xác thực ứng dụng/TLS. Wi-Fi đã bắt buộc WPA2, nhưng TCP raw vẫn không có
  mật khẩu thứ hai; chỉ dùng trong LAN tin cậy hoặc VLAN riêng.
- OTA. Bo còn nằm trong tầm tay khi đang phát triển.

## Cảnh báo

Firmware này yêu cầu mật khẩu để gia nhập Wi-Fi, nhưng **bất kỳ thiết bị nào
đã ở cùng LAN** vẫn có thể truy cập UART raw và API SWD có lệnh ghi. Chỉ cấp
nguồn khi cần dùng, hoặc đặt vào VLAN riêng.
