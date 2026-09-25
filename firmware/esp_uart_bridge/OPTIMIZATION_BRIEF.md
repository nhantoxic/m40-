# ESP32-S2 bridge — bộ tối ưu firmware (chuẩn bị 2026-09-25, đêm)

Người dùng cắm ESP32-S2 mini để flash (COM5 lúc flash → COM8 sau flash) và yêu cầu
tối ưu firmware, rồi đi ngủ. File này ghi lại: đã làm gì, còn vướng gì, và các bước
build/flash để chạy một phát khi thức dậy.

## TRẠNG THÁI: BUILD XONG ✅ — CHỈ CÒN FLASH (cần bấm tay download mode)

Firmware tối ưu (WIFI_PS_NONE) đã **build thành công**:
`D:\pio-build\esp_uart_bridge\lolin_s2_mini\firmware.bin` (815 KB, Flash 77.7%, RAM 11.5%)
cùng `bootloader.bin`, `partitions.bin`. Toolchain đã sửa (xem mục 0).

**FLASH (bạn làm — cần download mode):**
1. Trên S2 mini: **giữ BOOT, nhấn RESET, thả BOOT** → board hiện thành **COM5**.
2. (khuyên) backup flash cũ trước:
   `py <esptool.py> --chip esp32s2 --port COM5 read_flash 0 0x400000 D:\esp_uart\backup_before_opt.bin`
   (esptool.py ở `C:\Users\ADMIN\.platformio\packages\tool-esptoolpy\esptool.py`)
3. Flash:
   `set TEMP=D:\tmp && cd D:\esp_uart\firmware\esp_uart_bridge && py -m platformio run -t upload --upload-port COM5`
4. Sau flash board tự reset → chạy lại thành **COM8**, tự join Wi-Fi. Kiểm tra bằng
   discovery UDP (mục 3) — kỳ vọng `uart_baud:230400`.

## 0. MÔI TRƯỜNG (đã xử lý đêm 2026-09-25)

- [ĐÃ XỬ LÝ] **C: đầy** → đã dọn ~9.7 GB (xóa 3 folder `7zO*` temp của 7-Zip trong
  `%TEMP%`). C: giờ ~9.7 GB trống.
- [ĐÃ XỬ LÝ] **PlatformIO Core** → đã cài lại: `py -m platformio` (Core 6.2.0,
  Python 3.14). Build bằng: `set TEMP=D:\tmp && cd <proj> && py -m platformio run`.
- [CÒN VƯỚNG] **Toolchain xtensa hỏng.** `xtensa-esp32s2-elf-gcc` (esp-15.2.0_20251204)
  báo `fatal error: cannot execute 'as'` KỂ CẢ khi để bin trên PATH → gcc không gọi
  được assembler (lỗi exec-prefix/relocation, nhiều khả năng hư từ lúc C: đầy).
  Build DỪNG ở bước compiler-check, chưa ra binary.
  **Cách sửa (làm khi thức):**
  1. Thử build lại bằng **VS Code PlatformIO** (nó có penv + có thể toolchain riêng nhất quán).
  2. Hoặc cài lại gói toolchain: xóa `~/.platformio/packages/toolchain-xtensa-esp-elf`
     rồi `py -m platformio run` (pio tự tải lại), hoặc `py -m platformio pkg install`.
  3. Nếu vẫn lỗi: kiểm tra platform espressif32 version bị nhảy lên bản GCC15 mới;
     pin về version đã từng build được.

CHƯA flash (build chưa thành công + không flash khi không giám sát; đây là cầu nối
duy nhất tới MCU). Board S2 luôn cứu được qua bootloader USB (giữ BOOT + nhấn RESET).

## 1. ĐÃ ÁP (có backup, an toàn)

- `main/wifi_sta.c`: `WIFI_PS_MIN_MODEM` → **`WIFI_PS_NONE`** (tắt modem-sleep).
  Lý do: cầu nối tải console thô + đọc SWD nhạy độ trễ; modem-sleep gây trễ vài ms
  làm chậm round-trip và thỉnh thoảng rớt/hủy giao dịch ngắn (đúng hiện tượng SWD
  bị abort khi gửi lệnh liên tiếp). Backup: `wifi_sta.c.bak_psnone_*`.
- Baud ESP↔MCU giữ **230400 8N1** (đã xác minh đúng qua discovery `uart_baud:230400`
  và đọc live; KHÔNG hạ 115200 — đó là baud COM ảo RobotMonitor, đường khác).

## 2. NÊN ÁP (diff sẵn — cần build để kiểm, làm khi thức)

### 2a. Địa chỉ ổn định — hết cảnh DHCP đổi IP (.51→.36)
Đây là phiền toái lớn nhất khi dùng. Chọn 1:

**mDNS `dreame-bridge.local`** (khuyên dùng): thêm managed component + init.
```
# tại D:\esp_uart\firmware\esp_uart_bridge
idf.py add-dependency "espressif/mdns"     # hoặc thêm vào main/idf_component.yml
```
Trong `main/main.c`, sau `wifi_sta_start_and_wait()`:
```c
#include "mdns.h"
...
mdns_init();
mdns_hostname_set(CONFIG_BRIDGE_HOSTNAME);          // -> dreame-bridge.local
mdns_service_add(NULL, "_dreame-bridge", "_tcp", CONFIG_BRIDGE_TCP_PORT, NULL, 0);
```
Sau đó truy cập `dreame-bridge.local:2324/2325` khỏi cần dò IP.

**Hoặc IP tĩnh** (không cần component): trong `wifi_sta.c` trước `esp_wifi_start()`:
```c
esp_netif_dhcpc_stop(netif);
esp_netif_ip_info_t ip = {0};
ip.ip.addr = esp_ip4addr_aton("192.168.1.50");
ip.gw.addr = esp_ip4addr_aton("192.168.1.1");
ip.netmask.addr = esp_ip4addr_aton("255.255.255.0");
esp_netif_set_ip_info(netif, &ip);
```
(Đơn giản nhất thực tế: đặt **DHCP reservation** cho MAC `48:F6:EE:6B:05:26` trên router.)

### 2b. Tối ưu build (qua menuconfig, an toàn hơn sửa tay sdkconfig)
- `Compiler options → Optimization Level → Optimize for performance (-O2)`
  (mặc định thường -Og). Tăng throughput bơm byte + độ nhạy console.
- Giữ log ở INFO (log `connected, ip=...` hữu ích để tìm board; log ra USB-CDC,
  không đụng đường UART tới MCU).

### 2c. SWD server bền hơn với nhiều lệnh/1 kết nối
Quan sát: gửi `PING` rồi `ID` liên tiếp trên 1 socket bị abort (WinError 10053).
Client hiện lách bằng "mỗi lệnh 1 kết nối" (xem swd_monitor_mop_pump.py) nên không
chặn việc dùng, nhưng nên soi vòng lặp dispatcher trong `swd_bridge.c` (~dòng 1080–1110,
1900+) xem có đóng socket sau 1 lệnh không, để cho phép pipelined reads nhanh hơn.

## 3. BUILD + FLASH (khi C: đã dọn & có pio/VS Code)

```
# ép temp sang D: để né C: đầy
set TMP=D:\tmp
set TEMP=D:\tmp
cd D:\esp_uart\firmware\esp_uart_bridge

# build (build_dir đã trỏ D:\pio-build\esp_uart_bridge trong platformio.ini)
pio run                      # hoặc nút Build trong VS Code PlatformIO

# BACKUP firmware đang chạy TRƯỚC KHI flash (để khôi phục nếu cần)
#   đưa S2 vào download mode: giữ BOOT, nhấn RESET (cổng COM5)
esptool --chip esp32s2 -p COM5 read_flash 0 0x400000 D:\esp_uart\backup_before_opt.bin

# flash
pio run -t upload            # dùng upload_port trong platformio.ini
#   nếu cần chỉ cổng download: pio run -t upload --upload-port COM5
# sau reset, board chạy lại như COM8 và tự join Wi-Fi
```

Kiểm tra sau flash (không mở RobotMonitorV4):
```
python -c "import socket;s=socket.socket(2,2);s.setsockopt(1,6,1);s.sendto(b'DREAME_BRIDGE_DISCOVER',('255.255.255.255',2326));s.settimeout(3);print(s.recvfrom(2048)[0].decode())"
```
Kỳ vọng JSON có `uart_baud:230400`, và sau khi robot bật thì `rx_bytes` tăng khi có
lệnh (đường RX GPIO33 ← MCU-TX phải thông).

## 4. LƯU Ý
- KHÔNG hạ baud xuống 115200. KHÔNG bật autobaud để "tự chạy" (nó chỉ LOG rồi khôi
  phục baud cũ — uart_tcp_bridge.c:481).
- Vấn đề "bơm không chạy ngoài sàn" KHÔNG phải lỗi cầu nối/UART (xem memory
  robotmon-m40-project): firmware stock lệnh bơm đúng khi giẻ hạ; nghi kênh đo dòng
  bơm (shunt/ADC) hở và/hoặc SoC không cấp nước khi lau — cần đọc schematic phần bơm.
