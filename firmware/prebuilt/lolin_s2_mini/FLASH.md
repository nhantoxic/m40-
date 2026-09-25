# Nạp sẵn cho LOLIN S2 mini (không cần build)

Build từ commit ghi trong git log của thư mục này, bằng ESP-IDF v6.1 và cấu hình mặc định:

- MCU UART: TX 35, RX 33 (tcp 2324)
- SoC shell: TX 37, RX 39 (tcp 2323)
- SWD: GPIO 16/18
- 115200 8N1

**Không có sẵn Wi-Fi**: lần đầu bridge phát AP `DreameBridge-XXXX` (mật khẩu `dreame-setup`). Vào
http://192.168.4.1 để chọn Wi-Fi nhà, hoặc cài qua cáp USB
(`bridge_tool.py --usb COM8 wifi.set "SSID" "mật khẩu"`).

## Nạp

Vào download mode: giữ BOOT → nhấn RESET → thả BOOT (board hiện COM5).

```powershell
cd firmware\prebuilt\lolin_s2_mini
py -m esptool --chip esp32s2 --port COM5 --baud 460800 --before no_reset --after no_reset `
   write_flash --flash_mode dio --flash_freq 80m --flash_size 4MB `
   0x1000 bootloader.bin 0x8000 partition-table.bin 0x10000 esp_uart_bridge.bin
py -m esptool --chip esp32s2 --port COM5 --before no_reset --after no_reset erase_region 0x1F0000 0x10000
```

Lệnh thứ hai xoá vùng coredump mới, để không còn dữ liệu cũ nằm ở đó. Nạp xong thì nhấn RESET.
Kiểm tra file bằng `SHA256SUMS`.

## Nếu bridge không lên

Firmware tự đếm số lần crash sớm (trong vòng 60 s sau khi boot) liên tiếp:

| Số crash liên tiếp | Chế độ | Cách lấy báo cáo |
|---|---|---|
| 0–1 | bình thường | `bridge_tool.py crash`, hoặc tab Wi-Fi trong app |
| 2–3 | **safe**: chỉ Wi-Fi, discovery, web và lệnh (tắt UART, SWD, mDNS; giảm công suất phát Wi-Fi) | `bridge_tool.py discover` → các trường `boot_mode`, `reset_reason`, `crash` |
| ≥ 4 | **usb_only**: tắt cả Wi-Fi | mở cổng COM của board (DTR=0, RTS=1): cứ 5 s in `[crashlog] …`; gửi `@CMD crash` |

Discovery còn có `free_heap`, `min_free_heap` và `http_ok` (web server có chạy không).

`crash` có dạng `task=<tên> pc=0x… cause=<n> vaddr=0x… bt=0x… 0x…`. `cause` là mã EXCCAUSE của
Xtensa: 28/29 = đọc/ghi địa chỉ sai (xem `vaddr`); 0 kèm `reset_reason` watchdog = CPU bị kẹt tại `pc`. Giải mã bằng ELF trong thư mục này:

```powershell
xz -d esp_uart_bridge.elf.xz
xtensa-esp32s2-elf-addr2line -pfiaC -e esp_uart_bridge.elf 0x4008xxxx 0x4009xxxx ...
```

Hoặc gửi nguyên dòng đó cho người hỗ trợ. `reset_reason: brownout` nghĩa là nguồn bị sụt áp
(thường khi Wi-Fi phát), không phải lỗi phần mềm. `bridge_tool.py crash.clear` rồi `reboot` để
quay về chế độ bình thường. `crash.test` cố ý gây crash để thử toàn bộ cơ chế này.
