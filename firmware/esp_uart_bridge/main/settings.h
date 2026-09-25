#pragma once

#include <stdbool.h>
#include <stddef.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Runtime settings kept in NVS namespace "bridge". Set from the app, the PC
 * tool or the USB console; survive reboots and firmware updates. */

/* Call once after nvs_flash_init(). */
void settings_load(void);

/* Wi-Fi station credentials. Falls back to the menuconfig SSID/password when
 * nothing was ever saved; returns false when there is nothing usable (never
 * configured, or erased with settings_forget_wifi()). */
bool settings_wifi(char ssid[33], char pass[65]);
esp_err_t settings_set_wifi(const char *ssid, const char *pass);
/* Stores an explicit "no network" so the menuconfig fallback is not used. */
esp_err_t settings_forget_wifi(void);

/* ch: BRIDGE_CH_MCU or BRIDGE_CH_SOC (see uart_tcp_bridge.h). */
int settings_uart_baud(int ch);
esp_err_t settings_set_uart_baud(int ch, int baud);

#ifdef __cplusplus
}
#endif
