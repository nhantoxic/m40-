#pragma once

#include <stdbool.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Bring up the Wi-Fi station and block until an IPv4 address is assigned.
 *
 * Returns ESP_OK once connected. Reconnection after a later drop is handled
 * internally and does not require the caller to do anything. */
esp_err_t wifi_sta_start_and_wait(void);

/* Latest IPv4 address as a dotted string, or "0.0.0.0" when disconnected.
 * The returned pointer is to static storage and stays valid forever. */
const char *wifi_sta_ip(void);

/* True while the station holds an IP address. */
bool wifi_sta_is_up(void);

#ifdef __cplusplus
}
#endif
