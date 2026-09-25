#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Starts AP+STA provisioning.
 *
 * SoftAP:
 *   SSID: bridge_esp_swd_XXXX  (XXXX = last 2 MAC bytes)
 *   PASS: bridge-swd
 *   URL : http://192.168.4.1/
 *
 * Saved STA credentials are kept in NVS namespace "bridgewifi".
 * Reset/reboot does NOT erase them.
 */
esp_err_t wifi_provision_start(void);

/* Erase saved STA credentials. Does not erase unrelated NVS data. */
esp_err_t wifi_provision_forget(void);

#ifdef __cplusplus
}
#endif
