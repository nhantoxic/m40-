#pragma once

#include <stdbool.h>
#include <stddef.h>

#include "esp_err.h"
#include "jbuf.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Wi-Fi station plus an on-demand, WPA2-protected setup access point.
 *
 * The station joins the saved network (NVS, else menuconfig). The setup AP
 * "DreameBridge-XXXX" (192.168.4.1) comes up when no network is configured,
 * when the station has had no IP for CONFIG_BRIDGE_SETUP_AP_DELAY_S, when the
 * setup button is held for 3 s, or on request; it goes away again once the
 * station is back and nobody is connected to it. Every service listens on
 * both interfaces, so the app works the same through either. */

esp_err_t wifi_mgr_start(void);   /* non-blocking */

bool wifi_mgr_sta_up(void);
const char *wifi_mgr_ip(void);    /* station IP, "0.0.0.0" when down */
bool wifi_mgr_ap_active(void);

/* Writes wifi fields into the currently open JSON object. */
void wifi_mgr_status_json(jbuf_t *jb);

/* Appends an array of visible networks under `key`. Blocks ~2-3 s. */
esp_err_t wifi_mgr_scan_json(jbuf_t *jb, const char *key);

/* Joins ssid/pass and saves them only if an IP is obtained within ~20 s;
 * otherwise the previous network is restored. On failure *reason explains
 * why (wrong password, not found, ...). */
esp_err_t wifi_mgr_set(const char *ssid, const char *pass,
                       char *ip_out, size_t ip_len, const char **reason);

/* Erases the saved network and brings the setup AP up. */
esp_err_t wifi_mgr_forget(void);

/* on: setup AP for 10 minutes (or until the station is back and idle).
 * off: drop it now if the station has an IP. */
esp_err_t wifi_mgr_ap(bool on);

#ifdef __cplusplus
}
#endif
