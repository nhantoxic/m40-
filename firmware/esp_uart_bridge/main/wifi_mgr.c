/* Wi-Fi station with unattended reconnect and an on-demand setup AP.
 *
 * The bridge sits inside a robot with no user interface. It must keep trying
 * its network forever, and when that network is gone (new router, changed
 * password) there has to be a way back in that does not involve opening the
 * robot: the setup AP. It is WPA2-protected and only up while needed, since
 * it leads to the same raw UART and SWD endpoints as the LAN.
 */

#include "wifi_mgr.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_wifi.h"

#include "settings.h"

static const char *TAG = "wifi";

#define BIT_GOT_IP        BIT0
#define BIT_DISCONNECTED  BIT1

#define TICK_MS           100
#define SET_TIMEOUT_MS    20000
#define FORCED_AP_MS      (10 * 60 * 1000)
#define AP_IDLE_OFF_MS    30000   /* station up + AP unused this long -> AP off */
#define BUTTON_HOLD_MS    3000
#define SCAN_MAX          20

static EventGroupHandle_t s_ev;
static SemaphoreHandle_t s_op;        /* serializes every esp_wifi_* reconfiguration */
static esp_netif_t *s_sta_netif;

static char s_ssid[33];
static char s_pass[65];
static bool s_have_creds;
static char s_ip[16] = "0.0.0.0";
static char s_ap_ssid[33];

static volatile bool s_sta_up;
static volatile bool s_connecting;
static volatile bool s_hold;          /* an operation owns the station; no auto-reconnect */
static volatile bool s_ap_on;
static volatile bool s_ap_forced;
static volatile int s_ap_clients;
static volatile uint8_t s_last_reason;
static volatile int s_retries;
static volatile TickType_t s_down_since;
static volatile TickType_t s_up_since;
static volatile TickType_t s_next_retry;
static TickType_t s_ap_until;

static const char *reason_text(uint8_t reason)
{
    switch (reason) {
    case 0: return "";
    case WIFI_REASON_4WAY_HANDSHAKE_TIMEOUT:
    case WIFI_REASON_HANDSHAKE_TIMEOUT:
        return "wrong password (handshake timeout)";
    case WIFI_REASON_AUTH_FAIL: return "authentication failed";
    case WIFI_REASON_NO_AP_FOUND: return "network not found";
    case WIFI_REASON_NO_AP_FOUND_W_COMPATIBLE_SECURITY:
        return "network found but its security mode is not supported";
    case WIFI_REASON_CONNECTION_FAIL: return "connection failed";
    case WIFI_REASON_ASSOC_LEAVE: return "left the network";
    case WIFI_REASON_BEACON_TIMEOUT: return "signal lost (beacon timeout)";
    default: return "disconnected";
    }
}

static TickType_t now(void)
{
    return xTaskGetTickCount();
}

static bool elapsed(TickType_t since, uint32_t ms)
{
    return (TickType_t)(now() - since) >= pdMS_TO_TICKS(ms);
}

static void apply_sta(const char *ssid, const char *pass)
{
    wifi_config_t cfg = { 0 };
    strlcpy((char *)cfg.sta.ssid, ssid, sizeof(cfg.sta.ssid));
    strlcpy((char *)cfg.sta.password, pass, sizeof(cfg.sta.password));
    /* WPA2 or better only: this bridge reaches the robot's consoles. */
    cfg.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
    cfg.sta.pmf_cfg.capable = true;
    cfg.sta.sae_pwe_h2e = WPA3_SAE_PWE_BOTH;
    /* Mesh/extender homes: pick the strongest AP for this SSID. */
    cfg.sta.scan_method = WIFI_ALL_CHANNEL_SCAN;
    cfg.sta.sort_method = WIFI_CONNECT_AP_BY_SIGNAL;
    esp_err_t err = esp_wifi_set_config(WIFI_IF_STA, &cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "station config: %s", esp_err_to_name(err));
    }
}

static void ap_on(bool forced)
{
    if (!s_ap_on) {
        esp_err_t err = esp_wifi_set_mode(WIFI_MODE_APSTA);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "setup AP: %s", esp_err_to_name(err));
            return;
        }
        wifi_config_t cfg = { 0 };
        strlcpy((char *)cfg.ap.ssid, s_ap_ssid, sizeof(cfg.ap.ssid));
        cfg.ap.ssid_len = strlen(s_ap_ssid);
        strlcpy((char *)cfg.ap.password, CONFIG_BRIDGE_SETUP_AP_PASSWORD,
                sizeof(cfg.ap.password));
        cfg.ap.authmode = WIFI_AUTH_WPA2_PSK;
        cfg.ap.channel = 1;          /* follows the station channel once it connects */
        cfg.ap.max_connection = 2;
        err = esp_wifi_set_config(WIFI_IF_AP, &cfg);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "setup AP config: %s", esp_err_to_name(err));
        }
        s_ap_on = true;
        s_ap_clients = 0;
        ESP_LOGW(TAG, "setup AP \"%s\" up: http://192.168.4.1/", s_ap_ssid);
    }
    if (forced) {
        s_ap_forced = true;
        s_ap_until = now() + pdMS_TO_TICKS(FORCED_AP_MS);
    }
}

static void ap_off(void)
{
    if (s_ap_on) {
        if (esp_wifi_set_mode(WIFI_MODE_STA) != ESP_OK) {
            return;
        }
        s_ap_on = false;
        s_ap_forced = false;
        s_ap_clients = 0;
        ESP_LOGI(TAG, "setup AP off");
    }
}

static void on_event(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    (void)arg;

    if (base == WIFI_EVENT) {
        switch (id) {
        case WIFI_EVENT_STA_DISCONNECTED: {
            const wifi_event_sta_disconnected_t *ev = data;
            s_last_reason = ev->reason;
            if (s_sta_up) {
                s_down_since = now();
                ESP_LOGW(TAG, "lost \"%s\": %s (%u)", s_ssid, reason_text(ev->reason),
                         ev->reason);
            }
            s_sta_up = false;
            s_connecting = false;
            strcpy(s_ip, "0.0.0.0");
            int r = ++s_retries;
            s_next_retry = now() + pdMS_TO_TICKS(r <= 5 ? 1000 : r <= 10 ? 5000 : 15000);
            xEventGroupClearBits(s_ev, BIT_GOT_IP);
            xEventGroupSetBits(s_ev, BIT_DISCONNECTED);
            break;
        }
        case WIFI_EVENT_AP_STACONNECTED:
            s_ap_clients++;
            break;
        case WIFI_EVENT_AP_STADISCONNECTED:
            if (s_ap_clients > 0) {
                s_ap_clients--;
            }
            break;
        default:
            break;
        }
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        const ip_event_got_ip_t *ev = data;
        snprintf(s_ip, sizeof(s_ip), IPSTR, IP2STR(&ev->ip_info.ip));
        s_sta_up = true;
        s_connecting = false;
        s_retries = 0;
        s_last_reason = 0;
        s_up_since = now();
        ESP_LOGI(TAG, "connected to \"%s\", ip=%s", s_ssid, s_ip);
        xEventGroupSetBits(s_ev, BIT_GOT_IP);
    }
}

static void connect_now(void)
{
    s_connecting = true;
    if (esp_wifi_connect() != ESP_OK) {
        s_connecting = false;
        s_next_retry = now() + pdMS_TO_TICKS(1000);
    }
}

static void manager_task(void *arg)
{
    (void)arg;
    TickType_t pressed_since = 0;
    bool press_handled = false;

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(TICK_MS));

#if CONFIG_BRIDGE_SETUP_BUTTON_GPIO >= 0
        if (gpio_get_level(CONFIG_BRIDGE_SETUP_BUTTON_GPIO) == 0) {
            if (pressed_since == 0) {
                pressed_since = now();
            } else if (!press_handled && elapsed(pressed_since, BUTTON_HOLD_MS)) {
                press_handled = true;
                if (xSemaphoreTake(s_op, portMAX_DELAY) == pdTRUE) {
                    ESP_LOGW(TAG, "setup button held");
                    ap_on(true);
                    xSemaphoreGive(s_op);
                }
            }
        } else {
            pressed_since = 0;
            press_handled = false;
        }
#else
        (void)pressed_since;
        (void)press_handled;
#endif

        /* An operation (set/scan/forget) is reconfiguring: stay out of it. */
        if (xSemaphoreTake(s_op, 0) != pdTRUE) {
            continue;
        }

        if (!s_hold && s_have_creds && !s_sta_up && !s_connecting &&
            (int32_t)(now() - s_next_retry) >= 0) {
            connect_now();
        }

        if (!s_ap_on && !s_sta_up &&
            (!s_have_creds || elapsed(s_down_since, CONFIG_BRIDGE_SETUP_AP_DELAY_S * 1000U))) {
            ap_on(false);
        }

        if (s_ap_on && s_ap_forced && (int32_t)(now() - s_ap_until) >= 0) {
            s_ap_forced = false;
        }
        if (s_ap_on && !s_ap_forced && s_sta_up && s_ap_clients == 0 &&
            elapsed(s_up_since, AP_IDLE_OFF_MS)) {
            ap_off();
        }

        xSemaphoreGive(s_op);
    }
}

esp_err_t wifi_mgr_start(void)
{
    s_ev = xEventGroupCreate();
    s_op = xSemaphoreCreateMutex();
    if (s_ev == NULL || s_op == NULL) {
        return ESP_ERR_NO_MEM;
    }

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    s_sta_netif = esp_netif_create_default_wifi_sta();
    esp_netif_create_default_wifi_ap();
    ESP_ERROR_CHECK(esp_netif_set_hostname(s_sta_netif, CONFIG_BRIDGE_HOSTNAME));

    wifi_init_config_t init = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&init));
    /* Credentials live in our own NVS namespace; keep the driver from
     * rewriting flash on every esp_wifi_set_config(). */
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, on_event, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, on_event, NULL, NULL));

    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_SOFTAP);
    snprintf(s_ap_ssid, sizeof(s_ap_ssid), "DreameBridge-%02X%02X", mac[4], mac[5]);

    s_have_creds = settings_wifi(s_ssid, s_pass);
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    if (s_have_creds) {
        apply_sta(s_ssid, s_pass);
    }
    ESP_ERROR_CHECK(esp_wifi_start());
    /* Modem sleep adds wake-up latency to every console byte and SWD round
     * trip; the board runs from the robot's supply. */
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));

#if CONFIG_BRIDGE_SETUP_BUTTON_GPIO >= 0
    const gpio_config_t btn = {
        .pin_bit_mask = 1ULL << CONFIG_BRIDGE_SETUP_BUTTON_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&btn));
#endif

    s_down_since = now();
    s_next_retry = now();
    if (s_have_creds) {
        ESP_LOGI(TAG, "joining \"%s\"", s_ssid);
        connect_now();
    } else {
        ESP_LOGW(TAG, "no Wi-Fi configured");
        ap_on(false);
    }

    xTaskCreate(manager_task, "wifi_mgr", 3072, NULL, 3, NULL);
    return ESP_OK;
}

bool wifi_mgr_sta_up(void)
{
    return s_sta_up;
}

const char *wifi_mgr_ip(void)
{
    return s_ip;
}

bool wifi_mgr_ap_active(void)
{
    return s_ap_on;
}

void wifi_mgr_status_json(jbuf_t *jb)
{
    jb_bool(jb, "connected", s_sta_up);
    jb_str(jb, "ssid", s_have_creds ? s_ssid : "");
    jb_str(jb, "ip", s_ip);
    int rssi = 0;
    wifi_ap_record_t ap;
    if (s_sta_up && esp_wifi_sta_get_ap_info(&ap) == ESP_OK) {
        rssi = ap.rssi;
    }
    jb_int(jb, "rssi", rssi);
    jb_int(jb, "last_reason", s_last_reason);
    jb_str(jb, "last_error", reason_text(s_last_reason));
    jb_bool(jb, "setup_ap", s_ap_on);
    jb_str(jb, "setup_ap_ssid", s_ap_ssid);
    jb_str(jb, "setup_ap_ip", "192.168.4.1");
    jb_int(jb, "setup_ap_clients", s_ap_clients);
    jb_str(jb, "hostname", CONFIG_BRIDGE_HOSTNAME ".local");
}

static const char *auth_name(wifi_auth_mode_t a)
{
    switch (a) {
    case WIFI_AUTH_OPEN: return "open";
    case WIFI_AUTH_WEP: return "wep";
    case WIFI_AUTH_WPA_PSK: return "wpa";
    case WIFI_AUTH_WPA2_PSK: return "wpa2";
    case WIFI_AUTH_WPA_WPA2_PSK: return "wpa/wpa2";
    case WIFI_AUTH_WPA3_PSK: return "wpa3";
    case WIFI_AUTH_WPA2_WPA3_PSK: return "wpa2/wpa3";
    default: return "other";
    }
}

esp_err_t wifi_mgr_scan_json(jbuf_t *jb, const char *key)
{
    xSemaphoreTake(s_op, portMAX_DELAY);
    s_hold = true;

    const wifi_scan_config_t scan = { .show_hidden = false, .scan_type = WIFI_SCAN_TYPE_ACTIVE };
    esp_err_t err = esp_wifi_scan_start(&scan, true);
    if (err == ESP_ERR_WIFI_STATE && !s_sta_up) {
        /* The station is mid-connect; a scan is not allowed until it stops. */
        esp_wifi_disconnect();
        vTaskDelay(pdMS_TO_TICKS(200));
        err = esp_wifi_scan_start(&scan, true);
    }

    wifi_ap_record_t *recs = NULL;
    uint16_t n = SCAN_MAX;
    if (err == ESP_OK) {
        recs = calloc(SCAN_MAX, sizeof(*recs));
        if (recs == NULL) {
            esp_wifi_clear_ap_list();
            err = ESP_ERR_NO_MEM;
        } else {
            err = esp_wifi_scan_get_ap_records(&n, recs);
        }
    }

    s_hold = false;
    s_next_retry = now();
    xSemaphoreGive(s_op);

    if (err != ESP_OK) {
        free(recs);
        return err;
    }

    /* Records come sorted by RSSI; keep the strongest entry per SSID. */
    jb_arr_open(jb, key);
    for (uint16_t i = 0; i < n; i++) {
        const char *ssid = (const char *)recs[i].ssid;
        bool dup = ssid[0] == '\0';
        for (uint16_t j = 0; j < i && !dup; j++) {
            dup = strcmp(ssid, (const char *)recs[j].ssid) == 0;
        }
        if (dup) {
            continue;
        }
        jb_obj_open(jb, NULL);
        jb_str(jb, "ssid", ssid);
        jb_int(jb, "rssi", recs[i].rssi);
        jb_int(jb, "channel", recs[i].primary);
        jb_str(jb, "auth", auth_name(recs[i].authmode));
        jb_bool(jb, "supported", recs[i].authmode >= WIFI_AUTH_WPA2_PSK &&
                                 recs[i].authmode != WIFI_AUTH_WEP);
        jb_obj_close(jb);
    }
    jb_arr_close(jb);
    free(recs);
    return ESP_OK;
}

static void stop_station(void)
{
    if (s_sta_up || s_connecting) {
        xEventGroupClearBits(s_ev, BIT_DISCONNECTED);
        esp_wifi_disconnect();
        xEventGroupWaitBits(s_ev, BIT_DISCONNECTED, pdTRUE, pdFALSE, pdMS_TO_TICKS(2000));
    }
    s_connecting = false;
}

esp_err_t wifi_mgr_set(const char *ssid, const char *pass,
                       char *ip_out, size_t ip_len, const char **reason)
{
    *reason = "";
    if (ssid == NULL || ssid[0] == '\0' || strlen(ssid) > 32) {
        *reason = "ssid must be 1..32 characters";
        return ESP_ERR_INVALID_ARG;
    }
    if (pass == NULL || strlen(pass) < 8 || strlen(pass) > 63) {
        *reason = "password must be 8..63 characters (open networks are not allowed)";
        return ESP_ERR_INVALID_ARG;
    }

    xSemaphoreTake(s_op, portMAX_DELAY);
    s_hold = true;
    ESP_LOGI(TAG, "trying \"%s\"", ssid);

    stop_station();
    apply_sta(ssid, pass);

    bool ok = false;
    uint8_t reason_code = 0;
    const TickType_t deadline = now() + pdMS_TO_TICKS(SET_TIMEOUT_MS);
    for (int attempt = 0; attempt < 3; attempt++) {
        const TickType_t left = deadline - now();
        if ((int32_t)left <= 0) {
            break;
        }
        xEventGroupClearBits(s_ev, BIT_GOT_IP | BIT_DISCONNECTED);
        connect_now();
        EventBits_t bits = xEventGroupWaitBits(s_ev, BIT_GOT_IP | BIT_DISCONNECTED,
                                               pdTRUE, pdFALSE, left);
        if (bits & BIT_GOT_IP) {
            ok = true;
            break;
        }
        reason_code = s_last_reason;
        if (!(bits & BIT_DISCONNECTED)) {
            stop_station();   /* timed out while associating */
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(300));
    }

    esp_err_t err;
    if (ok) {
        err = settings_set_wifi(ssid, pass);
        strlcpy(s_ssid, ssid, sizeof(s_ssid));
        strlcpy(s_pass, pass, sizeof(s_pass));
        s_have_creds = true;
        strlcpy(ip_out, s_ip, ip_len);
        ESP_LOGI(TAG, "saved \"%s\"", ssid);
    } else {
        *reason = reason_code ? reason_text(reason_code) : "timed out waiting for an IP address";
        ESP_LOGW(TAG, "\"%s\" failed: %s; keeping the previous network", ssid, *reason);
        stop_station();
        if (s_have_creds) {
            apply_sta(s_ssid, s_pass);
        }
        err = ESP_FAIL;
    }

    s_retries = 0;
    s_next_retry = now();
    s_hold = false;
    xSemaphoreGive(s_op);
    return err;
}

esp_err_t wifi_mgr_forget(void)
{
    xSemaphoreTake(s_op, portMAX_DELAY);
    s_hold = true;
    esp_err_t err = settings_forget_wifi();
    s_have_creds = false;
    s_ssid[0] = '\0';
    s_pass[0] = '\0';
    stop_station();
    ap_on(false);
    s_hold = false;
    xSemaphoreGive(s_op);
    return err;
}

esp_err_t wifi_mgr_ap(bool on)
{
    esp_err_t err = ESP_OK;
    xSemaphoreTake(s_op, portMAX_DELAY);
    if (on) {
        ap_on(true);
    } else if (s_sta_up) {
        ap_off();
    } else {
        err = ESP_ERR_INVALID_STATE;   /* would leave no way in */
    }
    xSemaphoreGive(s_op);
    return err;
}
