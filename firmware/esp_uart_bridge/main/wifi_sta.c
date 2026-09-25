/* Wi-Fi station bring-up with unattended reconnect.
 *
 * The bridge is meant to sit inside a robot vacuum with no user interface, so
 * the only acceptable failure mode is "keeps trying". After CONFIG_BRIDGE_WIFI_MAX_RETRY
 * consecutive failures we back off for 30 s instead of hammering the AP, then
 * resume. There is deliberately no provisioning portal and no AP fallback:
 * an open AP bridged to a root console is a worse problem than a typo in the SSID.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"

#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"

#include "wifi_sta.h"

static const char *TAG = "wifi";

#define WIFI_CONNECTED_BIT BIT0

static EventGroupHandle_t s_events;
static char s_ip[16] = "0.0.0.0";
static int s_retries;
static volatile bool s_up;
static volatile bool s_allow_connect;
static bool s_using_fallback;

#define SSID_FALLBACK CONFIG_BRIDGE_WIFI_SSID_FALLBACK

static void apply_ssid(const char *ssid, bool fallback)
{
    wifi_config_t cfg = { 0 };

    strlcpy((char *)cfg.sta.ssid, ssid, sizeof(cfg.sta.ssid));
    strlcpy((char *)cfg.sta.password, CONFIG_BRIDGE_WIFI_PASSWORD,
            sizeof(cfg.sta.password));
    /* Never silently fall back to an open network: this bridge reaches the
     * robot's MCU console and must only join a password-protected AP. */
    cfg.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;

    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &cfg));
    s_using_fallback = fallback;
}

static void backoff_task(void *arg)
{
    (void)arg;

    if (SSID_FALLBACK[0] != '\0' && !s_using_fallback) {
        ESP_LOGW(TAG, "primary SSID unreachable, switching to fallback \"%s\"",
                 SSID_FALLBACK);
        apply_ssid(SSID_FALLBACK, true);
        s_retries = 0;
        esp_wifi_connect();
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGW(TAG, "%d consecutive failures, backing off for 30 s", s_retries);
    vTaskDelay(pdMS_TO_TICKS(30000));
    s_retries = 0;
    esp_wifi_connect();
    vTaskDelete(NULL);
}

/* Logs every AP in range once at boot. When the configured SSID is wrong or a
 * name only exists at another site, this is the difference between "the board
 * is broken" and "the board cannot see that AP from inside the robot". */
static void log_visible_aps(void)
{
    wifi_scan_config_t scan = { .show_hidden = true };
    uint16_t count = 0;

    if (esp_wifi_scan_start(&scan, true) != ESP_OK) {
        ESP_LOGW(TAG, "scan failed to start");
        return;
    }
    esp_wifi_scan_get_ap_num(&count);
    if (count == 0) {
        ESP_LOGW(TAG, "scan: no AP visible");
        return;
    }
    if (count > 20) {
        count = 20;
    }

    wifi_ap_record_t *records = calloc(count, sizeof(*records));
    if (records == NULL) {
        return;
    }
    if (esp_wifi_scan_get_ap_records(&count, records) == ESP_OK) {
        ESP_LOGI(TAG, "scan: %u AP(s) visible", (unsigned)count);
        for (uint16_t i = 0; i < count; i++) {
            ESP_LOGI(TAG, "  ssid=\"%s\" rssi=%d ch=%u auth=%d",
                     (const char *)records[i].ssid, records[i].rssi,
                     records[i].primary, (int)records[i].authmode);
        }
    }
    free(records);
}

static void on_wifi_event(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    (void)arg;
    (void)base;
    (void)data;

    switch (id) {
    case WIFI_EVENT_STA_START:
        /* Held off until the boot scan finishes, otherwise the connect races
         * the scan and the scan either fails or returns nothing. */
        if (s_allow_connect) {
            esp_wifi_connect();
        }
        break;

    case WIFI_EVENT_STA_DISCONNECTED:
        s_up = false;
        strcpy(s_ip, "0.0.0.0");
        xEventGroupClearBits(s_events, WIFI_CONNECTED_BIT);

        if (++s_retries <= CONFIG_BRIDGE_WIFI_MAX_RETRY) {
            ESP_LOGI(TAG, "disconnected, retry %d/%d", s_retries, CONFIG_BRIDGE_WIFI_MAX_RETRY);
            esp_wifi_connect();
        } else {
            /* Delay on a separate task: blocking the event loop would stall
             * every other esp_event consumer, including the IP stack. */
            xTaskCreate(backoff_task, "wifi_backoff", 2048, NULL, 4, NULL);
        }
        break;

    default:
        break;
    }
}

static void on_got_ip(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    (void)arg;
    (void)base;
    (void)id;

    const ip_event_got_ip_t *event = (const ip_event_got_ip_t *)data;

    snprintf(s_ip, sizeof(s_ip), IPSTR, IP2STR(&event->ip_info.ip));
    s_retries = 0;
    s_up = true;

    ESP_LOGI(TAG, "connected, ip=%s", s_ip);
    xEventGroupSetBits(s_events, WIFI_CONNECTED_BIT);
}

esp_err_t wifi_sta_start_and_wait(void)
{
    const size_t password_len = strlen(CONFIG_BRIDGE_WIFI_PASSWORD);
    if (password_len < 8 || strcmp(CONFIG_BRIDGE_WIFI_PASSWORD, "changeme") == 0) {
        ESP_LOGE(TAG, "set a real Wi-Fi password (at least 8 characters; 'changeme' is rejected)");
        return ESP_ERR_INVALID_ARG;
    }

    s_events = xEventGroupCreate();
    if (s_events == NULL) {
        return ESP_ERR_NO_MEM;
    }

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    esp_netif_t *netif = esp_netif_create_default_wifi_sta();
    ESP_ERROR_CHECK(esp_netif_set_hostname(netif, CONFIG_BRIDGE_HOSTNAME));

    wifi_init_config_t init = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&init));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, on_wifi_event, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, on_got_ip, NULL, NULL));

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    apply_ssid(CONFIG_BRIDGE_WIFI_SSID, false);

    /* Disable modem sleep: this bridge carries a latency-sensitive raw console
     * and SWD memory reads, where the few ms of modem-sleep wake latency cause
     * sluggish round-trips and occasional dropped/aborted short transactions.
     * Idle current rises modestly, but the board runs off the robot's supply,
     * so responsiveness and link reliability win. (Was WIFI_PS_MIN_MODEM.) */
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
    ESP_ERROR_CHECK(esp_wifi_start());

    log_visible_aps();

    s_allow_connect = true;
    ESP_LOGI(TAG, "joining \"%s\"", CONFIG_BRIDGE_WIFI_SSID);
    esp_wifi_connect();
    xEventGroupWaitBits(s_events, WIFI_CONNECTED_BIT, pdFALSE, pdTRUE, portMAX_DELAY);

    return ESP_OK;
}

const char *wifi_sta_ip(void)
{
    return s_ip;
}

bool wifi_sta_is_up(void)
{
    return s_up;
}
