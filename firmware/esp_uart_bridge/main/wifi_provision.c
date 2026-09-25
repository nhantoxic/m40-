#include "wifi_provision.h"

#include <ctype.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_check.h"
#include "esp_event.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs.h"
#include "nvs_flash.h"

#define PROV_AP_BASE_SSID      "bridge_esp_swd"
#define PROV_AP_PASSWORD       "bridge-swd"
#define PROV_AP_CHANNEL        1
#define PROV_AP_MAX_CLIENTS    4

#define PROV_NVS_NS            "bridgewifi"
#define PROV_NVS_SSID          "ssid"
#define PROV_NVS_PASS          "pass"

#define PROV_SCAN_MAX          24
#define PROV_HTTP_STACK        6144

static const char *TAG = "wifi_prov";

static esp_netif_t *s_ap_netif;
static esp_netif_t *s_sta_netif;
static httpd_handle_t s_httpd;

static bool s_wifi_started;
static bool s_have_credentials;
static volatile bool s_sta_got_ip;
static volatile bool s_reconnect_requested;
static unsigned s_disconnect_count;

static char s_saved_ssid[33];
static char s_saved_pass[65];
static char s_ap_ssid[33];
static char s_sta_ip[16] = "0.0.0.0";

static esp_event_handler_instance_t s_wifi_any_id;
static esp_event_handler_instance_t s_ip_got_ip;

static esp_err_t nvs_init_safe(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES ||
        err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    return err;
}

static esp_err_t net_stack_init_safe(void)
{
    esp_err_t err = esp_netif_init();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        return err;
    }

    err = esp_event_loop_create_default();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        return err;
    }
    return ESP_OK;
}

static bool load_credentials(void)
{
    memset(s_saved_ssid, 0, sizeof(s_saved_ssid));
    memset(s_saved_pass, 0, sizeof(s_saved_pass));

    nvs_handle_t nvs;
    esp_err_t err = nvs_open(PROV_NVS_NS, NVS_READONLY, &nvs);
    if (err != ESP_OK) {
        return false;
    }

    size_t ssid_len = sizeof(s_saved_ssid);
    size_t pass_len = sizeof(s_saved_pass);

    err = nvs_get_str(nvs, PROV_NVS_SSID, s_saved_ssid, &ssid_len);
    if (err == ESP_OK) {
        esp_err_t pe = nvs_get_str(nvs, PROV_NVS_PASS, s_saved_pass, &pass_len);
        if (pe == ESP_ERR_NVS_NOT_FOUND) {
            s_saved_pass[0] = '\0'; /* open network */
            pe = ESP_OK;
        }
        err = pe;
    }
    nvs_close(nvs);

    if (err != ESP_OK || s_saved_ssid[0] == '\0') {
        memset(s_saved_ssid, 0, sizeof(s_saved_ssid));
        memset(s_saved_pass, 0, sizeof(s_saved_pass));
        return false;
    }
    return true;
}

static esp_err_t save_credentials(const char *ssid, const char *pass)
{
    if (!ssid || ssid[0] == '\0' || strlen(ssid) > 32) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!pass) {
        pass = "";
    }
    if (strlen(pass) > 64) {
        return ESP_ERR_INVALID_ARG;
    }

    nvs_handle_t nvs;
    ESP_RETURN_ON_ERROR(nvs_open(PROV_NVS_NS, NVS_READWRITE, &nvs),
                        TAG, "nvs_open");

    esp_err_t err = nvs_set_str(nvs, PROV_NVS_SSID, ssid);
    if (err == ESP_OK) {
        err = nvs_set_str(nvs, PROV_NVS_PASS, pass);
    }
    if (err == ESP_OK) {
        err = nvs_commit(nvs);
    }
    nvs_close(nvs);

    if (err == ESP_OK) {
        strlcpy(s_saved_ssid, ssid, sizeof(s_saved_ssid));
        strlcpy(s_saved_pass, pass, sizeof(s_saved_pass));
        s_have_credentials = true;
    }
    return err;
}

esp_err_t wifi_provision_forget(void)
{
    nvs_handle_t nvs;
    esp_err_t err = nvs_open(PROV_NVS_NS, NVS_READWRITE, &nvs);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        s_have_credentials = false;
        s_saved_ssid[0] = '\0';
        s_saved_pass[0] = '\0';
        return ESP_OK;
    }
    if (err != ESP_OK) {
        return err;
    }

    err = nvs_erase_all(nvs);
    if (err == ESP_OK) {
        err = nvs_commit(nvs);
    }
    nvs_close(nvs);

    if (err == ESP_OK) {
        s_have_credentials = false;
        s_saved_ssid[0] = '\0';
        s_saved_pass[0] = '\0';
    }
    return err;
}

static void apply_sta_config(void)
{
    if (!s_have_credentials) {
        return;
    }

    wifi_config_t sta = {0};
    strlcpy((char *)sta.sta.ssid, s_saved_ssid, sizeof(sta.sta.ssid));
    strlcpy((char *)sta.sta.password, s_saved_pass, sizeof(sta.sta.password));
    sta.sta.scan_method = WIFI_ALL_CHANNEL_SCAN;
    sta.sta.sort_method = WIFI_CONNECT_AP_BY_SIGNAL;
    sta.sta.threshold.authmode =
        s_saved_pass[0] ? WIFI_AUTH_WPA2_PSK : WIFI_AUTH_OPEN;

    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &sta));
}

static void wifi_event(void *arg, esp_event_base_t base,
                       int32_t id, void *data)
{
    (void)arg;
    (void)data;

    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        if (s_have_credentials) {
            s_reconnect_requested = true;
        }
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        s_sta_got_ip = false;
        strlcpy(s_sta_ip, "0.0.0.0", sizeof(s_sta_ip));
        s_disconnect_count++;
        if (s_have_credentials) {
            s_reconnect_requested = true;
        }
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *ev = (ip_event_got_ip_t *)data;
        snprintf(s_sta_ip, sizeof(s_sta_ip), IPSTR, IP2STR(&ev->ip_info.ip));
        s_sta_got_ip = true;
        s_disconnect_count = 0;
        ESP_LOGI(TAG, "STA connected: ssid='%s' ip=%s",
                 s_saved_ssid, s_sta_ip);
    }
}

static void reconnect_task(void *arg)
{
    (void)arg;
    for (;;) {
        if (s_reconnect_requested && s_have_credentials && !s_sta_got_ip) {
            s_reconnect_requested = false;

            /* Keep retries gentle. AP provisioning stays online meanwhile. */
            unsigned delay_ms = 250;
            if (s_disconnect_count > 2) delay_ms = 1000;
            if (s_disconnect_count > 5) delay_ms = 3000;
            vTaskDelay(pdMS_TO_TICKS(delay_ms));

            esp_err_t err = esp_wifi_connect();
            if (err != ESP_OK) {
                ESP_LOGW(TAG, "esp_wifi_connect: %s", esp_err_to_name(err));
                s_reconnect_requested = true;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

static int hex_value(char c)
{
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
}

static void url_decode(char *dst, size_t dst_len, const char *src)
{
    size_t di = 0;
    while (*src && di + 1 < dst_len) {
        if (*src == '+') {
            dst[di++] = ' ';
            src++;
        } else if (*src == '%' &&
                   isxdigit((unsigned char)src[1]) &&
                   isxdigit((unsigned char)src[2])) {
            int hi = hex_value(src[1]);
            int lo = hex_value(src[2]);
            dst[di++] = (char)((hi << 4) | lo);
            src += 3;
        } else {
            dst[di++] = *src++;
        }
    }
    dst[di] = '\0';
}

static bool form_value(const char *body, const char *key,
                       char *out, size_t out_len)
{
    size_t key_len = strlen(key);
    const char *p = body;

    while (*p) {
        if ((p == body || p[-1] == '&') &&
            strncmp(p, key, key_len) == 0 &&
            p[key_len] == '=') {
            p += key_len + 1;
            const char *end = strchr(p, '&');
            size_t n = end ? (size_t)(end - p) : strlen(p);

            char tmp[256];
            if (n >= sizeof(tmp)) n = sizeof(tmp) - 1;
            memcpy(tmp, p, n);
            tmp[n] = '\0';
            url_decode(out, out_len, tmp);
            return true;
        }
        p = strchr(p, '&');
        if (!p) break;
        p++;
    }
    if (out_len) out[0] = '\0';
    return false;
}

static void html_escape(httpd_req_t *req, const char *s)
{
    for (; *s; ++s) {
        switch (*s) {
        case '&': httpd_resp_sendstr_chunk(req, "&amp;"); break;
        case '<': httpd_resp_sendstr_chunk(req, "&lt;"); break;
        case '>': httpd_resp_sendstr_chunk(req, "&gt;"); break;
        case '"': httpd_resp_sendstr_chunk(req, "&quot;"); break;
        default: {
            char c[2] = {*s, 0};
            httpd_resp_sendstr_chunk(req, c);
            break;
        }
        }
    }
}

static esp_err_t root_get(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");

    httpd_resp_sendstr_chunk(req,
        "<!doctype html><html><head>"
        "<meta name='viewport' content='width=device-width,initial-scale=1'>"
        "<title>ESP SWD Bridge Wi-Fi</title>"
        "<style>"
        "body{font-family:system-ui;max-width:620px;margin:24px auto;padding:0 16px}"
        "input,select,button{font-size:16px;padding:10px;margin:5px 0;width:100%;box-sizing:border-box}"
        "code{background:#eee;padding:2px 5px}.ok{color:#075}.muted{color:#666}"
        "</style></head><body>"
        "<h2>ESP SWD Bridge</h2>");

    char line[256];
    snprintf(line, sizeof(line),
             "<p>AP: <code>%s</code> &nbsp; Portal: <code>192.168.4.1</code></p>"
             "<p>STA: <b>%s</b> &nbsp; IP: <code>%s</code></p>",
             s_ap_ssid,
             s_sta_got_ip ? "connected" :
                 (s_have_credentials ? "connecting" : "not configured"),
             s_sta_ip);
    httpd_resp_sendstr_chunk(req, line);

    if (s_have_credentials) {
        httpd_resp_sendstr_chunk(req, "<p>Saved SSID: <b>");
        html_escape(req, s_saved_ssid);
        httpd_resp_sendstr_chunk(req, "</b></p>");
    }

    httpd_resp_sendstr_chunk(req,
        "<h3>Select Wi-Fi</h3>"
        "<form method='POST' action='/save'>"
        "<select name='ssid'><option value=''>-- scan --</option>");

    /*
     * Blocking scan is intentional here: it only runs when the user opens
     * the provisioning page. AP+STA mode supports scan.
     */
    wifi_scan_config_t scan = {
        .ssid = NULL,
        .bssid = NULL,
        .channel = 0,
        .show_hidden = true,
        .scan_type = WIFI_SCAN_TYPE_ACTIVE,
    };

    esp_err_t scan_err = esp_wifi_scan_start(&scan, true);
    if (scan_err == ESP_OK) {
        uint16_t count = PROV_SCAN_MAX;
        wifi_ap_record_t *aps = calloc(PROV_SCAN_MAX, sizeof(*aps));
        if (aps) {
            if (esp_wifi_scan_get_ap_records(&count, aps) == ESP_OK) {
                for (uint16_t i = 0; i < count; ++i) {
                    if (aps[i].ssid[0] == '\0') continue;
                    httpd_resp_sendstr_chunk(req, "<option value=\"");
                    html_escape(req, (const char *)aps[i].ssid);
                    httpd_resp_sendstr_chunk(req, "\">");
                    html_escape(req, (const char *)aps[i].ssid);
                    snprintf(line, sizeof(line), " (%d dBm)</option>",
                             aps[i].rssi);
                    httpd_resp_sendstr_chunk(req, line);
                }
            }
            free(aps);
        } else {
            /* Free driver scan list even on allocation failure. */
            esp_wifi_clear_ap_list();
        }
    } else {
        snprintf(line, sizeof(line),
                 "<option value=''>scan failed: %s</option>",
                 esp_err_to_name(scan_err));
        httpd_resp_sendstr_chunk(req, line);
    }

    httpd_resp_sendstr_chunk(req,
        "</select>"
        "<p class='muted'>Or type SSID manually:</p>"
        "<input name='ssid_manual' maxlength='32' placeholder='SSID'>"
        "<input name='pass' maxlength='64' type='password' placeholder='Wi-Fi password'>"
        "<button type='submit'>Save and connect</button>"
        "</form>"
        "<form method='POST' action='/reconnect'>"
        "<button type='submit'>Reconnect saved Wi-Fi</button></form>"
        "<form method='POST' action='/forget' "
        "onsubmit=\"return confirm('Forget saved Wi-Fi?')\">"
        "<button type='submit'>Forget Wi-Fi</button></form>"
        "<form method='POST' action='/reboot'>"
        "<button type='submit'>Reboot ESP</button></form>"
        "<p class='muted'>RST only reboots; saved Wi-Fi remains in NVS.</p>"
        "</body></html>");

    return httpd_resp_sendstr_chunk(req, NULL);
}

static esp_err_t read_body(httpd_req_t *req, char *buf, size_t cap)
{
    if (req->content_len <= 0 || (size_t)req->content_len >= cap) {
        return ESP_ERR_INVALID_SIZE;
    }
    int got = 0;
    while (got < req->content_len) {
        int n = httpd_req_recv(req, buf + got, req->content_len - got);
        if (n <= 0) {
            return ESP_FAIL;
        }
        got += n;
    }
    buf[got] = '\0';
    return ESP_OK;
}

static void respond_back(httpd_req_t *req, const char *message)
{
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    httpd_resp_sendstr_chunk(req,
        "<!doctype html><meta name='viewport' content='width=device-width,initial-scale=1'>"
        "<body style='font-family:system-ui;max-width:600px;margin:30px auto;padding:0 16px'>"
        "<p>");
    httpd_resp_sendstr_chunk(req, message);
    httpd_resp_sendstr_chunk(req,
        "</p><p><a href='/'>Back to Wi-Fi setup</a></p></body>");
    httpd_resp_sendstr_chunk(req, NULL);
}

static esp_err_t save_post(httpd_req_t *req)
{
    char body[384];
    esp_err_t err = read_body(req, body, sizeof(body));
    if (err != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "invalid form");
        return ESP_OK;
    }

    char picked[33], manual[33], pass[65];
    form_value(body, "ssid", picked, sizeof(picked));
    form_value(body, "ssid_manual", manual, sizeof(manual));
    form_value(body, "pass", pass, sizeof(pass));

    const char *ssid = manual[0] ? manual : picked;
    if (!ssid[0]) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "SSID required");
        return ESP_OK;
    }

    err = save_credentials(ssid, pass);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "save credentials: %s", esp_err_to_name(err));
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                            "NVS save failed");
        return ESP_OK;
    }

    ESP_LOGI(TAG, "new Wi-Fi credentials saved for SSID '%s'", ssid);

    (void)esp_wifi_disconnect();
    apply_sta_config();
    s_disconnect_count = 0;
    s_reconnect_requested = true;

    respond_back(req,
        "Saved. ESP is connecting now. The setup AP stays available.");
    return ESP_OK;
}

static esp_err_t forget_post(httpd_req_t *req)
{
    esp_err_t err = wifi_provision_forget();
    (void)esp_wifi_disconnect();
    if (err == ESP_OK) {
        respond_back(req, "Saved Wi-Fi credentials erased.");
    } else {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                            "NVS erase failed");
    }
    return ESP_OK;
}

static esp_err_t reconnect_post(httpd_req_t *req)
{
    if (s_have_credentials) {
        (void)esp_wifi_disconnect();
        apply_sta_config();
        s_reconnect_requested = true;
        respond_back(req, "Reconnect requested.");
    } else {
        respond_back(req, "No saved Wi-Fi credentials.");
    }
    return ESP_OK;
}

static esp_err_t status_get(httpd_req_t *req)
{
    char json[320];
    snprintf(json, sizeof(json),
             "{\"ap_ssid\":\"%s\",\"ap_ip\":\"192.168.4.1\","
             "\"saved\":%s,\"sta_ssid\":\"%s\",\"connected\":%s,"
             "\"sta_ip\":\"%s\",\"disconnects\":%u}",
             s_ap_ssid,
             s_have_credentials ? "true" : "false",
             s_have_credentials ? s_saved_ssid : "",
             s_sta_got_ip ? "true" : "false",
             s_sta_ip,
             s_disconnect_count);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_sendstr(req, json);
}

static void delayed_restart_task(void *arg)
{
    (void)arg;
    vTaskDelay(pdMS_TO_TICKS(400));
    esp_restart();
}

static esp_err_t reboot_post(httpd_req_t *req)
{
    respond_back(req, "Rebooting ESP...");
    xTaskCreate(delayed_restart_task, "prov_restart", 2048, NULL, 3, NULL);
    return ESP_OK;
}

static esp_err_t captive_redirect(httpd_req_t *req)
{
    httpd_resp_set_status(req, "302 Found");
    httpd_resp_set_hdr(req, "Location", "http://192.168.4.1/");
    return httpd_resp_send(req, NULL, 0);
}

static esp_err_t start_http(void)
{
    if (s_httpd) return ESP_OK;

    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.stack_size = PROV_HTTP_STACK;
    cfg.max_uri_handlers = 12;
    cfg.lru_purge_enable = true;

    ESP_RETURN_ON_ERROR(httpd_start(&s_httpd, &cfg), TAG, "httpd_start");

    const httpd_uri_t root = {
        .uri = "/", .method = HTTP_GET, .handler = root_get
    };
    const httpd_uri_t save = {
        .uri = "/save", .method = HTTP_POST, .handler = save_post
    };
    const httpd_uri_t forget = {
        .uri = "/forget", .method = HTTP_POST, .handler = forget_post
    };
    const httpd_uri_t reconnect = {
        .uri = "/reconnect", .method = HTTP_POST, .handler = reconnect_post
    };
    const httpd_uri_t reboot = {
        .uri = "/reboot", .method = HTTP_POST, .handler = reboot_post
    };
    const httpd_uri_t status = {
        .uri = "/status", .method = HTTP_GET, .handler = status_get
    };
    const httpd_uri_t android = {
        .uri = "/generate_204", .method = HTTP_GET, .handler = captive_redirect
    };
    const httpd_uri_t apple = {
        .uri = "/hotspot-detect.html", .method = HTTP_GET, .handler = captive_redirect
    };
    const httpd_uri_t windows = {
        .uri = "/connecttest.txt", .method = HTTP_GET, .handler = captive_redirect
    };
    const httpd_uri_t ncsi = {
        .uri = "/ncsi.txt", .method = HTTP_GET, .handler = captive_redirect
    };

    ESP_ERROR_CHECK(httpd_register_uri_handler(s_httpd, &root));
    ESP_ERROR_CHECK(httpd_register_uri_handler(s_httpd, &save));
    ESP_ERROR_CHECK(httpd_register_uri_handler(s_httpd, &forget));
    ESP_ERROR_CHECK(httpd_register_uri_handler(s_httpd, &reconnect));
    ESP_ERROR_CHECK(httpd_register_uri_handler(s_httpd, &reboot));
    ESP_ERROR_CHECK(httpd_register_uri_handler(s_httpd, &status));
    ESP_ERROR_CHECK(httpd_register_uri_handler(s_httpd, &android));
    ESP_ERROR_CHECK(httpd_register_uri_handler(s_httpd, &apple));
    ESP_ERROR_CHECK(httpd_register_uri_handler(s_httpd, &windows));
    ESP_ERROR_CHECK(httpd_register_uri_handler(s_httpd, &ncsi));

    ESP_LOGI(TAG, "portal: http://192.168.4.1/");
    return ESP_OK;
}

esp_err_t wifi_provision_start(void)
{
    if (s_wifi_started) {
        return ESP_OK;
    }

    ESP_RETURN_ON_ERROR(nvs_init_safe(), TAG, "NVS init");
    ESP_RETURN_ON_ERROR(net_stack_init_safe(), TAG, "net stack init");

    /*
     * Create netifs after esp_netif/event-loop init and before Wi-Fi driver init,
     * matching ESP-IDF's official station/AP examples.
     */
    s_sta_netif = esp_netif_create_default_wifi_sta();
    s_ap_netif = esp_netif_create_default_wifi_ap();
    if (!s_sta_netif || !s_ap_netif) {
        return ESP_FAIL;
    }

    wifi_init_config_t init = WIFI_INIT_CONFIG_DEFAULT();
    ESP_RETURN_ON_ERROR(esp_wifi_init(&init), TAG, "esp_wifi_init");

    /*
     * We keep bridge credentials in our own NVS namespace, so Wi-Fi driver
     * configuration itself can stay in RAM.
     */
    ESP_RETURN_ON_ERROR(esp_wifi_set_storage(WIFI_STORAGE_RAM),
                        TAG, "wifi storage");

    s_have_credentials = load_credentials();

    uint8_t mac[6] = {0};
    ESP_ERROR_CHECK(esp_read_mac(mac, ESP_MAC_WIFI_SOFTAP));
    snprintf(s_ap_ssid, sizeof(s_ap_ssid),
             "%s_%02X%02X", PROV_AP_BASE_SSID, mac[4], mac[5]);

    wifi_config_t ap = {0};
    strlcpy((char *)ap.ap.ssid, s_ap_ssid, sizeof(ap.ap.ssid));
    ap.ap.ssid_len = strlen(s_ap_ssid);
    strlcpy((char *)ap.ap.password, PROV_AP_PASSWORD,
            sizeof(ap.ap.password));
    ap.ap.channel = PROV_AP_CHANNEL;
    ap.ap.max_connection = PROV_AP_MAX_CLIENTS;
    ap.ap.authmode = WIFI_AUTH_WPA2_PSK;
    ap.ap.pmf_cfg.required = false;

    ESP_RETURN_ON_ERROR(
        esp_event_handler_instance_register(
            WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event, NULL, &s_wifi_any_id),
        TAG, "register WIFI event");

    ESP_RETURN_ON_ERROR(
        esp_event_handler_instance_register(
            IP_EVENT, IP_EVENT_STA_GOT_IP, wifi_event, NULL, &s_ip_got_ip),
        TAG, "register IP event");

    ESP_RETURN_ON_ERROR(esp_wifi_set_mode(WIFI_MODE_APSTA),
                        TAG, "WIFI_MODE_APSTA");
    ESP_RETURN_ON_ERROR(esp_wifi_set_config(WIFI_IF_AP, &ap),
                        TAG, "AP config");

    if (s_have_credentials) {
        apply_sta_config();
    }

    ESP_RETURN_ON_ERROR(esp_wifi_start(), TAG, "esp_wifi_start");
    s_wifi_started = true;

    xTaskCreate(reconnect_task, "wifi_reconnect", 3072, NULL, 4, NULL);

    ESP_RETURN_ON_ERROR(start_http(), TAG, "portal");

    ESP_LOGI(TAG, "provision AP: ssid='%s' pass='%s' ip=192.168.4.1",
             s_ap_ssid, PROV_AP_PASSWORD);

    if (s_have_credentials) {
        ESP_LOGI(TAG, "saved STA SSID='%s'; auto-connect enabled",
                 s_saved_ssid);
        s_reconnect_requested = true;
    } else {
        ESP_LOGW(TAG, "no saved STA Wi-Fi; connect to '%s' and open "
                      "http://192.168.4.1/", s_ap_ssid);
    }

    return ESP_OK;
}
