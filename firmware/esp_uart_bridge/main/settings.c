#include "settings.h"

#include <string.h>

#include "esp_log.h"
#include "nvs.h"

static const char *TAG = "settings";

#define NS          "bridge"
#define KEY_SSID    "wifi_ssid"
#define KEY_PASS    "wifi_pass"
#define KEY_BAUD    "uart_baud"
#define KEY_SOC_BAUD "soc_baud"

#ifndef CONFIG_BRIDGE_SOC_UART_BAUD
#define CONFIG_BRIDGE_SOC_UART_BAUD 115200
#endif

static char s_ssid[33];
static char s_pass[65];
static bool s_wifi_saved;   /* NVS holds a value, possibly "" (= forgotten) */
static int s_baud[2] = { CONFIG_BRIDGE_UART_BAUD, CONFIG_BRIDGE_SOC_UART_BAUD };
static const char *const BAUD_KEYS[2] = { KEY_BAUD, KEY_SOC_BAUD };

void settings_load(void)
{
    nvs_handle_t h;
    if (nvs_open(NS, NVS_READONLY, &h) != ESP_OK) {
        return;   /* nothing saved yet */
    }

    size_t len = sizeof(s_ssid);
    if (nvs_get_str(h, KEY_SSID, s_ssid, &len) == ESP_OK) {
        s_wifi_saved = true;
        len = sizeof(s_pass);
        if (nvs_get_str(h, KEY_PASS, s_pass, &len) != ESP_OK) {
            s_pass[0] = '\0';
        }
    }

    for (int ch = 0; ch < 2; ch++) {
        int32_t baud = 0;
        if (nvs_get_i32(h, BAUD_KEYS[ch], &baud) == ESP_OK && baud >= 1200 && baud <= 5000000) {
            s_baud[ch] = (int)baud;
        }
    }
    nvs_close(h);

    ESP_LOGI(TAG, "wifi=%s mcu_baud=%d soc_baud=%d",
             s_wifi_saved ? (s_ssid[0] ? "saved" : "forgotten") : "menuconfig",
             s_baud[0], s_baud[1]);
}

bool settings_wifi(char ssid[33], char pass[65])
{
    if (s_wifi_saved) {
        strlcpy(ssid, s_ssid, 33);
        strlcpy(pass, s_pass, 65);
        return s_ssid[0] != '\0';
    }
    /* Never saved: use the build-time credentials if they are real. */
    if (CONFIG_BRIDGE_WIFI_SSID[0] == '\0' ||
        strcmp(CONFIG_BRIDGE_WIFI_SSID, "changeme") == 0 ||
        strlen(CONFIG_BRIDGE_WIFI_PASSWORD) < 8 ||
        strcmp(CONFIG_BRIDGE_WIFI_PASSWORD, "changeme") == 0) {
        return false;
    }
    strlcpy(ssid, CONFIG_BRIDGE_WIFI_SSID, 33);
    strlcpy(pass, CONFIG_BRIDGE_WIFI_PASSWORD, 65);
    return true;
}

static esp_err_t write_wifi(const char *ssid, const char *pass)
{
    nvs_handle_t h;
    esp_err_t err = nvs_open(NS, NVS_READWRITE, &h);
    if (err != ESP_OK) {
        return err;
    }
    err = nvs_set_str(h, KEY_SSID, ssid);
    if (err == ESP_OK) {
        err = nvs_set_str(h, KEY_PASS, pass);
    }
    if (err == ESP_OK) {
        err = nvs_commit(h);
    }
    nvs_close(h);
    if (err == ESP_OK) {
        strlcpy(s_ssid, ssid, sizeof(s_ssid));
        strlcpy(s_pass, pass, sizeof(s_pass));
        s_wifi_saved = true;
    }
    return err;
}

esp_err_t settings_set_wifi(const char *ssid, const char *pass)
{
    if (ssid == NULL || ssid[0] == '\0' || strlen(ssid) > 32 ||
        pass == NULL || strlen(pass) > 64) {
        return ESP_ERR_INVALID_ARG;
    }
    return write_wifi(ssid, pass);
}

esp_err_t settings_forget_wifi(void)
{
    return write_wifi("", "");
}

int settings_uart_baud(int ch)
{
    return (ch == 1) ? s_baud[1] : s_baud[0];
}

esp_err_t settings_set_uart_baud(int ch, int baud)
{
    ch = (ch == 1) ? 1 : 0;
    nvs_handle_t h;
    esp_err_t err = nvs_open(NS, NVS_READWRITE, &h);
    if (err != ESP_OK) {
        return err;
    }
    err = nvs_set_i32(h, BAUD_KEYS[ch], baud);
    if (err == ESP_OK) {
        err = nvs_commit(h);
    }
    nvs_close(h);
    if (err == ESP_OK) {
        s_baud[ch] = baud;
    }
    return err;
}
