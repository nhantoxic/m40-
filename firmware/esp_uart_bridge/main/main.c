/* Dreame robot debug bridge - phase P0.
 *
 * Exposes the selected robot UART on TCP. In the temporary S2 mini profile the
 * primary UART is the MCU parameter UART on GPIO35/GPIO33; GPIO16/GPIO18 are
 * reserved for the future SWD path. Nothing else: no USB host, no ADB, no
 * authentication. See prototype/s2_mini/WIRING.md before connecting this to a
 * robot, and hardware/rev_a/DESIGN_REVIEW.md for the current limitations.
 *
 * SECURITY: the bridge requires a WPA2-protected Wi-Fi network, but the raw
 * TCP console itself has no second application password. Keep it unpowered
 * when not in use, or put it on an isolated VLAN.
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_system.h"
#include "nvs_flash.h"
#if CONFIG_BRIDGE_MDNS_ENABLE
#include "mdns.h"
#endif

#include "uart_tcp_bridge.h"
#include "swd_bridge.h"
#include "discovery.h"
#include "wifi_sta.h"

static const char *TAG = "main";

/* The board routes SWD and the temporary SoC UART profile to the same pins.
 * Do not let a menuconfig change silently start both peripherals on one net. */
#if defined(CONFIG_BRIDGE_SWD_ENABLE) && CONFIG_BRIDGE_SWD_ENABLE
#if CONFIG_BRIDGE_UART_TX_GPIO == CONFIG_BRIDGE_SWDIO_GPIO || \
    CONFIG_BRIDGE_UART_TX_GPIO == CONFIG_BRIDGE_SWCLK_GPIO || \
    CONFIG_BRIDGE_UART_RX_GPIO == CONFIG_BRIDGE_SWDIO_GPIO || \
    CONFIG_BRIDGE_UART_RX_GPIO == CONFIG_BRIDGE_SWCLK_GPIO
#error "Primary UART GPIOs overlap SWD; move the UART pins or disable SWD"
#endif
#if defined(CONFIG_BRIDGE_MCU_UART_ENABLE) && CONFIG_BRIDGE_MCU_UART_ENABLE
#if CONFIG_BRIDGE_MCU_UART_TX_GPIO == CONFIG_BRIDGE_SWDIO_GPIO || \
    CONFIG_BRIDGE_MCU_UART_TX_GPIO == CONFIG_BRIDGE_SWCLK_GPIO || \
    CONFIG_BRIDGE_MCU_UART_RX_GPIO == CONFIG_BRIDGE_SWDIO_GPIO || \
    CONFIG_BRIDGE_MCU_UART_RX_GPIO == CONFIG_BRIDGE_SWCLK_GPIO
#error "MCU UART GPIOs overlap SWD; move the UART pins or disable SWD"
#endif
#endif
#endif

#if CONFIG_BRIDGE_MDNS_ENABLE
/* <hostname>.local survives DHCP handing out a new address. Not fatal: the
 * UDP discovery responder still works without it. */
static void mdns_start(void)
{
    esp_err_t err = mdns_init();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "mdns_init failed: %s", esp_err_to_name(err));
        return;
    }
    mdns_hostname_set(CONFIG_BRIDGE_HOSTNAME);
    mdns_instance_name_set("Dreame UART/SWD bridge");
    mdns_service_add(NULL, "_dreame-bridge", "_tcp", CONFIG_BRIDGE_TCP_PORT, NULL, 0);
#if CONFIG_BRIDGE_SWD_ENABLE
    mdns_service_add(NULL, "_dreame-swd", "_tcp", CONFIG_BRIDGE_SWD_TCP_PORT, NULL, 0);
#endif
    ESP_LOGI(TAG, "mDNS: %s.local", CONFIG_BRIDGE_HOSTNAME);
}
#endif

#if CONFIG_BRIDGE_LED_GPIO >= 0

/* Status at a glance, since the board lives under a robot's top cover:
 *   fast blink (100 ms)  - joining Wi-Fi, or the link dropped
 *   slow blink (1000 ms) - on the network, idle, waiting for a client
 *   solid on             - a client holds the console
 */
static void led_task(void *arg)
{
    (void)arg;

    const gpio_config_t io = {
        .pin_bit_mask = 1ULL << CONFIG_BRIDGE_LED_GPIO,
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&io));

    bool on = false;

    for (;;) {
        if (uart_tcp_bridge_has_client()) {
            gpio_set_level(CONFIG_BRIDGE_LED_GPIO, 1);
            vTaskDelay(pdMS_TO_TICKS(200));
            continue;
        }

        on = !on;
        gpio_set_level(CONFIG_BRIDGE_LED_GPIO, on);
        vTaskDelay(pdMS_TO_TICKS(wifi_sta_is_up() ? 1000 : 100));
    }
}
#endif /* CONFIG_BRIDGE_LED_GPIO >= 0 */

void app_main(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

#if CONFIG_BRIDGE_LED_GPIO >= 0
    xTaskCreate(led_task, "led", 2048, NULL, 2, NULL);
#endif

    ESP_ERROR_CHECK(wifi_sta_start_and_wait());
#if CONFIG_BRIDGE_MDNS_ENABLE
    mdns_start();
#endif
    uart_tcp_bridge_start();
#if CONFIG_BRIDGE_SWD_ENABLE
    swd_bridge_start();
#endif
#if CONFIG_BRIDGE_DISCOVERY_ENABLE
    discovery_start();
#endif

    ESP_LOGI(TAG, "ready:  nc %s %d", wifi_sta_ip(), CONFIG_BRIDGE_TCP_PORT);
    ESP_LOGW(TAG, "raw TCP console has no application password or encryption");
}
