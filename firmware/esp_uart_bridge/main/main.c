/* Dreame robot debug bridge (ESP32-S2 / ESP32-S3).
 *
 * Wi-Fi <-> robot UART and SWD:
 *   http://<bridge>/     web app (Wi-Fi setup, terminal, commands, SWD)
 *   POST /api/cmd        command API shared with the PC tool and AI agents
 *   tcp/2324             raw UART (RobotMonitor)
 *   tcp/2325, tcp/3335   SWD text API, OpenOCD remote_bitbang
 *   udp/2326             LAN discovery
 * Wi-Fi is configured at runtime through the setup AP (DreameBridge-XXXX) or
 * "wifi.set"; see README.md.
 *
 * SECURITY: nothing on the LAN side is authenticated beyond the Wi-Fi itself.
 * Keep it on a trusted network or an isolated VLAN.
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
#if CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG
#include "driver/usb_serial_jtag.h"
#include "driver/usb_serial_jtag_vfs.h"
#elif CONFIG_ESP_CONSOLE_UART
#include "driver/uart.h"
#include "driver/uart_vfs.h"
#endif

#include "cmd.h"
#include "discovery.h"
#include "settings.h"
#include "swd_bridge.h"
#include "uart_tcp_bridge.h"
#include "web.h"
#include "wifi_mgr.h"

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

/* The USB console also carries "@CMD"/"@SWD" requests. USB-Serial-JTAG (S3)
 * and UART consoles need their driver for blocking line reads from stdin; the
 * S2 ROM CDC console works without one. */
static void console_input_init(void)
{
#if CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG
    usb_serial_jtag_driver_config_t cfg = USB_SERIAL_JTAG_DRIVER_CONFIG_DEFAULT();
    if (usb_serial_jtag_driver_install(&cfg) == ESP_OK) {
        usb_serial_jtag_vfs_use_driver();
    }
#elif CONFIG_ESP_CONSOLE_UART && !CONFIG_BRIDGE_MCU_UART_ENABLE
    if (uart_driver_install(CONFIG_ESP_CONSOLE_UART_NUM, 512, 0, 0, NULL, 0) == ESP_OK) {
        uart_vfs_dev_use_driver(CONFIG_ESP_CONSOLE_UART_NUM);
    }
#endif
}

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
    mdns_service_add(NULL, "_http", "_tcp", 80, NULL, 0);
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
 *   double blink         - setup AP is up (connect to DreameBridge-XXXX)
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

        if (wifi_mgr_ap_active()) {
            for (int i = 0; i < 2; i++) {
                gpio_set_level(CONFIG_BRIDGE_LED_GPIO, 1);
                vTaskDelay(pdMS_TO_TICKS(80));
                gpio_set_level(CONFIG_BRIDGE_LED_GPIO, 0);
                vTaskDelay(pdMS_TO_TICKS(120));
            }
            vTaskDelay(pdMS_TO_TICKS(600));
            continue;
        }

        on = !on;
        gpio_set_level(CONFIG_BRIDGE_LED_GPIO, on);
        vTaskDelay(pdMS_TO_TICKS(wifi_mgr_sta_up() ? 1000 : 100));
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

    settings_load();
    console_input_init();

    /* Wi-Fi does not block: the services listen on every interface, so they
     * work through the setup AP as well as the home network. */
    ESP_ERROR_CHECK(wifi_mgr_start());

#if CONFIG_BRIDGE_LED_GPIO >= 0
    xTaskCreate(led_task, "led", 2048, NULL, 2, NULL);
#endif
#if CONFIG_BRIDGE_MDNS_ENABLE
    mdns_start();
#endif
    uart_tcp_bridge_start();
    cmd_start();
#if CONFIG_BRIDGE_SWD_ENABLE
    swd_bridge_start();
#endif
#if CONFIG_BRIDGE_DISCOVERY_ENABLE
    discovery_start();
#endif
    web_start();

    ESP_LOGI(TAG, "ready: app http://%s.local/, raw UART tcp/%d, SWD tcp/%d",
             CONFIG_BRIDGE_HOSTNAME, CONFIG_BRIDGE_TCP_PORT, CONFIG_BRIDGE_SWD_TCP_PORT);
    ESP_LOGW(TAG, "LAN services have no application password; use a trusted network");
}
