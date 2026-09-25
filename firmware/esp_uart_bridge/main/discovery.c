/* Small, read-only LAN discovery responder for the RobotMonitor bridge. */

#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "lwip/inet.h"
#include "lwip/sockets.h"

#include "discovery.h"
#include "uart_tcp_bridge.h"
#include "wifi_mgr.h"

static const char *TAG = "discovery";
static const char QUERY[] = "DREAME_BRIDGE_DISCOVER";

static void discovery_task(void *arg)
{
    (void)arg;

    int fd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (fd < 0) {
        ESP_LOGE(TAG, "socket failed");
        vTaskDelete(NULL);
        return;
    }

    int reuse = 1;
    setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

    struct sockaddr_in local = { 0 };
    local.sin_family = AF_INET;
    local.sin_port = htons(CONFIG_BRIDGE_DISCOVERY_PORT);
    local.sin_addr.s_addr = htonl(INADDR_ANY);
    if (bind(fd, (struct sockaddr *)&local, sizeof(local)) < 0) {
        ESP_LOGE(TAG, "bind UDP %d failed", CONFIG_BRIDGE_DISCOVERY_PORT);
        close(fd);
        vTaskDelete(NULL);
        return;
    }

    struct timeval timeout = { .tv_sec = 1, .tv_usec = 0 };
    setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
    ESP_LOGI(TAG, "listening UDP %d", CONFIG_BRIDGE_DISCOVERY_PORT);

    for (;;) {
        char request[96] = { 0 };
        struct sockaddr_in peer = { 0 };
        socklen_t peer_len = sizeof(peer);
        int received = recvfrom(fd, request, sizeof(request) - 1, 0,
                                (struct sockaddr *)&peer, &peer_len);
        if (received < (int)sizeof(QUERY) - 1 ||
            memcmp(request, QUERY, sizeof(QUERY) - 1) != 0) {
            continue;
        }

        uint8_t mac[6] = { 0 };
        esp_read_mac(mac, ESP_MAC_WIFI_STA);

        /* Report the MCU UART when dual-UART mode is enabled. */
#if CONFIG_BRIDGE_MCU_UART_ENABLE
        const int mcu_port = CONFIG_BRIDGE_MCU_TCP_PORT;
        const int report_channel = 1;
        const int report_baud = CONFIG_BRIDGE_MCU_UART_BAUD;
        const int report_tx_gpio = CONFIG_BRIDGE_MCU_UART_TX_GPIO;
        const int report_rx_gpio = CONFIG_BRIDGE_MCU_UART_RX_GPIO;
#ifdef CONFIG_BRIDGE_MCU_UART_EVEN_PARITY
        const char *uart_mode = "8E1";
#else
        const char *uart_mode = "8N1";
#endif
#else
        const int mcu_port = CONFIG_BRIDGE_TCP_PORT;
        const int report_channel = 0;
        const int report_baud = uart_tcp_bridge_baud();
        const int report_tx_gpio = CONFIG_BRIDGE_UART_TX_GPIO;
        const int report_rx_gpio = CONFIG_BRIDGE_UART_RX_GPIO;
#ifdef CONFIG_BRIDGE_UART_EVEN_PARITY
        const char *uart_mode = "8E1";
#else
        const char *uart_mode = "8N1";
#endif
#endif

        uart_tcp_bridge_stats_t stats = { 0 };
        uart_tcp_bridge_get_stats(report_channel, &stats);

        char response[512];
        int length = snprintf(
            response, sizeof(response),
            "{\"protocol\":\"dreame-bridge-discovery/1\","
            "\"name\":\"%s\",\"ip\":\"%s\","
            "\"mac\":\"%02X:%02X:%02X:%02X:%02X:%02X\","
            "\"uart_port\":%d,\"mcu_port\":%d,\"uart_mode\":\"%s\","
            "\"uart_baud\":%d,\"tx_gpio\":%d,\"rx_gpio\":%d,"
            "\"uart_tx_level\":%d,\"uart_rx_level\":%d,"
            "\"swd_port\":%d,\"bitbang_port\":%d,\"http_port\":80,\"setup_ap\":%s,"
            "\"uart_stats\":{\"rx_bytes\":%u,\"tx_bytes\":%u,"
            "\"frame_err\":%u,\"parity_err\":%u,\"break\":%u,"
            "\"fifo_ovf\":%u,\"buf_full\":%u}}\n",
            CONFIG_BRIDGE_HOSTNAME,
            wifi_mgr_sta_up() ? wifi_mgr_ip() : (wifi_mgr_ap_active() ? "192.168.4.1" : "0.0.0.0"),
            mac[0], mac[1], mac[2], mac[3], mac[4], mac[5],
            CONFIG_BRIDGE_TCP_PORT, mcu_port, uart_mode,
            report_baud,
            report_tx_gpio, report_rx_gpio,
            gpio_get_level(report_tx_gpio),
            gpio_get_level(report_rx_gpio),
            CONFIG_BRIDGE_SWD_TCP_PORT,
            CONFIG_BRIDGE_SWD_REMOTE_BITBANG_TCP_PORT,
            wifi_mgr_ap_active() ? "true" : "false",
            (unsigned)stats.rx_bytes, (unsigned)stats.tx_bytes,
            (unsigned)stats.frame_err, (unsigned)stats.parity_err,
            (unsigned)stats.break_evt, (unsigned)stats.fifo_ovf,
            (unsigned)stats.buf_full);
        if (length > 0 && length < (int)sizeof(response)) {
            sendto(fd, response, length, 0, (struct sockaddr *)&peer, peer_len);
        }
    }
}

void discovery_start(void)
{
    BaseType_t result = xTaskCreate(discovery_task, "discovery", 3072, NULL, 3, NULL);
    if (result != pdPASS) {
        ESP_LOGE(TAG, "cannot create discovery task");
    }
}
