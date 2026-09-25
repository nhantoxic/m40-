#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Configure the MCU UART and, when enabled, the SoC UART, then start one TCP
 * server per channel. Never returns an error to the caller: any fault is fatal
 * and asserted, because a half-started bridge is worse than a reboot loop you
 * can see. */
void uart_tcp_bridge_start(void);

/* True while a TCP client holds the console. Used to drive the status LED. */
bool uart_tcp_bridge_has_client(void);

/* Counters that answer "did anything ever arrive on this UART?".
 *
 * rx_bytes > 0 means the peer is electrically alive and the bridge is sampling
 * it. frame_err/parity_err rising with rx_bytes near zero is the signature of a
 * baud or parity mismatch: the line toggles, but not at the framing we expect.
 * All zero means the line never carried a start bit, so the fault is wiring,
 * power, or the peer never transmitting - not the parser and not the protocol. */
typedef struct {
    uint32_t rx_bytes;
    uint32_t tx_bytes;
    uint32_t frame_err;
    uint32_t parity_err;
    uint32_t break_evt;
    uint32_t fifo_ovf;
    uint32_t buf_full;
} uart_tcp_bridge_stats_t;

/* Channels. MCU is always present; SoC exists when CONFIG_BRIDGE_SOC_UART_ENABLE. */
#define BRIDGE_CH_MCU    0   /* robot MCU UART, tcp/CONFIG_BRIDGE_TCP_PORT */
#define BRIDGE_CH_SOC    1   /* robot SoC Linux shell, tcp/CONFIG_BRIDGE_SOC_TCP_PORT */
#define BRIDGE_CH_COUNT  2

bool uart_tcp_bridge_enabled(int ch);
bool uart_tcp_bridge_channel_has_client(int ch);
int uart_tcp_bridge_tcp_port(int ch);

/* Zeroes the output for a channel that is not built in. */
void uart_tcp_bridge_get_stats(int ch, uart_tcp_bridge_stats_t *out);

/* Baud rate the channel is running at (saved setting, else Kconfig default). */
int uart_tcp_bridge_baud(int ch);

/* Changes a channel's baud immediately (not persisted here). */
esp_err_t uart_tcp_bridge_set_baud(int ch, int baud);

/* Writes to a channel's UART from the app/command path. Returns bytes queued, -1 if absent. */
int uart_tcp_bridge_write(int ch, const uint8_t *data, size_t len);

/* Called from the UART reader tasks with every chunk received on any channel,
 * whether or not a TCP client is connected. Keep it short. */
typedef void (*uart_tcp_bridge_tap_t)(int ch, const uint8_t *data, size_t len);
void uart_tcp_bridge_add_tap(uart_tcp_bridge_tap_t tap);

#ifdef __cplusplus
}
#endif
