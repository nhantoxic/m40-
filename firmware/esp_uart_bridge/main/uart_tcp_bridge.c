/* Raw TCP <-> UART pumps for the robot SoC console and MCU parameter UART.
 *
 * Each channel owns its UART and listening socket. The byte stream is kept
 * deliberately raw: no Telnet negotiation, CR/LF translation, framing, or
 * command rewriting. This is important for both the root console and the
 * RobotMonitor protocol.
 *
 * The ESP console is native USB CDC, so UART0 is available for the MCU channel.
 * The primary channel uses UART1; in the temporary S2 mini profile it is
 * routed to the MCU UART pins instead of the SoC pins.
 *
 * Two tasks per channel, both fully blocking (no polling):
 *   uart_rx task  - sole reader of the UART. Wakes on the driver's event queue,
 *                   drains the RX ring and forwards it to the current client.
 *                   Also counts line errors, with or without a client.
 *   tcp task      - owns the listening socket and the client socket lifetime.
 *                   Forwards client bytes to the UART.
 *
 * One client per channel. A new connection takes over from the old one: after
 * a Wi-Fi drop the old socket is half-open and would otherwise hold the port
 * until TCP keepalive gives up.
 */

#include <assert.h>
#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "soc/gpio_reg.h"

#include "lwip/sockets.h"

#include "bridge_cores.h"
#include "settings.h"
#include "uart_tcp_bridge.h"

static const char *TAG = "bridge";

/* Kconfig omits a bool macro when the option is disabled. Keep the channel
 * initializer valid in both cases; the MCU application UART is 8N1. */
#ifdef CONFIG_BRIDGE_UART_EVEN_PARITY
#define PRIMARY_UART_EVEN_PARITY CONFIG_BRIDGE_UART_EVEN_PARITY
#else
#define PRIMARY_UART_EVEN_PARITY 0
#endif

/* Pre-connect backlog only exists when buffered data is kept on connect. */
#if !CONFIG_BRIDGE_FLUSH_ON_CONNECT && defined(CONFIG_BRIDGE_UART_BACKLOG) && CONFIG_BRIDGE_UART_BACKLOG > 0
#define BACKLOG_SIZE CONFIG_BRIDGE_UART_BACKLOG
#else
#define BACKLOG_SIZE 0
#endif

#define SOC_UART_PORT       UART_NUM_1
#define MCU_UART_PORT       UART_NUM_0
#define CHUNK               1024
#define LISTEN_BACKLOG      2
#define UART_EVT_QUEUE_LEN  32
/* Safety net only: a dropped UART_DATA event must not strand bytes in the ring. */
#define RX_EVT_TIMEOUT_MS   20
/* Idle time (in character times) after which the UART hands a partial FIFO to
 * the driver. The driver default is 10; 4 cuts ~0.5 ms off every reply at
 * 115200 while still batching back-to-back bytes. */
#define RX_TIMEOUT_SYMBOLS  4
#define MAX_TAPS            2

/* A client that vanished without FIN (Wi-Fi drop, laptop sleep) is detected
 * after IDLE + INTVL * CNT = 11 s instead of lwIP's default two hours. */
#define KEEPALIVE_IDLE_S    5
#define KEEPALIVE_INTVL_S   2
#define KEEPALIVE_CNT       3
/* A client that stops reading must not stall the UART reader forever. */
#define SEND_TIMEOUT_S      10

/* ESP-IDF applies software-flow-control thresholds to the hardware RX FIFO,
 * not the driver's much larger RX ring. ESP32-S2/S3 UARTs have a 128-byte
 * FIFO; leave 32 bytes of hysteresis between XOFF and XON. */
#define SW_FLOW_XON_THRESH  32
#define SW_FLOW_XOFF_THRESH 96

typedef struct {
    const char *name;
    uart_port_t uart;
    int tcp_port;
    int tx_gpio;
    int rx_gpio;
    int baud;
    bool even_parity;
    int rx_buf;
    bool sw_flowctrl;
    volatile bool *client_flag;
    QueueHandle_t evt_queue;
    /* Guards `client` and the backlog; held by the UART task while sending so
     * the TCP task never closes a socket that is mid-send. */
    SemaphoreHandle_t lock;
    int client;
    volatile uint32_t rx_bytes;
    volatile uint32_t tx_bytes;
    volatile uint32_t frame_err;
    volatile uint32_t parity_err;
    volatile uint32_t break_evt;
    volatile uint32_t fifo_ovf;
    volatile uint32_t buf_full;
    uint8_t rx_chunk[CHUNK];   /* UART -> TCP, owned by the uart_rx task */
    uint8_t tx_chunk[CHUNK];   /* TCP -> UART, owned by the tcp task */
#if BACKLOG_SIZE > 0
    uint8_t backlog[BACKLOG_SIZE];
    size_t backlog_head;       /* index of the oldest byte */
    size_t backlog_len;
#endif
} channel_t;

static uart_tcp_bridge_tap_t s_taps[MAX_TAPS];

static volatile bool s_soc_client;
#if CONFIG_BRIDGE_MCU_UART_ENABLE
static volatile bool s_mcu_client;
#endif

/* File scope so uart_tcp_bridge_get_stats() can reach them. Each channel owns
 * exactly one UART and its tasks live for the lifetime of the firmware. */
static channel_t s_channel_primary = {
    /* In the temporary MCU profile this proven UART1 path is connected to the
     * MCU instead of the SoC. Keep the transport-neutral label so the log
     * cannot claim the wrong target. */
    .name         = "primary",
    .uart         = SOC_UART_PORT,
    .tcp_port     = CONFIG_BRIDGE_TCP_PORT,
    .tx_gpio      = CONFIG_BRIDGE_UART_TX_GPIO,
    .rx_gpio      = CONFIG_BRIDGE_UART_RX_GPIO,
    .baud         = CONFIG_BRIDGE_UART_BAUD,
    .even_parity  = PRIMARY_UART_EVEN_PARITY,
    .rx_buf       = CONFIG_BRIDGE_UART_RX_BUF,
#if CONFIG_BRIDGE_UART_SW_FLOWCTRL
    .sw_flowctrl  = true,
#else
    .sw_flowctrl  = false,
#endif
    .client_flag  = &s_soc_client,
    .client       = -1,
};

#if CONFIG_BRIDGE_MCU_UART_ENABLE
static channel_t s_channel_mcu = {
    .name         = "mcu",
    .uart         = MCU_UART_PORT,
    .tcp_port     = CONFIG_BRIDGE_MCU_TCP_PORT,
    .tx_gpio      = CONFIG_BRIDGE_MCU_UART_TX_GPIO,
    .rx_gpio      = CONFIG_BRIDGE_MCU_UART_RX_GPIO,
    .baud         = CONFIG_BRIDGE_MCU_UART_BAUD,
#ifdef CONFIG_BRIDGE_MCU_UART_EVEN_PARITY
    .even_parity  = true,
#else
    .even_parity  = false,
#endif
    .rx_buf       = CONFIG_BRIDGE_MCU_UART_RX_BUF,
    .sw_flowctrl  = false,
    .client_flag  = &s_mcu_client,
    .client       = -1,
};
#endif

bool uart_tcp_bridge_has_client(void)
{
    return s_soc_client
#if CONFIG_BRIDGE_MCU_UART_ENABLE
           || s_mcu_client
#endif
           ;
}

static void uart_init_channel(channel_t *ch)
{
    const uart_config_t cfg = {
        .baud_rate  = ch->baud,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = ch->even_parity ? UART_PARITY_EVEN : UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ch->lock = xSemaphoreCreateMutex();
    assert(ch->lock != NULL);

    ESP_ERROR_CHECK(uart_driver_install(ch->uart, ch->rx_buf, ch->rx_buf,
                                        UART_EVT_QUEUE_LEN, &ch->evt_queue, 0));
    ESP_ERROR_CHECK(uart_param_config(ch->uart, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(ch->uart, ch->tx_gpio, ch->rx_gpio,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_set_rx_timeout(ch->uart, RX_TIMEOUT_SYMBOLS));

    if (ch->sw_flowctrl) {
        ESP_ERROR_CHECK(uart_set_sw_flow_ctrl(ch->uart, true,
                                              SW_FLOW_XON_THRESH,
                                              SW_FLOW_XOFF_THRESH));
    }

    ESP_LOGI(TAG, "%s: uart%d %d baud %s, tx=gpio%d rx=gpio%d, tcp/%d, ixoff=%d",
             ch->name, (int)ch->uart, ch->baud,
             ch->even_parity ? "8E1" : "8N1", ch->tx_gpio, ch->rx_gpio,
             ch->tcp_port, ch->sw_flowctrl);
}

/* send() can return short. Losing the tail of a line in the middle of a shell
 * or parameter transaction is the kind of bug that gets blamed on the robot
 * for weeks. */
static bool send_all(int sock, const uint8_t *buf, size_t len)
{
    size_t sent = 0;

    while (sent < len) {
        int n = send(sock, buf + sent, len - sent, 0);
        if (n < 0) {
            if (errno == EINTR) {
                continue;
            }
            return false;
        }
        if (n == 0) {
            return false;
        }
        sent += (size_t)n;
    }
    return true;
}

#if BACKLOG_SIZE > 0
/* Keeps the newest BACKLOG_SIZE bytes. Called with ch->lock held. */
static void backlog_push(channel_t *ch, const uint8_t *data, size_t len)
{
    if (len >= BACKLOG_SIZE) {
        data += len - BACKLOG_SIZE;
        len = BACKLOG_SIZE;
    }
    for (size_t i = 0; i < len; i++) {
        size_t tail = (ch->backlog_head + ch->backlog_len) % BACKLOG_SIZE;
        ch->backlog[tail] = data[i];
        if (ch->backlog_len < BACKLOG_SIZE) {
            ch->backlog_len++;
        } else {
            ch->backlog_head = (ch->backlog_head + 1) % BACKLOG_SIZE;
        }
    }
}

/* Sends and clears the backlog. Called with ch->lock held. */
static void backlog_replay(channel_t *ch, int sock)
{
    size_t first = BACKLOG_SIZE - ch->backlog_head;
    if (first > ch->backlog_len) {
        first = ch->backlog_len;
    }
    if (ch->backlog_len > 0) {
        ESP_LOGI(TAG, "%s: replaying %u byte(s) received before connect",
                 ch->name, (unsigned)ch->backlog_len);
        if (send_all(sock, ch->backlog + ch->backlog_head, first)) {
            (void)send_all(sock, ch->backlog, ch->backlog_len - first);
        }
    }
    ch->backlog_head = 0;
    ch->backlog_len = 0;
}
#endif

/* ---------------------------------------------------------------- UART -> TCP */

static void forward_to_client(channel_t *ch, const uint8_t *data, size_t len)
{
    xSemaphoreTake(ch->lock, portMAX_DELAY);
    if (ch->client >= 0) {
        if (!send_all(ch->client, data, len)) {
            /* Wake the tcp task out of recv(); it owns close(). */
            ESP_LOGW(TAG, "%s: send failed (%d), dropping client", ch->name, errno);
            shutdown(ch->client, SHUT_RDWR);
            ch->client = -1;
            *ch->client_flag = false;
        }
    }
#if BACKLOG_SIZE > 0
    else {
        backlog_push(ch, data, len);
    }
#endif
    xSemaphoreGive(ch->lock);
}

static void drain_uart(channel_t *ch)
{
    for (;;) {
        size_t avail = 0;
        if (uart_get_buffered_data_len(ch->uart, &avail) != ESP_OK || avail == 0) {
            return;
        }
        int n = uart_read_bytes(ch->uart, ch->rx_chunk,
                                avail < CHUNK ? avail : CHUNK, 0);
        if (n <= 0) {
            return;
        }
        ch->rx_bytes += (uint32_t)n;
        forward_to_client(ch, ch->rx_chunk, (size_t)n);
        if (ch == &s_channel_primary) {
            for (int i = 0; i < MAX_TAPS; i++) {
                if (s_taps[i] != NULL) {
                    s_taps[i](ch->rx_chunk, (size_t)n);
                }
            }
        }
    }
}

/* Sole reader of the UART. Keeps counting while no TCP client is attached, so
 * the discovery endpoint can say whether the peer ever spoke. */
static void uart_rx_task(void *arg)
{
    channel_t *ch = (channel_t *)arg;

    for (;;) {
        uart_event_t evt;
        if (xQueueReceive(ch->evt_queue, &evt, pdMS_TO_TICKS(RX_EVT_TIMEOUT_MS)) == pdTRUE) {
            switch (evt.type) {
            case UART_FRAME_ERR:   ch->frame_err++;  break;
            case UART_PARITY_ERR:  ch->parity_err++; break;
            case UART_BREAK:       ch->break_evt++;  break;
            /* The driver already reset the FIFO; count it and drain the ring. */
            case UART_FIFO_OVF:    ch->fifo_ovf++;   break;
            /* RX interrupts stay off until the ring is read; drain_uart does that. */
            case UART_BUFFER_FULL: ch->buf_full++;   break;
            default: break;
            }
        }
        drain_uart(ch);
    }
}

/* ---------------------------------------------------------------- TCP -> UART */

static void configure_client_socket(int sock)
{
    const int one = 1;
    const int idle = KEEPALIVE_IDLE_S;
    const int intvl = KEEPALIVE_INTVL_S;
    const int cnt = KEEPALIVE_CNT;
    const struct timeval snd_to = { .tv_sec = SEND_TIMEOUT_S, .tv_usec = 0 };

    setsockopt(sock, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
    setsockopt(sock, SOL_SOCKET, SO_KEEPALIVE, &one, sizeof(one));
    setsockopt(sock, IPPROTO_TCP, TCP_KEEPIDLE, &idle, sizeof(idle));
    setsockopt(sock, IPPROTO_TCP, TCP_KEEPINTVL, &intvl, sizeof(intvl));
    setsockopt(sock, IPPROTO_TCP, TCP_KEEPCNT, &cnt, sizeof(cnt));
    setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, &snd_to, sizeof(snd_to));
}

static void attach_client(channel_t *ch, int sock)
{
    xSemaphoreTake(ch->lock, portMAX_DELAY);
#if CONFIG_BRIDGE_FLUSH_ON_CONNECT
    uart_flush_input(ch->uart);
#elif BACKLOG_SIZE > 0
    backlog_replay(ch, sock);
#endif
    ch->client = sock;
    *ch->client_flag = true;
    xSemaphoreGive(ch->lock);
}

static void detach_client(channel_t *ch, int sock)
{
    xSemaphoreTake(ch->lock, portMAX_DELAY);
    if (ch->client == sock) {
        ch->client = -1;
        *ch->client_flag = false;
    }
    xSemaphoreGive(ch->lock);
    shutdown(sock, SHUT_RDWR);
    close(sock);
}

#if CONFIG_BRIDGE_UART_AUTOBAUD
/* The robot's debug header carries more than one UART depending on the machine
 * generation, and the log of which baud once worked is easy to misremember.
 * Listening at each candidate for about a second is cheaper than another round
 * of "try a build, guess, try again".
 *
 * This runs before the monitor and TCP tasks exist, so nothing else is draining
 * the driver ring while we count. */
static const int PROBE_BAUDS[] = { 115200, 57600, 9600 };

/* The X40 debug header carries more than the MCU UART, and the project's own
 * wiring note warns that its pin order differs from the older machines the
 * Rev A drawings were derived from. Listening on the second pair costs nothing
 * electrically - RX is an input, and TX idling high into a pulled-up SWDIO is
 * the same few milliamps the 1 kohm series resistors already cover. */
static const struct {
    int tx;
    int rx;
    const char *label;
} PROBE_PINS[] = {
    { CONFIG_BRIDGE_UART_TX_GPIO, CONFIG_BRIDGE_UART_RX_GPIO, "configured" },
    { 16, 18, "alternate" },
};

/* The vendor SoC pokes the MCU with these framed keepalive/query packets at
 * boot (/dev/ttyS4, see mcu_update.sh); the MCU parameter table only prints
 * after one of these or a CLI line. Probe them all so a silent wire and an
 * unhandled command are not confused. */
static const uint8_t PROBE_FRAME_MON1[] =
    { 0x3C, 0x00, 0x01, 0x01, 0x0E, 0x00, 0x01, 0x06, 0x00, 0x0E, 0x08, 0x3E };
static const uint8_t PROBE_FRAME_MON2[] =
    { 0x3C, 0x00, 0x01, 0x00, 0x03, 0x00, 0x02, 0x03, 0x01, 0x0A, 0x01, 0x3E };
static const uint8_t PROBE_FRAME_MON3[] = { 0x3C, 0x01, 0x02, 0x00, 0x60, 0x21, 0x3E };
static const uint8_t PROBE_FRAME_MON4[] = { 0x3C, 0x01, 0x1E, 0x01, 0x60, 0xE8, 0x3E };

static const struct {
    const char *name;
    const uint8_t *data;
    size_t len;
} PROBE_CMDS[] = {
    { "crlf",      (const uint8_t *)"\r\n",            2 },
    { "info -a",   (const uint8_t *)"info -a\r\n",      9 },
    { "info",      (const uint8_t *)"info\r\n",         6 },
    { "help",      (const uint8_t *)"help\r\n",         6 },
    { "backslash", (const uint8_t *)"\\\r\n",           3 },
    { "frame1",    PROBE_FRAME_MON1, sizeof(PROBE_FRAME_MON1) },
    { "frame2",    PROBE_FRAME_MON2, sizeof(PROBE_FRAME_MON2) },
    { "frame3",    PROBE_FRAME_MON3, sizeof(PROBE_FRAME_MON3) },
    { "frame4",    PROBE_FRAME_MON4, sizeof(PROBE_FRAME_MON4) },
};

static int listen_window(channel_t *ch, uint8_t *scratch, size_t scratch_len,
                         uint32_t ms)
{
    int got = 0;
    const TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(ms);
    while (xTaskGetTickCount() < deadline) {
        int n = uart_read_bytes(ch->uart, scratch, scratch_len,
                                pdMS_TO_TICKS(40));
        if (n > 0) {
            got += n;
        }
    }
    return got;
}

static void probe_pair(channel_t *ch, int tx_gpio, int rx_gpio,
                       const char *label)
{
    uint8_t scratch[128];

    ESP_ERROR_CHECK(uart_set_pin(ch->uart, tx_gpio, rx_gpio,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    uart_flush_input(ch->uart);

    ESP_LOGW(TAG, "autobaud: %s pair gpio%d(tx)/gpio%d(rx), idle %d/%d",
             label, tx_gpio, rx_gpio,
             gpio_get_level(tx_gpio), gpio_get_level(rx_gpio));

    for (size_t b = 0; b < sizeof(PROBE_BAUDS) / sizeof(PROBE_BAUDS[0]); b++) {
        ESP_ERROR_CHECK(uart_set_baudrate(ch->uart, PROBE_BAUDS[b]));
        uart_flush_input(ch->uart);
        int total = 0;

        for (size_t c = 0; c < sizeof(PROBE_CMDS) / sizeof(PROBE_CMDS[0]); c++) {
            uart_write_bytes(ch->uart, (const char *)PROBE_CMDS[c].data,
                             (int)PROBE_CMDS[c].len);
            int got = listen_window(ch, scratch, sizeof(scratch), 250);
            total += got;
            if (got > 0) {
                ESP_LOGW(TAG, "autobaud: %s %d 8N1, cmd=%s -> %d byte(s)",
                         label, PROBE_BAUDS[b], PROBE_CMDS[c].name, got);
            }
        }
        ESP_LOGW(TAG, "autobaud: %s %d 8N1 -> %d byte(s)", label,
                 PROBE_BAUDS[b], total);
    }
}

/* Map activity across the whole debug header before trusting any pin pair.
 * A live UART shows up as thousands of edges on its TX pin; a powered-off or
 * wrong header shows nothing anywhere. This replaces guessing. */
static const int SCAN_PINS[] = { 16, 18, 33, 35, 37, 39 };
#define SCAN_N ((int)(sizeof(SCAN_PINS) / sizeof(SCAN_PINS[0])))

/* GPIO33+ are NOT in GPIO_IN_REG (covers GPIO0..31). On ESP32-S2 they live in
 * GPIO_IN1_REG at bit (pin-32). The old code did `1u << 33` on GPIO_IN_REG which
 * is undefined (shift >= 32) and silently never sampled 33/35, so its "0
 * transitions" was meaningless. Sample both registers. */
static inline uint32_t pin_vec(void)
{
    const uint32_t in0 = REG_READ(GPIO_IN_REG);
    const uint32_t in1 = REG_READ(GPIO_IN1_REG);
    uint32_t v = 0;
    for (int i = 0; i < SCAN_N; i++) {
        const int p = SCAN_PINS[i];
        const uint32_t hi = (p >= 32) ? ((in1 >> (p - 32)) & 1U)
                                      : ((in0 >> p) & 1U);
        v |= hi << i;
    }
    return v;
}

static void scan_header_activity(void)
{
    uint32_t trans[SCAN_N] = { 0 };
    uint32_t prev = pin_vec();

    const TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(3000);
    while (xTaskGetTickCount() < deadline) {
        const uint32_t now = pin_vec();
        const uint32_t changed = now ^ prev;
        for (int i = 0; i < SCAN_N; i++) {
            if (changed & (1u << i)) {
                trans[i]++;
            }
        }
        prev = now;
    }

    for (int i = 0; i < SCAN_N; i++) {
        ESP_LOGW(TAG, "pin-scan: gpio%d -> %u transitions",
                 SCAN_PINS[i], (unsigned)trans[i]);
    }
}

static void autobaud_probe(channel_t *ch)
{
    /* DIAGNOSTIC: emit a burst so scan_header_activity() can observe our OWN
     * TX pin (GPIO35) toggling. If gpio35 shows transitions here, the command
     * physically leaves the ESP; if it stays 0, the TX path/pin is broken. */
    uart_write_bytes(ch->uart, "info -a\r\n", 9);
    vTaskDelay(pdMS_TO_TICKS(50));
    scan_header_activity();

    ESP_LOGW(TAG, "autobaud: probing %u pin pair(s) x %u baud(s)",
             (unsigned)(sizeof(PROBE_PINS) / sizeof(PROBE_PINS[0])),
             (unsigned)(sizeof(PROBE_BAUDS) / sizeof(PROBE_BAUDS[0])));

    for (size_t p = 0; p < sizeof(PROBE_PINS) / sizeof(PROBE_PINS[0]); p++) {
        probe_pair(ch, PROBE_PINS[p].tx, PROBE_PINS[p].rx,
                   PROBE_PINS[p].label);
    }

    /* Restore the configured pair; a silent wire teaches us nothing about the
     * MCU real baud, so the bridge keeps its configured settings. */
    ESP_ERROR_CHECK(uart_set_pin(ch->uart, ch->tx_gpio, ch->rx_gpio,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_set_baudrate(ch->uart, ch->baud));
    uart_flush_input(ch->uart);
    ESP_LOGW(TAG, "autobaud: restored gpio%d/gpio%d %d 8N1", ch->tx_gpio,
             ch->rx_gpio, ch->baud);
}
#endif /* CONFIG_BRIDGE_UART_AUTOBAUD */

/* DIAGNOSTIC (temporary): report per-pin transition counts every 3 s forever.
 * Uses pin_vec() so GPIO33..39 (GPIO_IN1_REG) are actually sampled. */
#if 0
static void pin_watch_task(void *arg)
{
    (void)arg;

    for (;;) {
        uint32_t trans[SCAN_N] = { 0 };
        uint32_t prev = pin_vec();
        const TickType_t dl = xTaskGetTickCount() + pdMS_TO_TICKS(3000);
        while (xTaskGetTickCount() < dl) {
            const uint32_t now = pin_vec();
            const uint32_t ch = now ^ prev;
            for (int i = 0; i < SCAN_N; i++) {
                if (ch & (1u << i)) {
                    trans[i]++;
                }
            }
            prev = now;
        }
        ESP_LOGW(TAG, "live: 16=%u 18=%u 33=%u 35=%u 37=%u 39=%u bits=0x%02x",
                 (unsigned)trans[0], (unsigned)trans[1], (unsigned)trans[2],
                 (unsigned)trans[3], (unsigned)trans[4], (unsigned)trans[5],
                 (unsigned)pin_vec());
    }
}
#endif /* temporary pin_watch_task disabled */

static void tcp_task(void *arg)
{
    channel_t *ch = (channel_t *)arg;
    int listener = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    assert(listener >= 0);

    const int one = 1;
    setsockopt(listener, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));

    const struct sockaddr_in addr = {
        .sin_family      = AF_INET,
        .sin_port        = htons(ch->tcp_port),
        .sin_addr.s_addr = htonl(INADDR_ANY),
    };

    ESP_ERROR_CHECK(bind(listener, (const struct sockaddr *)&addr, sizeof(addr))
                        == 0 ? ESP_OK : ESP_FAIL);
    ESP_ERROR_CHECK(listen(listener, LISTEN_BACKLOG) == 0 ? ESP_OK : ESP_FAIL);

    ESP_LOGI(TAG, "%s: listening on tcp/%d", ch->name, ch->tcp_port);

    int client = -1;
    char client_ip[16] = "";

    for (;;) {
        fd_set rfds;
        FD_ZERO(&rfds);
        FD_SET(listener, &rfds);
        int maxfd = listener;
        if (client >= 0) {
            FD_SET(client, &rfds);
            if (client > maxfd) {
                maxfd = client;
            }
        }

        int ready = select(maxfd + 1, &rfds, NULL, NULL, NULL);
        if (ready < 0) {
            if (errno != EINTR) {
                ESP_LOGW(TAG, "%s: select failed: %d", ch->name, errno);
                vTaskDelay(pdMS_TO_TICKS(100));
            }
            continue;
        }

        /* Read the current client before looking at a new connection, so the
         * bytes it already sent still reach the UART on takeover. */
        if (client >= 0 && FD_ISSET(client, &rfds)) {
            int n = recv(client, ch->tx_chunk, CHUNK, 0);
            if (n > 0) {
                if (uart_write_bytes(ch->uart, (const char *)ch->tx_chunk,
                                     (size_t)n) < 0) {
                    ESP_LOGW(TAG, "%s: uart write failed", ch->name);
                } else {
                    __atomic_fetch_add(&ch->tx_bytes, (uint32_t)n, __ATOMIC_RELAXED);
                }
            } else if (n == 0 || (errno != EINTR && errno != EAGAIN)) {
                if (n == 0) {
                    ESP_LOGI(TAG, "%s: client %s closed", ch->name, client_ip);
                } else {
                    ESP_LOGW(TAG, "%s: client %s recv failed: %d", ch->name,
                             client_ip, errno);
                }
                detach_client(ch, client);
                client = -1;
            }
        }

        if (FD_ISSET(listener, &rfds)) {
            struct sockaddr_in peer;
            socklen_t peer_len = sizeof(peer);
            int sock = accept(listener, (struct sockaddr *)&peer, &peer_len);
            if (sock < 0) {
                ESP_LOGW(TAG, "%s: accept failed: %d", ch->name, errno);
                vTaskDelay(pdMS_TO_TICKS(100));
                continue;
            }
            configure_client_socket(sock);

            char ip[16];
            inet_ntoa_r(peer.sin_addr, ip, sizeof(ip));
            if (client >= 0) {
                ESP_LOGW(TAG, "%s: client %s replaced by %s", ch->name,
                         client_ip, ip);
                detach_client(ch, client);
            } else {
                ESP_LOGI(TAG, "%s: client %s connected", ch->name, ip);
            }
            client = sock;
            strlcpy(client_ip, ip, sizeof(client_ip));
            attach_client(ch, client);
        }
    }
}

void uart_tcp_bridge_get_stats(int index, uart_tcp_bridge_stats_t *out)
{
    const channel_t *ch = NULL;

    if (out == NULL) {
        return;
    }
    *out = (uart_tcp_bridge_stats_t){ 0 };

    switch (index) {
    case 0:
        ch = &s_channel_primary;
        break;
#if CONFIG_BRIDGE_MCU_UART_ENABLE
    case 1:
        ch = &s_channel_mcu;
        break;
#endif
    default:
        return;
    }

    out->rx_bytes   = ch->rx_bytes;
    out->tx_bytes   = ch->tx_bytes;
    out->frame_err  = ch->frame_err;
    out->parity_err = ch->parity_err;
    out->break_evt  = ch->break_evt;
    out->fifo_ovf   = ch->fifo_ovf;
    out->buf_full   = ch->buf_full;
}

int uart_tcp_bridge_baud(void)
{
    return s_channel_primary.baud;
}

esp_err_t uart_tcp_bridge_set_baud(int baud)
{
    if (baud < 1200 || baud > 5000000) {
        return ESP_ERR_INVALID_ARG;
    }
    esp_err_t err = uart_set_baudrate(s_channel_primary.uart, (uint32_t)baud);
    if (err == ESP_OK) {
        s_channel_primary.baud = baud;
        ESP_LOGI(TAG, "primary: baud -> %d", baud);
    }
    return err;
}

int uart_tcp_bridge_write(const uint8_t *data, size_t len)
{
    int n = uart_write_bytes(s_channel_primary.uart, (const char *)data, len);
    if (n > 0) {
        __atomic_fetch_add(&s_channel_primary.tx_bytes, (uint32_t)n, __ATOMIC_RELAXED);
    }
    return n;
}

void uart_tcp_bridge_add_tap(uart_tcp_bridge_tap_t tap)
{
    for (int i = 0; i < MAX_TAPS; i++) {
        if (s_taps[i] == NULL || s_taps[i] == tap) {
            s_taps[i] = tap;
            return;
        }
    }
    ESP_LOGE(TAG, "no free tap slot");
}

static void start_channel(channel_t *ch, const char *rx_name, const char *tcp_name)
{
    /* The UART reader runs above the TCP side so a burst from the robot is
     * moved out of the 128-byte FIFO/ring before anything else. */
    xTaskCreatePinnedToCore(uart_rx_task, rx_name, 3072, ch, 6, NULL, BRIDGE_IO_CORE);
    xTaskCreatePinnedToCore(tcp_task, tcp_name, 4096, ch, 5, NULL, BRIDGE_IO_CORE);
}

void uart_tcp_bridge_start(void)
{
    s_channel_primary.baud = settings_uart_baud();
    uart_init_channel(&s_channel_primary);
#if CONFIG_BRIDGE_UART_AUTOBAUD
    autobaud_probe(&s_channel_primary);
#endif
    start_channel(&s_channel_primary, "uart_rx", "uart_tcp");

#if CONFIG_BRIDGE_MCU_UART_ENABLE
    if (CONFIG_BRIDGE_UART_TX_GPIO == CONFIG_BRIDGE_MCU_UART_TX_GPIO ||
        CONFIG_BRIDGE_UART_TX_GPIO == CONFIG_BRIDGE_MCU_UART_RX_GPIO ||
        CONFIG_BRIDGE_UART_RX_GPIO == CONFIG_BRIDGE_MCU_UART_TX_GPIO ||
        CONFIG_BRIDGE_UART_RX_GPIO == CONFIG_BRIDGE_MCU_UART_RX_GPIO) {
        ESP_LOGE(TAG, "SoC and MCU UART GPIOs overlap");
        abort();
    }
    if (CONFIG_BRIDGE_TCP_PORT == CONFIG_BRIDGE_MCU_TCP_PORT) {
        ESP_LOGE(TAG, "SoC and MCU TCP ports overlap");
        abort();
    }

    uart_init_channel(&s_channel_mcu);
    start_channel(&s_channel_mcu, "mcu_rx", "mcu_tcp");
#endif
}
