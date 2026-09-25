#include "cmd.h"

#include <assert.h>
#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "esp_app_desc.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"

#include "jbuf.h"
#include "settings.h"
#include "swd_bridge.h"
#include "uart_tcp_bridge.h"
#include "wifi_mgr.h"

static const char *TAG = "cmd";

#if CONFIG_IDF_TARGET_ESP32S3
#define CAP_SIZE        16384     /* UART bytes kept for uart.read / uart.xfer */
#else
#define CAP_SIZE        4096
#endif
#define CAP_BIT         BIT0
#define MAX_WAIT_MS     30000
#define READ_IDLE_MS    60        /* uart.read: reply considered complete after this gap */
#define XFER_IDLE_MS    100       /* uart.xfer: same, for a command's reply */
#define MAX_DATA        1024      /* bytes per uart.send */

/* ------------------------------------------------------------ UART capture */

static uint8_t s_cap[CAP_SIZE];
static size_t s_cap_head;
static size_t s_cap_len;
static bool s_cap_overflow;
static SemaphoreHandle_t s_cap_lock;
static SemaphoreHandle_t s_xfer_lock;   /* one uart.read/xfer at a time */
static EventGroupHandle_t s_cap_ev;

static void cap_tap(const uint8_t *data, size_t len)
{
    xSemaphoreTake(s_cap_lock, portMAX_DELAY);
    for (size_t i = 0; i < len; i++) {
        s_cap[(s_cap_head + s_cap_len) % CAP_SIZE] = data[i];
        if (s_cap_len < CAP_SIZE) {
            s_cap_len++;
        } else {
            s_cap_head = (s_cap_head + 1) % CAP_SIZE;
            s_cap_overflow = true;
        }
    }
    xSemaphoreGive(s_cap_lock);
    xEventGroupSetBits(s_cap_ev, CAP_BIT);
}

static size_t cap_len(void)
{
    xSemaphoreTake(s_cap_lock, portMAX_DELAY);
    size_t n = s_cap_len;
    xSemaphoreGive(s_cap_lock);
    return n;
}

static size_t cap_take(uint8_t *out, bool *overflow)
{
    xSemaphoreTake(s_cap_lock, portMAX_DELAY);
    size_t n = s_cap_len;
    for (size_t i = 0; i < n; i++) {
        out[i] = s_cap[(s_cap_head + i) % CAP_SIZE];
    }
    s_cap_head = 0;
    s_cap_len = 0;
    *overflow = s_cap_overflow;
    s_cap_overflow = false;
    xSemaphoreGive(s_cap_lock);
    return n;
}

/* Waits up to wait_ms for data, then until the line has been quiet for idle_ms. */
static void cap_wait(uint32_t wait_ms, uint32_t idle_ms)
{
    const int64_t deadline = esp_timer_get_time() + (int64_t)wait_ms * 1000;
    bool got = false;
    for (;;) {
        xEventGroupClearBits(s_cap_ev, CAP_BIT);
        if (!got && cap_len() > 0) {
            got = true;
        }
        int64_t left_ms = (deadline - esp_timer_get_time()) / 1000;
        if (left_ms <= 0) {
            return;
        }
        uint32_t step = got ? (idle_ms < left_ms ? idle_ms : (uint32_t)left_ms) : (uint32_t)left_ms;
        EventBits_t bits = xEventGroupWaitBits(s_cap_ev, CAP_BIT, pdTRUE, pdFALSE,
                                               pdMS_TO_TICKS(step));
        if (bits & CAP_BIT) {
            got = true;
        } else if (got) {
            return;   /* quiet for idle_ms after data: reply complete */
        }
    }
}

/* ------------------------------------------------------------ parsing */

static const char *skip_ws(const char *p)
{
    while (*p == ' ' || *p == '\t') {
        p++;
    }
    return p;
}

/* Next token: "double quoted" (\" and \\ escapes) or up to whitespace. */
static const char *next_token(const char *p, char *out, size_t cap)
{
    p = skip_ws(p);
    size_t n = 0;
    if (*p == '"') {
        p++;
        while (*p && *p != '"') {
            char c = *p++;
            if (c == '\\' && (*p == '"' || *p == '\\')) {
                c = *p++;
            }
            if (n + 1 < cap) {
                out[n++] = c;
            }
        }
        if (*p == '"') {
            p++;
        }
    } else {
        while (*p && *p != ' ' && *p != '\t') {
            if (n + 1 < cap) {
                out[n++] = *p;
            }
            p++;
        }
    }
    out[n] = '\0';
    return p;
}

static int hexval(char c)
{
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
}

/* Data argument: the rest of the line, optionally wrapped in one pair of
 * double quotes. Escapes: \r \n \t \0 \\ \" \xHH. */
static size_t parse_data(const char *rest, uint8_t *out, size_t cap)
{
    rest = skip_ws(rest);
    size_t len = strlen(rest);
    if (len >= 2 && rest[0] == '"' && rest[len - 1] == '"') {
        rest++;
        len -= 2;
    }
    size_t n = 0;
    for (size_t i = 0; i < len && n < cap; i++) {
        char c = rest[i];
        if (c == '\\' && i + 1 < len && rest[i + 1] == 'x' && i + 3 < len &&
            hexval(rest[i + 2]) >= 0 && hexval(rest[i + 3]) >= 0) {
            c = (char)((hexval(rest[i + 2]) << 4) | hexval(rest[i + 3]));
            i += 3;
        } else if (c == '\\' && i + 1 < len) {
            char e = rest[++i];
            switch (e) {
            case 'r': c = '\r'; break;
            case 'n': c = '\n'; break;
            case 't': c = '\t'; break;
            case '0': c = '\0'; break;
            case '\\': c = '\\'; break;
            case '"': c = '"'; break;
            default:   /* unknown escape: keep it literally */
                if (n < cap) {
                    out[n++] = '\\';
                }
                c = e;
                break;
            }
        }
        if (n < cap) {
            out[n++] = (uint8_t)c;
        }
    }
    return n;
}

/* "3C 00 0x01,3e" -> bytes. Returns -1 on an odd digit count. */
static int parse_hex(const char *s, uint8_t *out, size_t cap)
{
    size_t n = 0;
    int hi = -1;
    for (; *s; s++) {
        if (s[0] == '0' && (s[1] == 'x' || s[1] == 'X')) {
            s++;
            continue;
        }
        int v = hexval(*s);
        if (v < 0) {
            continue;
        }
        if (hi < 0) {
            hi = v;
        } else {
            if (n >= cap) {
                return -1;
            }
            out[n++] = (uint8_t)((hi << 4) | v);
            hi = -1;
        }
    }
    return hi >= 0 ? -1 : (int)n;
}

static bool parse_uint(const char *s, unsigned long max, unsigned long *out)
{
    if (s == NULL || !isdigit((unsigned char)*s)) {
        return false;
    }
    char *end;
    unsigned long v = strtoul(s, &end, 10);
    if (*end != '\0' || v > max) {
        return false;
    }
    *out = v;
    return true;
}

/* ------------------------------------------------------------ commands */

static bool fail(jbuf_t *jb, const char *error)
{
    jb_bool(jb, "ok", false);
    jb_str(jb, "error", error);
    return false;
}

static void put_data(jbuf_t *jb, const uint8_t *data, size_t len)
{
    jb_int(jb, "len", (long long)len);
    jb_bytes(jb, "text", data, len);
    jb_hex(jb, "hex", data, len);
}

typedef bool (*cmd_fn_t)(jbuf_t *jb, const char *args);

typedef struct {
    const char *name;
    const char *usage;
    const char *help;
    cmd_fn_t fn;
} cmd_t;

static const cmd_t *commands(size_t *count);

static bool c_help(jbuf_t *jb, const char *args)
{
    (void)args;
    size_t n;
    const cmd_t *t = commands(&n);
    jb_bool(jb, "ok", true);
    jb_arr_open(jb, "commands");
    for (size_t i = 0; i < n; i++) {
        jb_obj_open(jb, NULL);
        jb_str(jb, "cmd", t[i].name);
        jb_str(jb, "usage", t[i].usage);
        jb_str(jb, "help", t[i].help);
        jb_obj_close(jb);
    }
    jb_arr_close(jb);
    return true;
}

static bool c_status(jbuf_t *jb, const char *args)
{
    (void)args;
    const esp_app_desc_t *app = esp_app_get_description();
    uart_tcp_bridge_stats_t st;
    uart_tcp_bridge_get_stats(0, &st);

    jb_bool(jb, "ok", true);
    jb_str(jb, "fw", app->version);
    jb_int(jb, "uptime_s", esp_timer_get_time() / 1000000);
    jb_int(jb, "free_heap", esp_get_free_heap_size());
    jb_int(jb, "min_free_heap", esp_get_minimum_free_heap_size());
    jb_obj_open(jb, "wifi");
    wifi_mgr_status_json(jb);
    jb_obj_close(jb);
    jb_obj_open(jb, "uart");
    jb_int(jb, "baud", uart_tcp_bridge_baud());
    jb_str(jb, "mode", "8N1");
    jb_int(jb, "tcp_port", CONFIG_BRIDGE_TCP_PORT);
    jb_bool(jb, "tcp_client", uart_tcp_bridge_has_client());
    jb_int(jb, "rx_bytes", st.rx_bytes);
    jb_int(jb, "tx_bytes", st.tx_bytes);
    jb_int(jb, "frame_err", st.frame_err);
    jb_int(jb, "parity_err", st.parity_err);
    jb_int(jb, "break", st.break_evt);
    jb_int(jb, "fifo_ovf", st.fifo_ovf);
    jb_int(jb, "buf_full", st.buf_full);
    jb_int(jb, "buffered", (long long)cap_len());
    jb_obj_close(jb);
    jb_obj_open(jb, "swd");
    jb_bool(jb, "enabled", CONFIG_BRIDGE_SWD_ENABLE);
    jb_int(jb, "tcp_port", CONFIG_BRIDGE_SWD_TCP_PORT);
    jb_int(jb, "bitbang_port", CONFIG_BRIDGE_SWD_REMOTE_BITBANG_TCP_PORT);
    jb_obj_close(jb);
    return true;
}

static bool c_wifi_scan(jbuf_t *jb, const char *args)
{
    (void)args;
    /* Writes nothing on failure, so "ok" can follow the array. */
    esp_err_t err = wifi_mgr_scan_json(jb, "networks");
    if (err != ESP_OK) {
        return fail(jb, esp_err_to_name(err));
    }
    jb_bool(jb, "ok", true);
    return true;
}

static bool c_wifi_set(jbuf_t *jb, const char *args)
{
    char ssid[40];
    char pass[72];
    args = next_token(args, ssid, sizeof(ssid));
    next_token(args, pass, sizeof(pass));
    char ip[16] = "";
    const char *reason = "";
    if (wifi_mgr_set(ssid, pass, ip, sizeof(ip), &reason) != ESP_OK) {
        return fail(jb, reason);
    }
    jb_bool(jb, "ok", true);
    jb_str(jb, "ssid", ssid);
    jb_str(jb, "ip", ip);
    return true;
}

static bool c_wifi_forget(jbuf_t *jb, const char *args)
{
    (void)args;
    if (wifi_mgr_forget() != ESP_OK) {
        return fail(jb, "could not erase the saved network");
    }
    jb_bool(jb, "ok", true);
    return true;
}

static bool c_wifi_ap(jbuf_t *jb, const char *args)
{
    char arg[8];
    next_token(args, arg, sizeof(arg));
    bool on = strcmp(arg, "on") == 0;
    if (!on && strcmp(arg, "off") != 0) {
        return fail(jb, "usage: wifi.ap on|off");
    }
    if (wifi_mgr_ap(on) != ESP_OK) {
        return fail(jb, "station is not connected; the setup AP stays up");
    }
    jb_bool(jb, "ok", true);
    jb_bool(jb, "setup_ap", wifi_mgr_ap_active());
    return true;
}

static bool c_uart_baud(jbuf_t *jb, const char *args)
{
    char arg[12];
    next_token(args, arg, sizeof(arg));
    if (arg[0] != '\0') {
        unsigned long baud;
        if (!parse_uint(arg, 5000000, &baud) || baud < 1200) {
            return fail(jb, "baud must be 1200..5000000");
        }
        if (uart_tcp_bridge_set_baud((int)baud) != ESP_OK ||
            settings_set_uart_baud((int)baud) != ESP_OK) {
            return fail(jb, "could not apply/save baud rate");
        }
    }
    jb_bool(jb, "ok", true);
    jb_int(jb, "baud", uart_tcp_bridge_baud());
    return true;
}

static bool send_bytes(jbuf_t *jb, const uint8_t *data, size_t len)
{
    if (len == 0) {
        return fail(jb, "nothing to send");
    }
    int n = uart_tcp_bridge_write(data, len);
    if (n < 0) {
        return fail(jb, "uart write failed");
    }
    jb_bool(jb, "ok", true);
    jb_int(jb, "sent", n);
    return true;
}

static bool c_uart_send(jbuf_t *jb, const char *args)
{
    uint8_t buf[MAX_DATA];
    return send_bytes(jb, buf, parse_data(args, buf, sizeof(buf)));
}

static bool c_uart_sendhex(jbuf_t *jb, const char *args)
{
    uint8_t buf[MAX_DATA];
    int n = parse_hex(args, buf, sizeof(buf));
    if (n < 0) {
        return fail(jb, "bad hex (odd digit count or longer than 1024 bytes)");
    }
    return send_bytes(jb, buf, (size_t)n);
}

static bool reply_captured(jbuf_t *jb)
{
    uint8_t *buf = malloc(CAP_SIZE);
    if (buf == NULL) {
        return fail(jb, "out of memory");
    }
    bool overflow;
    size_t n = cap_take(buf, &overflow);
    jb_bool(jb, "ok", true);
    put_data(jb, buf, n);
    jb_bool(jb, "overflow", overflow);
    free(buf);
    return true;
}

static bool c_uart_read(jbuf_t *jb, const char *args)
{
    char arg[12];
    next_token(args, arg, sizeof(arg));
    unsigned long wait = 0;
    if (arg[0] != '\0' && !parse_uint(arg, MAX_WAIT_MS, &wait)) {
        return fail(jb, "usage: uart.read [wait_ms<=30000]");
    }
    xSemaphoreTake(s_xfer_lock, portMAX_DELAY);
    if (wait > 0) {
        cap_wait(wait, READ_IDLE_MS);
    }
    bool ok = reply_captured(jb);
    xSemaphoreGive(s_xfer_lock);
    return ok;
}

static bool c_uart_xfer(jbuf_t *jb, const char *args)
{
    char arg[12];
    const char *rest = next_token(args, arg, sizeof(arg));
    unsigned long wait;
    if (!parse_uint(arg, MAX_WAIT_MS, &wait)) {
        return fail(jb, "usage: uart.xfer <wait_ms<=30000> <data>");
    }
    uint8_t buf[MAX_DATA];
    size_t len = parse_data(rest, buf, sizeof(buf));
    if (len == 0) {
        return fail(jb, "nothing to send");
    }

    xSemaphoreTake(s_xfer_lock, portMAX_DELAY);
    xSemaphoreTake(s_cap_lock, portMAX_DELAY);   /* drop stale bytes */
    s_cap_head = 0;
    s_cap_len = 0;
    s_cap_overflow = false;
    xSemaphoreGive(s_cap_lock);

    int n = uart_tcp_bridge_write(buf, len);
    bool ok;
    if (n < 0) {
        ok = fail(jb, "uart write failed");
    } else {
        cap_wait(wait, XFER_IDLE_MS);
        jb_int(jb, "sent", n);
        ok = reply_captured(jb);
    }
    xSemaphoreGive(s_xfer_lock);
    return ok;
}

static bool c_swd(jbuf_t *jb, const char *args)
{
    args = skip_ws(args);
    char verb[12];
    next_token(args, verb, sizeof(verb));
    for (char *p = verb; *p; p++) {
        *p = (char)toupper((unsigned char)*p);
    }
    if (verb[0] == '\0') {
        return fail(jb, "usage: swd <command>, e.g. swd ID, swd READ 0x08000000 64");
    }
    if (strcmp(verb, "WRITE") == 0 || strcmp(verb, "MWRITE") == 0 ||
        strcmp(verb, "DUMP") == 0) {
        return fail(jb, "binary transfer: use the raw SWD port (tcp/2325)");
    }
    if (strlen(args) > 90) {
        return fail(jb, "command too long");
    }

    /* The SWD text protocol is upper-case; accept "swd read ..." too. */
    char line[96];
    strlcpy(line, args, sizeof(line));
    for (char *p = line; *p && *p != ' '; p++) {
        *p = (char)toupper((unsigned char)*p);
    }

    uint8_t *out = NULL;
    size_t out_len = 0;
    esp_err_t err = swd_bridge_exec(line, &out, &out_len, CONFIG_BRIDGE_SWD_MAX_READ + 128);
    if (err == ESP_ERR_NOT_FINISHED) {
        return fail(jb, "SWD busy: an OpenOCD session is connected");
    }
    if (err != ESP_OK) {
        free(out);
        return fail(jb, err == ESP_ERR_INVALID_STATE ? "SWD disabled" : esp_err_to_name(err));
    }

    /* First line is the status; READ appends the binary body after it. */
    size_t line_end = 0;
    while (line_end < out_len && out[line_end] != '\n') {
        line_end++;
    }
    const bool ok = line_end >= 2 && (memcmp(out, "OK", 2) == 0 || memcmp(out, "PONG", 4) == 0);
    jb_bool(jb, "ok", ok);
    jb_bytes(jb, "response", out, line_end);
    if (!ok) {
        jb_bytes(jb, "error", out, line_end);
    }
    if (line_end + 1 < out_len) {
        jb_hex(jb, "data", out + line_end + 1, out_len - line_end - 1);
    }
    free(out);
    return ok;
}

static void reboot_task(void *arg)
{
    (void)arg;
    vTaskDelay(pdMS_TO_TICKS(500));   /* let the reply leave */
    esp_restart();
}

static bool c_reboot(jbuf_t *jb, const char *args)
{
    (void)args;
    xTaskCreate(reboot_task, "reboot", 2048, NULL, 1, NULL);
    jb_bool(jb, "ok", true);
    return true;
}

static const cmd_t COMMANDS[] = {
    { "help", "", "list commands", c_help },
    { "status", "", "firmware, Wi-Fi, UART counters, SWD ports", c_status },
    { "wifi.scan", "", "list visible networks (strongest first)", c_wifi_scan },
    { "wifi.set", "<ssid> <password>",
      "join a network; saved only if it gets an IP, else the old one is kept. Quote values with spaces",
      c_wifi_set },
    { "wifi.forget", "", "erase the saved network and start the setup AP", c_wifi_forget },
    { "wifi.ap", "on|off", "setup AP on (10 min) / off (only while connected)", c_wifi_ap },
    { "uart.baud", "[rate]", "show or set (and save) the robot UART baud rate", c_uart_baud },
    { "uart.send", "<data>", "send text; escapes \\r \\n \\t \\0 \\\\ \\\" \\xHH; no line ending is added",
      c_uart_send },
    { "uart.sendhex", "<hex>", "send raw bytes, e.g. 3C 00 01 3E", c_uart_sendhex },
    { "uart.read", "[wait_ms]",
      "return bytes received since the last read/xfer (buffer: 4 KB on S2, 16 KB on S3); wait up to wait_ms for data",
      c_uart_read },
    { "uart.xfer", "<wait_ms> <data>",
      "clear the buffer, send data, return the reply (ends after 100 ms of silence or wait_ms)",
      c_uart_xfer },
    { "swd", "<command>",
      "SWD text command: PING ID DPID PID CTRL RAW HALT RESUME STEP REGREAD n REGWRITE n v RUNUNTIL ... READ addr len",
      c_swd },
    { "reboot", "", "restart the bridge", c_reboot },
};

static const cmd_t *commands(size_t *count)
{
    *count = sizeof(COMMANDS) / sizeof(COMMANDS[0]);
    return COMMANDS;
}

char *cmd_exec(const char *line)
{
    char name[24];
    const char *args = next_token(line ? line : "", name, sizeof(name));

    jbuf_t jb;
    jb_init(&jb, 512);
    jb_obj_open(&jb, NULL);

    size_t n;
    const cmd_t *t = commands(&n);
    const cmd_t *c = NULL;
    for (size_t i = 0; i < n; i++) {
        if (strcmp(name, t[i].name) == 0) {
            c = &t[i];
            break;
        }
    }
    if (c == NULL) {
        fail(&jb, name[0] ? "unknown command, try \"help\"" : "empty command");
    } else {
        c->fn(&jb, args);
    }
    jb_obj_close(&jb);

    char *out = jb_finish(&jb);
    if (out == NULL) {
        out = strdup("{\"ok\":false,\"error\":\"out of memory\"}");
    }
    return out;
}

void cmd_usb_line(const char *line)
{
    char *out = cmd_exec(line);
    if (out != NULL) {
        printf("@CMD %s\n", out);
        fflush(stdout);
        free(out);
    }
}

#if !CONFIG_BRIDGE_SWD_ENABLE
/* Without the SWD USB RPC task nobody reads the console; do it here. */
static void usb_task(void *arg)
{
    (void)arg;
    static char line[600];
    setvbuf(stdin, NULL, _IONBF, 0);
    for (;;) {
        if (fgets(line, sizeof(line), stdin) == NULL) {
            clearerr(stdin);
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }
        line[strcspn(line, "\r\n")] = '\0';
        if (strncmp(line, "@CMD ", 5) == 0) {
            cmd_usb_line(line + 5);
        }
    }
}
#endif

void cmd_start(void)
{
    s_cap_lock = xSemaphoreCreateMutex();
    s_xfer_lock = xSemaphoreCreateMutex();
    s_cap_ev = xEventGroupCreate();
    assert(s_cap_lock && s_xfer_lock && s_cap_ev);
    uart_tcp_bridge_add_tap(cap_tap);
#if !CONFIG_BRIDGE_SWD_ENABLE
    xTaskCreate(usb_task, "usb_cmd", 4096, NULL, 3, NULL);
#endif
    ESP_LOGI(TAG, "command API ready (HTTP POST /api/cmd, USB \"@CMD <line>\")");
}
