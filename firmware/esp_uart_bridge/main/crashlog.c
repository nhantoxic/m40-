#include "crashlog.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_attr.h"
#include "esp_log.h"
#include "esp_system.h"

#if CONFIG_ESP_COREDUMP_ENABLE_TO_FLASH
#include "esp_core_dump.h"
#endif

static const char *TAG = "crashlog";

#define MAGIC      0xB21D6E57u
#define STABLE_S   60

/* Survives software resets, panics and watchdog resets (not power-off). */
RTC_NOINIT_ATTR static uint32_t s_magic;
RTC_NOINIT_ATTR static uint32_t s_crashes;

static boot_mode_t s_mode;
static uint32_t s_boot_crashes;   /* counter value at this boot (for reports) */
static esp_reset_reason_t s_reason;
static char s_summary[240];

static const char *reason_name(esp_reset_reason_t r)
{
    switch (r) {
    case ESP_RST_POWERON: return "power_on";
    case ESP_RST_EXT: return "external_pin";
    case ESP_RST_SW: return "software";
    case ESP_RST_PANIC: return "panic";
    case ESP_RST_INT_WDT: return "interrupt_watchdog";
    case ESP_RST_TASK_WDT: return "task_watchdog";
    case ESP_RST_WDT: return "other_watchdog";
    case ESP_RST_DEEPSLEEP: return "deep_sleep";
    case ESP_RST_BROWNOUT: return "brownout";
    case ESP_RST_SDIO: return "sdio";
    default: return "unknown";
    }
}

static bool is_crash(esp_reset_reason_t r)
{
    return r == ESP_RST_PANIC || r == ESP_RST_INT_WDT || r == ESP_RST_TASK_WDT ||
           r == ESP_RST_WDT || r == ESP_RST_BROWNOUT;
}

static void load_coredump(void)
{
#if CONFIG_ESP_COREDUMP_ENABLE_TO_FLASH
    if (esp_core_dump_image_check() != ESP_OK) {
        return;
    }
    esp_core_dump_summary_t *sum = calloc(1, sizeof(*sum));
    if (sum == NULL) {
        return;
    }
    if (esp_core_dump_get_summary(sum) == ESP_OK) {
        /* cause: Xtensa EXCCAUSE (e.g. 28/29 = load/store to a bad address,
         * 0 with a watchdog reset = the CPU was stuck at pc). */
        int n = snprintf(s_summary, sizeof(s_summary), "task=%s pc=0x%08lx cause=%lu vaddr=0x%08lx bt=",
                         sum->exc_task, (unsigned long)sum->exc_pc,
                         (unsigned long)sum->ex_info.exc_cause,
                         (unsigned long)sum->ex_info.exc_vaddr);
        for (uint32_t i = 0; i < sum->exc_bt_info.depth && i < 12 && n < (int)sizeof(s_summary) - 12; i++) {
            n += snprintf(s_summary + n, sizeof(s_summary) - n, "%s0x%08lx", i ? " " : "",
                          (unsigned long)sum->exc_bt_info.bt[i]);
        }
        if (sum->exc_bt_info.corrupted && n < (int)sizeof(s_summary) - 12) {
            snprintf(s_summary + n, sizeof(s_summary) - n, " (corrupted)");
        }
    }
    free(sum);
#endif
}

static void stable_task(void *arg)
{
    (void)arg;
    vTaskDelay(pdMS_TO_TICKS(STABLE_S * 1000));
    s_crashes = 0;
    ESP_LOGI(TAG, "stable for %d s, crash counter cleared", STABLE_S);
    vTaskDelete(NULL);
}

static void usb_report_task(void *arg)
{
    (void)arg;
    for (;;) {
        printf("\n[crashlog] USB-only safe mode after %lu early crashes; last reset: %s\n"
               "[crashlog] %s\n"
               "[crashlog] send \"@CMD crash\" / \"@CMD crash.clear\" / \"@CMD reboot\"\n",
               (unsigned long)s_boot_crashes, reason_name(s_reason),
               s_summary[0] ? s_summary : "(no core dump stored)");
        fflush(stdout);
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

boot_mode_t crashlog_init(void)
{
    s_reason = esp_reset_reason();
    if (s_magic != MAGIC) {
        s_magic = MAGIC;
        s_crashes = 0;
    }
    if (is_crash(s_reason)) {
        s_crashes++;
    } else if (s_reason == ESP_RST_POWERON || s_reason == ESP_RST_EXT) {
        /* A deliberate reset: keep counting only if we were already crash-looping. */
        if (s_crashes > 0 && s_crashes < 2) {
            s_crashes = 0;
        }
    } else {
        s_crashes = 0;
    }

    load_coredump();

    s_boot_crashes = s_crashes;
    s_mode = s_crashes >= 4 ? BOOT_USB_ONLY : s_crashes >= 2 ? BOOT_SAFE : BOOT_NORMAL;
    ESP_LOGW(TAG, "last reset: %s, early crashes: %lu, mode: %s", reason_name(s_reason),
             (unsigned long)s_crashes, crashlog_mode_name());
    if (s_summary[0]) {
        ESP_LOGW(TAG, "stored crash: %s", s_summary);
    }

    xTaskCreate(stable_task, "stable", 2048, NULL, 1, NULL);
    if (s_mode == BOOT_USB_ONLY) {
        xTaskCreate(usb_report_task, "crash_usb", 3072, NULL, 1, NULL);
    }
    return s_mode;
}

boot_mode_t crashlog_mode(void)
{
    return s_mode;
}

const char *crashlog_mode_name(void)
{
    return s_mode == BOOT_USB_ONLY ? "usb_only" : s_mode == BOOT_SAFE ? "safe" : "normal";
}

const char *crashlog_reset_reason(void)
{
    return reason_name(s_reason);
}

int crashlog_crash_count(void)
{
    return (int)s_boot_crashes;
}

const char *crashlog_summary(void)
{
    return s_summary;
}

void crashlog_json(jbuf_t *jb)
{
    jb_str(jb, "boot_mode", crashlog_mode_name());
    jb_str(jb, "reset_reason", reason_name(s_reason));
    jb_int(jb, "early_crashes", s_boot_crashes);
    jb_str(jb, "crash", s_summary);
}

void crashlog_clear(void)
{
#if CONFIG_ESP_COREDUMP_ENABLE_TO_FLASH
    esp_core_dump_image_erase();
#endif
    s_summary[0] = '\0';
    s_crashes = 0;
    s_boot_crashes = 0;
}
