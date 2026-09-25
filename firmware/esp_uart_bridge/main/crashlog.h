#pragma once

#include <stdbool.h>

#include "jbuf.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Boot diagnostics and crash-loop protection.
 *
 * Every boot records why the previous run ended (esp_reset_reason) and, after a
 * panic, the core-dump summary (task, PC, backtrace) that the panic handler
 * wrote to the "coredump" flash partition. Consecutive early crashes (within
 * STABLE_S seconds of boot) are counted in RTC memory:
 *
 *   0-1 crashes  BOOT_NORMAL     everything starts
 *   2-3 crashes  BOOT_SAFE       Wi-Fi + discovery + web/commands only (no UART,
 *                                SWD, mDNS), Wi-Fi TX power reduced
 *   4+  crashes  BOOT_USB_ONLY   no Wi-Fi either; USB console prints the crash
 *                                report every 5 s and answers "@CMD" lines
 *
 * Running STABLE_S seconds clears the counter, so the next reset is normal. */

typedef enum {
    BOOT_NORMAL = 0,
    BOOT_SAFE,
    BOOT_USB_ONLY,
} boot_mode_t;

/* Call first in app_main (after nvs_flash_init). */
boot_mode_t crashlog_init(void);
boot_mode_t crashlog_mode(void);
const char *crashlog_mode_name(void);
const char *crashlog_reset_reason(void);   /* why the previous run ended */
int crashlog_crash_count(void);

/* One line, e.g. "panic task=wifi_mgr pc=0x40081234 bt=0x4008.. 0x4009.." or "". */
const char *crashlog_summary(void);

/* Writes {mode, reset_reason, crash_count, crash{...}} fields into the open object. */
void crashlog_json(jbuf_t *jb);

/* Erases the stored core dump and the crash counter. */
void crashlog_clear(void);

#ifdef __cplusplus
}
#endif
