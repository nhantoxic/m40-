#pragma once

/* SWD-over-TCP bridge for the robot MCU (includes memory write operations). */
void swd_bridge_start(void);

#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

/* Runs one text command (same syntax as tcp/2325, e.g. "ID", "READ 0x08000000 64")
 * without a socket. *out receives the exact bytes the TCP client would get
 * (caller frees). Commands that stream a payload in (WRITE/MWRITE) fail.
 * ESP_ERR_NOT_FINISHED: an OpenOCD session currently owns the probe. */
esp_err_t swd_bridge_exec(const char *line, uint8_t **out, size_t *out_len,
                          size_t max_out);
