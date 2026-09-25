#pragma once

#include "freertos/FreeRTOS.h"

/* ESP32-S3 has two cores: Wi-Fi and lwIP are pinned to core 0 (sdkconfig),
 * the byte pumps and the SWD bit-bang run on core 1 so Wi-Fi interrupts do
 * not stretch SWD clock phases or delay UART draining. Single-core chips
 * (ESP32-S2) just let the scheduler decide. */
#if CONFIG_FREERTOS_UNICORE || (portNUM_PROCESSORS < 2)
#define BRIDGE_IO_CORE tskNO_AFFINITY
#else
#define BRIDGE_IO_CORE 1
#endif
