/* ARM SWD bridge.
 *
 * GPIO16 is SWDIO and GPIO18 is SWCLK in the S2-mini wiring. The text endpoint
 * exposes memory reads, core debug controls, SRAM writes, and Flash writes.
 * The remote_bitbang endpoint can also write the target if OpenOCD requests
 * it. Neither endpoint is authenticated; use only on a trusted network.
 */

#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "esp_cpu.h"

#include "lwip/sockets.h"
#include "soc/gpio_reg.h"

#include "swd_bridge.h"

static const char *TAG = "swd";

#define SWD_ACK_OK       0x1U
#define SWD_ACK_WAIT     0x2U
#define SWD_ACK_FAULT    0x4U
#define SWD_ACK_PROTOCOL 0x7U
#define SWD_DP_IDCODE    0x00U
#define SWD_DP_ABORT     0x00U
#define SWD_DP_CTRLSTAT  0x04U
#define SWD_DP_SELECT    0x08U
#define SWD_DP_RDBUFF    0x0CU
#define SWD_AP_CSW       0x00U
#define SWD_AP_TAR       0x04U
#define SWD_AP_DRW       0x0CU
#define SWD_AP_IDR       0xFCU
#define SWD_MEM_FLASH_BASE 0x08000000U
#define SWD_MEM_FLASH_END  0x08080000U
#define SWD_MEM_SRAM_BASE  0x20000000U
#define SWD_MEM_SRAM_END   0x20010000U /* 64 KiB GD32F303ZET6 SRAM */
#define SWD_MEM_NVM_BASE   0x0807C000U
#define SWD_FMC_PID        0x40022100U
#define SWD_MAX_LINE 96
#define SWD_IDLE_CYCLES 8U

typedef enum {
    SWD_OK = 0,
    SWD_ERR_ACK,
    SWD_ERR_PARITY,
    SWD_ERR_POWER,
    SWD_ERR_RANGE,
    SWD_ERR_CORE_RUNNING,
    SWD_ERR_VERIFY,
} swd_status_t;

/* Subtract only after proving address < end. This prevents overflow when
 * validating untrusted address/length pairs received over TCP. */
static bool swd_range_contains(uint32_t address, size_t length,
                               uint32_t start, uint32_t end)
{
    return length != 0 && address >= start && address < end &&
           length <= (size_t)(end - address);
}

static SemaphoreHandle_t s_swd_lock;
static uint32_t s_half_period_cycles;
static volatile unsigned s_last_ack;
static volatile bool s_swdio_output;

/* Direct-register bit-bang -> SWD clock far above gpio_set_level(). SWDIO and
 * SWCLK are <32 on the S2-mini wiring, so the GPIO0..31 register bank applies. */
#define SWDIO_MASK (1U << CONFIG_BRIDGE_SWDIO_GPIO)
#define SWCLK_MASK (1U << CONFIG_BRIDGE_SWCLK_GPIO)
_Static_assert(CONFIG_BRIDGE_SWDIO_GPIO < 32 && CONFIG_BRIDGE_SWCLK_GPIO < 32,
               "SWD bit-bang uses the GPIO0..31 register bank");

static inline void swd_delay(void)
{
    if (s_half_period_cycles == 0) {
        return;
    }

    uint32_t start = (uint32_t)esp_cpu_get_cycle_count();

    while ((uint32_t)((uint32_t)esp_cpu_get_cycle_count() - start) <
           s_half_period_cycles) {
        ;
    }
}

static inline void swd_clock_cycle(void)
{
    REG_WRITE(GPIO_OUT_W1TC_REG, SWCLK_MASK);
    swd_delay();
    REG_WRITE(GPIO_OUT_W1TS_REG, SWCLK_MASK);
    swd_delay();
}

static void swdio_drive(int level)
{
    /* Set the latch while still high-Z so changing direction cannot create a
     * low pulse on SWDIO. */
    if (level) {
        REG_WRITE(GPIO_OUT_W1TS_REG, SWDIO_MASK);
    } else {
        REG_WRITE(GPIO_OUT_W1TC_REG, SWDIO_MASK);
    }
    gpio_set_pull_mode(CONFIG_BRIDGE_SWDIO_GPIO, GPIO_FLOATING);
    gpio_set_direction(CONFIG_BRIDGE_SWDIO_GPIO, GPIO_MODE_OUTPUT);
    s_swdio_output = true;
}

static void swdio_release(void)
{
    gpio_set_direction(CONFIG_BRIDGE_SWDIO_GPIO, GPIO_MODE_INPUT);
    gpio_set_pull_mode(CONFIG_BRIDGE_SWDIO_GPIO, GPIO_PULLUP_ONLY);
    s_swdio_output = false;
}

static void swd_write_bit(unsigned bit)
{
    if (bit) {
        REG_WRITE(GPIO_OUT_W1TS_REG, SWDIO_MASK);
    } else {
        REG_WRITE(GPIO_OUT_W1TC_REG, SWDIO_MASK);
    }

    REG_WRITE(GPIO_OUT_W1TC_REG, SWCLK_MASK);
    swd_delay();
    REG_WRITE(GPIO_OUT_W1TS_REG, SWCLK_MASK);
    swd_delay();
}

static unsigned swd_read_bit(void)
{
    REG_WRITE(GPIO_OUT_W1TC_REG, SWCLK_MASK);
    swd_delay();

    const unsigned bit =
        (REG_READ(GPIO_IN_REG) & SWDIO_MASK) ? 1U : 0U;

    REG_WRITE(GPIO_OUT_W1TS_REG, SWCLK_MASK);
    swd_delay();

    return bit;
}

static void swd_write_bits(uint32_t value, unsigned count)
{
    for (unsigned i = 0; i < count; ++i) {
        swd_write_bit((value >> i) & 1U);
    }
}

static uint32_t swd_read_bits(unsigned count)
{
    uint32_t value = 0;
    for (unsigned i = 0; i < count; ++i) {
        value |= swd_read_bit() << i;
    }
    return value;
}

static void swd_turn_to_target(void)
{
    swdio_release();

    /* One input turnaround cycle. */
    (void)swd_read_bit();
}

static void swd_turn_to_host_preload(unsigned level)
{
    swdio_release();

    /* Preload first host-owned data bit while SWDIO is still Hi-Z. */
    if (level) {
        REG_WRITE(GPIO_OUT_W1TS_REG, SWDIO_MASK);
    } else {
        REG_WRITE(GPIO_OUT_W1TC_REG, SWDIO_MASK);
    }

    /* One target->host turnaround cycle, ending SWCLK HIGH. */
    swd_clock_cycle();

    gpio_set_pull_mode(CONFIG_BRIDGE_SWDIO_GPIO, GPIO_FLOATING);
    gpio_set_direction(CONFIG_BRIDGE_SWDIO_GPIO, GPIO_MODE_OUTPUT);
    s_swdio_output = true;
}

static void swd_turn_to_host(void)
{
    swdio_release();

    /* One turnaround after target-driven read data/parity. */
    (void)swd_read_bit();

    /* Host resumes ownership while SWCLK is HIGH. */
    swdio_drive(0);
}

static void swd_idle_cycles(unsigned count)
{
    /* SWD idle cycles are clocked with SWDIO low. */
    swdio_drive(0);
    swd_delay();
    for (unsigned i = 0; i < count; ++i) {
        swd_clock_cycle();
    }
}

static void swd_line_reset(void)
{
    swdio_drive(1);
    swd_delay();
    for (unsigned i = 0; i < 64; ++i) {
        swd_clock_cycle();
    }
}

static void swd_select_protocol(void)
{
    /* ARM ADIv5 JTAG-to-SWD selection sequence, LSB first. */
    swdio_drive(1);
    for (unsigned i = 0; i < 64; ++i) {
        swd_clock_cycle();
    }
    swd_write_bits(0xE79EU, 16);
    swd_line_reset();
    swdio_drive(0);
    swd_clock_cycle();
    swd_clock_cycle();
    /* Give the target time to settle after the protocol switch before the
     * first transfer. */
    vTaskDelay(pdMS_TO_TICKS(2));
}

static uint8_t make_request(bool ap, bool read, uint8_t addr);

static void swd_session_begin(void)
{
    swd_select_protocol();
    swdio_drive(1);
    REG_WRITE(GPIO_OUT_W1TS_REG, SWCLK_MASK);
    swd_delay();
}

/* DIAGNOSTIC: after line-reset + JTAG->SWD switch, send a DP IDCODE read
 * request and return the next 8 raw bits (turnaround + ACK + ...). Lets the
 * host see whether the ACK is present / off-by-one. */
static uint32_t swd_raw_probe(void)
{
    swd_select_protocol();
    swdio_drive(1);
    swd_write_bits(make_request(false, true, SWD_DP_IDCODE), 8);
    swd_turn_to_target();
    return swd_read_bits(8) & 0xFFU;
}

static unsigned request_parity(bool ap, bool read, uint8_t addr)
{
    return ((unsigned)ap ^ (unsigned)read ^ ((addr >> 2) & 1U) ^
            ((addr >> 3) & 1U)) & 1U;
}

static uint8_t make_request(bool ap, bool read, uint8_t addr)
{
    return (uint8_t)(1U | ((unsigned)ap << 1) | ((unsigned)read << 2) |
                     (((addr >> 2) & 3U) << 3) |
                     (request_parity(ap, read, addr) << 5) | (1U << 7));
}

static swd_status_t swd_transfer(bool ap, bool read, uint8_t addr,
                                 uint32_t *data, unsigned *ack_out)
{
    swdio_drive(1);
    swd_write_bits(make_request(ap, read, addr), 8);
    swd_turn_to_target();

    const unsigned ack = (unsigned)swd_read_bits(3);
    s_last_ack = ack;
    if (ack_out != NULL) {
        *ack_out = ack;
    }
    if (ack != SWD_ACK_OK) {
        swd_turn_to_host();
        swd_idle_cycles(SWD_IDLE_CYCLES);
        return SWD_ERR_ACK;
    }

    if (read) {
        uint32_t value = swd_read_bits(32);
        const unsigned parity = swd_read_bit();
        if (parity != (__builtin_parity(value) & 1U)) {
            swd_turn_to_host();
            return SWD_ERR_PARITY;
        }
        if (data != NULL) {
            *data = value;
        }
        swd_turn_to_host();
        swd_idle_cycles(SWD_IDLE_CYCLES);
    } else {
        /* ACK is target-driven. SWD requires one turnaround cycle before
         * the host starts the write-data phase. */
        const uint32_t value = data != NULL ? *data : 0;
        swd_turn_to_host_preload(value & 1U);
        swd_write_bits(value, 32);
        swd_write_bit(__builtin_parity(value) & 1U);
        swd_idle_cycles(SWD_IDLE_CYCLES);
    }
    return SWD_OK;
}

static swd_status_t swd_transfer_retry(bool ap, bool read, uint8_t addr,
                                       uint32_t *data, unsigned *ack_out)
{
    unsigned local_ack = 0;
    unsigned *ack = ack_out != NULL ? ack_out : &local_ack;
    for (unsigned attempt = 0; attempt < 80; ++attempt) {
        swd_status_t status = swd_transfer(ap, read, addr, data, ack);
        if (status == SWD_OK) {
            return status;
        }
        if (*ack != SWD_ACK_WAIT) {
            return status;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    return SWD_ERR_ACK;
}

static swd_status_t dp_read(uint8_t addr, uint32_t *value)
{
    return swd_transfer_retry(false, true, addr, value, NULL);
}

static swd_status_t dp_write(uint8_t addr, uint32_t value)
{
    return swd_transfer_retry(false, false, addr, &value, NULL);
}

/* Minimal diagnostic path: protocol select + a single DP IDCODE read.
 * This deliberately avoids ABORT, power-up, SELECT and all AP accesses. */
static swd_status_t swd_read_dpid_only(uint32_t *dp_id)
{
    swd_session_begin();
    return dp_read(SWD_DP_IDCODE, dp_id);
}

static swd_status_t ap_read(uint8_t addr, uint32_t *value)
{
    uint32_t discarded = 0;
    swd_status_t status = swd_transfer_retry(true, true, addr, &discarded, NULL);
    if (status != SWD_OK) {
        return status;
    }
    return dp_read(SWD_DP_RDBUFF, value);
}

static swd_status_t ap_write(uint8_t addr, uint32_t value)
{
    return swd_transfer_retry(true, false, addr, &value, NULL);
}

static swd_status_t swd_power_and_select(uint32_t *dp_id, uint32_t *ap_id)
{
    swd_session_begin();

    /* Read IDCODE first, exactly like OpenOCD does right after the switch;
     * only then clear errors / power up. */
    swd_status_t status = dp_read(SWD_DP_IDCODE, dp_id);
    if (status != SWD_OK) {
        return status;
    }

    /* Clear sticky errors, request system/debug power, and select AP0 bank 0. */
    (void)dp_write(SWD_DP_ABORT, 0x0000001EU);
    status = dp_write(SWD_DP_CTRLSTAT, 0x50000000U);
    if (status != SWD_OK) {
        return status;
    }

    uint32_t ctrl = 0;
    for (unsigned i = 0; i < 100; ++i) {
        status = dp_read(SWD_DP_CTRLSTAT, &ctrl);
        if (status == SWD_OK && (ctrl & 0xF0000000U) == 0xF0000000U) {
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    if ((ctrl & 0xF0000000U) != 0xF0000000U) {
        return SWD_ERR_POWER;
    }

    /* AP IDR is at AP register 0xFC: bank 0xF, address bits A[3:2]=3.
     * The SWD request carries only A[3:2], so APBANKSEL must be 0xF. */
    status = dp_write(SWD_DP_SELECT, 0x000000F0U);
    if (status != SWD_OK) {
        return status;
    }

    status = ap_read(SWD_AP_IDR, ap_id);
    if (status != SWD_OK) {
        return status;
    }

    /* Restore AP bank 0 before CSW/TAR/DRW accesses. */
    return dp_write(SWD_DP_SELECT, 0x00000000U);
}

/* ---- GD32 FMC (flash memory controller) helpers for high-speed flashing ---- */
#define FMC_BASE   0x40022000U
#define FMC_KEY    (FMC_BASE + 0x04U)
#define FMC_STAT   (FMC_BASE + 0x0CU)
#define FMC_CTL    (FMC_BASE + 0x10U)
#define FMC_ADDR   (FMC_BASE + 0x14U)
#define FMC_STAT_BUSY   (1U << 0)
#define FMC_STAT_PGERR  (1U << 2)
#define FMC_STAT_WRPERR (1U << 4)
#define FMC_STAT_EOP    (1U << 5)
#define FMC_CTL_PG      (1U << 0)
#define FMC_CTL_PER     (1U << 1)
#define FMC_CTL_START   (1U << 6)
#define FMC_CTL_LK      (1U << 7)
#define CSW_32BIT   0x23000052U
#define CSW_16BIT   0x23000051U

/* AHB memory access through the AP (TAR + DRW). */
static swd_status_t mem_write32(uint32_t addr, uint32_t value)
{
    swd_status_t s = ap_write(SWD_AP_TAR, addr);
    if (s != SWD_OK) {
        return s;
    }
    return ap_write(SWD_AP_DRW, value);
}

static swd_status_t mem_read32(uint32_t addr, uint32_t *value)
{
    swd_status_t s = ap_write(SWD_AP_TAR, addr);
    if (s != SWD_OK) {
        return s;
    }
    return ap_read(SWD_AP_DRW, value);
}


/* ============================================================
 * SRAM write + Cortex-M core debug helpers.
 *
 * MWRITE is intentionally restricted to SRAM.
 * Existing WRITE remains flash-only.
 * ============================================================ */

#define DBG_DHCSR 0xE000EDF0U
#define DBG_DCRSR 0xE000EDF4U
#define DBG_DCRDR 0xE000EDF8U

#define DHCSR_DBGKEY      0xA05F0000U
#define DHCSR_C_DEBUGEN   (1U << 0)
#define DHCSR_C_HALT      (1U << 1)
#define DHCSR_C_STEP      (1U << 2)
#define DHCSR_C_MASKINTS  (1U << 3)
#define DHCSR_S_REGRDY    (1U << 16)
#define DHCSR_S_HALT      (1U << 17)


static swd_status_t swd_prepare_mem_access_locked(void)
{
    uint32_t dp_id = 0;
    uint32_t ap_id = 0;

    swd_status_t st = swd_power_and_select(&dp_id, &ap_id);
    if (st != SWD_OK) {
        return st;
    }

    return ap_write(SWD_AP_CSW, CSW_32BIT);
}


/* Arbitrary-byte SRAM write using 32-bit read-modify-write.
 * Does NOT access the flash programming controller.
 */
static swd_status_t swd_ram_write_locked(uint32_t address,
                                         const uint8_t *data,
                                         size_t length)
{
    if (!swd_range_contains(address, length,
                            SWD_MEM_SRAM_BASE, SWD_MEM_SRAM_END)) {
        return SWD_ERR_RANGE;
    }

    swd_status_t st = swd_prepare_mem_access_locked();
    if (st != SWD_OK) {
        return st;
    }

    size_t done = 0;

    while (done < length) {

        uint32_t current = address + (uint32_t)done;
        uint32_t aligned = current & ~3U;

        uint32_t word = 0;

        st = mem_read32(aligned, &word);
        if (st != SWD_OK) {
            return st;
        }

        unsigned byte_off = current & 3U;
        unsigned count = 4U - byte_off;

        if (count > length - done) {
            count = (unsigned)(length - done);
        }

        uint8_t tmp[4];

        tmp[0] = (uint8_t)(word);
        tmp[1] = (uint8_t)(word >> 8);
        tmp[2] = (uint8_t)(word >> 16);
        tmp[3] = (uint8_t)(word >> 24);

        for (unsigned i = 0; i < count; ++i) {
            tmp[byte_off + i] = data[done + i];
        }

        word =
            ((uint32_t)tmp[0]) |
            ((uint32_t)tmp[1] << 8) |
            ((uint32_t)tmp[2] << 16) |
            ((uint32_t)tmp[3] << 24);

        st = mem_write32(aligned, word);
        if (st != SWD_OK) {
            return st;
        }

        uint32_t readback = 0;
        st = mem_read32(aligned, &readback);
        if (st != SWD_OK) {
            return st;
        }
        if (readback != word) {
            return SWD_ERR_VERIFY;
        }

        done += count;
    }

    return SWD_OK;
}


static swd_status_t core_halt_locked(uint32_t *dhcsr_out)
{
    swd_status_t st = swd_prepare_mem_access_locked();
    if (st != SWD_OK) {
        return st;
    }

    st = mem_write32(DBG_DHCSR,
                     DHCSR_DBGKEY |
                     DHCSR_C_DEBUGEN |
                     DHCSR_C_HALT);

    if (st != SWD_OK) {
        return st;
    }

    for (unsigned i = 0; i < 10000U; ++i) {

        uint32_t dhcsr = 0;

        st = mem_read32(DBG_DHCSR, &dhcsr);
        if (st != SWD_OK) {
            return st;
        }

        if (dhcsr & DHCSR_S_HALT) {

            if (dhcsr_out != NULL) {
                *dhcsr_out = dhcsr;
            }

            return SWD_OK;
        }
    }

    return SWD_ERR_ACK;
}


static swd_status_t core_resume_locked(void)
{
    swd_status_t st = swd_prepare_mem_access_locked();

    if (st != SWD_OK) {
        return st;
    }

    /*
     * Clear C_MASKINTS while the core is still halted.
     */
    st = mem_write32(
        DBG_DHCSR,
        DHCSR_DBGKEY |
        DHCSR_C_DEBUGEN |
        DHCSR_C_HALT
    );

    if (st != SWD_OK) {
        return st;
    }

    /*
     * Release the halt.
     */
    return mem_write32(
        DBG_DHCSR,
        DHCSR_DBGKEY |
        DHCSR_C_DEBUGEN
    );
}



static swd_status_t core_step_locked(uint32_t *dhcsr_out)
{
    swd_status_t st = swd_prepare_mem_access_locked();

    if (st != SWD_OK) {
        return st;
    }

    /*
     * MASKINTS must be changed while the core is halted.
     * Keep the target halted and mask configurable interrupts first.
     */
    st = mem_write32(
        DBG_DHCSR,
        DHCSR_DBGKEY |
        DHCSR_C_DEBUGEN |
        DHCSR_C_HALT |
        DHCSR_C_MASKINTS
    );

    if (st != SWD_OK) {
        return st;
    }

    /*
     * Execute exactly one instruction.
     * C_HALT=0 + C_STEP=1.
     * Keep MASKINTS asserted during the step.
     */
    st = mem_write32(
        DBG_DHCSR,
        DHCSR_DBGKEY |
        DHCSR_C_DEBUGEN |
        DHCSR_C_STEP |
        DHCSR_C_MASKINTS
    );

    if (st != SWD_OK) {
        return st;
    }

    /*
     * Avoid accepting the stale S_HALT state from before
     * the step command reached the Cortex-M core.
     */
    esp_rom_delay_us(10);

    for (unsigned i = 0; i < 100000U; ++i) {

        uint32_t dhcsr = 0;

        st = mem_read32(DBG_DHCSR, &dhcsr);

        if (st != SWD_OK) {
            return st;
        }

        if (dhcsr & DHCSR_S_HALT) {

            if (dhcsr_out != NULL) {
                *dhcsr_out = dhcsr;
            }

            return SWD_OK;
        }
    }

    return SWD_ERR_ACK;
}

static swd_status_t core_wait_regrdy_locked(void)
{
    for (unsigned i = 0; i < 10000U; ++i) {

        uint32_t dhcsr = 0;

        swd_status_t st = mem_read32(DBG_DHCSR, &dhcsr);
        if (st != SWD_OK) {
            return st;
        }

        if (dhcsr & DHCSR_S_REGRDY) {
            return SWD_OK;
        }
    }

    return SWD_ERR_ACK;
}

static swd_status_t core_require_halted_locked(void)
{
    uint32_t dhcsr = 0;
    swd_status_t st = mem_read32(DBG_DHCSR, &dhcsr);
    if (st != SWD_OK) {
        return st;
    }
    return (dhcsr & DHCSR_S_HALT) ? SWD_OK : SWD_ERR_CORE_RUNNING;
}


static swd_status_t core_reg_read_locked(unsigned regno,
                                         uint32_t *value)
{
    if (regno > 19U || value == NULL) {
        return SWD_ERR_RANGE;
    }

    swd_status_t st = swd_prepare_mem_access_locked();
    if (st != SWD_OK) {
        return st;
    }

    st = core_require_halted_locked();
    if (st != SWD_OK) {
        return st;
    }

    st = mem_write32(DBG_DCRSR, regno);
    if (st != SWD_OK) {
        return st;
    }

    st = core_wait_regrdy_locked();
    if (st != SWD_OK) {
        return st;
    }

    return mem_read32(DBG_DCRDR, value);
}


static swd_status_t core_reg_write_locked(unsigned regno,
                                          uint32_t value)
{
    if (regno > 19U) {
        return SWD_ERR_RANGE;
    }

    swd_status_t st = swd_prepare_mem_access_locked();
    if (st != SWD_OK) {
        return st;
    }

    st = core_require_halted_locked();
    if (st != SWD_OK) {
        return st;
    }

    st = mem_write32(DBG_DCRDR, value);
    if (st != SWD_OK) {
        return st;
    }

    st = mem_write32(DBG_DCRSR,
                     (1U << 16) | regno);

    if (st != SWD_OK) {
        return st;
    }

    return core_wait_regrdy_locked();
}



static swd_status_t core_run_until_locked(uint32_t stop0,
                                          uint32_t stop1,
                                          uint32_t stop2,
                                          uint32_t max_steps,
                                          uint32_t *pc_out,
                                          uint32_t *steps_out)
{
    uint32_t pc = 0;

    for (uint32_t i = 0; i <= max_steps; ++i) {
        swd_status_t st = core_reg_read_locked(15U, &pc);
        if (st != SWD_OK) {
            return st;
        }

        if (pc == stop0 || pc == stop1 || pc == stop2) {
            if (pc_out != NULL) {
                *pc_out = pc;
            }
            if (steps_out != NULL) {
                *steps_out = i;
            }
            return SWD_OK;
        }

        if (i == max_steps) {
            break;
        }

        st = core_step_locked(NULL);
        if (st != SWD_OK) {
            return st;
        }

        /* ESP32-S2 is single-core. Let the idle task run periodically so a
         * long single-step scan does not starve the task watchdog. The ARM
         * core is halted between steps, so this does not change target state. */
        if ((i & 0x3FFU) == 0x3FFU) {
            vTaskDelay(1);
        }
    }

    if (pc_out != NULL) {
        *pc_out = pc;
    }
    if (steps_out != NULL) {
        *steps_out = max_steps;
    }
    return SWD_ERR_ACK;
}

static bool fmc_wait_idle(void)
{
    uint32_t s = 0;
    for (unsigned i = 0; i < 200000U; ++i) {
        if (mem_read32(FMC_STAT, &s) != SWD_OK) {
            return false;
        }
        if ((s & FMC_STAT_BUSY) == 0U) {
            return true;
        }
    }
    return false;
}

/* Program flash from a RAM buffer. Erases the covering 2 KiB pages first.
 * Halfwords only; address must be even and inside the flash range.
 * NOTE: requires the SWD engine to actually talk to the target (see ID). */
static swd_status_t swd_flash_write_locked(uint32_t address, const uint8_t *data,
                                           size_t length)
{
    if ((address & 1U) != 0 ||
        !swd_range_contains(address, length,
                            SWD_MEM_FLASH_BASE, SWD_MEM_NVM_BASE)) {
        return SWD_ERR_RANGE;
    }

    uint32_t dp_id = 0, ap_id = 0;
    swd_status_t st = swd_power_and_select(&dp_id, &ap_id);
    if (st != SWD_OK) {
        return st;
    }

    /* FMC register accesses are 32-bit. */
    st = ap_write(SWD_AP_CSW, CSW_32BIT);
    if (st != SWD_OK) {
        return st;
    }
    if (!fmc_wait_idle()) {
        return SWD_ERR_ACK;
    }
    if (mem_write32(FMC_KEY, 0x45670123U) != SWD_OK ||
        mem_write32(FMC_KEY, 0xCDEF89ABU) != SWD_OK) {
        return SWD_ERR_ACK;
    }

    const uint32_t first = address & ~0x7FFU;
    const uint32_t last = (address + (uint32_t)length + 0x7FFU) & ~0x7FFU;
    for (uint32_t page = first; page < last; page += 0x800U) {
        uint32_t ctl = 0;
        if (mem_read32(FMC_CTL, &ctl) != SWD_OK) {
            return SWD_ERR_ACK;
        }
        if (mem_write32(FMC_CTL, (ctl & ~(FMC_CTL_PG | FMC_CTL_START | FMC_CTL_LK)) | FMC_CTL_PER) != SWD_OK ||
            mem_write32(FMC_ADDR, page) != SWD_OK ||
            mem_write32(FMC_CTL, (ctl & ~(FMC_CTL_PG | FMC_CTL_LK)) | FMC_CTL_PER | FMC_CTL_START) != SWD_OK) {
            return SWD_ERR_ACK;
        }
        if (!fmc_wait_idle()) {
            return SWD_ERR_ACK;
        }
        uint32_t stat = 0;
        if (mem_read32(FMC_STAT, &stat) != SWD_OK) {
            return SWD_ERR_ACK;
        }
        if (stat & (FMC_STAT_PGERR | FMC_STAT_WRPERR)) {
            return SWD_ERR_ACK;
        }
        (void)mem_write32(FMC_STAT, FMC_STAT_EOP);
    }

    /* Program halfwords. */
    uint32_t ctl = 0;
    if (mem_read32(FMC_CTL, &ctl) != SWD_OK) {
        return SWD_ERR_ACK;
    }
    if (mem_write32(FMC_CTL, (ctl & ~(FMC_CTL_PER | FMC_CTL_START | FMC_CTL_LK)) | FMC_CTL_PG) != SWD_OK) {
        return SWD_ERR_ACK;
    }
    if (ap_write(SWD_AP_CSW, CSW_16BIT) != SWD_OK) {
        return SWD_ERR_ACK;
    }
    for (size_t i = 0; i < length; i += 2) {
        uint32_t hw = (uint32_t)data[i] | ((uint32_t)data[i + 1] << 8);
        /* A 16-bit MEM-AP access places the halfword in the DRW byte lane
         * selected by address[1]. For odd halfwords (addr & 2) the data must
         * sit in DRW[31:16]; otherwise the flash programs 0x0000 there. */
        uint32_t drw = ((address + (uint32_t)i) & 2U) ? (hw << 16) : hw;
        if (mem_write32(address + (uint32_t)i, drw) != SWD_OK) {
            return SWD_ERR_ACK;
        }
        if (ap_write(SWD_AP_CSW, CSW_32BIT) != SWD_OK) {
            return SWD_ERR_ACK;
        }
        if (!fmc_wait_idle()) {
            return SWD_ERR_ACK;
        }
        uint32_t stat = 0;
        if (mem_read32(FMC_STAT, &stat) != SWD_OK) {
            return SWD_ERR_ACK;
        }
        if (stat & (FMC_STAT_PGERR | FMC_STAT_WRPERR)) {
            return SWD_ERR_ACK;
        }
        (void)mem_write32(FMC_STAT, FMC_STAT_EOP);
        if (ap_write(SWD_AP_CSW, CSW_16BIT) != SWD_OK) {
            return SWD_ERR_ACK;
        }
    }
    (void)ap_write(SWD_AP_CSW, CSW_32BIT);

    uint32_t c2 = 0;
    if (mem_read32(FMC_CTL, &c2) == SWD_OK) {
        (void)mem_write32(FMC_CTL, c2 | FMC_CTL_LK);
    }
    return SWD_OK;
}

static swd_status_t swd_read_memory_locked(uint32_t address, uint8_t *out,
                                           size_t length)
{
    const bool flash_range =
        swd_range_contains(address, length,
                           SWD_MEM_FLASH_BASE, SWD_MEM_FLASH_END);

    const bool sram_range =
        swd_range_contains(address, length,
                           SWD_MEM_SRAM_BASE, SWD_MEM_SRAM_END);

    const bool pid_range =
        address == SWD_FMC_PID && length == 4;

    if (length == 0 || (address & 3U) != 0 || (length & 3U) != 0 ||
        (!flash_range && !sram_range && !pid_range)) {
        return SWD_ERR_RANGE;
    }

    uint32_t dp_id = 0;
    uint32_t ap_id = 0;
    swd_status_t status = swd_power_and_select(&dp_id, &ap_id);
    if (status != SWD_OK) {
        return status;
    }
    (void)dp_id;
    (void)ap_id;

    /* CSW: debug access, auto-increment, 32-bit transfers. */
    status = ap_write(SWD_AP_CSW, 0x23000052U);
    if (status != SWD_OK) {
        return status;
    }

    size_t done = 0;
    while (done < length) {
        uint32_t current = address + (uint32_t)done;
        size_t block = 0x400U - (current & 0x3FFU);
        if (block > length - done) {
            block = length - done;
        }
        status = ap_write(SWD_AP_TAR, current);
        if (status != SWD_OK) {
            return status;
        }
        for (size_t i = 0; i < block; i += 4) {
            uint32_t word = 0;
            status = ap_read(SWD_AP_DRW, &word);
            if (status != SWD_OK) {
                return status;
            }
            out[done + i + 0] = (uint8_t)(word >> 0);
            out[done + i + 1] = (uint8_t)(word >> 8);
            out[done + i + 2] = (uint8_t)(word >> 16);
            out[done + i + 3] = (uint8_t)(word >> 24);
        }
        done += block;
    }
    return SWD_OK;
}

static const char *swd_status_name(swd_status_t status)
{
    switch (status) {
    case SWD_OK: return "ok";
    case SWD_ERR_ACK: return "ack";
    case SWD_ERR_PARITY: return "parity";
    case SWD_ERR_POWER: return "power";
    case SWD_ERR_RANGE: return "range";
    case SWD_ERR_CORE_RUNNING: return "core-running";
    case SWD_ERR_VERIFY: return "verify";
    default: return "unknown";
    }
}

static const char *swd_ack_name(unsigned ack)
{
    switch (ack) {
    case SWD_ACK_OK: return "OK";
    case SWD_ACK_WAIT: return "WAIT";
    case SWD_ACK_FAULT: return "FAULT";
    case SWD_ACK_PROTOCOL: return "PROTOCOL";
    default: return "INVALID";
    }
}

static bool send_all(int sock, const void *data, size_t length)
{
    const uint8_t *p = (const uint8_t *)data;
    size_t sent = 0;
    while (sent < length) {
        int n = send(sock, p + sent, length - sent, 0);
        if (n < 0 && errno == EINTR) {
            continue;
        }
        if (n <= 0) {
            return false;
        }
        sent += (size_t)n;
    }
    return true;
}

/* A text client may now hold its connection for many commands, so an idle or
 * vanished peer must not lock the port out: drop it after this long without a
 * byte. Dead peers (Wi-Fi drop) are also caught by keepalive in ~11 s. */
#define SWD_TEXT_IDLE_TIMEOUT_S 60

static void configure_client_socket(int sock, int rcv_timeout_s)
{
    const int one = 1;
    const int idle = 5;
    const int intvl = 2;
    const int cnt = 3;

    setsockopt(sock, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
    setsockopt(sock, SOL_SOCKET, SO_KEEPALIVE, &one, sizeof(one));
    setsockopt(sock, IPPROTO_TCP, TCP_KEEPIDLE, &idle, sizeof(idle));
    setsockopt(sock, IPPROTO_TCP, TCP_KEEPINTVL, &intvl, sizeof(intvl));
    setsockopt(sock, IPPROTO_TCP, TCP_KEEPCNT, &cnt, sizeof(cnt));
    if (rcv_timeout_s > 0) {
        const struct timeval to = { .tv_sec = rcv_timeout_s, .tv_usec = 0 };
        setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &to, sizeof(to));
    }
}

static int recv_line(int sock, char *line, size_t capacity)
{
    size_t used = 0;
    while (used + 1 < capacity) {
        char ch;
        int n = recv(sock, &ch, 1, 0);
        if (n <= 0) {
            return -1;
        }
        if (ch == '\n') {
            line[used] = '\0';
            return (int)used;
        }
        if (ch != '\r') {
            line[used++] = ch;
        }
    }
    line[capacity - 1] = '\0';
    return -2;
}

/* Handles one command. Returns false when the connection must be closed:
 * the peer is gone, or a binary transfer was cut short so the byte stream no
 * longer lines up with the command framing. */
static bool handle_command(int sock)
{
    char line[SWD_MAX_LINE];
    int line_len = recv_line(sock, line, sizeof(line));
    if (line_len < 0) {
        return false;
    }

    if (strcmp(line, "RAW") == 0) {
        xSemaphoreTake(s_swd_lock, portMAX_DELAY);
        uint32_t raw = swd_raw_probe();
        xSemaphoreGive(s_swd_lock);
        char response[64];
        int n = snprintf(response, sizeof(response), "OK RAW=0x%02lX\n",
                         (unsigned long)raw);
        (void)send_all(sock, response, (size_t)n);
        return true;
    }

    if (strcmp(line, "PING") == 0) {
        (void)send_all(sock, "PONG PROXY2\n", 12);
        return true;
    }

    if (strcmp(line, "CTRL") == 0) {
        uint32_t dp_id = 0;
        uint32_t before = 0;
        uint32_t after = 0;
        const char *stage = "ID";

        xSemaphoreTake(s_swd_lock, portMAX_DELAY);

        swd_session_begin();
        swd_status_t status = dp_read(SWD_DP_IDCODE, &dp_id);

        if (status == SWD_OK) {
            stage = "ABORT";
            status = dp_write(SWD_DP_ABORT, 0x0000001EU);
        }
        if (status == SWD_OK) {
            stage = "BEFORE";
            status = dp_read(SWD_DP_CTRLSTAT, &before);
        }
        if (status == SWD_OK) {
            stage = "WRITE";
            status = dp_write(SWD_DP_CTRLSTAT, 0x50000000U);
        }
        if (status == SWD_OK) {
            vTaskDelay(pdMS_TO_TICKS(5));
            stage = "AFTER";
            status = dp_read(SWD_DP_CTRLSTAT, &after);
        }

        xSemaphoreGive(s_swd_lock);

        char response[160];
        int n;
        if (status == SWD_OK) {
            n = snprintf(response, sizeof(response),
                         "OK CTRL DPIDR=0x%08lX BEFORE=0x%08lX AFTER=0x%08lX ACK=0x%X\n",
                         (unsigned long)dp_id,
                         (unsigned long)before,
                         (unsigned long)after,
                         s_last_ack);
        } else {
            n = snprintf(response, sizeof(response),
                         "ERR CTRL stage=%s swd=%s ACK=0x%X BEFORE=0x%08lX AFTER=0x%08lX\n",
                         stage, swd_status_name(status), s_last_ack,
                         (unsigned long)before,
                         (unsigned long)after);
        }
        (void)send_all(sock, response, (size_t)n);
        return true;
    }
    if (strcmp(line, "DPID") == 0) {
        uint32_t dp_id = 0;
        xSemaphoreTake(s_swd_lock, portMAX_DELAY);
        swd_status_t status = swd_read_dpid_only(&dp_id);
        xSemaphoreGive(s_swd_lock);

        char response[96];
        int n;
        if (status == SWD_OK) {
            n = snprintf(response, sizeof(response),
                         "OK DPIDR=0x%08lX ACK=0x%X\n",
                         (unsigned long)dp_id, s_last_ack);
        } else {
            n = snprintf(response, sizeof(response),
                         "ERR SWD %s %s ACK=0x%X DIO=%d\n",
                         swd_status_name(status), swd_ack_name(s_last_ack),
                         s_last_ack, gpio_get_level(CONFIG_BRIDGE_SWDIO_GPIO));
        }
        (void)send_all(sock, response, (size_t)n);
        return true;
    }

    if (strcmp(line, "ID") == 0) {
        uint32_t dp_id = 0;
        uint32_t ap_id = 0;
        xSemaphoreTake(s_swd_lock, portMAX_DELAY);
        swd_status_t status = swd_power_and_select(&dp_id, &ap_id);
        xSemaphoreGive(s_swd_lock);
        char response[128];
        if (status == SWD_OK) {
            int n = snprintf(response, sizeof(response),
                             "OK DPIDR=0x%08lX APIDR=0x%08lX CLK=%lukHz\n",
                             (unsigned long)dp_id, (unsigned long)ap_id,
                             (unsigned long)CONFIG_BRIDGE_SWD_CLOCK_KHZ);
            (void)send_all(sock, response, (size_t)n);
        } else {
            int n = snprintf(response, sizeof(response),
                             "ERR SWD %s %s ACK=0x%X DIO=%d\n",
                             swd_status_name(status), swd_ack_name(s_last_ack),
                             s_last_ack, gpio_get_level(CONFIG_BRIDGE_SWDIO_GPIO));
            (void)send_all(sock, response, (size_t)n);
        }
        return true;
    }

    if (strcmp(line, "PID") == 0) {
        uint32_t pid = 0;
        xSemaphoreTake(s_swd_lock, portMAX_DELAY);
        swd_status_t status = swd_read_memory_locked(SWD_FMC_PID,
                                                      (uint8_t *)&pid, 4);
        xSemaphoreGive(s_swd_lock);
        char response[96];
        if (status == SWD_OK) {
            int n = snprintf(response, sizeof(response),
                             "OK FMC_PID=0x%08lX\n", (unsigned long)pid);
            (void)send_all(sock, response, (size_t)n);
        } else {
            int n = snprintf(response, sizeof(response),
                             "ERR SWD %s %s ACK=0x%X\n",
                             swd_status_name(status), swd_ack_name(s_last_ack),
                             s_last_ack);
            (void)send_all(sock, response, (size_t)n);
        }
        return true;
    }

    if (strncmp(line, "READ ", 5) == 0) {
        char *end = NULL;
        unsigned long address = strtoul(line + 5, &end, 0);
        if (end == line + 5 || *end != ' ') {
            (void)send_all(sock, "ERR syntax\n", 11);
            return true;
        }
        unsigned long length = strtoul(end + 1, &end, 0);
        if (*end != '\0' || length == 0 ||
            length > (unsigned long)CONFIG_BRIDGE_SWD_MAX_READ) {
            (void)send_all(sock, "ERR range\n", 10);
            return true;
        }

        uint8_t *buffer = (uint8_t *)malloc((size_t)length);
        if (buffer == NULL) {
            (void)send_all(sock, "ERR memory\n", 11);
            return true;
        }
        xSemaphoreTake(s_swd_lock, portMAX_DELAY);
        swd_status_t status = swd_read_memory_locked((uint32_t)address,
                                                      buffer, (size_t)length);
        xSemaphoreGive(s_swd_lock);
        if (status != SWD_OK) {
            char response[64];
            int n = snprintf(response, sizeof(response), "ERR SWD %s\n",
                             swd_status_name(status));
            (void)send_all(sock, response, (size_t)n);
            free(buffer);
            return true;
        }

        char header[32];
        int n = snprintf(header, sizeof(header), "OK %lu\n", length);
        bool ok = send_all(sock, header, (size_t)n) &&
                  send_all(sock, buffer, (size_t)length);
        if (!ok) {
            ESP_LOGW(TAG, "READ client disconnected while sending data");
        }
        free(buffer);
        return ok;
    }

    if (strncmp(line, "DUMP ", 5) == 0) {
        char *end = NULL;
        unsigned long address = strtoul(line + 5, &end, 0);
        if (end == line + 5 || *end != ' ') {
            (void)send_all(sock, "ERR syntax\n", 11);
            return true;
        }
        unsigned long length = strtoul(end + 1, &end, 0);
        if (*end != '\0' || length == 0 || (length & 3U) != 0 ||
            (address & 3UL) != 0 ||
            length > 0x80000UL ||
            !swd_range_contains((uint32_t)address, (size_t)length,
                                SWD_MEM_FLASH_BASE, SWD_MEM_FLASH_END)) {
            (void)send_all(sock, "ERR range\n", 10);
            return true;
        }
        char header[32];
        int hn = snprintf(header, sizeof(header), "OK %lu\n", length);
        if (hn <= 0 || !send_all(sock, header, (size_t)hn)) {
            return false;
        }
        const size_t CH = 4096U;
        uint8_t *chunk = (uint8_t *)malloc(CH);
        if (chunk == NULL) {
            return false;
        }
        xSemaphoreTake(s_swd_lock, portMAX_DELAY);
        unsigned long done = 0;
        swd_status_t st = SWD_OK;
        while (done < length) {
            size_t n = (size_t)(length - done);
            if (n > CH) {
                n = CH;
            }
            st = swd_read_memory_locked((uint32_t)(address + done), chunk, n);
            if (st != SWD_OK) {
                break;
            }
            if (!send_all(sock, chunk, n)) {
                break;
            }
            done += n;
        }
        xSemaphoreGive(s_swd_lock);
        free(chunk);
        if (st != SWD_OK) {
            /* The OK header promised a fixed-length binary body. Appending a
             * text error here would be mistaken for payload bytes by clients. */
            ESP_LOGW(TAG, "DUMP stopped after %lu/%lu bytes: %s",
                     done, length, swd_status_name(st));
            return false;
        }
        return done == length;
    }

    if (strncmp(line, "MWRITE ", 7) == 0) {

        char *end = NULL;

        unsigned long address =
            strtoul(line + 7, &end, 0);

        if (end == line + 7 || *end != ' ') {
            (void)send_all(sock, "ERR syntax\n", 11);
            return true;
        }

        unsigned long length =
            strtoul(end + 1, &end, 0);

        if (*end != '\0' ||
            length == 0 ||
            length > 1024UL ||
            !swd_range_contains((uint32_t)address, (size_t)length,
                                SWD_MEM_SRAM_BASE, SWD_MEM_SRAM_END)) {

            (void)send_all(sock, "ERR range\n", 10);
            return true;
        }

        if (!send_all(sock, "OK\n", 3)) {
            return false;
        }

        uint8_t *payload =
            (uint8_t *)malloc((size_t)length);

        if (payload == NULL) {
            (void)send_all(sock, "ERR memory\n", 11);
            return false;
        }

        size_t got = 0;

        while (got < (size_t)length) {

            int n = recv(
                sock,
                payload + got,
                (size_t)length - got,
                0
            );

            if (n <= 0) {
                free(payload);
                return false;
            }

            got += (size_t)n;
        }

        xSemaphoreTake(s_swd_lock, portMAX_DELAY);

        swd_status_t st =
            swd_ram_write_locked(
                (uint32_t)address,
                payload,
                (size_t)length
            );

        xSemaphoreGive(s_swd_lock);

        free(payload);

        char response[64];

        int n;

        if (st == SWD_OK) {
            n = snprintf(
                response,
                sizeof(response),
                "OK MWRITTEN %lu\n",
                length
            );
        } else {
            n = snprintf(
                response,
                sizeof(response),
                "ERR SWD %s\n",
                swd_status_name(st)
            );
        }

        (void)send_all(sock,response,(size_t)n);
        return true;
    }


    if (strcmp(line, "HALT") == 0) {

        uint32_t dhcsr = 0;

        xSemaphoreTake(s_swd_lock, portMAX_DELAY);

        swd_status_t st =
            core_halt_locked(&dhcsr);

        xSemaphoreGive(s_swd_lock);

        char response[96];

        int n;

        if (st == SWD_OK) {
            n = snprintf(
                response,
                sizeof(response),
                "OK HALTED DHCSR=0x%08lX\n",
                (unsigned long)dhcsr
            );
        } else {
            n = snprintf(
                response,
                sizeof(response),
                "ERR SWD %s\n",
                swd_status_name(st)
            );
        }

        (void)send_all(sock,response,(size_t)n);
        return true;
    }


    if (strcmp(line, "STEP") == 0) {

        uint32_t dhcsr = 0;
        uint32_t pc = 0;

        xSemaphoreTake(s_swd_lock, portMAX_DELAY);

        swd_status_t st =
            core_step_locked(&dhcsr);

        if (st == SWD_OK) {
            st = core_reg_read_locked(15U, &pc);
        }

        xSemaphoreGive(s_swd_lock);

        char response[96];
        int n;

        if (st == SWD_OK) {

            n = snprintf(
                response,
                sizeof(response),
                "OK STEP PC=0x%08lX DHCSR=0x%08lX\n",
                (unsigned long)pc,
                (unsigned long)dhcsr
            );

        } else {

            n = snprintf(
                response,
                sizeof(response),
                "ERR SWD %s\n",
                swd_status_name(st)
            );
        }

        (void)send_all(sock, response, (size_t)n);
        return true;
    }


    if (strncmp(line, "RUNUNTIL ", 9) == 0) {
        unsigned long max_steps = 0;
        unsigned long stop0 = 0;
        unsigned long stop1 = 0;
        unsigned long stop2 = 0;

        if (sscanf(line + 9, "%lu %lx %lx %lx",
                   &max_steps, &stop0, &stop1, &stop2) != 4 ||
            max_steps == 0 || max_steps > 2000000UL) {
            (void)send_all(sock, "ERR syntax\n", 11);
            return true;
        }

        uint32_t pc = 0;
        uint32_t steps = 0;

        xSemaphoreTake(s_swd_lock, portMAX_DELAY);

        swd_status_t st = core_run_until_locked(
            (uint32_t)stop0,
            (uint32_t)stop1,
            (uint32_t)stop2,
            (uint32_t)max_steps,
            &pc,
            &steps
        );

        xSemaphoreGive(s_swd_lock);

        char response[112];
        int n;

        if (st == SWD_OK) {
            n = snprintf(
                response,
                sizeof(response),
                "OK RUN PC=0x%08lX STEPS=%lu\n",
                (unsigned long)pc,
                (unsigned long)steps
            );
        } else {
            n = snprintf(
                response,
                sizeof(response),
                "ERR RUN PC=0x%08lX STEPS=%lu SWD=%s\n",
                (unsigned long)pc,
                (unsigned long)steps,
                swd_status_name(st)
            );
        }

        (void)send_all(sock, response, (size_t)n);
        return true;
    }

    if (strcmp(line, "RESUME") == 0) {

        xSemaphoreTake(s_swd_lock, portMAX_DELAY);

        swd_status_t st =
            core_resume_locked();

        xSemaphoreGive(s_swd_lock);

        if (st == SWD_OK) {
            (void)send_all(sock,"OK RESUMED\n",11);
        } else {
            char response[64];

            int n = snprintf(
                response,
                sizeof(response),
                "ERR SWD %s\n",
                swd_status_name(st)
            );

            (void)send_all(sock,response,(size_t)n);
        }

        return true;
    }


    if (strncmp(line, "REGREAD ", 8) == 0) {

        char *end = NULL;

        unsigned long regno =
            strtoul(line + 8, &end, 0);

        if (end == line + 8 ||
            *end != '\0' ||
            regno > 19UL) {

            (void)send_all(sock,"ERR syntax\n",11);
            return true;
        }

        uint32_t value = 0;

        xSemaphoreTake(s_swd_lock, portMAX_DELAY);

        swd_status_t st =
            core_reg_read_locked(
                (unsigned)regno,
                &value
            );

        xSemaphoreGive(s_swd_lock);

        char response[96];

        int n;

        if (st == SWD_OK) {

            n = snprintf(
                response,
                sizeof(response),
                "OK R%lu=0x%08lX\n",
                regno,
                (unsigned long)value
            );

        } else {

            n = snprintf(
                response,
                sizeof(response),
                "ERR SWD %s\n",
                swd_status_name(st)
            );
        }

        (void)send_all(sock,response,(size_t)n);
        return true;
    }


    if (strncmp(line, "REGWRITE ", 9) == 0) {

        char *end = NULL;

        unsigned long regno =
            strtoul(line + 9, &end, 0);

        if (end == line + 9 || *end != ' ') {
            (void)send_all(sock,"ERR syntax\n",11);
            return true;
        }

        unsigned long value =
            strtoul(end + 1, &end, 0);

        if (*end != '\0' || regno > 19UL) {
            (void)send_all(sock,"ERR syntax\n",11);
            return true;
        }

        xSemaphoreTake(s_swd_lock, portMAX_DELAY);

        swd_status_t st =
            core_reg_write_locked(
                (unsigned)regno,
                (uint32_t)value
            );

        xSemaphoreGive(s_swd_lock);

        char response[96];

        int n;

        if (st == SWD_OK) {

            n = snprintf(
                response,
                sizeof(response),
                "OK R%lu=0x%08lX\n",
                regno,
                value
            );

        } else {

            n = snprintf(
                response,
                sizeof(response),
                "ERR SWD %s\n",
                swd_status_name(st)
            );
        }

        (void)send_all(sock,response,(size_t)n);
        return true;
    }

    if (strncmp(line, "WRITE ", 6) == 0) {
        char *end = NULL;
        unsigned long address = strtoul(line + 6, &end, 0);
        if (end == line + 6 || *end != ' ') {
            (void)send_all(sock, "ERR syntax\n", 11);
            return true;
        }
        unsigned long length = strtoul(end + 1, &end, 0);
        if (*end != '\0' || length == 0 || (length & 1U) != 0 ||
            (address & 1UL) != 0 ||
            length > 0x40000UL ||
            !swd_range_contains((uint32_t)address, (size_t)length,
                                SWD_MEM_FLASH_BASE, SWD_MEM_NVM_BASE)) {
            (void)send_all(sock, "ERR range\n", 10);
            return true;
        }
        if (!send_all(sock, "OK\n", 3)) {
            return false;
        }
        uint8_t *payload = (uint8_t *)malloc((size_t)length);
        if (payload == NULL) {
            (void)send_all(sock, "ERR memory\n", 11);
            return false;
        }
        size_t got = 0;
        while (got < (size_t)length) {
            int n = recv(sock, payload + got, (size_t)length - got, 0);
            if (n <= 0) {
                free(payload);
                return false;
            }
            got += (size_t)n;
        }
        xSemaphoreTake(s_swd_lock, portMAX_DELAY);
        swd_status_t st = swd_flash_write_locked((uint32_t)address, payload, (size_t)length);
        xSemaphoreGive(s_swd_lock);
        free(payload);
        char resp[64];
        int rn;
        if (st == SWD_OK) {
            rn = snprintf(resp, sizeof(resp), "OK WRITTEN %lu\n", length);
        } else {
            rn = snprintf(resp, sizeof(resp), "ERR SWD %s\n", swd_status_name(st));
        }
        (void)send_all(sock, resp, (size_t)rn);
        return true;
    }

    (void)send_all(sock, "ERR command\n", 12);
    return true;
}

/* OpenOCD remote_bitbang wire protocol.
 *
 * For SWD, OpenOCD sends:
 *   d..g : set SWCLK/SWDIO (bit 1 / bit 0)
 *   O/o  : drive/release SWDIO
 *   c    : sample SWDIO; reply with ASCII '0' or '1'
 *   r..u : adapter reset state (there is no NRST/TRST wired here)
 *   Z/z  : optional remote sleep (normally disabled in the host config)
 *   Q    : close the session
 */
/* Commands are processed from a batched recv() and the sampled bits are
 * batched into one send(). OpenOCD streams many commands before it waits for
 * a reply, so this removes one lwIP round trip per bit; replies are flushed
 * before every blocking recv() so the host can never wait on us. */
#define RBB_IN_BUF  512
#define RBB_OUT_BUF 256

static void remote_bitbang_handle_client(int sock)
{
    uint8_t in[RBB_IN_BUF];
    char out[RBB_OUT_BUF];
    size_t out_len = 0;

    for (;;) {
        int got = recv(sock, in, sizeof(in), 0);
        if (got <= 0) {
            return;
        }

        for (int i = 0; i < got; i++) {
            const char command = (char)in[i];

            if (command >= 'd' && command <= 'g') {
                const unsigned value = (unsigned)(command - 'd');
                if (s_swdio_output) {
                    REG_WRITE((value & 0x1U) ? GPIO_OUT_W1TS_REG : GPIO_OUT_W1TC_REG,
                              SWDIO_MASK);
                }
                REG_WRITE((value & 0x2U) ? GPIO_OUT_W1TS_REG : GPIO_OUT_W1TC_REG,
                          SWCLK_MASK);
                continue;
            }

            switch (command) {
            case 'O':
                swdio_drive(1);
                break;
            case 'o':
                swdio_release();
                break;
            case 'c':
            case 'R':
                out[out_len++] = (REG_READ(GPIO_IN_REG) & SWDIO_MASK) ? '1' : '0';
                if (out_len == sizeof(out)) {
                    if (!send_all(sock, out, out_len)) {
                        return;
                    }
                    out_len = 0;
                }
                break;
            case 'r':
            case 's':
            case 't':
            case 'u':
                /* No NRST/TRST is connected; leave the SWD clock idle and release
                 * the bidirectional data line. OpenOCD performs the SWD line
                 * reset itself through subsequent d* commands. */
                REG_WRITE(GPIO_OUT_W1TC_REG, SWCLK_MASK);
                swdio_release();
                break;
            case 'Z':
                esp_rom_delay_us(1000);
                break;
            case 'z':
                esp_rom_delay_us(1);
                break;
            case 'B':
            case 'b':
                /* LED indication is not wired to this protocol endpoint. */
                break;
            case 'Q':
                if (out_len > 0) {
                    (void)send_all(sock, out, out_len);
                }
                return;
            default:
                ESP_LOGW(TAG, "remote_bitbang: unsupported command 0x%02X",
                         (unsigned char)command);
                if (out_len > 0) {
                    (void)send_all(sock, out, out_len);
                }
                return;
            }
        }

        if (out_len > 0) {
            if (!send_all(sock, out, out_len)) {
                return;
            }
            out_len = 0;
        }
    }
}

static void remote_bitbang_task(void *arg)
{
    (void)arg;
    int listener = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (listener < 0) {
        ESP_LOGE(TAG, "remote_bitbang socket failed: %d", errno);
        vTaskDelete(NULL);
        return;
    }
    const int one = 1;
    setsockopt(listener, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
    const struct sockaddr_in addr = {
        .sin_family = AF_INET,
        .sin_port = htons(CONFIG_BRIDGE_SWD_REMOTE_BITBANG_TCP_PORT),
        .sin_addr.s_addr = htonl(INADDR_ANY),
    };
    if (bind(listener, (const struct sockaddr *)&addr, sizeof(addr)) != 0 ||
        listen(listener, 1) != 0) {
        ESP_LOGE(TAG, "remote_bitbang bind/listen tcp/%d failed: %d",
                 CONFIG_BRIDGE_SWD_REMOTE_BITBANG_TCP_PORT, errno);
        close(listener);
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGI(TAG, "OpenOCD remote_bitbang: tcp/%d, SWDIO=gpio%d SWCLK=gpio%d",
             CONFIG_BRIDGE_SWD_REMOTE_BITBANG_TCP_PORT,
             CONFIG_BRIDGE_SWDIO_GPIO, CONFIG_BRIDGE_SWCLK_GPIO);

    for (;;) {
        int sock = accept(listener, NULL, NULL);
        if (sock < 0) {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }
        /* No idle timeout: OpenOCD may sit quietly between commands. */
        configure_client_socket(sock, 0);
        xSemaphoreTake(s_swd_lock, portMAX_DELAY);
        remote_bitbang_handle_client(sock);
        xSemaphoreGive(s_swd_lock);
        shutdown(sock, SHUT_RDWR);
        close(sock);
    }
}


#if CONFIG_ESP_CONSOLE_USB_CDC

static int usb_hex_nibble(char c)
{
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
}

static void usb_rpc_reply(const char *text)
{
    printf("@SWD %s\n", text);
    fflush(stdout);
}

static void swd_usb_rpc_task(void *arg)
{
    (void)arg;

    char line[1200];

    /* USB CDC console can carry normal ESP_LOG output too.
     * The host ignores everything that does not start with "@SWD ". */
    setvbuf(stdin, NULL, _IONBF, 0);
    setvbuf(stdout, NULL, _IONBF, 0);

    for (;;) {
        if (fgets(line, sizeof(line), stdin) == NULL) {
            clearerr(stdin);
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }

        size_t len = strlen(line);
        while (len > 0 &&
               (line[len - 1] == '\r' || line[len - 1] == '\n')) {
            line[--len] = '\0';
        }

        if (strncmp(line, "@SWD ", 5) != 0) {
            continue;
        }

        char *cmd = line + 5;

        if (strcmp(cmd, "PING") == 0) {
            usb_rpc_reply("PONG PROXY2");
            continue;
        }

        if (strcmp(cmd, "HALT") == 0) {
            uint32_t dhcsr = 0;

            xSemaphoreTake(s_swd_lock, portMAX_DELAY);
            swd_status_t st = core_halt_locked(&dhcsr);
            xSemaphoreGive(s_swd_lock);

            char out[96];
            if (st == SWD_OK) {
                snprintf(out, sizeof(out),
                         "OK HALTED DHCSR=0x%08lX",
                         (unsigned long)dhcsr);
            } else {
                snprintf(out, sizeof(out),
                         "ERR SWD %s", swd_status_name(st));
            }
            usb_rpc_reply(out);
            continue;
        }

        if (strcmp(cmd, "RESUME") == 0) {
            xSemaphoreTake(s_swd_lock, portMAX_DELAY);
            swd_status_t st = core_resume_locked();
            xSemaphoreGive(s_swd_lock);

            usb_rpc_reply(st == SWD_OK ? "OK RESUMED" : "ERR RESUME");
            continue;
        }

        if (strcmp(cmd, "STEP") == 0) {
            uint32_t dhcsr = 0;
            uint32_t pc = 0;

            xSemaphoreTake(s_swd_lock, portMAX_DELAY);
            swd_status_t st = core_step_locked(&dhcsr);
            if (st == SWD_OK) {
                st = core_reg_read_locked(15U, &pc);
            }
            xSemaphoreGive(s_swd_lock);

            char out[112];
            if (st == SWD_OK) {
                snprintf(out, sizeof(out),
                         "OK STEP PC=0x%08lX DHCSR=0x%08lX",
                         (unsigned long)pc,
                         (unsigned long)dhcsr);
            } else {
                snprintf(out, sizeof(out),
                         "ERR SWD %s", swd_status_name(st));
            }
            usb_rpc_reply(out);
            continue;
        }

        if (strncmp(cmd, "REGREAD ", 8) == 0) {
            unsigned long regno = 0;

            if (sscanf(cmd + 8, "%lu", &regno) != 1 || regno > 19UL) {
                usb_rpc_reply("ERR syntax");
                continue;
            }

            uint32_t value = 0;

            xSemaphoreTake(s_swd_lock, portMAX_DELAY);
            swd_status_t st =
                core_reg_read_locked((unsigned)regno, &value);
            xSemaphoreGive(s_swd_lock);

            char out[96];
            if (st == SWD_OK) {
                snprintf(out, sizeof(out),
                         "OK R%lu=0x%08lX",
                         regno, (unsigned long)value);
            } else {
                snprintf(out, sizeof(out),
                         "ERR SWD %s", swd_status_name(st));
            }
            usb_rpc_reply(out);
            continue;
        }

        if (strncmp(cmd, "REGWRITE ", 9) == 0) {
            unsigned long regno = 0;
            unsigned long value = 0;

            if (sscanf(cmd + 9, "%lu %lx", &regno, &value) != 2 ||
                regno > 19UL) {
                usb_rpc_reply("ERR syntax");
                continue;
            }

            xSemaphoreTake(s_swd_lock, portMAX_DELAY);
            swd_status_t st =
                core_reg_write_locked((unsigned)regno, (uint32_t)value);
            xSemaphoreGive(s_swd_lock);

            char out[96];
            if (st == SWD_OK) {
                snprintf(out, sizeof(out),
                         "OK R%lu=0x%08lX",
                         regno, value);
            } else {
                snprintf(out, sizeof(out),
                         "ERR SWD %s", swd_status_name(st));
            }
            usb_rpc_reply(out);
            continue;
        }

        if (strncmp(cmd, "RUNUNTIL ", 9) == 0) {
            unsigned long max_steps = 0;
            unsigned long stop0 = 0;
            unsigned long stop1 = 0;
            unsigned long stop2 = 0;

            if (sscanf(cmd + 9, "%lu %lx %lx %lx",
                       &max_steps, &stop0, &stop1, &stop2) != 4 ||
                max_steps == 0 || max_steps > 2000000UL) {
                usb_rpc_reply("ERR syntax");
                continue;
            }

            uint32_t pc = 0;
            uint32_t steps = 0;

            xSemaphoreTake(s_swd_lock, portMAX_DELAY);
            swd_status_t st = core_run_until_locked(
                (uint32_t)stop0,
                (uint32_t)stop1,
                (uint32_t)stop2,
                (uint32_t)max_steps,
                &pc,
                &steps
            );
            xSemaphoreGive(s_swd_lock);

            char out[128];
            if (st == SWD_OK) {
                snprintf(out, sizeof(out),
                         "OK RUN PC=0x%08lX STEPS=%lu",
                         (unsigned long)pc,
                         (unsigned long)steps);
            } else {
                snprintf(out, sizeof(out),
                         "ERR RUN PC=0x%08lX STEPS=%lu SWD=%s",
                         (unsigned long)pc,
                         (unsigned long)steps,
                         swd_status_name(st));
            }
            usb_rpc_reply(out);
            continue;
        }

        if (strncmp(cmd, "READ ", 5) == 0) {
            unsigned long address = 0;
            unsigned long count = 0;

            if (sscanf(cmd + 5, "%lx %lu", &address, &count) != 2 ||
                count == 0 || count > 256UL ||
                (address & 3UL) != 0 || (count & 3UL) != 0) {
                usb_rpc_reply("ERR syntax");
                continue;
            }

            uint8_t data[256];

            xSemaphoreTake(s_swd_lock, portMAX_DELAY);
            swd_status_t st =
                swd_read_memory_locked((uint32_t)address,
                                       data,
                                       (size_t)count);
            xSemaphoreGive(s_swd_lock);

            if (st != SWD_OK) {
                char out[64];
                snprintf(out, sizeof(out),
                         "ERR SWD %s", swd_status_name(st));
                usb_rpc_reply(out);
                continue;
            }

            printf("@SWD OK ");
            for (unsigned long i = 0; i < count; ++i) {
                printf("%02X", data[i]);
            }
            printf("\n");
            fflush(stdout);
            continue;
        }

        if (strncmp(cmd, "MWRITE ", 7) == 0) {
            char *p = cmd + 7;
            char *end = NULL;
            unsigned long address = strtoul(p, &end, 0);

            if (end == p || *end != ' ') {
                usb_rpc_reply("ERR syntax");
                continue;
            }

            while (*end == ' ') ++end;
            char *hex = end;
            size_t hex_len = strlen(hex);

            if (hex_len == 0 || (hex_len & 1U) != 0 ||
                hex_len > 512U) {
                usb_rpc_reply("ERR syntax");
                continue;
            }

            uint8_t data[256];
            size_t count = hex_len / 2U;
            bool bad = false;

            for (size_t i = 0; i < count; ++i) {
                int hi = usb_hex_nibble(hex[i * 2U]);
                int lo = usb_hex_nibble(hex[i * 2U + 1U]);
                if (hi < 0 || lo < 0) {
                    bad = true;
                    break;
                }
                data[i] = (uint8_t)((hi << 4) | lo);
            }

            if (bad) {
                usb_rpc_reply("ERR hex");
                continue;
            }

            xSemaphoreTake(s_swd_lock, portMAX_DELAY);
            swd_status_t st =
                swd_ram_write_locked((uint32_t)address, data, count);
            xSemaphoreGive(s_swd_lock);

            if (st == SWD_OK) {
                usb_rpc_reply("OK");
            } else {
                char out[64];
                snprintf(out, sizeof(out),
                         "ERR SWD %s", swd_status_name(st));
                usb_rpc_reply(out);
            }
            continue;
        }

        usb_rpc_reply("ERR command");
    }
}

#endif /* CONFIG_ESP_CONSOLE_USB_CDC */

static void swd_task(void *arg)
{
    (void)arg;
    int listener = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (listener < 0) {
        ESP_LOGE(TAG, "socket failed: %d", errno);
        vTaskDelete(NULL);
        return;
    }
    const int one = 1;
    setsockopt(listener, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
    const struct sockaddr_in addr = {
        .sin_family = AF_INET,
        .sin_port = htons(CONFIG_BRIDGE_SWD_TCP_PORT),
        .sin_addr.s_addr = htonl(INADDR_ANY),
    };
    if (bind(listener, (const struct sockaddr *)&addr, sizeof(addr)) != 0 ||
        listen(listener, 1) != 0) {
        ESP_LOGE(TAG, "bind/listen tcp/%d failed: %d",
                 CONFIG_BRIDGE_SWD_TCP_PORT, errno);
        close(listener);
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGI(TAG, "SWD text bridge: tcp/%d, SWDIO=gpio%d SWCLK=gpio%d",
             CONFIG_BRIDGE_SWD_TCP_PORT, CONFIG_BRIDGE_SWDIO_GPIO,
             CONFIG_BRIDGE_SWCLK_GPIO);

    for (;;) {
        int sock = accept(listener, NULL, NULL);
        if (sock < 0) {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }
        configure_client_socket(sock, SWD_TEXT_IDLE_TIMEOUT_S);
        /* Several commands per connection. One-command-per-connection clients
         * keep working: they close, recv_line() sees EOF, we loop back. */
        while (handle_command(sock)) {
        }
        shutdown(sock, SHUT_RDWR);
        close(sock);
    }
}

void swd_bridge_start(void)
{
    const gpio_config_t clk = {
        .pin_bit_mask = 1ULL << CONFIG_BRIDGE_SWCLK_GPIO,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&clk));
    ESP_ERROR_CHECK(gpio_set_level(CONFIG_BRIDGE_SWCLK_GPIO, 0));
    ESP_ERROR_CHECK(gpio_set_drive_capability(CONFIG_BRIDGE_SWCLK_GPIO,
                                               GPIO_DRIVE_CAP_3));

    const gpio_config_t dio = {
        .pin_bit_mask = 1ULL << CONFIG_BRIDGE_SWDIO_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&dio));
    ESP_ERROR_CHECK(gpio_set_drive_capability(CONFIG_BRIDGE_SWDIO_GPIO,
                                               GPIO_DRIVE_CAP_3));
    s_swdio_output = false;

    unsigned khz = CONFIG_BRIDGE_SWD_CLOCK_KHZ;
    if (khz < 10U) {
        khz = 10U;
    }

    /*
     * Half SWCLK period expressed in CPU cycles.
     * cycles = CPU_MHz * 1000 / (2 * SWD_kHz)
     *
     * 240 MHz CPU + 2000 kHz SWD -> 60 cycles ~= 250 ns.
     */
    s_half_period_cycles =
        ((uint32_t)CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ * 1000U) /
        (2U * khz);

    if (s_half_period_cycles < 1U) {
        s_half_period_cycles = 1U;
    }
    s_swd_lock = xSemaphoreCreateMutex();
    configASSERT(s_swd_lock != NULL);
    xTaskCreate(swd_task, "swd", 6144, NULL, 5, NULL);
#if CONFIG_ESP_CONSOLE_USB_CDC
    xTaskCreate(swd_usb_rpc_task, "swd_usb_rpc", 6144, NULL, 4, NULL);
#endif
    xTaskCreate(remote_bitbang_task, "remote_bitbang", 4096, NULL, 5, NULL);
}
