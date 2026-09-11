/**
 * swd.h — Bit-banged ARM SWD (Serial Wire Debug) driver for Flipper Zero
 *
 * Target:  N32G031K8Q7-1 (ARM Cortex-M0) inside RAZ DC25000 vape
 * Host:    Flipper Zero (STM32WB55, 64 MHz)
 *
 * Pin wiring (Flipper GPIO header → USB-C plug on vape):
 *   Flipper PA7  (header pin 2, label "A7")  →  SWDIO  →  USB-C CC1
 *   Flipper PA6  (header pin 3, label "A6")  →  SWCLK  →  USB-C CC2
 *   Flipper GND  (header pin 8 or 18)        →  GND    →  USB-C GND shell
 *
 * The vape is self-powered; USB-C carries only the CC/SWD signals.
 *
 * Protocol:  ARM ADIv5 SWD
 * Reference: ARM IHI0031 (ADIv5 Architecture Specification)
 */

#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---------------------------------------------------------------------------
 * SWD pin definitions
 * ------------------------------------------------------------------------- */
#include <furi_hal.h>

#define SWD_SWDIO (&gpio_ext_pa7)
/* SWCLK moved off PA6 (header pin 3) → PC3 (header pin 7): the A6 contact
 * proved dead on this Flipper (line probed FLOAT through both jumper wires
 * and both breakout pads). */
#define SWD_SWCLK (&gpio_ext_pc3)

/* ---------------------------------------------------------------------------
 * Return / status codes
 *
 * SWD_ACK_OK, SWD_ACK_WAIT, SWD_ACK_FAULT match the 3-bit ACK field values
 * driven by the target on the wire.  SWD_ERR_* are host-generated errors.
 * ------------------------------------------------------------------------- */
typedef enum {
    SWD_ACK_OK        = 1,  /**< Target accepted the transaction              */
    SWD_ACK_WAIT      = 2,  /**< Target busy — caller may retry               */
    SWD_ACK_FAULT     = 4,  /**< Target flagged a fault in CTRL/STAT          */
    SWD_ERR_NO_TARGET = 8,  /**< No valid IDCODE after 3 line-reset attempts  */
    SWD_ERR_TIMEOUT   = 16, /**< Power-up ack or other poll timed out         */
} SWDAck;

/* ---------------------------------------------------------------------------
 * Public API
 * ------------------------------------------------------------------------- */

/**
 * swd_connect() — full ARM ADIv5 connect sequence.
 *
 * Steps performed:
 *   1. Line reset (56 clocks HIGH) — up to 3 retries
 *   2. JTAG-to-SWD switch (0xE79E, 16 bits LSB-first)
 *   3. Line reset again + 4 idle clocks
 *   4. Read DP IDCODE — verify bit 0 == 1
 *   5. Write DP CTRL/STAT = 0x50000000 (request system + debug power)
 *   6. Poll CTRL/STAT until CSYSPWRUPACK (bit 31) + CDBGPWRUPACK (bit 29) set
 *   7. Write DP SELECT = 0x00000000 (AP 0, bank 0)
 *   8. Write AP CSW = 0x00000002 (32-bit word transfers, AddrInc off)
 *
 * Returns SWD_ACK_OK on success, SWD_ERR_NO_TARGET or SWD_ERR_TIMEOUT on
 * failure, or the raw ACK value from a failed DP/AP access.
 */
SWDAck swd_connect(void);

/**
 * swd_disconnect() — tri-state both SWD pins and release them.
 *
 * Call this when you are finished; it returns the Flipper GPIO pins to
 * high-impedance input so they do not interfere with other use.
 */
void swd_disconnect(void);

/**
 * swd_read32() — read one 32-bit word from the target's memory map.
 *
 * Uses MEM-AP TAR/DRW with AP-read pipeline correction (result captured via
 * DP RDBUFF).
 *
 * @param addr   Target byte address (must be 32-bit aligned).
 * @param out    Receives the value read from the target.
 * @return       SWD_ACK_OK on success, or an ACK/error code on failure.
 */
SWDAck swd_read32(uint32_t addr, uint32_t* out);

/**
 * swd_write32() — write one 32-bit word to the target's memory map.
 *
 * Uses MEM-AP TAR/DRW.
 *
 * @param addr   Target byte address (must be 32-bit aligned).
 * @param val    Value to write.
 * @return       SWD_ACK_OK on success, or an ACK/error code on failure.
 */
SWDAck swd_write32(uint32_t addr, uint32_t val);

/**
 * swd_write16() — write one 16-bit half-word to the target's memory map.
 *
 * Switches the MEM-AP to 16-bit transfers and places the half-word in the
 * DRW byte lane selected by address bit 1, as the MEM-AP requires.
 *
 * Needed for flash programming: the N32G031 flash controller is STM32F0
 * compatible and accepts ONLY half-word programming writes.  A 32-bit write
 * to a flash address in PG mode sets PGERR instead of programming.
 *
 * @param addr   Target byte address (must be 16-bit aligned).
 * @param val    Half-word to write.
 * @return       SWD_ACK_OK on success, or an ACK/error code on failure.
 */
SWDAck swd_write16(uint32_t addr, uint16_t val);

/**
 * swd_read16() — read one 16-bit half-word from the target's memory map.
 *
 * Lets the flash programming loop poll the status register without switching
 * the MEM-AP back to 32-bit between every half-word written.
 *
 * @param addr   Target byte address (must be 16-bit aligned).
 * @param out    Receives the half-word read from the target.
 * @return       SWD_ACK_OK on success, or an ACK/error code on failure.
 */
SWDAck swd_read16(uint32_t addr, uint16_t* out);

/**
 * swd_halt() — halt the Cortex-M0 core via DHCSR.
 *
 * Writes 0xA05F0003 (DBGKEY | C_HALT | C_DEBUGEN) to 0xE000EDF0.
 *
 * @return SWD_ACK_OK on success, or an ACK/error code on failure.
 */
SWDAck swd_halt(void);

/**
 * swd_probe() — passive electrical probe of both SWD lines.
 *
 * For each line, reads the pin under an internal pull-down and then an
 * internal pull-up (2-bit result: bit0 = level under pull-down, bit1 =
 * level under pull-up):
 *   0b10 (2) — follows our pulls  → line FLOATING (no electrical path)
 *   0b11 (3) — high both times    → externally pulled/driven HIGH (connected)
 *   0b00 (0) — low both times     → externally pulled/driven LOW  (connected)
 *   0b01 (1) — inverted           → oscillating / being actively driven
 *
 * Leaves both pins as no-pull inputs. Purely passive — never drives the
 * lines, safe to call at any time.
 */
void swd_probe(uint8_t* swdio_state, uint8_t* swclk_state);

/**
 * Diagnostics describing the most recent swd_connect() attempt.
 *
 * swd_last_stage() returns the step that was in progress when the connect
 * gave up (0 = completed successfully):
 *   1 = reading DP IDCODE
 *   2 = DP SELECT write
 *   3 = CTRL/STAT power-up request write
 *   4 = polling for the power-up acknowledge
 *   5 = MEM-AP CSW write
 *
 * swd_last_idcode() returns the raw IDCODE if one was read (0 if never).
 * A plausible IDCODE with bit 0 set proves the wire protocol is working.
 *
 * swd_last_stat() returns the last CTRL/STAT value seen while polling for the
 * power-up acknowledge — bit 31 is CSYSPWRUPACK, bit 29 is CDBGPWRUPACK.
 */
uint8_t  swd_last_stage(void);
uint32_t swd_last_idcode(void);
uint32_t swd_last_stat(void);

/**
 * swd_last_trn() — the write-data turnaround length swd_connect() settled on
 * while calibrating the write frame (0-3 clocks).
 */
uint8_t  swd_last_trn(void);

/**
 * swd_last_ack_err() — the most recent non-OK ACK returned by any SWD
 * transaction.  Useful for telling apart the causes of a flash-time
 * "SWD communication error": 4 = FAULT, 2 = WAIT (retries exhausted),
 * 7 = line floating, 0 = line stuck low.
 */
SWDAck swd_last_ack_err(void);

/**
 * swd_raw_capture() — alignment diagnostic.
 *
 * Performs a line reset + JTAG-to-SWD switch, sends the DP IDCODE read
 * request, then clocks in 64 raw bits making NO assumption about where the
 * turnaround ends or where the ACK begins.  Bit 0 of *first32 is the first
 * bit clocked in after the request byte.
 *
 * This makes the real framing visible: find the first 1 (the target starting
 * to drive ACK[0]) and the 32 bits of IDCODE that follow it.
 */
void swd_raw_capture(uint32_t* first32, uint32_t* second32);

/**
 * swd_reset_and_run() — trigger a system reset and release halt.
 *
 * Writes AIRCR = 0x05FA0004 (VECTKEY | SYSRESETREQ) to 0xE000ED0C.
 * The core runs from reset; no halt is re-applied.
 *
 * @return SWD_ACK_OK on success, or an ACK/error code on failure.
 */
SWDAck swd_reset_and_run(void);

#ifdef __cplusplus
}
#endif
