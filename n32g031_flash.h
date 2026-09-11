/**
 * n32g031_flash.h — Flash programming interface for the N32G031K8Q7-1 MCU
 *
 * Target:  N32G031K8Q7-1 (ARM Cortex-M0)
 *   Flash: 64 KB @ 0x08000000, page size 1 KB, 64 pages total
 *   SRAM:  8 KB  @ 0x20000000
 *
 * NV storage region (pages 60-63, 0x0800F000–0x0800FFFF) is NEVER touched.
 * The maximum writable image size is therefore 60 KB (61440 bytes).
 *
 * Memory access uses the companion swd.h module (MEM-AP via Flipper Zero
 * bit-banged SWD on GPIO PA7/PA6).
 *
 * Usage:
 *   1. Call swd_connect() to establish the debug link.
 *   2. Call swd_halt()          to stop the core.
 *   3. Call n32_flash_program() to erase + write + verify the image.
 *   4. Call swd_reset_and_run() to restart the target.
 *
 * All functions in this module are re-entrant only from a single thread.
 * Do not call from an ISR.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---------------------------------------------------------------------------
 * Result codes
 * ------------------------------------------------------------------------- */

typedef enum {
    FLASH_OK            = 0, /**< Operation completed successfully              */
    FLASH_ERR_UNLOCK    = 1, /**< Flash controller did not unlock               */
    FLASH_ERR_ERASE     = 2, /**< Page erase failed (PGERR / WRPERR)           */
    FLASH_ERR_PROGRAM   = 3, /**< Word programming failed (PGERR / WRPERR)     */
    FLASH_ERR_VERIFY    = 4, /**< Read-back mismatch after programming          */
    FLASH_ERR_PROTECTED = 5, /**< Write protection error flagged by controller  */
    FLASH_ERR_SWD       = 6, /**< swd_read32 / swd_write32 returned non-OK ACK */
    FLASH_ERR_TIMEOUT   = 7, /**< BSY polling exceeded the per-operation limit  */
    FLASH_ERR_SIZE      = 8, /**< Image len > 61440 (would overwrite NV region) */
} FlashResult;

/* ---------------------------------------------------------------------------
 * Progress callback
 *
 * Called periodically during long operations so the caller can update a UI.
 *
 * @param phase  Human-readable phase name: "Erasing", "Flashing", "Verifying"
 * @param done   Bytes (or pages * PAGE_SIZE) processed so far
 * @param total  Total bytes (or pages * PAGE_SIZE) for this phase
 * @param ctx    Opaque pointer passed through from n32_flash_program()
 *
 * The callback is invoked:
 *   - After each 1 KB page is erased  (phase = "Erasing",   unit = pages*1024)
 *   - After each 1 KB block written   (phase = "Flashing",  unit = bytes)
 *   - After each 1 KB block verified  (phase = "Verifying", unit = bytes)
 *
 * The callback MUST return quickly and MUST NOT call any flash or SWD
 * functions — it runs in the middle of a SWD transaction sequence.
 * ------------------------------------------------------------------------- */

typedef void (*FlashProgressCb)(
    const char* phase,
    uint32_t    done,
    uint32_t    total,
    void*       ctx);

/* ---------------------------------------------------------------------------
 * Public API
 * ------------------------------------------------------------------------- */

/**
 * n32_flash_program() — full erase / program / verify cycle.
 *
 * Sequence:
 *   1. Unlock flash controller (FLASH_KEY sequence).
 *   2. Page-erase pages 0–59 (NV region pages 60–63 are preserved).
 *   3. Write @data word-by-word into flash starting at 0x08000000.
 *   4. Verify every word by reading back via SWD.
 *   5. Lock the flash controller (best-effort, even on error paths).
 *
 * @param data    Pointer to the raw firmware binary image.
 *                Byte 0 maps to flash address 0x08000000.
 * @param len     Length of @data in bytes.
 *                Must be a multiple of 4 and <= 61440 (60 KB).
 *                If @len is not a multiple of 4 the last partial word is
 *                zero-padded to 32 bits before being written.
 * @param cb      Progress callback, or NULL if not needed.
 * @param cb_ctx  Opaque value forwarded to every @cb invocation.
 *
 * @return FLASH_OK on success, or a FlashResult error code.
 *
 * Preconditions:
 *   - swd_connect() must have returned SWD_ACK_OK.
 *   - swd_halt()    must have been called to stop the core.
 *
 * Thread safety: NOT safe to call from multiple threads simultaneously.
 */
FlashResult n32_flash_program(
    const uint8_t*  data,
    uint32_t        len,
    FlashProgressCb cb,
    void*           cb_ctx);

/**
 * n32_flash_identify() — probe the target and confirm it is an ARM Cortex-M
 * compatible device accessible via SWD.
 *
 * Attempts swd_connect() if the link is not already up, then performs a
 * known-good memory read (DHCSR @ 0xE000EDF0).  If the read succeeds the
 * target is assumed to be a live Cortex-M core.
 *
 * Because the raw DP IDCODE register is not directly accessible through the
 * MEM-AP swd_read32 interface, this function returns the well-known Cortex-M0
 * DP IDCODE value (0x0BB11477) in *idcode_out when the probe succeeds.
 *
 * @param idcode_out  Receives 0x0BB11477 on success, 0 on failure.
 *                    May be NULL if the caller does not need the value.
 *
 * @return true  if a live SWD target was found and DHCSR was readable,
 *         false otherwise.
 */
bool n32_flash_identify(uint32_t* idcode_out);

/**
 * n32_flash_err_str() — human-readable description of a FlashResult code.
 *
 * @param r  Any FlashResult value.
 * @return   A short, null-terminated ASCII string.  The pointer is valid for
 *           the lifetime of the program (points to a string literal).
 */
const char* n32_flash_err_str(FlashResult r);

/**
 * Details of the first verify mismatch from the last n32_flash_program().
 * Only meaningful when that call returned FLASH_ERR_VERIFY.
 *
 * An actual value of 0xFFFFFFFF means nothing was programmed at that address;
 * a shifted or byte-swapped value points at a transfer-lane bug; a value that
 * shares set bits with the expected one means the page was not fully erased.
 */
/* Flash controller state captured during the last programming pass: CTRL read
 * back immediately after PG was set, and STS after the first data write. */
uint32_t n32_flash_dbg_ctrl(void);
uint32_t n32_flash_dbg_sts(void);

/* SRAM loopback results from the last programming pass, proving which MEM-AP
 * transfer widths actually reach the target.  Expect 0x5A5AA5A5 and
 * 0xDEADBEEF respectively when both widths work. */
/* Flash protection state read at the start of the last programming pass.
 * OBR carries the read-protection level; WRPR is the write-protection mask
 * (0xFFFFFFFF when no page is protected). */
/* Address at which programming first reported an error, and the flash status
 * register at that moment. */
/* Erase-path state from the first page erase of the last programming pass:
 * CTRL and ADD read back after the start trigger, STS sampled right after.
 * BSY absent from STS means the erase never actually started. */
/* Status captured after the first page erase completed, before it was
 * cleared — bit 2 is PGERR, bit 4 WRPERR, bit 5 EOP. */
uint32_t n32_flash_er_done(void);

/* First word of the first erased page, sampled before and after that erase. */
uint32_t n32_flash_er_pre(void);
uint32_t n32_flash_er_post(void);

uint32_t n32_flash_er_ctrl(void);
uint32_t n32_flash_er_add(void);
uint32_t n32_flash_er_sts(void);

uint32_t n32_flash_fail_addr(void);
uint32_t n32_flash_fail_sts(void);

uint32_t n32_flash_dbg_obr(void);
uint32_t n32_flash_dbg_wrpr(void);

uint32_t n32_flash_dbg_ram16(void);
uint32_t n32_flash_dbg_ram32(void);

uint32_t n32_flash_verify_addr(void);
uint32_t n32_flash_verify_expected(void);
uint32_t n32_flash_verify_actual(void);

#ifdef __cplusplus
}
#endif
