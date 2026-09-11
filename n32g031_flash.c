/**
 * n32g031_flash.c — Flash programmer for N32G031K8Q7-1 via ARM SWD
 *
 * Target MCU : N32G031K8Q7-1 (ARM Cortex-M0)
 * Flash      : 64 KB @ 0x08000000, 512 B pages, 128 pages
 * SRAM       : 8 KB  @ 0x20000000
 * Flash ctrl : N32G031 register-compatible subset of STM32F0 flash IP
 *
 * NV storage: pages 120-127 (0x0800F000–0x0800FFFF) are read-only from the
 * perspective of this module.  The erase loop hard-stops at page 119.
 *
 * SWD access: every target memory read/write goes through swd_read32() /
 * swd_write32() from swd.h.  A non-OK ACK from either function causes the
 * current operation to return FLASH_ERR_SWD immediately.
 *
 * BSY polling: uses furi_delay_us(100) between iterations.  Per-operation
 * maximum iteration counts are pre-computed from the timeout budget so that
 * no floating-point arithmetic or division is performed inside the loop.
 *
 * Lock on error: the flash controller is locked (best-effort) even when an
 * intermediate step fails, to leave the target in a safe state.
 *
 * Note on naming: all macros describing the N32G031 flash controller use the
 * N32_ prefix to avoid collisions with STM32WB CMSIS definitions (which
 * define their own FLASH_BASE, FLASH_KEY1, FLASH_KEY2, etc.) pulled in
 * transitively through furi_hal.h.
 *
 * Author: generated for the Flipper Zero RAZ DC25000 vape-reflash project.
 */

#include "n32g031_flash.h"
#include "swd.h"

#include <furi.h>       /* furi_delay_us()  */
#include <stddef.h>
#include <string.h>

/* ============================================================================
 * Flash controller register addresses  (N32G031 peripheral space)
 * ========================================================================= */

#define N32_FLASH_BASE    0x40022000UL
#define N32_FLASH_AC      (N32_FLASH_BASE + 0x00U)  /* Access control (wait states)  */
#define N32_FLASH_KEY     (N32_FLASH_BASE + 0x04U)  /* Unlock key register           */
#define N32_FLASH_OPTKEY  (N32_FLASH_BASE + 0x08U)  /* Option byte unlock key        */
#define N32_FLASH_STS     (N32_FLASH_BASE + 0x0CU)  /* Status register               */
#define N32_FLASH_CTRL    (N32_FLASH_BASE + 0x10U)  /* Control register              */
#define N32_FLASH_ADD     (N32_FLASH_BASE + 0x14U)  /* Page-erase address register   */
#define N32_FLASH_OBR     (N32_FLASH_BASE + 0x1CU)  /* Option byte register          */
#define N32_FLASH_WRPR    (N32_FLASH_BASE + 0x20U)  /* Write protection register     */

/* N32_FLASH_CTRL bits */
#define N32_CTRL_PG    (1UL << 0)  /* Program enable                               */
#define N32_CTRL_PER   (1UL << 1)  /* Page erase                                   */
#define N32_CTRL_MER   (1UL << 2)  /* Mass erase                                   */
#define N32_CTRL_STRT  (1UL << 6)  /* Start (trigger erase)                        */
#define N32_CTRL_LOCK  (1UL << 7)  /* Lock bit (1 = locked; clear via keys)        */

/* N32_FLASH_STS bits */
#define N32_STS_BSY    (1UL << 0)  /* Controller busy                              */
#define N32_STS_PGERR  (1UL << 2)  /* Program error                                */
#define N32_STS_WRPERR (1UL << 4)  /* Write-protection error                       */
#define N32_STS_EOP    (1UL << 5)  /* End of operation (write 1 to clear)          */

/* Unlock key sequence */
#define N32_KEY1  0x45670123UL
#define N32_KEY2  0xCDEF89ABUL

/* ============================================================================
 * Flash geometry
 * ========================================================================= */

#define N32_FLASH_ORIGIN      0x08000000UL   /* First byte of flash in target space  */

/* Page size is 512 bytes, NOT the 1 KB this file originally assumed.
 *
 * Measured directly: after erasing one page, the first surviving word sat at
 * offset 0x200.  A 1 KB assumption therefore erased only the first half of
 * each page and left the second half holding data, so programming failed with
 * PGERR the moment it crossed into a half that had never been cleared.
 *
 * The SDK's own nv.h agrees — it describes each NV key as owning "one
 * dedicated 512-byte flash page", with the NV region starting at 0x0800F000
 * (offset 61440 = page 120). */
#define N32_PAGE_SIZE         512U           /* 512 B per page                       */
#define N32_TOTAL_PAGES       128U           /* 128 pages = 64 KB                    */

/* NV region: pages 120-127 (0x0800F000-0x0800FFFF) must not be erased/written */
#define N32_NV_FIRST_PAGE    120U            /* First NV page (inclusive)            */
#define N32_MAX_USER_PAGES   120U            /* Pages 0-119 are user-writable        */
#define N32_MAX_BYTES        (N32_MAX_USER_PAGES * N32_PAGE_SIZE) /* 61440           */

/* ============================================================================
 * Polling timeouts
 *
 * All delays are multiples of furi_delay_us(100) = 100 µs per iteration.
 *
 * Page erase: 50 ms budget  → 50000 µs / 100 µs = 500 iterations
 * Word write:  5 ms budget  →  5000 µs / 100 µs =  50 iterations
 * ========================================================================= */

#define POLL_DELAY_US       100U
#define POLL_MAX_ERASE      500U   /* 50 ms  */
#define POLL_MAX_PROGRAM     50U   /*  5 ms  */

/* ============================================================================
 * Cortex-M debug register used for identification
 * ========================================================================= */

#define DHCSR_ADDR      0xE000EDF0UL   /* Debug Halting Control and Status Reg  */

/* Known DP IDCODE for Cortex-M0 devices (returned by n32_flash_identify)    */
#define CORTEX_M0_DP_IDCODE  0x0BB11477UL

/* ============================================================================
 * First verify mismatch seen during the last n32_flash_program() call.
 * ========================================================================= */

static uint32_t g_verify_addr     = 0;
static uint32_t g_verify_expected = 0;
static uint32_t g_verify_actual   = 0;

uint32_t n32_flash_verify_addr(void) {
    return g_verify_addr;
}

uint32_t n32_flash_verify_expected(void) {
    return g_verify_expected;
}

uint32_t n32_flash_verify_actual(void) {
    return g_verify_actual;
}

/* Flash controller state captured during programming, for diagnosis:
 * CTRL read back right after PG is set, and STS after the first data write. */
static uint32_t g_dbg_ctrl  = 0;
static uint32_t g_dbg_sts   = 0;
static uint8_t  g_dbg_first = 1;

uint32_t n32_flash_dbg_ctrl(void) {
    return g_dbg_ctrl;
}

uint32_t n32_flash_dbg_sts(void) {
    return g_dbg_sts;
}

/* Flash protection state: OBR carries the read-protection level, WRPR the
 * per-page write-protection mask (all ones = nothing protected).  With read
 * protection active, debug reads of flash return 0xFFFFFFFF whatever is
 * actually stored, so a protected part looks identical to an erased one. */
static uint32_t g_dbg_obr  = 0;
static uint32_t g_dbg_wrpr = 0;

/* Where programming first failed, and the controller status at that moment. */
static uint32_t g_fail_addr = 0;
static uint32_t g_fail_sts  = 0;

/* Erase-path state captured on the first page erase: CTRL and ADD read back
 * after the start trigger, and STS sampled immediately afterwards.  If BSY
 * never appears in STS the erase never began, which is indistinguishable from
 * success to the polling loop. */
static uint32_t g_er_ctrl  = 0;
static uint32_t g_er_add   = 0;
static uint32_t g_er_sts   = 0;
static uint8_t  g_er_first = 1;

/* Status read after the first page erase completes, before it is cleared. */
static uint32_t g_er_done       = 0;
static uint8_t  g_er_done_first = 1;

/* The first word of the first page erased, sampled immediately before and
 * immediately after the erase.  If these are equal the erase did not touch
 * the address we asked it to, whatever the status register claims. */
static uint32_t g_er_pre  = 0;
static uint32_t g_er_post = 0;

uint32_t n32_flash_er_pre(void) {
    return g_er_pre;
}

uint32_t n32_flash_er_post(void) {
    return g_er_post;
}

uint32_t n32_flash_er_done(void) {
    return g_er_done;
}

uint32_t n32_flash_er_ctrl(void) {
    return g_er_ctrl;
}

uint32_t n32_flash_er_add(void) {
    return g_er_add;
}

uint32_t n32_flash_er_sts(void) {
    return g_er_sts;
}

uint32_t n32_flash_fail_addr(void) {
    return g_fail_addr;
}

uint32_t n32_flash_fail_sts(void) {
    return g_fail_sts;
}

uint32_t n32_flash_dbg_obr(void) {
    return g_dbg_obr;
}

uint32_t n32_flash_dbg_wrpr(void) {
    return g_dbg_wrpr;
}

/* SRAM loopback results proving which MEM-AP transfer widths actually work. */
static uint32_t g_dbg_ram16 = 0;
static uint32_t g_dbg_ram32 = 0;

uint32_t n32_flash_dbg_ram16(void) {
    return g_dbg_ram16;
}

uint32_t n32_flash_dbg_ram32(void) {
    return g_dbg_ram32;
}

/* ============================================================================
 * Internal helpers
 * ========================================================================= */

/**
 * swd_ok() — return true iff the ACK code indicates a successful transaction.
 */
static inline bool swd_ok(SWDAck ack) {
    return ack == SWD_ACK_OK;
}

/**
 * flash_write32() — write a single 32-bit word via SWD, return FlashResult.
 */
static inline FlashResult flash_write32(uint32_t addr, uint32_t val) {
    return swd_ok(swd_write32(addr, val)) ? FLASH_OK : FLASH_ERR_SWD;
}

/**
 * flash_read32() — read a single 32-bit word via SWD, return FlashResult.
 */
static inline FlashResult flash_read32(uint32_t addr, uint32_t* out) {
    return swd_ok(swd_read32(addr, out)) ? FLASH_OK : FLASH_ERR_SWD;
}

/**
 * flash_poll_bsy() — spin until N32_FLASH_STS BSY clears or timeout expires.
 *
 * @param max_iters  Maximum number of 100-µs polling iterations.
 * @return FLASH_OK, FLASH_ERR_TIMEOUT, or FLASH_ERR_SWD.
 */
static FlashResult flash_poll_bsy(uint32_t max_iters) {
    for(uint32_t i = 0; i < max_iters; i++) {
        uint32_t sts = 0;
        FlashResult r = flash_read32(N32_FLASH_STS, &sts);
        if(r != FLASH_OK) return FLASH_ERR_SWD;
        if(!(sts & N32_STS_BSY)) return FLASH_OK;
        furi_delay_us(POLL_DELAY_US);
    }
    return FLASH_ERR_TIMEOUT;
}

/* flash_check_errors() used to live here.  It read the status register only
 * after flash_clear_status() had already wiped PGERR and WRPERR, so it could
 * never see a failure.  Both the erase and program paths now sample the
 * status themselves before clearing it. */

/**
 * flash_clear_status() — acknowledge end-of-op and clear latched error flags.
 *
 * EOP, PGERR and WRPERR are all write-1-to-clear.  Clearing only EOP (as this
 * code used to) leaves PGERR set forever: the flags are sticky across
 * operations and even across debug sessions, so one failed programming write
 * makes every later erase and program report an error that already happened.
 * Clearing all three keeps each operation's status check about that operation.
 */
static FlashResult flash_clear_status(void) {
    return flash_write32(
        N32_FLASH_STS, N32_STS_EOP | N32_STS_PGERR | N32_STS_WRPERR);
}

/**
 * flash_unlock() — perform the two-key unlock sequence.
 *
 * After writing both keys, reads back N32_FLASH_CTRL to verify the LOCK bit
 * is clear.  Returns FLASH_ERR_UNLOCK if CTRL cannot be read or LOCK is still
 * set (e.g. wrong key order, or controller already in an error state).
 *
 * @return FLASH_OK on success.
 */
static FlashResult flash_unlock(void) {
    FlashResult r;

    r = flash_write32(N32_FLASH_KEY, N32_KEY1);
    if(r != FLASH_OK) return FLASH_ERR_UNLOCK;

    r = flash_write32(N32_FLASH_KEY, N32_KEY2);
    if(r != FLASH_OK) return FLASH_ERR_UNLOCK;

    /* Verify LOCK bit cleared */
    uint32_t ctrl = 0;
    r = flash_read32(N32_FLASH_CTRL, &ctrl);
    if(r != FLASH_OK) return FLASH_ERR_UNLOCK;
    if(ctrl & N32_CTRL_LOCK) return FLASH_ERR_UNLOCK;

    return FLASH_OK;
}

/**
 * flash_lock() — set the LOCK bit via read-modify-write.
 *
 * Called as a best-effort cleanup on both success and error paths; the
 * return value is intentionally discarded by callers that are already
 * propagating a different error.
 */
static FlashResult flash_lock(void) {
    uint32_t ctrl = 0;
    FlashResult r = flash_read32(N32_FLASH_CTRL, &ctrl);
    if(r != FLASH_OK) return FLASH_ERR_SWD;

    ctrl |= N32_CTRL_LOCK;
    return flash_write32(N32_FLASH_CTRL, ctrl);
}

/**
 * flash_erase_page() — erase one 1 KB page by its page index (0-59).
 *
 * Sequence per N32G031 reference manual:
 *   1. Set PER in CTRL.
 *   2. Load page start address into ADD.
 *   3. Set PER | STRT in CTRL to trigger erase.
 *   4. Poll BSY.
 *   5. Clear EOP.
 *   6. Check PGERR / WRPERR.
 *
 * @param page_index  0-based page number; must be < N32_NV_FIRST_PAGE.
 * @return FLASH_OK, FLASH_ERR_ERASE, FLASH_ERR_PROTECTED, FLASH_ERR_SWD,
 *         or FLASH_ERR_TIMEOUT.
 */
static FlashResult flash_erase_page(uint32_t page_index) {
    FlashResult r;
    uint32_t page_addr = N32_FLASH_ORIGIN + (page_index * N32_PAGE_SIZE);

    if(g_er_first) flash_read32(page_addr, &g_er_pre);

    /* Step 1: set PER mode */
    r = flash_write32(N32_FLASH_CTRL, N32_CTRL_PER);
    if(r != FLASH_OK) return r;

    /* Step 2: write the page address to ADD */
    r = flash_write32(N32_FLASH_ADD, page_addr);
    if(r != FLASH_OK) return r;

    /* Step 3: trigger erase */
    r = flash_write32(N32_FLASH_CTRL, N32_CTRL_PER | N32_CTRL_STRT);
    if(r != FLASH_OK) return r;

    /* Snapshot the controller the instant after the trigger, once per pass. */
    if(g_er_first) {
        flash_read32(N32_FLASH_STS, &g_er_sts);
        flash_read32(N32_FLASH_CTRL, &g_er_ctrl);
        flash_read32(N32_FLASH_ADD, &g_er_add);
        g_er_first = 0;
    }

    /* Step 4: poll busy */
    r = flash_poll_bsy(POLL_MAX_ERASE);
    if(r != FLASH_OK) return r;  /* timeout or SWD error */

    /* Step 5: read the status BEFORE clearing it.
     *
     * Clearing first (as this used to) wipes PGERR and WRPERR, so the check
     * that follows always inspects a blank register and every failed erase
     * looks like a success — which is exactly why the erase appeared to work
     * while leaving pages full of data. */
    uint32_t sts = 0;
    if(!swd_ok(swd_read32(N32_FLASH_STS, &sts))) return FLASH_ERR_SWD;

    if(g_er_done_first) {
        g_er_done       = sts;
        g_er_done_first = 0;
        flash_read32(page_addr, &g_er_post);
    }

    /* Step 6: acknowledge, then judge what we captured */
    flash_clear_status();

    if(sts & N32_STS_WRPERR) return FLASH_ERR_PROTECTED;
    if(sts & N32_STS_PGERR) return FLASH_ERR_ERASE;
    return FLASH_OK;
}

/**
 * flash_write_halfword() — program one 16-bit half-word at a flash address.
 *
 * HALF-WORD ONLY: the N32G031 flash controller is STM32F0-compatible IP and
 * accepts only 16-bit programming writes.  A 32-bit write to a flash address
 * while PG is set does not program — it sets PGERR.  Each 32-bit word of the
 * image is therefore written as two half-words.
 *
 * Sequence per N32G031 reference manual:
 *   1. Set PG in CTRL.
 *   2. 16-bit write to the target flash address via SWD.
 *   3. Poll BSY.
 *   4. Clear EOP.
 *   5. Check PGERR / WRPERR.
 *
 * @param flash_addr  Target address in flash (must be 16-bit aligned).
 * @param half        Half-word to write.
 * @return FLASH_OK, FLASH_ERR_PROGRAM, FLASH_ERR_PROTECTED, FLASH_ERR_SWD,
 *         or FLASH_ERR_TIMEOUT.
 */
static FlashResult flash_write_word(uint32_t flash_addr, uint32_t word) {
    /* PG is set once by the caller for the whole programming pass rather than
     * per word, which keeps the transaction count (and the bus faults that
     * came with it) down. */
    if(!swd_ok(swd_write32(flash_addr, word))) return FLASH_ERR_SWD;

    /* Poll BSY (bit 0) in the status register. */
    for(uint32_t i = 0; i < POLL_MAX_PROGRAM; i++) {
        uint32_t sts = 0;
        if(!swd_ok(swd_read32(N32_FLASH_STS, &sts))) return FLASH_ERR_SWD;

        if(!(sts & N32_STS_BSY)) {
            /* Snapshot the very first raw status, before the flags below are
             * cleared — this is the only look we get at how the controller
             * actually responded to a programming write. */
            if(g_dbg_first) {
                g_dbg_sts   = sts;
                g_dbg_first = 0;
            }

            /* Acknowledge EOP and clear any error flags (all write-1-to-clear
             * and all in the low half-word). */
            if(!swd_ok(swd_write32(
                   N32_FLASH_STS,
                   N32_STS_EOP | N32_STS_PGERR | N32_STS_WRPERR))) {
                return FLASH_ERR_SWD;
            }

            if(sts & (N32_STS_WRPERR | N32_STS_PGERR)) {
                g_fail_addr = flash_addr;
                g_fail_sts  = sts;
            }
            if(sts & N32_STS_WRPERR) return FLASH_ERR_PROTECTED;
            if(sts & N32_STS_PGERR) return FLASH_ERR_PROGRAM;
            return FLASH_OK;
        }

        furi_delay_us(POLL_DELAY_US);
    }

    return FLASH_ERR_TIMEOUT;
}

/* ============================================================================
 * Helper: invoke the progress callback if one was provided
 * ========================================================================= */

static inline void report_progress(
    FlashProgressCb cb,
    void*           ctx,
    const char*     phase,
    uint32_t        done,
    uint32_t        total) {

    if(cb) cb(phase, done, total, ctx);
}

/* ============================================================================
 * Public API implementation
 * ========================================================================= */

FlashResult n32_flash_program(
    const uint8_t*  data,
    uint32_t        len,
    FlashProgressCb cb,
    void*           cb_ctx) {

    FlashResult r;

    /* ------------------------------------------------------------------
     * Parameter validation
     * ------------------------------------------------------------------ */
    if(len == 0 || len > N32_MAX_BYTES) {
        return FLASH_ERR_SIZE;
    }

    /* Number of pages that need to be erased to cover 'len' bytes */
    uint32_t pages_needed = (len + N32_PAGE_SIZE - 1) / N32_PAGE_SIZE;
    /* Clamp to the writable region — the size check above already ensures
     * pages_needed <= N32_MAX_USER_PAGES, but be explicit. */
    if(pages_needed > N32_MAX_USER_PAGES) {
        pages_needed = N32_MAX_USER_PAGES;
    }

    /* Number of 32-bit words to write (ceiling division).
     * If len is not a multiple of 4 the last word will be zero-padded. */
    uint32_t words_to_write = (len + 3U) / 4U;

    /* Total bytes for the flash and verify progress phases.
     * Use words_to_write * 4 so the callback total is always word-aligned. */
    uint32_t prog_total = words_to_write * 4U;

    /* ------------------------------------------------------------------
     * Step 1: Unlock
     * ------------------------------------------------------------------ */
    /* ------------------------------------------------------------------
     * Transfer-width loopback test against SRAM.
     *
     * Flash programming needs 16-bit writes, but a flash write that does not
     * take is indistinguishable from one the MEM-AP never issued.  SRAM has
     * no controller in the way, so writing there and reading it back says
     * plainly whether each transfer width works.  The core is halted and the
     * application firmware is about to be erased, so scribbling here is safe.
     *
     * Expect ram16 == 0x5A5AA5A5 and ram32 == 0xDEADBEEF.
     * ------------------------------------------------------------------ */
    /* Protection state, read before anything else touches the controller. */
    flash_read32(N32_FLASH_OBR, &g_dbg_obr);
    flash_read32(N32_FLASH_WRPR, &g_dbg_wrpr);
    g_dbg_first     = 1;
    g_er_first      = 1;
    g_er_done_first = 1;

    swd_write16(0x20000100UL, 0xA5A5U);
    swd_write16(0x20000102UL, 0x5A5AU);
    flash_read32(0x20000100UL, &g_dbg_ram16);

    flash_write32(0x20000104UL, 0xDEADBEEFUL);
    flash_read32(0x20000104UL, &g_dbg_ram32);

    r = flash_unlock();
    if(r != FLASH_OK) return r;  /* Never locked yet — no lock cleanup needed */

    /* Wipe status flags left over from any earlier attempt.  They are sticky,
     * so a previous run's PGERR would otherwise be reported against our first
     * erase — an error from a completely different session. */
    flash_clear_status();

    /* ------------------------------------------------------------------
     * Step 2: Page erase (pages 0 .. pages_needed-1)
     * ------------------------------------------------------------------ */
    uint32_t erase_total = pages_needed * N32_PAGE_SIZE;

    for(uint32_t page = 0; page < pages_needed; page++) {
        r = flash_erase_page(page);
        if(r != FLASH_OK) {
            flash_lock();  /* best-effort */
            return r;
        }
        report_progress(cb, cb_ctx, "Erasing",
                        (page + 1U) * N32_PAGE_SIZE,
                        erase_total);
    }

    /* ------------------------------------------------------------------
     * Step 2b: Confirm the erase actually blanked every page, and re-erase
     * any that did not.
     *
     * flash_erase_page() reporting success is not proof: a page can come back
     * still holding data with no error flag set, and programming into it then
     * fails with PGERR partway through the image (observed on the last page).
     * Erase is all-or-nothing per page, so a sparse scan is enough to spot a
     * page that did not take, and is far cheaper than reading every word.
     * ------------------------------------------------------------------ */
    for(uint32_t page = 0; page < pages_needed; page++) {
        uint32_t page_addr = N32_FLASH_ORIGIN + (page * N32_PAGE_SIZE);

        for(uint32_t attempt = 0;; attempt++) {
            bool blank = true;

            for(uint32_t off = 0; off < N32_PAGE_SIZE; off += 64U) {
                uint32_t v = 0;
                if(flash_read32(page_addr + off, &v) != FLASH_OK) {
                    flash_lock();
                    return FLASH_ERR_SWD;
                }
                if(v != 0xFFFFFFFFUL) {
                    /* Record exactly where the erase stopped reaching: the
                     * offset of the first surviving word reveals the real
                     * erase granularity. */
                    g_fail_addr = page_addr + off;
                    g_fail_sts  = v;
                    blank       = false;
                    break;
                }
            }

            if(blank) break;

            if(attempt >= 2U) {
                /* Three erases and it still holds data.  g_fail_addr already
                 * points at the first word that survived. */
                flash_lock();
                return FLASH_ERR_ERASE;
            }

            r = flash_erase_page(page);
            if(r != FLASH_OK) {
                flash_lock();
                return r;
            }
        }
    }

    /* ------------------------------------------------------------------
     * Step 3: Program words
     * ------------------------------------------------------------------ */
    uint32_t progress_cb_threshold = N32_PAGE_SIZE; /* report every 1 KB */
    uint32_t bytes_since_last_cb   = 0;

    /* Set PG once for the entire programming pass.  The bit stays set until
     * we clear it after the loop, so the inner loop never has to touch CTRL
     * (and never has to leave 16-bit transfer mode). */
    r = flash_write32(N32_FLASH_CTRL, N32_CTRL_PG);
    if(r != FLASH_OK) {
        flash_lock();
        return r;
    }

    /* Read CTRL back: if PG is not actually set here, the data writes below
     * will be quietly swallowed by the flash controller — no BSY, no error
     * flag, and a blank flash at verify time. */
    flash_read32(N32_FLASH_CTRL, &g_dbg_ctrl);

    for(uint32_t i = 0; i < words_to_write; i++) {
        /* Build the 32-bit word from the source buffer.
         * Bytes beyond 'len' are zero-padded (last partial word only). */
        uint32_t word = 0;
        uint32_t byte_offset = i * 4U;
        uint32_t bytes_left  = len - byte_offset;
        uint32_t copy_bytes  = (bytes_left >= 4U) ? 4U : bytes_left;

        /* Use memcpy for safe unaligned byte assembly */
        memcpy(&word, data + byte_offset, copy_bytes);
        /* Any remaining bytes in 'word' are already 0 (zero-initialised) */

        /* 32-bit word programming.  Half-word writes were tried first (the
         * flash IP looked STM32F0-like) but the controller ignores them
         * outright: no BSY, no EOP, no error, flash left blank.  Word writes
         * do reach it. */
        uint32_t flash_addr = N32_FLASH_ORIGIN + byte_offset;
        r = flash_write_word(flash_addr, word);
        if(r != FLASH_OK) {
            flash_lock();  /* best-effort */
            return r;
        }


        bytes_since_last_cb += 4U;
        if(bytes_since_last_cb >= progress_cb_threshold) {
            report_progress(cb, cb_ctx, "Flashing",
                            (i + 1U) * 4U,
                            prog_total);
            bytes_since_last_cb = 0;
        }
    }
    /* Final progress tick for "Flashing" if last block wasn't on a threshold */
    report_progress(cb, cb_ctx, "Flashing", prog_total, prog_total);

    /* Clear PG mode before verify reads */
    r = flash_write32(N32_FLASH_CTRL, 0UL);
    if(r != FLASH_OK) {
        flash_lock();
        return r;
    }

    /* ------------------------------------------------------------------
     * Step 4: Verify
     * ------------------------------------------------------------------ */
    bytes_since_last_cb = 0;

    for(uint32_t i = 0; i < words_to_write; i++) {
        uint32_t byte_offset = i * 4U;

        /* Reconstruct expected word (same logic as the write loop) */
        uint32_t expected = 0;
        uint32_t bytes_left = len - byte_offset;
        uint32_t copy_bytes = (bytes_left >= 4U) ? 4U : bytes_left;
        memcpy(&expected, data + byte_offset, copy_bytes);

        uint32_t flash_addr = N32_FLASH_ORIGIN + byte_offset;
        uint32_t actual     = 0;
        r = flash_read32(flash_addr, &actual);
        if(r != FLASH_OK) {
            flash_lock();
            return FLASH_ERR_SWD;
        }

        if(actual != expected) {
            /* Capture the first mismatch so the UI can show what actually came
             * back — 0xFFFFFFFF means nothing was programmed there, a shifted
             * or byte-swapped value means a lane/ordering bug, and a partial
             * match means the erase did not take. */
            g_verify_addr     = flash_addr;
            g_verify_expected = expected;
            g_verify_actual   = actual;
            flash_lock();
            return FLASH_ERR_VERIFY;
        }

        bytes_since_last_cb += 4U;
        if(bytes_since_last_cb >= progress_cb_threshold) {
            report_progress(cb, cb_ctx, "Verifying",
                            (i + 1U) * 4U,
                            prog_total);
            bytes_since_last_cb = 0;
        }
    }
    /* Final progress tick for "Verifying" */
    report_progress(cb, cb_ctx, "Verifying", prog_total, prog_total);

    /* ------------------------------------------------------------------
     * Step 5: Lock
     * ------------------------------------------------------------------ */
    flash_lock();  /* best-effort; ignore return value on success path */

    return FLASH_OK;
}

bool n32_flash_identify(uint32_t* idcode_out) {
    /* Attempt to bring the SWD link up.  If the target is already connected
     * from a previous call, swd_connect() re-runs the line-reset and power-up
     * sequence, which is harmless. */
    SWDAck ack = swd_connect();
    if(ack != SWD_ACK_OK) {
        if(idcode_out) *idcode_out = 0;
        return false;
    }

    /* Halt the core so DHCSR is accessible (not strictly required on M0, but
     * avoids any risk of the core interfering with the debug bus). */
    swd_halt();

    /* Read DHCSR (Debug Halting Control and Status Register @ 0xE000EDF0).
     * This register is present and readable on all ARM Cortex-M cores.
     * A successful read confirms we have a live SWD target. */
    uint32_t dhcsr = 0;
    ack = swd_read32(DHCSR_ADDR, &dhcsr);
    if(ack != SWD_ACK_OK) {
        if(idcode_out) *idcode_out = 0;
        return false;
    }

    /* DHCSR[16] (S_HALT) may or may not be set depending on core state, but
     * the register must always be readable as a non-zero value on a live
     * Cortex-M (at minimum DBGKEY bits survive reset as 0xA05F0000).
     * We treat any successful read as a valid target — a bus fault on the
     * debug bus would have returned a non-OK ACK above. */
    (void)dhcsr;  /* value not inspected further */

    /* The DP IDCODE register is not directly addressable through MEM-AP
     * swd_read32().  Return the known Cortex-M0 DP IDCODE for this family. */
    if(idcode_out) *idcode_out = CORTEX_M0_DP_IDCODE;

    return true;
}

const char* n32_flash_err_str(FlashResult r) {
    switch(r) {
    case FLASH_OK:            return "OK";
    case FLASH_ERR_UNLOCK:    return "Flash unlock failed";
    case FLASH_ERR_ERASE:     return "Page erase error";
    case FLASH_ERR_PROGRAM:   return "Programming error";
    case FLASH_ERR_VERIFY:    return "Verify mismatch";
    case FLASH_ERR_PROTECTED: return "Write protection error";
    case FLASH_ERR_SWD:       return "SWD communication error";
    case FLASH_ERR_TIMEOUT:   return "BSY poll timeout";
    case FLASH_ERR_SIZE:      return "Image too large (max 60 KB)";
    default:                  return "Unknown error";
    }
}
