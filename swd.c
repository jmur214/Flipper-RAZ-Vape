/**
 * swd.c — Bit-banged ARM SWD (Serial Wire Debug) driver for Flipper Zero
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
 * Timing note:
 *   Each half-clock is separated by SWD_HALF_PERIOD_US microseconds.
 *   At 64 MHz the GPIO toggle + furi_delay_us(1) yields a safe ~200 kHz
 *   SWD clock, well within what the N32G031 debug port handles.
 *
 * Protocol:  ARM ADIv5 SWD
 * Reference: ARM IHI0031 (ADIv5 Architecture Specification)
 */

#include "swd.h"

#include <stdint.h>
#include <stdbool.h>
#include <furi_hal.h>
#include <furi.h>

/* ---------------------------------------------------------------------------
 * Timing
 * ------------------------------------------------------------------------- */

/**
 * Half-period of the SWD clock in microseconds.
 *
 * furi_hal_gpio_init() (called on every direction switch in dio_input /
 * dio_output) takes several microseconds of overhead.  A 2 µs half-period
 * is therefore too aggressive — the actual waveform ends up much slower and
 * asymmetric.  10 µs (~50 kHz) gives comfortable margin and is well within
 * what the N32G031 debug port handles.
 */
#define SWD_HALF_PERIOD_US (10U)

/* ---------------------------------------------------------------------------
 * Frame alignment — NO leading turnaround clock
 *
 * The ARM spec describes a turnaround period between the host's 8-bit request
 * and the target's 3-bit ACK.  On this target (N32G031 reached over the USB-C
 * CC lines) that turnaround is absorbed: the target is already driving ACK[0]
 * on the first clock after the request byte.
 *
 * Verified by capturing 64 raw bits immediately after an IDCODE request:
 *   bits 0..2   = 1,0,0        -> ACK = OK
 *   bits 3..34  = 0x0BB11477   -> Cortex-M0 DP IDCODE
 *   bit  35     = 1            -> correct even parity for that IDCODE
 *
 * Clocking an extra turnaround cycle first swallows ACK[0] and shifts the ACK
 * to 0,0,1, which decodes as FAULT — making every connect fail against a
 * target that was in fact responding perfectly.  So the ACK is read directly
 * after releasing SWDIO, with no turnaround clock.  The TRAILING turnaround
 * (host taking the line back) is still required and is kept.
 * ------------------------------------------------------------------------- */

/* ---------------------------------------------------------------------------
 * DP register addresses  (A[3:2] field of the 8-bit request byte)
 * The address encodes bits [3:2] only; bit 1 and bit 0 are always 0.
 * We pass the full byte-address and extract A[3:2] in swd_build_request().
 * ------------------------------------------------------------------------- */
#define DP_REG_IDCODE    (0x00U) /* read  */
#define DP_REG_ABORT     (0x00U) /* write */
#define DP_REG_CTRLSTAT  (0x04U)
#define DP_REG_SELECT    (0x08U)
#define DP_REG_RDBUFF    (0x0CU)

/* ---------------------------------------------------------------------------
 * AP (MEM-AP) register addresses — bank 0 assumed via SELECT=0
 * ------------------------------------------------------------------------- */
#define AP_REG_CSW (0x00U)
#define AP_REG_TAR (0x04U)
#define AP_REG_DRW (0x0CU)

/* ---------------------------------------------------------------------------
 * Cortex-M0 debug registers
 * ------------------------------------------------------------------------- */
#define DHCSR_ADDR  (0xE000EDF0UL)
#define AIRCR_ADDR  (0xE000ED0CUL)

#define DHCSR_HALT_VAL  (0xA05F0003UL) /* DBGKEY | C_HALT | C_DEBUGEN */
#define AIRCR_RESET_VAL (0x05FA0004UL) /* VECTKEY | SYSRESETREQ       */

/* ---------------------------------------------------------------------------
 * ABORT register: clear all sticky error flags.
 * STKCMPCLR | STKERRCLR | WDERRCLR | ORUNERRCLR = 0x1E.
 *
 * A DP that has latched a sticky error answers subsequent transactions with
 * FAULT (ACK=0x04) until these are cleared.  This is the usual reason a
 * connect fails on a target that is wired correctly and clearly responding.
 * ------------------------------------------------------------------------- */
#define DP_ABORT_CLEAR_ALL (0x0000001EUL)

/* ---------------------------------------------------------------------------
 * Power-up polling limit
 * ------------------------------------------------------------------------- */
#define PWRUP_POLL_LIMIT (1000U)

/* ---------------------------------------------------------------------------
 * Diagnostics for the last swd_connect() attempt — see swd.h.
 * ------------------------------------------------------------------------- */
static uint8_t  g_last_stage  = 0;
static uint32_t g_last_idcode = 0;
static uint32_t g_last_stat   = 0;

/* Turnaround clocks inserted between the target's ACK and the host's write
 * data.  Reads need none before the ACK (see the frame-alignment note), but
 * the write data phase is a separate turnaround whose length this target does
 * not follow the spec on.  swd_connect() calibrates it empirically. */
static uint8_t  g_write_trn   = 1;
static uint8_t  g_last_trn    = 1;

/* MEM-AP CSW.
 *
 * Prot[30:24] carries HPROT onto the bus, and it matters: HPROT[0] (bit 24)
 * selects data access (1) versus opcode fetch (0), and HPROT[1] (bit 25)
 * selects privileged.  Leaving Prot at zero presents every write as an
 * unprivileged instruction fetch — SRAM does not care, but the flash
 * controller ignores it as a programming write, so the flash silently stays
 * erased.  0x23000000 is the value debuggers conventionally use here.
 *
 * Low bits are Size[2:0], with AddrInc left off. */
#define CSW_PROT    (0x23000000UL)
#define CSW_SIZE_32 (CSW_PROT | 0x00000002UL)
#define CSW_SIZE_16 (CSW_PROT | 0x00000001UL)

/* Cached CSW size so the width is only reprogrammed when it actually changes. */
static uint32_t g_csw_size = CSW_SIZE_32;

/* ===========================================================================
 * Low-level bit-bang helpers
 * ======================================================================== */

/** Drive SWCLK high. */
static inline void clk_high(void) {
    furi_hal_gpio_write(SWD_SWCLK, true);
}

/** Drive SWCLK low. */
static inline void clk_low(void) {
    furi_hal_gpio_write(SWD_SWCLK, false);
}

/** Drive SWDIO high (output mode must already be set). */
static inline void dio_high(void) {
    furi_hal_gpio_write(SWD_SWDIO, true);
}

/** Drive SWDIO low (output mode must already be set). */
static inline void dio_low(void) {
    furi_hal_gpio_write(SWD_SWDIO, false);
}

/** Switch SWDIO to output push-pull, very-high speed. */
static inline void dio_output(void) {
    furi_hal_gpio_init(SWD_SWDIO, GpioModeOutputPushPull, GpioPullNo, GpioSpeedVeryHigh);
}

/**
 * Switch SWDIO to input with pull-up.
 *
 * ARM SWD requires SWDIO to be pulled high during turnaround cycles and
 * whenever the host is not driving the line.  Without the pull-up the pin
 * floats, ACK bits read as garbage, and swd_connect() always fails.
 */
static inline void dio_input(void) {
    furi_hal_gpio_init(SWD_SWDIO, GpioModeInput, GpioPullUp, GpioSpeedVeryHigh);
}

/** Read SWDIO (input mode must already be set). */
static inline bool dio_read(void) {
    return furi_hal_gpio_read(SWD_SWDIO);
}

/** Delay one half-clock period. */
static inline void half_clk(void) {
    furi_delay_us(SWD_HALF_PERIOD_US);
}

/**
 * Clock out one bit on SWDIO (host → target, output mode assumed).
 * SWDIO is set before the rising edge and held through the falling edge.
 */
static void swd_write_bit(bool bit) {
    if(bit) {
        dio_high();
    } else {
        dio_low();
    }
    half_clk();
    clk_high();
    half_clk();
    clk_low();
}

/**
 * Clock in one bit from SWDIO (input mode assumed).
 * Samples on the rising edge.
 */
static bool swd_read_bit(void) {
    half_clk();
    clk_high();
    half_clk();
    bool bit = dio_read();
    clk_low();
    return bit;
}

/**
 * Execute one turnaround clock cycle.
 * The direction of SWDIO is NOT changed here — the caller must handle that
 * before and after as required by the SWD spec.
 */
static void swd_turnaround(void) {
    half_clk();
    clk_high();
    half_clk();
    clk_low();
}

/* ===========================================================================
 * Multi-bit helpers
 * ======================================================================== */

/**
 * Write 'count' bits LSB-first from 'data' (output mode assumed).
 */
static void swd_write_bits(uint32_t data, uint8_t count) {
    for(uint8_t i = 0; i < count; i++) {
        swd_write_bit((data >> i) & 1U);
    }
}

/**
 * Read 'count' bits LSB-first into *out (input mode assumed).
 */
static void swd_read_bits(uint32_t* out, uint8_t count) {
    *out = 0;
    for(uint8_t i = 0; i < count; i++) {
        if(swd_read_bit()) {
            *out |= (1U << i);
        }
    }
}

/* ===========================================================================
 * Parity
 * ======================================================================== */

/** Compute even parity (returns 1 if popcount of val is odd, 0 otherwise). */
static bool swd_parity32(uint32_t val) {
    val ^= val >> 16;
    val ^= val >> 8;
    val ^= val >> 4;
    val ^= val >> 2;
    val ^= val >> 1;
    return (bool)(val & 1U);
}

/** Even parity over the 4 bits APnDP, RnW, A2, A3. */
static bool swd_request_parity(bool apndp, bool rnw, uint8_t addr) {
    uint8_t bits = (uint8_t)apndp ^ (uint8_t)rnw ^
                   ((addr >> 2) & 1U) ^ ((addr >> 3) & 1U);
    return (bool)(bits & 1U);
}

/* ===========================================================================
 * Request byte builder
 * ======================================================================== */

/**
 * Build the 8-bit SWD request byte.
 *
 * Bit layout (LSB → MSB):
 *   [0] Start  = 1
 *   [1] APnDP  (0=DP, 1=AP)
 *   [2] RnW    (0=write, 1=read)
 *   [3] A2     (addr bit 2)
 *   [4] A3     (addr bit 3)
 *   [5] Parity (even parity over bits 1-4)
 *   [6] Stop   = 0
 *   [7] Park   = 1
 *
 * @param apndp  false=DP register, true=AP register
 * @param rnw    false=write, true=read
 * @param addr   Register address (only bits [3:2] are used)
 */
static uint8_t swd_build_request(bool apndp, bool rnw, uint8_t addr) {
    uint8_t a2 = (addr >> 2) & 1U;
    uint8_t a3 = (addr >> 3) & 1U;
    bool parity = swd_request_parity(apndp, rnw, addr);

    uint8_t req = 0;
    req |= (1U);                          /* bit 0: Start */
    req |= ((uint8_t)apndp << 1);        /* bit 1: APnDP */
    req |= ((uint8_t)rnw   << 2);        /* bit 2: RnW   */
    req |= (a2             << 3);        /* bit 3: A2    */
    req |= (a3             << 4);        /* bit 4: A3    */
    req |= ((uint8_t)parity << 5);       /* bit 5: Parity*/
    /* bit 6: Stop = 0 (already 0)                        */
    req |= (1U             << 7);        /* bit 7: Park  */
    return req;
}

/* ===========================================================================
 * Core SWD read / write transaction
 * ======================================================================== */

/**
 * swd_dp_read() — read a DP register.
 *
 * @param addr  DP register address (DP_REG_* constant)
 * @param out   Receives the 32-bit register value on SWD_ACK_OK
 * @return      ACK code
 */
static SWDAck swd_dp_read_once(uint8_t addr, uint32_t* out) {
    uint8_t req = swd_build_request(false, true, addr);

    /* Phase 1: send request (host drives SWDIO) */
    dio_output();
    swd_write_bits(req, 8);

    /* Phase 2: release SWDIO.  No turnaround clock — see the frame-alignment
     * note at the top of this file: ACK[0] is already on the wire. */
    dio_input();

    /* Phase 3: read 3-bit ACK (target drives) */
    uint32_t ack = 0;
    swd_read_bits(&ack, 3);

    if(ack == (uint32_t)SWD_ACK_OK) {
        /* Phase 4: read 32-bit data + 1 parity bit (target still drives) */
        uint32_t data = 0;
        swd_read_bits(&data, 32);
        bool parity_bit = swd_read_bit();

        /* Phase 5: turnaround — host takes SWDIO back, clock 1 cycle */
        dio_output();
        swd_turnaround();

        /* Phase 6: idle */
        swd_write_bit(false);

        /* Verify parity */
        if(parity_bit != swd_parity32(data)) {
            /* Parity error — treat as fault */
            *out = 0;
            return SWD_ACK_FAULT;
        }

        *out = data;
        return SWD_ACK_OK;
    } else {
        /*
         * Non-OK ACK (WAIT or FAULT):
         * Clock 33 bits while SWDIO is still in input mode, then reclaim it.
         */
        for(int i = 0; i < 33; i++) {
            swd_read_bit();
        }
        dio_output();
        swd_turnaround();
        swd_write_bit(false); /* idle */
        *out = 0;
        return (SWDAck)ack;
    }
}

/**
 * swd_dp_write() — write a DP register.
 *
 * @param addr  DP register address (DP_REG_* constant)
 * @param val   32-bit value to write
 * @return      ACK code
 */
static SWDAck swd_dp_write_once(uint8_t addr, uint32_t val) {
    uint8_t req = swd_build_request(false, false, addr);

    /* Phase 1: send request (host drives SWDIO) */
    dio_output();
    swd_write_bits(req, 8);

    /* Phase 2: release SWDIO.  No turnaround clock — see the frame-alignment
     * note at the top of this file: ACK[0] is already on the wire. */
    dio_input();

    /* Phase 3: read 3-bit ACK */
    uint32_t ack = 0;
    swd_read_bits(&ack, 3);

    if(ack == (uint32_t)SWD_ACK_OK) {
        /* Phase 4: host takes SWDIO back, then g_write_trn turnaround clocks
         * before driving data.  The correct count is target-specific and is
         * calibrated by swd_connect(); get it wrong and every data bit plus
         * the parity shifts, so the DP silently discards the write. */
        dio_output();
        for(uint8_t t = 0; t < g_write_trn; t++) swd_turnaround();

        /* Phase 5: write 32-bit data + parity */
        swd_write_bits(val, 32);
        swd_write_bit(swd_parity32(val));

        /* Phase 6: idle */
        swd_write_bit(false);
        return SWD_ACK_OK;
    } else {
        /*
         * Non-OK ACK (WAIT or FAULT):
         * Reclaim SWDIO, clock 1 TRN, drive 33 zero bits, idle.
         */
        dio_output();
        swd_turnaround();
        swd_write_bits(0, 33);
        swd_write_bit(false); /* idle */
        return (SWDAck)ack;
    }
}

/**
 * swd_ap_read() — read an AP register.
 *
 * AP reads are pipelined; the value returned is from the PREVIOUS read.
 * Callers that need actual data must follow up with swd_dp_read(DP_REG_RDBUFF).
 *
 * @param addr  AP register address (AP_REG_* constant)
 * @param out   Receives the (pipelined) 32-bit value on SWD_ACK_OK
 * @return      ACK code
 */
static SWDAck swd_ap_read_once(uint8_t addr, uint32_t* out) {
    uint8_t req = swd_build_request(true, true, addr);

    /* Phase 1: send request */
    dio_output();
    swd_write_bits(req, 8);

    /* Phase 2: release SWDIO (no turnaround clock — see alignment note). */
    dio_input();

    /* Phase 3: 3-bit ACK */
    uint32_t ack = 0;
    swd_read_bits(&ack, 3);

    if(ack == (uint32_t)SWD_ACK_OK) {
        /* Phase 4: 32-bit data + parity */
        uint32_t data = 0;
        swd_read_bits(&data, 32);
        bool parity_bit = swd_read_bit();

        /* Phase 5: turnaround */
        dio_output();
        swd_turnaround();

        /* Phase 6: idle */
        swd_write_bit(false);

        if(parity_bit != swd_parity32(data)) {
            *out = 0;
            return SWD_ACK_FAULT;
        }

        *out = data;
        return SWD_ACK_OK;
    } else {
        for(int i = 0; i < 33; i++) {
            swd_read_bit();
        }
        dio_output();
        swd_turnaround();
        swd_write_bit(false);
        *out = 0;
        return (SWDAck)ack;
    }
}

/**
 * swd_ap_write() — write an AP register.
 *
 * @param addr  AP register address (AP_REG_* constant)
 * @param val   32-bit value to write
 * @return      ACK code
 */
static SWDAck swd_ap_write_once(uint8_t addr, uint32_t val) {
    uint8_t req = swd_build_request(true, false, addr);

    /* Phase 1: send request */
    dio_output();
    swd_write_bits(req, 8);

    /* Phase 2: release SWDIO (no turnaround clock — see alignment note). */
    dio_input();

    /* Phase 3: 3-bit ACK */
    uint32_t ack = 0;
    swd_read_bits(&ack, 3);

    if(ack == (uint32_t)SWD_ACK_OK) {
        /* Phase 4: host takes SWDIO back + calibrated turnaround (see note). */
        dio_output();
        for(uint8_t t = 0; t < g_write_trn; t++) swd_turnaround();

        /* Phase 5: 32-bit data + parity */
        swd_write_bits(val, 32);
        swd_write_bit(swd_parity32(val));

        /* Phase 6: idle */
        swd_write_bit(false);
        return SWD_ACK_OK;
    } else {
        dio_output();
        swd_turnaround();
        swd_write_bits(0, 33);
        swd_write_bit(false);
        return (SWDAck)ack;
    }
}

/* ===========================================================================
 * WAIT retry wrappers
 *
 * A target answers WAIT whenever it cannot accept a transaction yet — which
 * during flash erase/program is most of the time, and became far more likely
 * once each word turned into two half-word writes plus CSW width switches.
 * WAIT is not an error: the transaction simply has to be repeated.  Treating
 * it as fatal (as this driver used to) surfaces as a spurious "SWD
 * communication error" partway through an otherwise healthy flash.
 * ======================================================================== */

#define SWD_WAIT_RETRIES  (200U)
#define SWD_FAULT_RETRIES (4U)

/* Last non-OK ACK seen, for diagnostics on the error screen. */
static SWDAck g_last_ack_err = SWD_ACK_OK;

SWDAck swd_last_ack_err(void) {
    return g_last_ack_err;
}

/* Clear the DP's sticky error flags.  Deliberately uses the _once form: the
 * retry wrappers below call this from inside their own loops.
 *
 * A FAULT is latched, not transient — once a sticky error is set the DP
 * answers FAULT to everything until ABORT clears it.  During flash work an
 * occasional error is normal, so recovering and retrying is the difference
 * between a flash that completes and one that dies partway through. */
static void swd_clear_sticky(void) {
    swd_dp_write_once(DP_REG_ABORT, DP_ABORT_CLEAR_ALL);
}

static SWDAck swd_dp_read(uint8_t addr, uint32_t* out) {
    SWDAck   r      = SWD_ACK_WAIT;
    uint32_t faults = 0;
    for(uint32_t i = 0; i < SWD_WAIT_RETRIES; i++) {
        r = swd_dp_read_once(addr, out);
        if(r == SWD_ACK_WAIT) {
            furi_delay_us(10);
            continue;
        }
        if(r == SWD_ACK_FAULT && faults++ < SWD_FAULT_RETRIES) {
            swd_clear_sticky();
            furi_delay_us(10);
            continue;
        }
        break;
    }
    if(r != SWD_ACK_OK) g_last_ack_err = r;
    return r;
}

static SWDAck swd_dp_write(uint8_t addr, uint32_t val) {
    SWDAck   r      = SWD_ACK_WAIT;
    uint32_t faults = 0;
    for(uint32_t i = 0; i < SWD_WAIT_RETRIES; i++) {
        r = swd_dp_write_once(addr, val);
        if(r == SWD_ACK_WAIT) {
            furi_delay_us(10);
            continue;
        }
        if(r == SWD_ACK_FAULT && faults++ < SWD_FAULT_RETRIES) {
            swd_clear_sticky();
            furi_delay_us(10);
            continue;
        }
        break;
    }
    if(r != SWD_ACK_OK) g_last_ack_err = r;
    return r;
}

static SWDAck swd_ap_read(uint8_t addr, uint32_t* out) {
    SWDAck   r      = SWD_ACK_WAIT;
    uint32_t faults = 0;
    for(uint32_t i = 0; i < SWD_WAIT_RETRIES; i++) {
        r = swd_ap_read_once(addr, out);
        if(r == SWD_ACK_WAIT) {
            furi_delay_us(10);
            continue;
        }
        if(r == SWD_ACK_FAULT && faults++ < SWD_FAULT_RETRIES) {
            swd_clear_sticky();
            furi_delay_us(10);
            continue;
        }
        break;
    }
    if(r != SWD_ACK_OK) g_last_ack_err = r;
    return r;
}

static SWDAck swd_ap_write(uint8_t addr, uint32_t val) {
    SWDAck   r      = SWD_ACK_WAIT;
    uint32_t faults = 0;
    for(uint32_t i = 0; i < SWD_WAIT_RETRIES; i++) {
        r = swd_ap_write_once(addr, val);
        if(r == SWD_ACK_WAIT) {
            furi_delay_us(10);
            continue;
        }
        if(r == SWD_ACK_FAULT && faults++ < SWD_FAULT_RETRIES) {
            swd_clear_sticky();
            furi_delay_us(10);
            continue;
        }
        break;
    }
    if(r != SWD_ACK_OK) g_last_ack_err = r;
    return r;
}

/* ===========================================================================
 * Line reset and JTAG-to-SWD switch
 * ======================================================================== */

/**
 * swd_line_reset() — send 56 clocks with SWDIO HIGH.
 * This resets the SWD state machine on the target.
 * Output mode is assumed; remains in output mode on return.
 */
static void swd_line_reset(void) {
    dio_output();
    dio_high();
    for(int i = 0; i < 56; i++) {
        half_clk();
        clk_high();
        half_clk();
        clk_low();
    }
}

/**
 * swd_jtag_to_swd() — send the 16-bit JTAG-to-SWD switch sequence 0xE79E
 * LSB-first, then float SWDIO back to idle.
 */
static void swd_jtag_to_swd(void) {
    dio_output();
    /* 0xE79E = 0b1110_0111_1001_1110 */
    swd_write_bits(0xE79EU, 16);
}

/**
 * swd_idle_cycles() — drive SWDIO LOW for 'n' clock cycles (idle / padding).
 * Output mode assumed.
 */
static void swd_idle_cycles(uint8_t n) {
    dio_output();
    dio_low();
    for(uint8_t i = 0; i < n; i++) {
        half_clk();
        clk_high();
        half_clk();
        clk_low();
    }
}

/* ===========================================================================
 * Public API implementation
 * ======================================================================== */

void swd_raw_capture(uint32_t* first32, uint32_t* second32) {
    furi_hal_gpio_init(SWD_SWCLK, GpioModeOutputPushPull, GpioPullNo, GpioSpeedVeryHigh);
    furi_hal_gpio_write(SWD_SWCLK, false);

    swd_line_reset();
    swd_jtag_to_swd();
    swd_line_reset();
    swd_idle_cycles(8);

    /* DP IDCODE read request (0xA5) */
    dio_output();
    swd_write_bits(swd_build_request(false, true, DP_REG_IDCODE), 8);

    /* Release the line and clock in raw bits — no turnaround assumed. */
    dio_input();
    swd_read_bits(first32, 32);
    swd_read_bits(second32, 32);
}

/**
 * swd_resync() — line reset + JTAG-to-SWD, then confirm the DP by reading
 * IDCODE.  Used both to establish the link and to recover between write-frame
 * calibration attempts, since a mis-framed write leaves the DP out of step.
 */
static SWDAck swd_resync(void) {
    SWDAck result = SWD_ERR_NO_TARGET;

    for(int attempt = 0; attempt < 4; attempt++) {
        swd_line_reset();

        /* Odd attempts add the JTAG-to-SWD switch, in case the debug port
         * came up in JTAG mode; the N32G031 is SWD-only so even attempts
         * skip it. */
        if(attempt & 1) {
            swd_jtag_to_swd();
            swd_line_reset();
        }

        swd_idle_cycles(8);

        uint32_t idcode = 0;
        result = swd_dp_read(DP_REG_IDCODE, &idcode);
        if(result != SWD_ACK_OK) {
            swd_dp_write(DP_REG_ABORT, DP_ABORT_CLEAR_ALL);
            continue;
        }

        g_last_idcode = idcode;

        /* Bit 0 of a valid IDCODE is always 1 per the ARM spec. */
        if((idcode & 1U) == 0) {
            result = SWD_ERR_NO_TARGET;
            continue;
        }

        return SWD_ACK_OK;
    }

    return result;
}

SWDAck swd_connect(void) {
    /* Initialise both pins at very-high speed; CLK starts low, SWDIO high. */
    furi_hal_gpio_init(SWD_SWCLK, GpioModeOutputPushPull, GpioPullNo, GpioSpeedVeryHigh);
    furi_hal_gpio_write(SWD_SWCLK, false);
    furi_hal_gpio_init(SWD_SWDIO, GpioModeOutputPushPull, GpioPullNo, GpioSpeedVeryHigh);
    furi_hal_gpio_write(SWD_SWDIO, true);

    SWDAck result = SWD_ERR_NO_TARGET;

    g_last_stage  = 1; /* reading IDCODE */
    g_last_idcode = 0;

    result = swd_resync();
    if(result != SWD_ACK_OK) {
        /* Return the real last ACK so the UI can show a useful code:
         *   2 = WAIT  — target busy
         *   4 = FAULT — target flagged an error
         *   7 = all-ones → SWDIO floating / nothing driving the bus
         *   0 = all-zeros → SWDIO stuck low (short / wrong pin) */
        return result;
    }

    /*
     * Calibrate the write frame.
     *
     * Reads are known-good at this point, but writes were being ACKed while
     * their data never landed — CTRL/STAT read back as zero no matter what we
     * wrote.  The unknown is how many turnaround clocks this target wants
     * between its ACK and the host's data, and guessing it one rebuild at a
     * time is hopeless, so try each candidate and keep whichever one sticks.
     *
     * The test is self-verifying: write CTRL/STAT, then read it back.  If the
     * power-up acknowledge appears, the data phase reached the DP intact.
     * Every attempt restarts from a line reset, because a mis-framed write
     * leaves the DP's state machine out of step with us.
     */
    g_last_stage = 2;

    uint32_t stat    = 0;
    bool     powered = false;

    for(uint8_t trn = 0; trn <= 3 && !powered; trn++) {
        g_write_trn = trn;

        if(swd_resync() != SWD_ACK_OK) continue;

        swd_dp_write(DP_REG_ABORT, DP_ABORT_CLEAR_ALL);

        /* SELECT must be written before CTRL/STAT: DPBANKSEL decides which
         * register is visible at DP address 0x04, and SELECT survives a line
         * reset, so a stale bank would send the reads below elsewhere. */
        if(swd_dp_write(DP_REG_SELECT, 0x00000000UL) != SWD_ACK_OK) continue;

        /* CSYSPWRUPREQ (bit 30) | CDBGPWRUPREQ (bit 28) */
        if(swd_dp_write(DP_REG_CTRLSTAT, 0x50000000UL) != SWD_ACK_OK) continue;

        for(uint32_t i = 0; i < 100U; i++) {
            if(swd_dp_read(DP_REG_CTRLSTAT, &stat) != SWD_ACK_OK) break;
            g_last_stat = stat;
            /* CDBGPWRUPACK (bit 29) is the one that matters — not every part
             * implements the system power domain and asserts bit 31. */
            if(stat & 0x20000000UL) {
                powered = true;
                break;
            }
            furi_delay_us(100);
        }
    }

    g_last_trn = g_write_trn;
    if(!powered) return SWD_ERR_TIMEOUT;

    g_last_stage = 5; /* MEM-AP CSW */

    /* Step 9: Configure MEM-AP CSW for 32-bit word transfers, AddrInc off */
    /* Size[2:0]=010 (32-bit), AddrInc[5:4]=00 (off) = 0x00000002        */
    result = swd_ap_write(AP_REG_CSW, CSW_SIZE_32);
    if(result == SWD_ACK_OK) {
        g_csw_size   = CSW_SIZE_32;
        g_last_stage = 0; /* completed */
    }
    return result;
}

static uint8_t probe_line(const GpioPin* pin) {
    uint8_t r = 0;
    furi_hal_gpio_init(pin, GpioModeInput, GpioPullDown, GpioSpeedLow);
    furi_delay_us(200);
    if(furi_hal_gpio_read(pin)) r |= 1;
    furi_hal_gpio_init(pin, GpioModeInput, GpioPullUp, GpioSpeedLow);
    furi_delay_us(200);
    if(furi_hal_gpio_read(pin)) r |= 2;
    furi_hal_gpio_init(pin, GpioModeInput, GpioPullNo, GpioSpeedLow);
    return r;
}

void swd_probe(uint8_t* swdio_state, uint8_t* swclk_state) {
    *swdio_state = probe_line(SWD_SWDIO);
    *swclk_state = probe_line(SWD_SWCLK);
}

uint8_t swd_last_stage(void) {
    return g_last_stage;
}

uint32_t swd_last_idcode(void) {
    return g_last_idcode;
}

uint32_t swd_last_stat(void) {
    return g_last_stat;
}

uint8_t swd_last_trn(void) {
    return g_last_trn;
}

void swd_disconnect(void) {
    /* Return both pins to floating input — no pull, low speed */
    furi_hal_gpio_init(SWD_SWDIO, GpioModeInput, GpioPullNo, GpioSpeedLow);
    furi_hal_gpio_init(SWD_SWCLK, GpioModeInput, GpioPullNo, GpioSpeedLow);
}

static SWDAck swd_set_csw(uint32_t size) {
    if(g_csw_size == size) return SWD_ACK_OK;
    SWDAck r = swd_ap_write(AP_REG_CSW, size);
    if(r == SWD_ACK_OK) g_csw_size = size;
    return r;
}

SWDAck swd_write16(uint32_t addr, uint16_t val) {
    SWDAck result = swd_set_csw(CSW_SIZE_16);
    if(result != SWD_ACK_OK) return result;

    result = swd_ap_write(AP_REG_TAR, addr);
    if(result != SWD_ACK_OK) return result;

    /* On a 16-bit transfer the MEM-AP takes the half-word from the DRW byte
     * lane picked out by address bit 1. */
    uint32_t data = (addr & 2U) ? ((uint32_t)val << 16) : (uint32_t)val;
    return swd_ap_write(AP_REG_DRW, data);
}

SWDAck swd_read16(uint32_t addr, uint16_t* out) {
    SWDAck result = swd_set_csw(CSW_SIZE_16);
    if(result != SWD_ACK_OK) return result;

    result = swd_ap_write(AP_REG_TAR, addr);
    if(result != SWD_ACK_OK) return result;

    /* AP reads are pipelined — the real data comes back via DP RDBUFF. */
    uint32_t discard = 0;
    result = swd_ap_read(AP_REG_DRW, &discard);
    if(result != SWD_ACK_OK) return result;

    uint32_t data = 0;
    result = swd_dp_read(DP_REG_RDBUFF, &data);
    if(result != SWD_ACK_OK) return result;

    *out = (uint16_t)((addr & 2U) ? (data >> 16) : data);
    return SWD_ACK_OK;
}

SWDAck swd_read32(uint32_t addr, uint32_t* out) {
    SWDAck result = swd_set_csw(CSW_SIZE_32);
    if(result != SWD_ACK_OK) return result;

    /* Write TAR with the target address */
    result = swd_ap_write(AP_REG_TAR, addr);
    if(result != SWD_ACK_OK) return result;

    /*
     * AP reads are pipelined:
     *   1. Initiate the read of DRW — result is from previous read (stale).
     *   2. Read DP RDBUFF — this captures the actual DRW data.
     */
    uint32_t discard = 0;
    result = swd_ap_read(AP_REG_DRW, &discard);
    if(result != SWD_ACK_OK) return result;

    result = swd_dp_read(DP_REG_RDBUFF, out);
    return result;
}

SWDAck swd_write32(uint32_t addr, uint32_t val) {
    SWDAck result = swd_set_csw(CSW_SIZE_32);
    if(result != SWD_ACK_OK) return result;

    /* Write TAR */
    result = swd_ap_write(AP_REG_TAR, addr);
    if(result != SWD_ACK_OK) return result;

    /* Write DRW — data goes straight through to the target */
    result = swd_ap_write(AP_REG_DRW, val);
    return result;
}

SWDAck swd_halt(void) {
    return swd_write32(DHCSR_ADDR, DHCSR_HALT_VAL);
}

SWDAck swd_reset_and_run(void) {
    return swd_write32(AIRCR_ADDR, AIRCR_RESET_VAL);
}
