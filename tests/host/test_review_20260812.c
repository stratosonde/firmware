/**
  ******************************************************************************
  * @file    test_review_20260812.c
  * @brief   Red-first regressions for the 2026-08-12 review: S-A, S-B, S-C
  ******************************************************************************
  * S-A and S-C are SOURCE SCANS (the invariant lives in HAL-bound lora_app.c /
  * main.c), following the R2-13 "grep-proof" precedent in R2_TEST_MAP.md.
  * S-B is BEHAVIOURAL — CalculateVoltageSlope is pure, so the backward-time
  * sign flip is provable by calling it.
  *
  *   make -C tests/host review0812        (red until the fixes land)
  *   EXPECT_UNFIXED=1 ./test_review0812   (green pre-fix gate)
  ******************************************************************************
  */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>

#include "power_model.h"

static int g_failures = 0;
static int g_checks = 0;
static int g_expected_failures = 0;

#define CHECK_REGRESSION(cond, id) do { \
    g_checks++; \
    if (!(cond)) { \
        g_failures++; \
        g_expected_failures++; \
        printf("FAIL [%s] %s:%d: %s\n", id, __FILE__, __LINE__, #cond); \
    } \
} while (0)

static char *slurp(const char *path)
{
    FILE *f = fopen(path, "rb");
    if (!f) { printf("FATAL: cannot open %s\n", path); exit(2); }
    fseek(f, 0, SEEK_END);
    long n = ftell(f);
    fseek(f, 0, SEEK_SET);
    char *buf = (char *)malloc((size_t)n + 1);
    if (!buf) exit(2);
    if (fread(buf, 1, (size_t)n, f) != (size_t)n) exit(2);
    buf[n] = '\0';
    fclose(f);
    /* Firmware sources are CRLF; multi-line scan anchors (function
     * signatures, "\n}\n" body terminators) are written LF. Normalise so the
     * scans match CODE, not line-ending accidents — without this, in_function
     * silently searched to EOF and F-3 false-passed on a later identifier. */
    {
        size_t r = 0, w = 0;
        while (r < n) {
            if (buf[r] != '\r') buf[w++] = buf[r];
            r++;
        }
        buf[w] = '\0';
    }
    return buf;
}

/* Strip block and line comments so a scan asserts on CODE, not on prose that
 * merely describes the bug (every finding below has a comment near it). */
static char *strip_comments(const char *src)
{
    size_t n = strlen(src);
    char *out = (char *)malloc(n + 1);
    size_t o = 0;
    for (size_t i = 0; i < n; i++) {
        if (src[i] == '/' && src[i + 1] == '*') {
            i += 2;
            while (i < n && !(src[i] == '*' && src[i + 1] == '/')) i++;
            i++;
            out[o++] = ' ';
        } else if (src[i] == '/' && src[i + 1] == '/') {
            while (i < n && src[i] != '\n') i++;
            out[o++] = '\n';
        } else {
            out[o++] = src[i];
        }
    }
    out[o] = '\0';
    return out;
}

/* Count non-overlapping occurrences of needle. */
static int count_occurrences(const char *hay, const char *needle)
{
    int c = 0;
    size_t nl = strlen(needle);
    const char *p = hay;
    while ((p = strstr(p, needle)) != NULL) { c++; p += nl; }
    return c;
}

/* True if `needle` appears inside the function body that starts at the first
 * occurrence of `sig` and ends at the next line starting with '}' at col 0. */
static bool in_function(const char *src, const char *sig, const char *needle)
{
    const char *start = strstr(src, sig);
    if (!start) { printf("FATAL: signature not found: %s\n", sig); exit(2); }
    const char *end = strstr(start, "\n}\n");
    if (!end) end = src + strlen(src);
    size_t len = (size_t)(end - start);
    char *body = (char *)malloc(len + 1);
    memcpy(body, start, len);
    body[len] = '\0';
    bool found = strstr(body, needle) != NULL;
    free(body);
    return found;
}


/* ========================================================================== */
/* S-A (P1) — the GPS-loss-silence forced retry must carry an acquisition
 *            budget, not just an enable flag                                 */
/* ========================================================================== */
/* STAB-03 (#150) forces gps_enabled_by_power_mgmt = true so a silenced unit
 * keeps trying for a fix. But ApplyOperatingMode() sets gps_timeout_ms = 0 for
 * exactly the modes that reach this code (REDUCED / RECOVERY), so
 * AcquireGnssFix(0, ...) evaluates `while ((HAL_GetTick() - gps_start) < 0)`
 * zero times and falls straight through to the stale-position branch, having
 * power-cycled the receiver for nothing. s_last_fresh_fix_s never advances, so
 * the 6 h silence is permanent. (Window is 24 h since 2026-08-13 — DDR-0015
 * BR-STALE-017; "permanent" was the point, and the fix below is what makes
 * BR-STALE-019 "one fix clears it" reachable at all.)
 *
 * Invariant the fix must establish: the block that forces the enable flag must
 * also raise a zero gps_timeout_ms. */
static void test_sa_gps_loss_retry_budget(const char *app)
{
    printf("-- S-A (P1): GPS-loss forced retry must set a non-zero timeout\n");

    /* Isolate the STAB-03 force block: the assignment plus a small window. */
    const char *force = strstr(app, "gps_enabled_by_power_mgmt = true;");
    bool found = (force != NULL);
    printf("   force-enable site present: %s\n", found ? "yes" : "no");
    if (!found) { printf("FATAL: S-A anchor missing\n"); exit(2); }

    /* Walk every force site; the one guarded by MODE_SURVIVAL is STAB-03's. */
    bool budget_raised = false;
    const char *p = app;
    while ((p = strstr(p, "current_mode != MODE_SURVIVAL")) != NULL) {
        const char *end = strstr(p, "SONDE_LOG_STR(\"GPS-LOSS SILENCE");
        if (!end) end = p + 900;
        size_t len = (size_t)(end - p);
        char *win = (char *)malloc(len + 1);
        memcpy(win, p, len); win[len] = '\0';
        if (strstr(win, "gps_timeout_ms") != NULL) budget_raised = true;
        free(win);
        p += 20;
    }
    printf("   forced retry assigns gps_timeout_ms: %s (want yes)\n",
           budget_raised ? "yes" : "no");
    CHECK_REGRESSION(budget_raised, "S-A");
}

/* ========================================================================== */
/* S-C (P1) — UART RX DMA must survive a reception error                      */
/* ========================================================================== */
/* stm32wlxx_hal_uart.c treats ANY error as blocking while CR3.DMAR is set, and
 * aborts the circular GNSS DMA. SP-01 (#244, verified against the vendored HAL
 * 2026-08-13): HAL_UART_IRQHandler NEVER consults the AdvancedInit
 * DMADisableonRxError bit, so the two advanced-feature assignments below CANNOT
 * save the transfer - they keep their hardware value (ORE overwrite, DDRE), but
 * the operative fix is the HAL_UART_ErrorCallback override that re-arms the
 * stream (usart_if.c -> GNSS_UART_ErrorCallback, covered behaviourally by
 * test_sp_20260813.c). */
static void test_sc_uart_dma_error_recovery(const char *mainc, const char *usart,
                                            const char *gnss)
{
    printf("-- S-C (P1): UART RX DMA must not die permanently on an RX error\n");

    bool ovr_disabled = strstr(mainc, "UART_ADVFEATURE_OVERRUN_DISABLE") != NULL;
    bool dma_survives = strstr(mainc, "UART_ADVFEATURE_DMA_ENABLEONRXERROR") != NULL;
    bool no_init_left = strstr(mainc, "AdvFeatureInit = UART_ADVFEATURE_NO_INIT") != NULL;

    /* SP-01: the error-callback re-arm is REQUIRED (bits alone are inert). */
    bool err_cb = (strstr(usart, "HAL_UART_ErrorCallback") != NULL) ||
                  (strstr(gnss,  "HAL_UART_ErrorCallback") != NULL);

    printf("   OverrunDisable: %s | DMA_ENABLEONRXERROR: %s | NO_INIT still set: %s\n",
           ovr_disabled ? "yes" : "no", dma_survives ? "yes" : "no",
           no_init_left ? "yes" : "no");
    printf("   HAL_UART_ErrorCallback override (operative fix, SP-01): %s\n",
           err_cb ? "yes" : "no");

    CHECK_REGRESSION(err_cb, "S-C-callback");
    CHECK_REGRESSION(!no_init_left || err_cb, "S-C-noinit");
}

/* ========================================================================== */
/* S-B (P2) — CalculateVoltageSlope must not evaluate a backward time delta   */
/* ========================================================================== */
/* RV-06 (#162) added this guard to Deadman_Check, LaunchDetector_Update and
 * FloatDetector_Update. CalculateVoltageSlope was missed. Its state is RAM-only
 * and survives an LSE->LSI failover (no MCU reset), so baseline_timestamp
 * outlives the RTC restart. A SMALL backward step casts to a small negative
 * int32_t and (dv * 3600) / -dt sign-flips AND magnifies. */
static void test_sb_slope_backward_time(void)
{
    printf("-- S-B (P2): backward RTC step must not invert the voltage slope\n");

    VoltageSlope_t s;
    memset(&s, 0, sizeof(s));
    CalculateVoltageSlope(&s, 5200, 1000);
    int16_t healthy = CalculateVoltageSlope(&s, 5170, 2000);
    printf("   healthy slope (discharging): %+d mV/h\n", healthy);
    CHECK_REGRESSION(healthy < 0, "S-B-setup");

    /* RTC restarts: now steps backward from 2000 to 990, battery still falling. */
    int16_t after = CalculateVoltageSlope(&s, 5160, 990);
    printf("   after backward step to t=990: %+d mV/h (must NOT be positive)\n", after);
    CHECK_REGRESSION(after <= 0, "S-B");

    /* And the poisoned value must not be republished by the FW-6 sticky path. */
    int16_t sticky = CalculateVoltageSlope(&s, 5159, 1000);
    printf("   next cycle (FW-6 sticky window): %+d mV/h (must NOT be positive)\n", sticky);
    CHECK_REGRESSION(sticky <= 0, "S-B-sticky");
}

int main(void)
{
    char *app   = strip_comments(slurp("../../LoRaWAN/App/lora_app.c"));
    char *mainc = strip_comments(slurp("../../Core/Src/main.c"));
    char *usart = strip_comments(slurp("../../Core/Src/usart_if.c"));
    char *gnss  = strip_comments(slurp("../../Core/Src/atgm336h.c"));

    printf("=== 2026-08-12 review regressions (S-A, S-B, S-C) ===\n\n");

    test_sa_gps_loss_retry_budget(app);
    printf("\n");
    test_sc_uart_dma_error_recovery(mainc, usart, gnss);
    printf("\n");
    test_sb_slope_backward_time();

    printf("\n%d checks, %d failures (%d expected pre-fix)\n",
           g_checks, g_failures, g_expected_failures);

    if (getenv("EXPECT_UNFIXED") && g_failures == g_expected_failures) {
        printf("BASELINE OK (all failures are known-unfixed findings)\n");
        return 0;
    }
    return g_failures ? 1 : 0;
}
