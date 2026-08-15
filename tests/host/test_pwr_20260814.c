/**
  ******************************************************************************
  * @file    test_pwr_20260814.c
  * @brief   Red-first regressions for the 2026-08-14 review PWR series
  *          (docs/temp/stratosonde-review-20260814.md).
  ******************************************************************************
  *   PWR-02 (P0): SelectModeFromPredictions opens with a temperature-blind
  *     raw floor (4300 mV). Per the firmware's OWN comp_table, the implied
  *     terminal voltage of a FULLY CHARGED pack crosses 4300 mV at -62.33 C;
  *     below that a full battery selects SURVIVAL unconditionally (GNSS off,
  *     60 min cadence, feeding the 24 h GPS-loss silence), and the STAB-03
  *     carve-out declines to force GNSS back on in SURVIVAL - an
  *     unrecoverable-while-cold mission-ender with a healthy pack. The fix is
  *     bench-gated: characterize the actual Nichicon flight pack, derive a
  *     temperature-scheduled floor(T) from that dataset (#261, #248). The
  *     checks below pin the desired post-fix shape and the R10 (#37) guard
  *     against over-correction; they are EXPECT_UNFIXED until the bench
  *     campaign lands.
  *
  *   PWR-01 (P1): compensation-table non-monotonicity (-40=700 -> -50=450 ->
  *     -55=430 -> -56=660). ALREADY red-gated as the LT-06 pair in
  *     test_lt_20260813.c; tracked as #248. Not duplicated here.
  *
  *   Run:
  *   make -C tests/host pwr          (red until the bench-gated fix lands)
  *   EXPECT_UNFIXED=1 ./test_pwr     (green pre-fix gate; CI shape via pwr-gate)
  ******************************************************************************
  */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>

#include "power_model.h"

static int g_failures = 0;
static int g_checks = 0;
static int g_expected_failures = 0;

#define CHECK(cond) do { \
    g_checks++; \
    if (!(cond)) { \
        g_failures++; \
        printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
    } \
} while (0)

#define CHECK_REGRESSION(cond, id) do { \
    g_checks++; \
    if (!(cond)) { \
        g_failures++; \
        g_expected_failures++; \
        printf("FAIL [%s] %s:%d: %s\n", id, __FILE__, __LINE__, #cond); \
    } \
} while (0)

/* Source-scan helpers (same shape as test_lt_20260813.c) */
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
    {   /* Firmware sources are CRLF; scan anchors are written LF. */
        size_t r = 0, w = 0;
        while (r < (size_t)n) {
            if (buf[r] != '\r') buf[w++] = buf[r];
            r++;
        }
        buf[w] = '\0';
    }
    return buf;
}

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

/* True when token appears in the function whose DEFINITION matches sig,
 * bounded by the next function's name (prototype-vs-definition lesson from
 * the FR wave). */
static bool in_function(const char *src, const char *sig, const char *next_fn,
                        const char *token)
{
    const char *def = strstr(src, sig);
    if (!def) { printf("FATAL: anchor not found: %s\n", sig); exit(2); }
    const char *end = strstr(def, next_fn);
    if (!end) { printf("FATAL: bound not found: %s\n", next_fn); exit(2); }
    size_t len = (size_t)(end - def);
    char *body = (char *)malloc(len + 1);
    memcpy(body, def, len);
    body[len] = '\0';
    bool found = strstr(body, token) != NULL;
    free(body);
    return found;
}

#define PWR_MODEL_SIG  "OperatingMode_t SelectModeFromPredictions(int16_t current_slope"
#define PWR_MODEL_NEXT "GetModeName"


/* ========================================================================== */
/* PWR-02 - BEHAVIOURAL: a fully charged pack must not select SURVIVAL just    */
/* because it is cold                                                          */
/* ========================================================================== */

static void test_pwr02_cold_full_not_survival(void)
{
    printf("PWR-02: full pack at deep cold must not select SURVIVAL\n");

    /* Implied FULL-pack terminal voltages, per power_model.c's own comp_table
     * (each row: Vmax_full = 5500 - compensation_mv). These constants describe
     * the OLD pack; when the Nichicon bench dataset lands, re-derive them from
     * that data (PWR-02 tracker issue). */
    static const struct { float temp_c; uint16_t raw_full_mv; } rows[] = {
        { -61.0f, 4550U },
        { -62.0f, 4400U },
        { -62.5f, 4250U },
        { -63.0f, 4100U },
        { -64.0f, 3810U },
        { -65.0f, 3330U },
    };

    int survival_count = 0;
    for (size_t i = 0; i < sizeof(rows) / sizeof(rows[0]); i++) {
        /* Flat slope, normalized 5500 mV, no predicted depletion: nothing but
         * the raw floor can justify SURVIVAL for a full pack. */
        OperatingMode_t m = SelectModeFromPredictions(
            0, 5500U, 0xFFFFU, rows[i].raw_full_mv);
        printf("  full pack @ %5.1f C (%u mV raw) -> %s\n",
               (double)rows[i].temp_c, rows[i].raw_full_mv, GetModeName(m));
        if (m == MODE_SURVIVAL) survival_count++;
    }

    /* Rows above the -62.33 C crossover pass today (ungated - also pins that
     * the floor is not absurdly LOW at shallower cold). */
    CHECK(SelectModeFromPredictions(0, 5500U, 0xFFFFU, 4550U) != MODE_SURVIVAL);
    CHECK(SelectModeFromPredictions(0, 5500U, 0xFFFFU, 4400U) != MODE_SURVIVAL);

    /* The finding itself: below -62.3 C every full-pack row latches SURVIVAL
     * with the fixed 4300 mV floor. Green only with a temperature-scheduled
     * floor(T) derived from the flight pack's bench data. */
    CHECK_REGRESSION(survival_count == 0, "PWR-02-cold-full-not-survival");
}

/* ========================================================================== */
/* PWR-02 - GUARD (ungated): a genuinely depleted pack must STILL select       */
/* SURVIVAL. The fix must not be a blanket floor reduction - that resurrects   */
/* the R10 (#37) brownout-masking bug. 2000 mV is below any plausible          */
/* discharge-cutoff floor(T) for an operating pack.                            */
/* ========================================================================== */

static void test_pwr02_survival_still_catches_brownout(void)
{
    printf("PWR-02 guard: genuinely depleted pack still selects SURVIVAL\n");
    CHECK(SelectModeFromPredictions(0, 5500U, 0xFFFFU, 2000U) == MODE_SURVIVAL);
}

/* ========================================================================== */
/* PWR-02 - STRUCTURAL: the raw floor must become temperature-scheduled, not   */
/* remain a bare constant. Red until the bench-derived floor(T) lands (the     */
/* signature is expected to gain a temperature parameter).                     */
/* ========================================================================== */

static void test_pwr02_floor_temp_scheduled(const char *pm)
{
    printf("PWR-02: structural - floor must be temperature-scheduled\n");
    CHECK_REGRESSION(
        !in_function(pm, PWR_MODEL_SIG, PWR_MODEL_NEXT, "< 4300"),
        "PWR-02-floor-fixed-constant");
    CHECK_REGRESSION(
        in_function(pm, PWR_MODEL_SIG, PWR_MODEL_NEXT, "temp"),
        "PWR-02-floor-references-temp");
}

int main(void)
{
    char *pm = strip_comments(slurp("../../Core/Src/power_model.c"));

    printf("=== PWR-series review regressions (2026-08-14) ===\n\n");

    test_pwr02_cold_full_not_survival();
    printf("\n");
    test_pwr02_survival_still_catches_brownout();
    printf("\n");
    test_pwr02_floor_temp_scheduled(pm);
    printf("\n");

    free(pm);

    printf("\n%d checks, %d failures (%d expected pre-fix)\n",
           g_checks, g_failures, g_expected_failures);

    if (getenv("EXPECT_UNFIXED") && g_failures == g_expected_failures) {
        printf("BASELINE OK (all failures are known-unfixed findings)\n");
        return 0;
    }
    return g_failures ? 1 : 0;
}
