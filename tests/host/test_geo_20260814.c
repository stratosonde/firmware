/* ARCHIVE — regression record for the 2026-08-14 GEO review. Module contracts
 * live in test_<module>.c (see R2_TEST_MAP.md). Do not extend; extend the
 * contract suite for the owning module instead. (Refactor stage 7.) */

/**
  ******************************************************************************
  * @file    test_geo_20260814.c
  * @brief   Red-first regressions for the 2026-08-14 review GEO series
  *          (docs/temp/stratosonde-review-20260814.md).
  *          Compiles against the REAL h3lite submodule sources; no firmware
  *          objects needed.
  ******************************************************************************
  *   GEO-01 (P0, firmware#257 / SP-02): the shipped region table contains
  *     ZERO REGION_RESTRICTED cells, so every enforcement consumer is dead
  *     code. Owner disposition: land cells to be marked RESTRICTED in the
  *     dataset (h3lite repo). The nonempty count below goes green when the
  *     regenerated table lands; the resolve-probe for a specific restricted
  *     coordinate is added with that dataset (probe coordinate is the
  *     owner's choice).
  *
  *   GEO-04 (P1, h3lite repo tracker): h3liteInit()'s F-013 self-check
  *     probes Paris->EU868 and mid-Atlantic->REGION_UNKNOWN but never proves
  *     the RESTRICTED enforcement set non-empty - it passes on today's empty
  *     table. Fix (submodule): init scans the table for REGION_RESTRICTED
  *     and probes a known restricted coordinate; generate_lookup_table.py
  *     asserts the set non-empty so a silent revert is impossible. MUST ride
  *     with the GEO-01 dataset (boot-fatal without it). Structural scans
  *     here pin both guards.
  *
  *   GEO-03 (P1, firmware#258 / SP-04): ring-search laundering + zero-
  *     candidate keep-current. OWNER DISPOSITION 2026-08-15: no behaviour
  *     rework - keep-current-on-UNKNOWN is sanctioned (ocean crossings);
  *     the dataset edit converts unmapped-land-laundering into enforced
  *     RESTRICTED. No checks here by design.
  *
  *   Run:
  *   make -C tests/host geo          (red until the dataset + guards land)
  *   EXPECT_UNFIXED=1 ./test_geo     (green pre-fix gate; CI shape via geo-gate)
  ******************************************************************************
  */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>

#include "h3lite.h"
#include "h3lite_regions_table.h"

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
    {   /* Sources are CRLF; scan anchors are written LF. */
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
 * bounded by the next function's name. */
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

/* ========================================================================== */
/* GREEN SANITY (ungated): the pipeline the F-013 self-check already probes    */
/* keeps working. These guard the table/geometry against regressions while     */
/* the gated findings stay red.                                                */
/* ========================================================================== */

static void test_geo_pipeline_sanity(void)
{
    printf("GEO sanity: init + Paris->EU868 + mid-Atlantic UNKNOWN\n");

    CHECK(h3liteInit() == true);
    CHECK(strcmp(getRegionName(latLngToRegion(48.8566, 2.3522)), "EU868") == 0);
    CHECK(latLngToRegion(0.0, -30.0) == REGION_UNKNOWN);
}

/* ========================================================================== */
/* GEO-01 - DATA (gated): the RESTRICTED enforcement set must be non-empty.    */
/* Green when the owner's dataset edit lands and the table is regenerated      */
/* (firmware#257 / SP-02).                                                     */
/* ========================================================================== */

static void test_geo01_restricted_nonempty(void)
{
    printf("GEO-01: region table must contain RESTRICTED cells\n");

    unsigned restricted = 0;
    unsigned total = 0;
    for (uint32_t i = 0; i < REGION_ENTRY_COUNT; i++) {
        if (RE_REGION(regionLookup[i]) == REGION_RESTRICTED) restricted++;
        total++;
    }
    printf("  %u of %u table entries are REGION_RESTRICTED\n", restricted, total);

    CHECK(total == REGION_ENTRY_COUNT);
    CHECK_REGRESSION(restricted > 0, "GEO-01-restricted-nonempty");

    /* The resolve-probe for a known restricted coordinate (review's
     * GEO-01-restricted-resolves anchor) is added with the dataset: the
     * probe coordinate is the owner's chosen restricted territory. */
}

/* ========================================================================== */
/* GEO-04 - STRUCTURAL (gated): h3liteInit() must prove the enforcement set    */
/* non-empty, and the generator must assert it so a future regeneration cannot */
/* silently revert. Both land in the h3lite submodule WITH the dataset (init   */
/* would be boot-fatal otherwise).                                             */
/* ========================================================================== */

static void test_geo04_guards(const char *h3c, const char *gen)
{
    printf("GEO-04: init self-check + generator guard for the restricted set\n");

    CHECK_REGRESSION(
        in_function(h3c, "bool h3liteInit(void) {", "H3Index latLngToH3",
                    "REGION_RESTRICTED"),
        "GEO-04-init-checks-restricted");

    /* Generator-side: an assertion naming RESTRICTED (e.g. a non-empty count
     * assert in generate_lookup_table.py). */
    bool has_assert = false;
    {
        const char *line = gen;
        while (line && *line) {
            const char *eol = strchr(line, '\n');
            size_t len = eol ? (size_t)(eol - line) : strlen(line);
            char *l = (char *)malloc(len + 1);
            memcpy(l, line, len);
            l[len] = '\0';
            if (strstr(l, "assert") && strstr(l, "RESTRICTED")) has_assert = true;
            free(l);
            if (!eol) break;
            line = eol + 1;
        }
    }
    CHECK_REGRESSION(has_assert, "GEO-04-generator-asserts-restricted");
}

int main(void)
{
    char *h3c = strip_comments(
        slurp("../../Middlewares/Third_Party/h3lite/src/h3lite.c"));
    char *gen = slurp("../../Middlewares/Third_Party/h3lite/generate_lookup_table.py");

    printf("=== GEO-series review regressions (2026-08-14) ===\n\n");

    test_geo_pipeline_sanity();
    printf("\n");
    test_geo01_restricted_nonempty();
    printf("\n");
    test_geo04_guards(h3c, gen);
    printf("\n");

    free(h3c); free(gen);

    printf("\n%d checks, %d failures (%d expected pre-fix)\n",
           g_checks, g_failures, g_expected_failures);

    if (getenv("EXPECT_UNFIXED") && g_failures == g_expected_failures) {
        printf("BASELINE OK (all failures are known-unfixed findings)\n");
        return 0;
    }
    return g_failures ? 1 : 0;
}

