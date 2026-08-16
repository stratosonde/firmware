/* GNSS fix-acceptance wiring gate (Phase A4 of the 2026-08-15 next-step
 * handoff; tracker #284 / H-08).
 *
 * RED-FIRST: this suite is red against the current tree for the intended
 * reasons and is added to `all` by the A5 implementation commit.
 *
 * The defect (two parts):
 *  1. The mission's fix-acceptance decision is a HARDCODED threshold pair
 *     (4 satellites / HDOP <= 5.0) inside GNSS_IsFixGoodQuality(), while the
 *     configured limits (gps_min_satellites default 4, gps_max_hdop_x10
 *     default 25 = 2.5) exist in config but are never consumed. Anything
 *     with HDOP in (2.5, 5.0] is accepted today that the configured mission
 *     policy must reject.
 *  2. Acceptance is decided by a low-level driver predicate rather than one
 *     authoritative, configured mission predicate in the pure gnss_acquire
 *     module.
 *
 * What this suite pins:
 *  - GREEN characterization of the current hardcoded behaviour (the defect,
 *    documented behaviourally);
 *  - RED wiring scans: the configured predicate exists in gnss_acquire, the
 *    adapter routes every acceptance decision through it with the configured
 *    limits, and the hardcoded predicate is no longer called by lora_app.c;
 *  - RED documentation scan: the retained low-level GNSS_IsFixGoodQuality
 *    is marked so it cannot be mistaken for the mission predicate.
 *
 * The direct behavioural boundary tests of the new predicate (satellite/HDOP
 * boundary equality, NaN/Inf/negative HDOP, missing tokens, invalid quality,
 * invalid coordinates, genuine (0,0), null inputs) land in
 * test_gnss_acquire.c WITH the A5 implementation - the symbol does not exist
 * before it, so those tests cannot run red first.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>

/* GNSS parser host stub surface (identical to test_dr_20260812.c). */
#include "config.h"
#include "timer_if.h"
#include "stm32_systime.h"

const SystemConfig_t *Config_Get(void) { return NULL; }
static uint32_t g_fake_rtc_seconds = 0;
uint32_t TIMER_IF_GetTime(uint16_t *ms) { if (ms) *ms = 0; return g_fake_rtc_seconds; }
uint16_t g_host_dma_cndtr = 0;
uint32_t HAL_GetTick(void) { return g_fake_rtc_seconds * 1000U; }
static uint32_t g_fake_epoch = 1754500000U;
SysTime_t SysTimeGet(void) { SysTime_t t = { g_fake_epoch, 0 }; return t; }

#include "../../Core/Src/atgm336h.c"

static int checks = 0, failures = 0;
#define CHECK(cond) do { checks++; if (!(cond)) { \
    failures++; printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); } } while (0)

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
    /* Firmware sources are CRLF; scan anchors are written LF. */
    {
        size_t r = 0, w = 0;
        while (r < (size_t)n) {
            if (buf[r] != '\r') buf[w++] = buf[r];
            r++;
        }
        buf[w] = '\0';
    }
    return buf;
}

/* Strip block and line comments so a scan asserts on CODE, not prose. */
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

/* Build a handle that the current hardcoded predicate must judge. */
static GNSS_HandleTypeDef make_fix(uint8_t sats, float hdop)
{
    GNSS_HandleTypeDef g;
    memset(&g, 0, sizeof(g));
    g.data.valid = true;
    g.data.position_present = true;
    g.data.fix_quality = GNSS_FIX_GPS;
    g.data.satellites = sats;
    g.data.hdop = hdop;
    g.data.latitude = 48.8566f;    /* Paris - real coordinates, not (0,0) */
    g.data.longitude = 2.3522f;
    return g;
}

int main(void)
{
    printf("-- A4 characterization: current acceptance is hardcoded 4 sats / HDOP 5.0\n");

    /* The defect, behaviourally: HDOP 3.0 with 5 satellites is OUTSIDE the
     * configured mission limit (gps_max_hdop_x10 = 25 -> 2.5) yet ACCEPTED
     * by the hardcoded 5.0 threshold. A5 routes this decision through
     * GnssAcquire_FixAccepted with the configured limits, which rejects it. */
    GNSS_HandleTypeDef g = make_fix(5, 3.0f);
    CHECK(GNSS_IsFixGoodQuality(&g));            /* current (wrong) behaviour */
    g = make_fix(5, 5.0f);
    CHECK(GNSS_IsFixGoodQuality(&g));            /* hardcoded boundary, accept */
    g = make_fix(5, 5.1f);
    CHECK(!GNSS_IsFixGoodQuality(&g));           /* hardcoded boundary, reject */

    printf("-- A4 wiring: one configured authoritative predicate (RED pre-A5)\n");

    char *app   = strip_comments(slurp("../../LoRaWAN/App/lora_app.c"));
    char *gacq  = strip_comments(slurp("../../Core/Src/gnss_acquire.c"));
    char *gacqh = strip_comments(slurp("../../Core/Inc/gnss_acquire.h"));
    char *raw_atgm = slurp("../../Core/Src/atgm336h.c");  /* comments intact */

    /* The configured predicate exists in the pure module. */
    CHECK(strstr(gacqh, "GnssAcquire_FixAccepted") != NULL);
    CHECK(strstr(gacq, "GnssAcquire_FixAccepted") != NULL);

    /* The adapter routes acceptance through it, with the configured limits
     * snapshotted into designated fields. */
    CHECK(strstr(app, "GnssAcquire_FixAccepted") != NULL);
    CHECK(strstr(app, ".minimum_satellites =") != NULL);
    CHECK(strstr(app, ".maximum_hdop_x10 =") != NULL);

    /* The hardcoded predicate is no longer an acceptance authority in the
     * adapter (it may remain as a documented low-level invariant). */
    CHECK(strstr(app, "GNSS_IsFixGoodQuality(") == NULL);

    /* The retained low-level predicate is documented so it cannot be
     * mistaken for the mission acceptance decision (comment scan on the
     * raw source, pre-strip). */
    CHECK(strstr(raw_atgm, "NOT the mission fix-acceptance predicate") != NULL);

    printf("%d checks, %d failures\n", checks, failures);
    return failures != 0;
}

