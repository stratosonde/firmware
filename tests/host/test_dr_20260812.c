/**
  ******************************************************************************
  * @file    test_dr_20260812.c
  * @brief   Red-first regressions for the 2026-08-12 independent deep review
  *          ("DR" series) + the 2026-08-12 deep stability review P1 GNSS item.
  ******************************************************************************
  *   DR-01 (#236): GNSS_ParseGGA commits to hgnss->data BEFORE the guards that
  *     reject the sentence; the discarded return value leaves mutations standing.
  *   STAB-P1#1 / DR-02 (#237): GNSS_IsFixGoodQuality does not require
  *     position_present; GNSS_HasPosition does not range-check;
  *     GeofenceRestricted fails open on invalid coordinates.
  *   DR-03: F8 mode-hysteresis hyst_last_ts has no backward-time-step re-seed
  *     (the one RV-06/#162 consumer that was missed).
  *
  *   Run:
  *   make -C tests/host dr        (red until the fixes land)
  *   EXPECT_UNFIXED=1 ./test_dr   (green pre-fix gate)
  ******************************************************************************
  */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>

#include "power_model.h"
#include "transmit_plan.h"
#include "config.h"

/* R47 precedent (test_main.c:456): Config_Get stubbed to NULL -> the
 * ApplyOperatingMode hardcoded-defaults path, same as the other suites. */
const SystemConfig_t *Config_Get(void) { return NULL; }

/* GNSS parser host stub surface (identical to test_stability_review.c). */
#include "timer_if.h"
#include "stm32_systime.h"

static uint32_t g_fake_rtc_seconds = 0;
uint32_t TIMER_IF_GetTime(uint16_t *ms) { if (ms) *ms = 0; return g_fake_rtc_seconds; }
uint16_t g_host_dma_cndtr = 0;
uint32_t HAL_GetTick(void) { return g_fake_rtc_seconds * 1000U; }

static uint32_t g_fake_epoch = 1754500000U;  /* 2025-08-06-ish UTC */
SysTime_t SysTimeGet(void) { SysTime_t t = { g_fake_epoch, 0 }; return t; }

#include "../../Core/Src/atgm336h.c"

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
        while (r < n) {
            if (buf[r] != '\r') buf[w++] = buf[r];
            r++;
        }
        buf[w] = '\0';
    }
    return buf;
}


/* ========================================================================== */
/* DR-01 (#236) — a REJECTED GGA must leave hgnss->data byte-for-byte unchanged */
/* ========================================================================== */
static void seed_known_good(GNSS_HandleTypeDef *g)
{
    memset(g, 0, sizeof(*g));
    g->data.timestamp = 120000;
    g->data.fix_quality = GNSS_FIX_GPS;
    g->data.satellites = 8;
    g->data.hdop = 1.2f;
    g->data.altitude = 1042.5f;
    g->data.latitude = 51.0f;
    g->data.longitude = -114.0f;
    g->data.position_present = true;
    g->data.valid = true;
}

static void check_rejected_gga_leaves_state(const char *sentence, const char *tag)
{
    GNSS_HandleTypeDef g;
    seed_known_good(&g);
    GNSS_Data_t before = g.data;

    int rc = GNSS_ParseGGA(&g, sentence);
    CHECK(rc == -1);                                   /* guard: truly rejected */
    CHECK_REGRESSION(memcmp(&g.data, &before, sizeof(before)) == 0, tag);
}

static void test_dr01_rejected_gga_is_transactional(void)
{
    printf("-- DR-01 (#236): rejected GGA must not mutate hgnss->data\n");

    /* F-11 scenario: garbage hemisphere char, coordinates otherwise valid. */
    check_rejected_gga_leaves_state(
        "$GNGGA,130000,4807.0380,X,01131.0000,W,1,08,0.9,545.4,M,46.9,M,,*00",
        "DR-01-hemisphere");

    /* Out-of-range coordinate (lat 91.0 > 90; 90.0000017 would round to
     * exactly 90.0f and legitimately pass the range check). */
    check_rejected_gga_leaves_state(
        "$GNGGA,130000,9100.0000,N,01131.0000,E,1,08,0.9,545.4,M,46.9,M,,*00",
        "DR-01-range");

    /* Position-less GGA with valid fix metrics (also the test_main.c:254
     * behaviour change: fix fields must NOT be committed either). */
    check_rejected_gga_leaves_state(
        "$GNGGA,130000,,,,,1,06,1.0,,M,,M,,*51",
        "DR-01-positionless");

    /* Guard the fix direction: a well-formed sentence still parses AND
     * commits (both hemispheres). */
    GNSS_HandleTypeDef g;
    memset(&g, 0, sizeof(g));
    int rc = GNSS_ParseGGA(&g, "$GNGGA,120000,4807.0380,S,01131.0000,W,1,08,0.9,545.4,M,46.9,M,,*4C");
    CHECK(rc == 0);
    CHECK(g.data.latitude < -48.11729 && g.data.latitude > -48.11731);
    CHECK(g.data.longitude < -11.5166 && g.data.longitude > -11.5167);
    CHECK(g.data.position_present);
    CHECK(g.data.satellites == 8);
}

/* ========================================================================== */
/* STAB-P1#1 + DR-02 (#237) — the position trust gates                         */
/* ========================================================================== */
static void test_stab_p1_positionless_never_good_quality(void)
{
    printf("-- STAB-P1#1 (#237): position-less GGA + RMC 'A' must not be good quality\n");

    /* Test A of the review: checksum-valid GGA with valid fix metrics but NO
     * lat/lon tokens, followed by an active RMC. */
    GNSS_HandleTypeDef g;
    memset(&g, 0, sizeof(g));
    int rc = GNSS_ParseGGA(&g, "$GNGGA,120000,,,,,1,06,1.0,,M,,M,,*51");
    CHECK(rc == -1);
    GNSS_ParseRMC(&g, "$GNRMC,120000,A,,,,,,,060825,,,N*00");
    CHECK(g.data.valid);                       /* RMC 'A' latches valid */
    CHECK(!g.data.position_present);
    CHECK_REGRESSION(!GNSS_IsFixGoodQuality(&g), "STAB-P1#1");
    CHECK(!GNSS_HasPosition(&g));              /* guard: already gated */

    /* Test B of the review: a REAL Null Island fix (fields present, 0.0/0.0)
     * with good metrics must remain good quality. */
    memset(&g, 0, sizeof(g));
    rc = GNSS_ParseGGA(&g, "$GNGGA,120000,0000.0000,N,0000.0000,E,1,05,1.0,15.0,M,0.0,M,,*6E");
    CHECK(rc == 0);
    GNSS_ParseRMC(&g, "$GNRMC,120000,A,,,,,,,060825,,,N*00");
    CHECK(g.data.position_present);
    CHECK(GNSS_IsFixGoodQuality(&g));          /* guard: (0,0) is legitimate */
    CHECK(GNSS_HasPosition(&g));
}

static void test_dr02_has_position_range_checks(void)
{
    printf("-- DR-02a (#237): GNSS_HasPosition must range-check\n");

    GNSS_HandleTypeDef g;
    memset(&g, 0, sizeof(g));
    g.data.valid = true;
    g.data.position_present = true;
    g.data.latitude = 95.0f;                   /* out of range */
    g.data.longitude = -114.0f;
    CHECK_REGRESSION(!GNSS_HasPosition(&g), "DR-02a-lat");

    g.data.latitude = 51.0f;
    g.data.longitude = 190.0f;                 /* out of range */
    CHECK_REGRESSION(!GNSS_HasPosition(&g), "DR-02a-lon");

    /* Guard: in-range still passes. */
    g.data.longitude = -114.0f;
    CHECK(GNSS_HasPosition(&g));
}

static void test_dr02_geofence_not_fail_open(void)
{
    printf("-- DR-02c (#237): GeofenceRestricted must not fail open on invalid coords\n");

    /* The fix replaces the boolean helper (implausible -> "not restricted")
     * with an explicit tri-state whose UNKNOWN case the caller maps to the
     * documented "never had a fix -> transmit" policy. Scan lora_app.c. */
    char *src = slurp("../../LoRaWAN/App/lora_app.c");
    CHECK_REGRESSION(strstr(src, "GEO_PERMISSION_UNKNOWN") != NULL, "DR-02c-tristate");
    CHECK_REGRESSION(strstr(src, "static bool GeofenceRestricted") == NULL, "DR-02c-old-helper-gone");
    free(src);
}

/* ========================================================================== */
/* DR-03 — F8 hysteresis must re-seed on a backward time step (LSE->LSI)       */
/* ========================================================================== */
static void test_dr03_hysteresis_backward_step(void)
{
    printf("-- DR-03: LSE->LSI failover must re-seed the F8 hysteresis epoch\n");

    VoltageSlope_t vs;
    memset(&vs, 0, sizeof(vs));

    /* Seed, then a 700 s discharge: slope ~-514 mV/h -> commits SURVIVAL
     * (same setup as test_stability_review.c F-4). */
    (void)DecideTransmitPlan(&vs, 5200, 25.0f, false, 1000, true, false, false);
    TransmitPlan_t p0 = DecideTransmitPlan(&vs, 5100, 25.0f, false, 1700, true, false, false);
    OperatingMode_t committed = p0.power_mode;
    printf("   committed after discharge: %d\n", (int)committed);
    CHECK(committed != MODE_NORMAL);           /* guard: a downgrade committed */

    /* LSE->LSI failover: the RTC counter restarts near zero, voltage_slope
     * (and hyst_last_ts) survives in RAM. Charging resumes. */
    (void)DecideTransmitPlan(&vs, 5210, 25.0f, false, 100, true, false, false);
    CHECK_REGRESSION(vs.hyst_last_ts == 100, "DR-03-reseed");

    /* Three consistent upgrade proposals separated in time must confirm the
     * upgrade (F8_UPGRADE_CONFIRM=3). Pre-fix the streak stays frozen until
     * now_timestamp passes the stale pre-failover epoch. */
    (void)DecideTransmitPlan(&vs, 5215, 25.0f, false, 800, true, false, false);
    (void)DecideTransmitPlan(&vs, 5220, 25.0f, false, 1500, true, false, false);
    TransmitPlan_t p = DecideTransmitPlan(&vs, 5225, 25.0f, false, 2200, true, false, false);
    printf("   post-failover mode after 3 upgrade proposals: %d (committed was %d)\n",
           (int)p.power_mode, (int)committed);
    CHECK_REGRESSION(p.power_mode == MODE_NORMAL, "DR-03");
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

/* ========================================================================== */
/* DR-06 (#241) — every RF-silence path must record WHY in plan.veto           */
/* ========================================================================== */
/* ArchiveSample packs plan.veto into record.flags bits 5-7 (DDR-0003 §6a),
 * but only DecideTransmitPlan's no-session veto ever reached it; the other
 * four silence causes set the local rf_silence and archived as VETO_NONE.
 * Fix: one Silence() helper at every site, first veto wins. */
static void test_dr06_silence_records_why(void)
{
    printf("-- DR-06 (#241): all RF-silence sites record the veto reason\n");

    char *src = slurp("../../LoRaWAN/App/lora_app.c");

    /* No BARE `rf_silence = true` outside the helper (the helper's own
     * `*rf_silence = true` is the one allowed instance). */
    int total = count_occurrences(src, "rf_silence = true");
    int in_helper = count_occurrences(src, "*rf_silence = true");
    printf("   bare rf_silence=true sites: %d (helper instances: %d)\n",
           total - in_helper, in_helper);
    CHECK_REGRESSION(total - in_helper == 0, "DR-06-no-bare-sites");

    /* The previously-unrecorded causes must name their veto at the site. */
    CHECK_REGRESSION(strstr(src, "VETO_GPS_LOSS") != NULL, "DR-06-gps-loss");
    CHECK_REGRESSION(strstr(src, "VETO_PRELAUNCH_QUIET") != NULL, "DR-06-prelaunch");
    CHECK_REGRESSION(count_occurrences(src, "VETO_RESTRICTED_REGION") >= 3, "DR-06-restricted");

    free(src);

    /* The enum must fit the 3-bit flags field (bits 5-7: max value 7). */
    char *tp = slurp("../../Core/Inc/transmit_plan.h");
    CHECK(strstr(tp, "VETO_GPS_LOSS") != NULL);
    CHECK(strstr(tp, "VETO_PRELAUNCH_QUIET") != NULL);
    free(tp);
}

int main(void)
{
    test_dr01_rejected_gga_is_transactional();
    test_stab_p1_positionless_never_good_quality();
    test_dr02_has_position_range_checks();
    test_dr02_geofence_not_fail_open();
    test_dr03_hysteresis_backward_step();
    test_dr06_silence_records_why();

    printf("\n%d checks, %d failures (%d expected-unfixed)\n",
           g_checks, g_failures, g_expected_failures);

    if (getenv("EXPECT_UNFIXED") && g_failures == g_expected_failures) {
        printf("BASELINE OK (all failures are known-unfixed findings)\n");
        return 0;
    }
    return (g_failures == 0) ? 0 : 1;
}
