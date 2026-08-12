/**
  ******************************************************************************
  * @file    test_stability_review.c
  * @brief   Red-first regressions for the 2026-08-11 (second pass)
  *          stability/predictability review: F-1 .. F-11
  ******************************************************************************
  * Two kinds of check live here:
  *
  *   SOURCE SCANS  (F-1, F-2, F-3, F-5, F-6, F-7) — the invariant lives in
  *   HAL-bound code (lora_app.c, stm32_lpm_if.c) that is not host-compilable
  *   without stubbing the whole HAL/LoRaMac surface. Following the R2-13
  *   "grep-proof" precedent (R2_TEST_MAP.md) these read the firmware sources
  *   as TEXT and assert the invariant the fix must establish.
  *
  *   BEHAVIOURAL   (F-4, F-10, F-11) — DecideTransmitPlan and the GNSS parser
  *   are pure, so the burst-defeats-hysteresis, HHMMSS-range and hemisphere-
  *   default bugs are provable by calling them.
  *
  * Every check is CHECK_REGRESSION (expected-fail-before-fix). Run:
  *   make -C tests/host stability     (red until the fixes land)
  *   EXPECT_UNFIXED=1 ./test_stability   (green pre-fix gate)
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
 * ApplyOperatingMode hardcoded-defaults path, same as the main suite. */
const SystemConfig_t *Config_Get(void) { return NULL; }

/* F-10/F-11: the GNSS parser is pure tokenizing + math — #include it like
 * test_main.c:40 does, with the same host stub surface. */
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
/* F-1 (P0) — STAB-01 persisted epochs are written in the wrong time base      */
/* ========================================================================== */
/* s_last_fresh_fix_s and now_timestamp both come from TIMER_IF_GetTime(),
 * which returns BOOT-RELATIVE seconds (GetTimerTicks() = UINT32_MAX - RTC->SSR,
 * zeroed at RTC init). STAB-01 then gates persist AND restore on
 * `>= 1700000000UL` ("real UTC, not boot-relative"). The gate therefore
 * rejects its own writes, 100% of the time:
 *
 *   - BKP_REG_LASTPOS_EPOCH is written with boot-relative seconds and never
 *     restored -> a restored position's true age is always unknown.
 *   - BKP_REG_GPS_LOSS_EPOCH is NEVER written at all (the write is itself
 *     behind `now_timestamp >= 1700000000UL`) -> the 6 h GPS-loss silence
 *     grace restarts on EVERY reset.
 *
 * A unit in a brownout/deadman/IWDG reset loop can therefore never reach RF
 * silence and keeps transmitting on days-old geography — the exact DDR-0015
 * violation #148 was written to close.
 *
 * Invariant the fix must establish: any value compared against the UTC epoch
 * threshold must be sourced from SysTime, not TIMER_IF_GetTime.
 */
static void test_f1_epoch_timebase(const char *app)
{
    printf("-- F-1 (P0): persisted fix/loss epochs must be UTC, not boot-relative\n");

    /* F-1a: the fresh-fix stamp must not be TIMER_IF_GetTime if it is
     * persisted behind a UTC plausibility gate. */
    int timer_if_stamps = count_occurrences(app, "s_last_fresh_fix_s = TIMER_IF_GetTime");
    printf("   s_last_fresh_fix_s = TIMER_IF_GetTime(...) sites: %d (want 0)\n",
           timer_if_stamps);
    CHECK_REGRESSION(timer_if_stamps == 0, "F-1a");

    /* F-1b: the GPS-loss grace epoch write must be reachable. Today the only
     * two BKUPWrite(BKP_REG_GPS_LOSS_EPOCH) sites sit behind
     * `now_timestamp >= 1700000000UL`, where now_timestamp is boot-relative. */
    bool loss_write_exists =
        strstr(app, "HAL_RTCEx_BKUPWrite(&hrtc, BKP_REG_GPS_LOSS_EPOCH") != NULL;
    bool gated_on_utc =
        strstr(app, "if (now_timestamp >= 1700000000UL)") != NULL;
    printf("   GPS-loss epoch write present: %s, gated on boot-relative >= UTC: %s\n",
           loss_write_exists ? "yes" : "no", gated_on_utc ? "yes (UNREACHABLE)" : "no");
    CHECK_REGRESSION(!(loss_write_exists && gated_on_utc), "F-1b");

    /* F-1c: the declared contract of s_last_fresh_fix_s and the plausibility
     * gate applied to it must agree on a time base. Today the declaration
     * comment says "RTC-second timestamp ... this boot" while the restore
     * gate demands a real UTC epoch. */
    bool declares_rtc_seconds =
        strstr(app, "#141: RTC-second timestamp of the last FRESH GNSS fix this boot") != NULL;
    bool restore_demands_utc =
        strstr(app, "if (fix_epoch >= 1700000000UL") != NULL;
    printf("   declares boot-relative: %s, restore demands UTC: %s\n",
           declares_rtc_seconds ? "yes" : "no", restore_demands_utc ? "yes" : "no");
    CHECK_REGRESSION(!(declares_rtc_seconds && restore_demands_utc), "F-1c");
}

/* ========================================================================== */
/* F-2 (P1) — F3's frozen-tick hazard survives in the GPS acquisition loop     */
/* ========================================================================== */
/* F3 (#169) fixed W25Q_WaitReady: a loop bounded by HAL_GetTick() (RTC-derived)
 * that refreshes the IWDG from inside hangs FOREVER if the RTC tick freezes —
 * timeout, watchdog and liveness all defeated by one fault. The GPS
 * acquisition loop in AcquireGnssFix has the identical structure:
 *
 *     while ((HAL_GetTick() - gps_start) < gps_timeout_ms) {
 *         ...
 *         HAL_IWDG_Refresh(&hiwdg);
 *         HAL_PWR_EnterSLEEPMode(...);
 *     }
 *
 * RTC_LivenessCheck() cannot rescue it: that runs in the main loop, and this
 * loop is inside a sequencer task.
 *
 * Invariant: any IWDG-refreshing loop must also carry a clock-INDEPENDENT
 * iteration bound (the W25Q_MAX_BUSY_POLLS_PER_MS pattern).
 */
static void test_f2_gps_loop_bound(const char *app, const char *w25q)
{
    printf("-- F-2 (P1): IWDG-refreshing GPS loop needs a clock-independent bound\n");

    bool w25q_has_bound = strstr(w25q, "max_polls") != NULL;
    printf("   W25Q_WaitReady has a poll bound (F3 precedent): %s\n",
           w25q_has_bound ? "yes" : "no");

    bool refreshes_iwdg = in_function(app, "static bool AcquireGnssFix(",
                                      "HAL_IWDG_Refresh");
    bool tick_bounded   = in_function(app, "static bool AcquireGnssFix(",
                                      "while ((HAL_GetTick() - gps_start) < gps_timeout_ms)");
    bool has_iter_bound = in_function(app, "static bool AcquireGnssFix(", "max_polls") ||
                          in_function(app, "static bool AcquireGnssFix(", "max_iters") ||
                          in_function(app, "static bool AcquireGnssFix(", "GNSS_MAX_ACQ_ITERS");
    printf("   AcquireGnssFix: refreshes IWDG=%s tick-bounded=%s iteration-bound=%s\n",
           refreshes_iwdg ? "yes" : "no", tick_bounded ? "yes" : "no",
           has_iter_bound ? "yes" : "NO");
    CHECK_REGRESSION(!(refreshes_iwdg && tick_bounded) || has_iter_bound, "F-2");
}

/* ========================================================================== */
/* F-3 (P2) — "stale position may INHIBIT but never SWITCH" is comment-only    */
/* ========================================================================== */
/* AcquireGnssFix returns TRUE on the GPS-timeout path too, after forging
 * hgnss.data from the last-known statics:
 *     hgnss.data.valid = true; fix_quality = GNSS_FIX_GPS; position_present = true;
 * SendTxData then calls SelectRegionAndSession(), which gates only on
 * GNSS_IsFixValid && GNSS_ValidateCoordinates — both now pass — and runs
 * MultiRegion_AutoSwitchToRegion(). The stale bit set by
 * EnvSensors_MarkGnssStale(true) is never consulted there.
 *
 * Invariant: the auto-switch site must consult freshness.
 */
static void test_f3_stale_autoswitch(const char *app)
{
    printf("-- F-3 (P2): region auto-switch must not run on a stale position\n");

    bool policy_documented =
        strstr(app, "may INHIBIT but never SWITCH") != NULL;
    bool switch_consults_freshness =
        in_function(app, "static void SelectRegionAndSession(", "stale") ||
        in_function(app, "static void SelectRegionAndSession(", "GnssIsStale") ||
        in_function(app, "static void SelectRegionAndSession(", "fresh");
    printf("   policy stated in comments: %s, enforced at the switch site: %s\n",
           policy_documented ? "yes" : "no",
           switch_consults_freshness ? "yes" : "NO");
    CHECK_REGRESSION(switch_consults_freshness, "F-3");
}

/* ========================================================================== */
/* F-4 (P2) — bulk bursts defeat the F8 mode hysteresis (BEHAVIOURAL)          */
/* ========================================================================== */
/* OnTxData re-arms SendTxData once per archive packet (up to 20), and
 * DecideTransmitPlan runs on every one of them. CalculateVoltageSlope is
 * protected by the FW-6 dt >= 600 s guard, so the slope is FROZEN across the
 * burst — the same mode is therefore proposed 20 times within a few seconds
 * and upgrade_streak reaches F8_UPGRADE_CONFIRM (3) on the third packet.
 * "Three consecutive work cycles" becomes "three seconds".
 *
 * Invariant: an upgrade must require confirmation across REAL elapsed time,
 * not merely across repeated calls.
 */
static void test_f4_hysteresis_burst(void)
{
    printf("-- F-4 (P2): mode upgrade must not confirm on same-timestamp calls\n");

    VoltageSlope_t vs;
    memset(&vs, 0, sizeof(vs));

    /* Establish history and commit a low-power mode: seed, then a discharging
     * sample 700 s later (dt > 600 so the slope is real and negative). */
    (void)DecideTransmitPlan(&vs, 5200, 25.0f, false, 1000, true, false, false);
    TransmitPlan_t p = DecideTransmitPlan(&vs, 5100, 25.0f, false, 1700, true, false, false);
    printf("   after discharge sample: committed mode=%s\n", GetModeName(p.power_mode));

    /* Now replay a bulk burst: the SAME timestamp, over and over. The slope is
     * frozen (dt < 600), so every call proposes whatever the frozen slope
     * implies — an upgrade must NOT be confirmable by repetition alone. */
    OperatingMode_t before = p.power_mode;
    OperatingMode_t after = before;
    for (int i = 0; i < 20; i++) {
        TransmitPlan_t b = DecideTransmitPlan(&vs, 5400, 25.0f, false, 1700, true, false, false);
        after = b.power_mode;
    }
    printf("   after 20 same-timestamp burst calls: %s -> %s (streak=%u)\n",
           GetModeName(before), GetModeName(after), vs.upgrade_streak);

    /* An upgrade is a numerically LOWER OperatingMode_t. */
    CHECK_REGRESSION(!(after < before), "F-4a");

    /* Directly: the confirmation counter must not advance without elapsed time. */
    VoltageSlope_t vs2;
    memset(&vs2, 0, sizeof(vs2));
    (void)DecideTransmitPlan(&vs2, 5200, 25.0f, false, 1000, true, false, false);
    (void)DecideTransmitPlan(&vs2, 5100, 25.0f, false, 1700, true, false, false);
    uint8_t streak_before = vs2.upgrade_streak;
    (void)DecideTransmitPlan(&vs2, 5400, 25.0f, false, 1700, true, false, false);
    (void)DecideTransmitPlan(&vs2, 5400, 25.0f, false, 1700, true, false, false);
    printf("   upgrade_streak across two zero-dt calls: %u -> %u\n",
           streak_before, vs2.upgrade_streak);
    CHECK_REGRESSION(vs2.upgrade_streak == streak_before, "F-4b");
}

/* ========================================================================== */
/* F-5 (P3) — sync_deferred latches when rf_silence interrupts a burst         */
/* ========================================================================== */
/* FlashLog_FlushHeaderSync is reachable only from the
 * (WAIT_PROBE_ACK || COMPLETE) branch at the top of RunTxStateMachine. If
 * rf_silence goes true while g_tx_state == TX_STATE_BULK_TRANSFER (restricted
 * region, GPS-loss silence, session loss), the state is parked to PROBE_SF10
 * without passing the flush and sync_deferred stays 1 indefinitely — commits
 * from then on live only in RAM.
 *
 * Invariant: the rf_silence park must flush too (or the flush must be
 * unconditional at cycle top).
 */
static void test_f5_deferred_sync_latch(const char *app)
{
    printf("-- F-5 (P3): rf_silence park must not strand a deferred header sync\n");

    int flush_sites = count_occurrences(app, "FlashLog_FlushHeaderSync(&hflashlog)");
    bool silence_parks =
        in_function(app, "static void RunTxStateMachine(", "if (rf_silence)");
    printf("   FlushHeaderSync call sites: %d, rf_silence parks the FSM: %s\n",
           flush_sites, silence_parks ? "yes" : "no");
    /* One flush site + a silence park that bypasses it == the latch. */
    CHECK_REGRESSION(!(flush_sites == 1 && silence_parks), "F-5");
}

/* ========================================================================== */
/* F-6 (P3) — boot-attempt evidence is cleared at cycle START                  */
/* ========================================================================== */
/* Deadman_MarkProgress() is the FIRST statement of SendTxData and calls
 * ResetCause_ClearBootAttempts(). Any deterministic fault INSIDE a work cycle
 * therefore clears the counter on every boot: FR-23's escape can never engage
 * and no evidence accumulates. The deadman wants marking at entry; the
 * boot-loop counter wants clearing at successful completion. They are
 * currently the same statement.
 */
static void test_f6_boot_attempts_placement(const char *app)
{
    printf("-- F-6 (P3): boot-attempt clear belongs at cycle completion\n");

    bool cleared_in_markprogress =
        in_function(app, "static void Deadman_MarkProgress(",
                    "ResetCause_ClearBootAttempts");
    bool marked_first =
        in_function(app, "static void SendTxData(void)\n{", "Deadman_MarkProgress();");
    printf("   cleared inside Deadman_MarkProgress: %s, marked at cycle start: %s\n",
           cleared_in_markprogress ? "yes" : "no", marked_first ? "yes" : "no");
    CHECK_REGRESSION(!(cleared_in_markprogress && marked_first), "F-6");
}

/* ========================================================================== */
/* F-7 (P3) — chunked-sleep ceiling is a literal, not derived                  */
/* ========================================================================== */
/* stm32_lpm_if.c:312 is `if (++chunks > 180)` while the comment claims the
 * bound "tracks IWDG_SAFE_SLEEP_SECONDS". It does not. 181 x 20 s = 3620 s,
 * but Config_Validate accepts tx_interval_survival up to 7 200 000 ms — a
 * config INSIDE its own validated range aborts chunked sleep every cycle and
 * spins the main loop at full power, in the mode that exists to save power.
 * This is #134 waiting to recur.
 *
 * Invariant: the ceiling must be derived from IWDG_SAFE_SLEEP_SECONDS and the
 * maximum permitted interval, not hardcoded.
 */
static void test_f7_chunk_ceiling_derived(const char *lpm, const char *cfg)
{
    printf("-- F-7 (P3): sleep-chunk ceiling must be derived, not a literal\n");

    bool literal_ceiling = strstr(lpm, "if (++chunks > 180)") != NULL;
    bool derived =
        strstr(lpm, "MAX_SLEEP_CHUNKS") != NULL ||
        strstr(lpm, "chunks > (") != NULL;
    bool cfg_allows_2h = strstr(cfg, "tx_interval_survival > 7200000") != NULL;
    printf("   literal 180: %s, derived symbol: %s, config ceiling 2h: %s\n",
           literal_ceiling ? "yes" : "no", derived ? "yes" : "no",
           cfg_allows_2h ? "yes" : "no");
    printf("   181 chunks x 20 s = %d s vs validated survival ceiling 7200 s\n",
           181 * 20);
    CHECK_REGRESSION(derived && !literal_ceiling, "F-7");
}


/* ========================================================================== */
/* F-8 (P2) — battery value and its staleness flag are different conversions  */
/* ========================================================================== */
/* SYS_GetBatteryVoltage() is NOT idempotent: each call runs a fresh ADC
 * conversion AND mutates s_batt_stale. Per work cycle it is called four times:
 *
 *   sys_sensors.c:276  conversion A -> sensor_data->battery_voltage
 *   sys_sensors.c:277               -> sensor_data->batt_stale  (describes A)
 *   sys_sensors.c:285  conversion B -> log only (overwrites s_batt_stale)
 *   lora_app.c:1699    conversion C -> battery_mv_raw
 *   lora_app.c:2104    conversion D -> the bulk-opportunity gate
 *
 * DecideTransmitPlan is then handed battery_mv_raw (C) paired with
 * sensor_data.batt_stale (A). If C is implausible and served from cache while
 * A was fine, batt_stale=false rides a cached voltage and RV-03's "a battery
 * nobody has measured this cycle is not entitled to the GPS-on modes" is
 * defeated. The archived record carries A, so the record does not even show
 * the voltage the power decision actually used.
 *
 * Invariant: one conversion per cycle, value and flag captured together.
 */
static void test_f8_battery_sample_coherence(const char *app, const char *sens)
{
    printf("-- F-8 (P2): power decision must use the sample its stale flag describes\n");

    int batt_reads = count_occurrences(sens, "SYS_GetBatteryVoltage()") +
                     count_occurrences(app,  "SYS_GetBatteryVoltage()");
    printf("   SYS_GetBatteryVoltage() call sites per cycle: %d (want 1)\n", batt_reads);
    CHECK_REGRESSION(batt_reads <= 1, "F-8a");

    bool plan_uses_own_read =
        in_function(app, "static void SendTxData(void)\n{",
                    "battery_mv_raw = SYS_GetBatteryVoltage()");
    bool plan_uses_foreign_flag =
        in_function(app, "static void SendTxData(void)\n{", "sensor_data.batt_stale");
    printf("   SendTxData takes its own reading: %s, passes sensor_data's flag: %s\n",
           plan_uses_own_read ? "yes" : "no", plan_uses_foreign_flag ? "yes" : "no");
    CHECK_REGRESSION(!(plan_uses_own_read && plan_uses_foreign_flag), "F-8b");
}

/* ========================================================================== */
/* F-9 (P3) — three ADC conversions per cycle exist only to feed dead logs     */
/* ========================================================================== */
/* sys_sensors.c:285-287 read battery, VDDA and solar solely to print them.
 * SONDE_LOG compiles to ((void)0) under SONDE_FLIGHT_BUILD, but the three
 * SYS_Get* CALLS are plain statements outside any #if, so the conversions
 * still run in flight. Same class as FR-16 (#97) and R2-15 (#119), which
 * gated the snprintf but not this. Cheap to fix, and it removes the
 * s_batt_stale overwrite that makes F-8 hard to reason about.
 */
static void test_f9_log_only_adc_reads(const char *sens)
{
    printf("-- F-9 (P3): log-only ADC conversions must be compiled out in flight\n");

    bool has_log_only_block = strstr(sens, "int batt_mv = SYS_GetBatteryVoltage();") != NULL;
    bool gated = strstr(sens, "#ifndef SONDE_FLIGHT_BUILD") != NULL;
    printf("   log-only ADC triple present: %s, file has a flight gate: %s\n",
           has_log_only_block ? "yes" : "no", gated ? "yes" : "no");
    CHECK_REGRESSION(!has_log_only_block || gated, "F-9");
}

/* ========================================================================== */
/* F-10 (P2) — GNSS HHMMSS time field is never range-checked                  */
/* ========================================================================== */
/* atgm336h.c GNSS_ParseRMC (:1075) and GNSS_ParseGGA (:983) store the UTC
 * time token on the sole condition strlen(token) >= 6, then atoi(). A
 * garbled-but-checksum-valid "995910" becomes timestamp=995910; upstream,
 * SysTimeSyncFromGnss turns that into hours=99 -> +356400 s discipline jump,
 * and SysTime is the clock every archive record AND the F-1 silence policy
 * read. RMC also stores time/date BEFORE consulting the status field, so a
 * 'V' (void) sentence still rewrites the clock inputs.
 *
 * Invariant: hh > 23 || mm > 59 || ss > 60 (60 = leap second) is rejected at
 * parse; a void RMC never stores time/date.
 */
static void test_f10_hhmmss_range_checked(void)
{
    printf("-- F-10 (P2): garbled HHMMSS must not discipline the clock inputs\n");

    GNSS_HandleTypeDef g;
    memset(&g, 0, sizeof(g));
    g.data.timestamp = 120000u;   /* known-good time already held */
    g.data.date      = 110826u;

    /* Garbled hours field, checksum-style sentence (parser called directly,
     * same as the R2-30 precedent in test_main.c). */
    GNSS_ParseRMC(&g, "$GNRMC,995910,A,4807.038,N,01131.000,E,0.5,180.0,060825,,,A*00");
    printf("   after '995910' RMC: timestamp=%lu (want 120000 untouched)\n",
           (unsigned long)g.data.timestamp);
    CHECK_REGRESSION(g.data.timestamp == 120000u, "F-10a");

    /* Void sentence must not store time/date at all. */
    g.data.timestamp = 120000u;
    g.data.date      = 110826u;
    GNSS_ParseRMC(&g, "$GNRMC,135900,V,,,,,,,070825,,,N*00");
    printf("   after void RMC: timestamp=%lu date=%lu (want both untouched)\n",
           (unsigned long)g.data.timestamp, (unsigned long)g.data.date);
    CHECK_REGRESSION(g.data.timestamp == 120000u && g.data.date == 110826u, "F-10b");

    /* Boundary: 23:59:60 (leap) must still be accepted; 24:00:00 rejected. */
    memset(&g, 0, sizeof(g));
    g.data.timestamp = 0xFFFFFFFFu;
    GNSS_ParseRMC(&g, "$GNRMC,235960,A,4807.038,N,01131.000,E,0.5,180.0,060825,,,A*00");
    CHECK_REGRESSION(g.data.timestamp == 235960u, "F-10c-leap-ok");
    g.data.timestamp = 0xFFFFFFFFu;
    GNSS_ParseRMC(&g, "$GNRMC,240000,A,4807.038,N,01131.000,E,0.5,180.0,060825,,,A*00");
    CHECK_REGRESSION(g.data.timestamp == 0xFFFFFFFFu, "F-10d-24h-reject");
}

/* ========================================================================== */
/* F-11 (P3) — hemisphere letter defaults to N/E and is applied by exclusion  */
/* ========================================================================== */
/* atgm336h.c:974 initialises lat_dir='N'/lon_dir='E' and only flips on an
 * exact 'S'/'W'. An empty direction token (truncated sentence whose '*'
 * landed after the numeric field, still checksum-valid) or any garbage
 * character yields a Northern/Eastern coordinate: a dropped 'W' puts a
 * Calgary launch (51N 114W) at +114E — Kazakhstan — and h3lite then picks a
 * completely wrong LoRaWAN region (see F-3).
 *
 * Invariant: a missing or non-{N,S}/{E,W} direction rejects the fix
 * (return -1), never defaults.
 */
static void test_f11_hemisphere_default(void)
{
    printf("-- F-11 (P3): missing/garbled hemisphere letter must reject the fix\n");

    GNSS_HandleTypeDef g;

    /* Dropped direction fields, coordinates otherwise valid. */
    memset(&g, 0, sizeof(g));
    int rc = GNSS_ParseGGA(&g, "$GNGGA,120000,4807.0380,,01131.0000,,1,08,0.9,545.4,M,46.9,M,,*00");
    printf("   empty N/S + E/W: rc=%d lat=%f lon=%f (want rc=-1)\n",
           rc, (double)g.data.latitude, (double)g.data.longitude);
    CHECK_REGRESSION(rc == -1, "F-11a");

    /* Garbage direction character. */
    memset(&g, 0, sizeof(g));
    rc = GNSS_ParseGGA(&g, "$GNGGA,120000,4807.0380,X,01131.0000,W,1,08,0.9,545.4,M,46.9,M,,*00");
    CHECK_REGRESSION(rc == -1, "F-11b");

    /* Guard: well-formed sentences still parse, both hemispheres. */
    memset(&g, 0, sizeof(g));
    rc = GNSS_ParseGGA(&g, "$GNGGA,120000,4807.0380,S,01131.0000,W,1,08,0.9,545.4,M,46.9,M,,*4C");
    CHECK_REGRESSION(rc == 0 && g.data.latitude < 0.0 && g.data.longitude < 0.0, "F-11c-guard");
    memset(&g, 0, sizeof(g));
    rc = GNSS_ParseGGA(&g, "$GNGGA,120000,4807.0380,N,01131.0000,E,1,08,0.9,545.4,M,46.9,M,,*00");
    CHECK_REGRESSION(rc == 0 && g.data.latitude > 0.0 && g.data.longitude > 0.0, "F-11d-guard");
}

/* ========================================================================== */
/* R5 (P1) — boot recovery must not anchor on US915 (ChatGPT review)           */
/* ========================================================================== */
/* lora_app.c LoRaWAN_Init gated session recovery on
 * MultiRegion_IsRegionJoined(LORAMAC_REGION_US915) and switched to US915
 * unconditionally - a valid non-US session bank fell through to RF silence.
 * Invariant: no hardcoded US915 switch target in lora_app.c.
 */
static void test_r5_boot_region_anchor(const char *app)
{
    printf("-- R5 (P1): boot session recovery must not be US915-anchored\n");

    bool hardcoded = strstr(app, "MultiRegion_SwitchToRegion(LORAMAC_REGION_US915)") != NULL;
    bool resume_logic = strstr(app, "MultiRegion_GetActiveRegion()") != NULL &&
                        strstr(app, "scan_regions") != NULL;
    printf("   hardcoded US915 switch: %s, resume-active-region logic: %s\n",
           hardcoded ? "yes (BAD)" : "no", resume_logic ? "yes" : "no");
    CHECK_REGRESSION(!hardcoded && resume_logic, "R5");
}

/* ========================================================================== */
/* R6 (P1) — provisioning must not enter flight (ChatGPT review)               */
/* ========================================================================== */
/* The PROVISIONING_BUILD (ABP table) path called MissionState_EnterFlight()
 * on success while the OTAA path deliberately holds COMMISSIONING (DDR-0002:
 * flight entry is explicit, never join/provisioning-triggered). Invariant:
 * lora_app.c never calls MissionState_EnterFlight.
 */
static void test_r6_provisioning_no_flight(const char *app)
{
    printf("-- R6 (P1): provisioning path must not enter flight\n");

    bool enters_flight = strstr(app, "MissionState_EnterFlight") != NULL;
    printf("   MissionState_EnterFlight in lora_app.c: %s\n", enters_flight ? "yes (BAD)" : "no");
    CHECK_REGRESSION(!enters_flight, "R6");
}

/* ========================================================================== */
/* R7 (P1) — every advertised region needs a complete DatarateFromSF mapping   */
/* ========================================================================== */
/* DatarateFromSF covered only US915/EU868/AS923/AU915; IN865/KR920 fell into
 * the default branch (LORAWAN_DEFAULT_DATA_RATE - not necessarily valid for
 * the active region, MAC rejects the uplink). Invariant: all six advertised
 * regions have an explicit table case.
 */
static void test_r7_region_completeness(const char *app)
{
    printf("-- R7 (P1): DatarateFromSF must cover all six advertised regions\n");

    const char *tables[] = { "DataratesUS915", "DataratesEU868", "DataratesAS923",
                             "DataratesAU915", "DataratesIN865", "DataratesKR920" };
    int covered = 0;
    for (int i = 0; i < 6; i++) {
        if (strstr(app, tables[i]) != NULL) covered++;
    }
    printf("   region datarate tables referenced: %d/6\n", covered);
    CHECK_REGRESSION(covered == 6, "R7");
}

/* ========================================================================== */
/* R9 (P1) — GNSS_PowerOn must be transactional (ChatGPT review)               */
/* ========================================================================== */
/* atgm336h.c GNSS_PowerOn asserts PWR/EN, disables STOP mode, then starts the
 * DMA. On HAL_UART_Receive_DMA failure it returned GNSS_ERROR with NO
 * rollback: module left powered (~25-30 mA), STOP left disabled, software
 * is_powered=false so nothing later cleans up. Invariant: any startup
 * failure returns ALL hardware to OFF (pins LOW, STOP re-enabled, sw OFF).
 */
static void test_r9_poweron_transaction(void)
{
    printf("-- R9 (P1): failed GNSS power-on must roll back to OFF\n");

    GNSS_HandleTypeDef g;
    memset(&g, 0, sizeof(g));
    UART_HandleTypeDef h = {0};
    GPIO_TypeDef port = {0};
    g.is_initialized = true;
    g.huart = &h;
    g.pwr_port = &port; g.pwr_pin = (1u << 10);
    g.en_port  = &port; g.en_pin  = (1u << 5);

    g_host_uart_dma_rc = HAL_ERROR;
    g_host_gpio_log_n = 0;
    g_host_lpm_stop_state = -1;

    GNSS_StatusTypeDef rc = GNSS_PowerOn(&g);
    printf("   DMA-start failure: rc=%d, is_powered=%d, lpm_state=%d\n",
           rc, g.is_powered, g_host_lpm_stop_state);
    CHECK_REGRESSION(rc == GNSS_ERROR, "R9-rc");
    CHECK_REGRESSION(!g.is_powered, "R9-sw-off");
    CHECK_REGRESSION(g_host_lpm_stop_state == UTIL_LPM_ENABLE, "R9-lpm");

    /* The LAST write to each power pin must be RESET (driven back off). */
    int pwr_final = -1, en_final = -1;
    for (int i = g_host_gpio_log_n - 1; i >= 0; i--) {
        if (pwr_final < 0 && g_host_gpio_log[i].pin == g.pwr_pin) pwr_final = g_host_gpio_log[i].state;
        if (en_final  < 0 && g_host_gpio_log[i].pin == g.en_pin)  en_final  = g_host_gpio_log[i].state;
    }
    printf("   final pin states: pwr=%d en=%d (want both RESET=0)\n", pwr_final, en_final);
    CHECK_REGRESSION(pwr_final == GPIO_PIN_RESET && en_final == GPIO_PIN_RESET, "R9-pins-off");

    g_host_uart_dma_rc = HAL_OK;
}

/* ========================================================================== */
/* R10 (P1) — a failed ADC stage must terminate the transaction (ChatGPT)      */
/* ========================================================================== */
/* adc_if.c ADC_ReadChannels: HAL_ADC_ConfigChannel and HAL_ADC_Start failures
 * called Error_Handler() and CONTINUED the transaction (config failure can
 * convert the previous channel's config -> plausible wrong-channel value
 * treated as fresh). Calibration (#136) and the bounded poll (F-014/#31) are
 * already safe. Invariant: config/start failure returns 0 (read-failure),
 * never falls through.
 */
static void test_r10_adc_transaction_abort(const char *adc)
{
    printf("-- R10 (P1): ADC config/start failure must abort, not continue\n");

    bool config_falls_through =
        strstr(adc, "if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)\n  {\n    Error_Handler();\n  }") != NULL;
    bool start_falls_through =
        strstr(adc, "if (HAL_ADC_Start(&hadc) != HAL_OK)\n  {\n    /* Start Error */\n    Error_Handler();\n  }") != NULL;
    printf("   config-failure fall-through: %s, start-failure fall-through: %s\n",
           config_falls_through ? "yes (BAD)" : "no",
           start_falls_through ? "yes (BAD)" : "no");
    CHECK_REGRESSION(!config_falls_through && !start_falls_through, "R10");
}

/* ========================================================================== */
/* R12 (P2) — config authority: frame_counter_save_interval must be consumed   */
/* ========================================================================== */
/* The config field existed but multiregion_context.c used the
 * FRAME_COUNTER_SAVE_INTERVAL macro - an operator edit changed nothing.
 * Invariant: the consumer reads the config field via the Config_Get
 * accessor (CfgFrameCounterSaveInterval), and Config_Validate range-checks
 * the now-live knob.
 */
static void test_r12_config_authority(const char *mreg, const char *cfg)
{
    printf("-- R12 (P2): frame_counter_save_interval must be config-authoritative\n");

    bool accessor = strstr(mreg, "CfgFrameCounterSaveInterval") != NULL &&
                    strstr(mreg, "Config_Get()") != NULL;
    bool validated = strstr(cfg, "frame_counter_save_interval < 1") != NULL;
    printf("   consumer accessor wired: %s, Config_Validate range check: %s\n",
           accessor ? "yes" : "no", validated ? "yes" : "no");
    CHECK_REGRESSION(accessor && validated, "R12");
}

int main(void)
{
    printf("=== 2026-08-11 (second pass) stability review regressions ===\n\n");

    char *app_raw = slurp("../../LoRaWAN/App/lora_app.c");
    char *app = strip_comments(app_raw);
    char *w25q = slurp("../../Core/Src/w25q16jv.c");
    char *lpm = slurp("../../Core/Src/stm32_lpm_if.c");
    char *cfg = slurp("../../Core/Src/config.c");
    char *sens = slurp("../../Core/Src/sys_sensors.c");
    char *adc = slurp("../../Core/Src/adc_if.c");
    char *mreg = slurp("../../Core/Src/multiregion_context.c");

    /* F-1c intentionally scans the UNSTRIPPED text: it asserts that the
     * declaration comment and the runtime gate agree on a time base. */
    test_f1_epoch_timebase(app);
    {
        bool declares_rtc_seconds =
            strstr(app_raw, "#141: RTC-second timestamp of the last FRESH GNSS fix this boot") != NULL;
        bool restore_demands_utc = strstr(app, "if (fix_epoch >= 1700000000UL") != NULL;
        printf("   [F-1c on raw text] declares boot-relative=%s gate demands UTC=%s\n",
               declares_rtc_seconds ? "yes" : "no", restore_demands_utc ? "yes" : "no");
        CHECK_REGRESSION(!(declares_rtc_seconds && restore_demands_utc), "F-1c-raw");
    }
    printf("\n");
    test_f2_gps_loop_bound(app, w25q);
    printf("\n");
    test_f3_stale_autoswitch(app);
    printf("\n");
    test_f4_hysteresis_burst();
    printf("\n");
    test_f5_deferred_sync_latch(app);
    printf("\n");
    test_f6_boot_attempts_placement(app);
    printf("\n");
    test_f7_chunk_ceiling_derived(lpm, cfg);
    printf("\n");
    test_f8_battery_sample_coherence(app, sens);
    printf("\n");
    test_f9_log_only_adc_reads(sens);
    printf("\n");
    test_f10_hhmmss_range_checked();
    printf("\n");
    test_f11_hemisphere_default();
    printf("\n");
    test_r5_boot_region_anchor(app);
    printf("\n");
    test_r6_provisioning_no_flight(app);
    printf("\n");
    test_r7_region_completeness(app);
    printf("\n");
    test_r9_poweron_transaction();
    printf("\n");
    test_r10_adc_transaction_abort(adc);
    printf("\n");
    test_r12_config_authority(mreg, cfg);

    printf("\n%d checks, %d failures (%d expected pre-fix)\n",
           g_checks, g_failures, g_expected_failures);

    if (getenv("EXPECT_UNFIXED") && g_failures == g_expected_failures) {
        printf("BASELINE OK (all failures are known-unfixed findings)\n");
        return 0;
    }
    return g_failures ? 1 : 0;
}
