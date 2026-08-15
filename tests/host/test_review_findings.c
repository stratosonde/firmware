/**
  ******************************************************************************
  * @file    test_review_findings.c
  * @brief   Source-scan regression tests for the 2026-08-10 flight-readiness
  *          review findings that live in HAL-bound code
  ******************************************************************************
  * Findings #1, #2, #3, #5 and the gating half of #6 live in stm32_lpm_if.c,
  * lora_app.c, usart_if.c and adc_if.c — none of which is host-compilable
  * without stubbing the whole HAL/LoRaMac surface. Following the R2-13
  * "grep-proof" precedent (see R2_TEST_MAP.md), these tests read the firmware
  * sources as TEXT and assert on the invariant the fix must establish.
  *
  * What a source scan proves: the buggy pattern is present today (red) and
  * cannot return without turning the suite red again. What it does NOT prove:
  * runtime/electrical behaviour — that stays bench-verified per the finding.
  *
  * Every check is EXPECTED-FAIL-BEFORE-FIX (CHECK_REGRESSION). Run:
  *   make -C tests/host findings    (red until fixes land)
  *   make -C tests/host baseline    (EXPECT_UNFIXED=1: green pre-fix CI gate)
  ******************************************************************************
  */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

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

#define CHECK_REGRESSION(cond, fr_id) do { \
    g_checks++; \
    if (!(cond)) { \
        g_failures++; \
        g_expected_failures++; \
        printf("FAIL [%s] %s:%d: %s\n", fr_id, __FILE__, __LINE__, #cond); \
    } \
} while (0)

/* Slurp a whole file; exits hard if missing (a moved file must be loud). */
static char *slurp(const char *path)
{
    FILE *f = fopen(path, "rb");
    if (!f) { printf("FATAL: cannot open %s\n", path); exit(2); }
    fseek(f, 0, SEEK_END);
    long n = ftell(f);
    fseek(f, 0, SEEK_SET);
    char *buf = (char *)malloc((size_t)n + 1);
    if (!buf) { exit(2); }
    if (fread(buf, 1, (size_t)n, f) != (size_t)n) { exit(2); }
    buf[n] = '\0';
    fclose(f);
    return buf;
}

/* Parse the integer following "#define <name>" — skips comment mentions. */
static int parse_define(const char *src, const char *name, int *out)
{
    const char *p = src;
    size_t nl = strlen(name);
    while ((p = strstr(p, "#define")) != NULL) {
        const char *q = p + 7;
        while (*q == ' ' || *q == '\t') q++;
        if (strncmp(q, name, nl) == 0 &&
            (q[nl] == ' ' || q[nl] == '\t')) {
            q += nl;
            while (*q == ' ' || *q == '\t') q++;
            *out = atoi(q);
            return 1;
        }
        p += 7;
    }
    return 0;
}

/* ========================================================================== */
/* Finding #3 — sleep chunk bound not rescaled after 25 s -> 20 s chunk        */
/* ========================================================================== */
/* stm32_lpm_if.c:84 shortened IWDG_SAFE_SLEEP_SECONDS to 20 (R2-09), but the
 * FW-4 chunk ceiling at :309 stayed `chunks > 150`. 150 x 20 s = 3000 s, so
 * SURVIVAL (3600 s) always hits the ceiling ~10 min early and pays a full
 * PWR_ExitStopMode()/re-enter cycle every hour — in the mode that exists to
 * save power. Fix: 180 chunks (180 x 20 s = 3600 s). */
/* F-7 (#182) update: the bound is now DERIVED - MAX_SLEEP_CHUNKS =
 * (MAX_TX_INTERVAL_MS/1000/IWDG_SAFE_SLEEP_SECONDS)+1, where MAX_TX_INTERVAL_MS
 * mirrors the Config_Validate tx_interval_survival ceiling (7200000 ms). The
 * invariant strengthens from "covers 3600 s SURVIVAL" to "covers the maximum
 * validated interval". */
static void test_chunk_bound_covers_survival_interval(void)
{
    printf("-- finding #3: IWDG chunk bound must cover the max validated interval\n");

    char *src = slurp("../../Core/Src/stm32_lpm_if.c");

    int seconds = 0;
    CHECK(parse_define(src, "IWDG_SAFE_SLEEP_SECONDS", &seconds));

    /* DR-04 (#240): the ceiling must be ONE constant Config_Validate actually
     * enforces - defined in config.h, referenced by stm32_lpm_if.c, checked
     * by config.c. (Was: a local 7200000 in lpm_if mirroring a ceiling S-04
     * had removed - two constants in one file, both wrong together.) */
    char *cfgh = slurp("../../Core/Inc/config.h");
    int max_interval_ms = 0;
    CHECK(parse_define(cfgh, "CONFIG_MAX_TX_INTERVAL_MS", &max_interval_ms));
    CHECK_REGRESSION(strstr(src, "CONFIG_MAX_TX_INTERVAL_MS") != NULL, "DR-04-lpm-uses-shared");
    free(cfgh);
    char *cfgc = slurp("../../Core/Src/config.c");
    CHECK_REGRESSION(strstr(cfgc, "tx_interval_survival > CONFIG_MAX_TX_INTERVAL_MS") != NULL,
                     "DR-04-validator-enforces");
    free(cfgc);

    /* derived bound: (max_s / seconds) + 1 chunks */
    long bound = (max_interval_ms / 1000L) / seconds + 1L;

    printf("   IWDG_SAFE_SLEEP_SECONDS=%d, MAX_TX_INTERVAL_MS=%d -> bound=%ld -> max sleep %ld s (need >= %d)\n",
           seconds, max_interval_ms, bound, bound * seconds, max_interval_ms / 1000);
    CHECK_REGRESSION(bound * seconds >= max_interval_ms / 1000L, "FINDING-3");

    free(src);
}

/* ========================================================================== */
/* Finding #2 — RxParams.LinkCheck is sticky; the burst gate goes inert        */
/* ========================================================================== */
/* LmHandler.c:1167 sets RxParams.LinkCheck = true on MLME_LINK_CHECK and
 * nothing ever clears it, so after the first LinkCheckAns every bulk burst
 * evaluates a STALE margin/gateway count (lora_app.c:739). Fix per the review:
 * the application consumes the flag — clears params->LinkCheck right after
 * reading it in OnRxData. Scan: lora_app.c must clear it. */
static void test_linkcheck_consumed_after_read(void)
{
    printf("-- finding #2: OnRxData must clear params->LinkCheck after reading it\n");

    char *src = slurp("../../LoRaWAN/App/lora_app.c");
    CHECK(strstr(src, "params->LinkCheck") != NULL);   /* anchor: gate still reads it */
    CHECK_REGRESSION(strstr(src, "params->LinkCheck = false") != NULL, "FINDING-2");
    free(src);
}


/* ========================================================================== */
/* Finding #5 — vcom_Resume() defeats the GPS power-off guard                  */
/* ========================================================================== */
/* PWR_ExitStopMode() (stm32_lpm_if.c:393-402) re-inits UART1 only when
 * hgnss.is_powered — then :420 calls vcom_Resume(), which unconditionally
 * HAL_UART_Init(&huart1)s, restoring PB6 AF push-pull idle-HIGH into the
 * unpowered GNSS RX pin. The guard is dead code. Fix: vcom_Resume() is a
 * no-op (its siblings vcom_Trace/vcom_Trace_DMA already are, FW-13) or is
 * gated on is_powered. Scan: the vcom_Resume body must not contain an
 * unconditional HAL_UART_Init. */
static void test_vcom_resume_gated(void)
{
    printf("-- finding #5: vcom_Resume must not unconditionally init huart1\n");

    char *src = slurp("../../Core/Src/usart_if.c");
    const char *fn = strstr(src, "void vcom_Resume(void)");
    CHECK(fn != NULL);
    if (fn) {
        /* Body ends at the closing USER CODE marker of the function. */
        const char *end = strstr(fn, "vcom_Resume_2");
        CHECK(end != NULL);
        if (end) {
            size_t body_len = (size_t)(end - fn);
            const char *init = strstr(fn, "HAL_UART_Init");
            const char *gate = strstr(fn, "is_powered");
            int init_in_body = (init != NULL) && ((size_t)(init - fn) < body_len);
            int gate_in_body = (gate != NULL) && ((size_t)(gate - fn) < body_len);
            CHECK_REGRESSION(!init_in_body || gate_in_body, "FINDING-5");
        }
    }
    free(src);
}

/* ========================================================================== */
/* Finding #1 — LoRaWAN MAC NVM store task is registered but never set         */
/* ========================================================================== */
/* lora_app.c:447 registers StoreContext under CFG_SEQ_Task_LoRaStoreContextEvent;
 * no UTIL_SEQ_SetTask caller exists anywhere, so the whole two-slot MAC NVM
 * store is dead code and in-flight MAC state is lost every reset. Fix: wire
 * the trigger (or delete the subsystem). Scan: a SetTask call naming the task
 * must exist. */
static void test_nvm_store_task_is_set(void)
{
    printf("-- finding #1: CFG_SEQ_Task_LoRaStoreContextEvent must have a setter\n");

    char *src = slurp("../../LoRaWAN/App/lora_app.c");
    CHECK(strstr(src, "CFG_SEQ_Task_LoRaStoreContextEvent") != NULL);  /* anchor */

    /* Find a SetTask that references the task (tolerant of one line break
     * between the macro and the task name). */
    int wired = 0;
    const char *p = src;
    while ((p = strstr(p, "UTIL_SEQ_SetTask")) != NULL) {
        const char *nl = strchr(p, '\n');
        const char *next = nl ? nl + 1 : p + strlen(p);
        const char *nl2 = strchr(next, '\n');
        size_t span = (size_t)((nl2 ? nl2 : next + strlen(next)) - p);
        const char *ref = strstr(p, "LoRaStoreContextEvent");
        if (ref && (size_t)(ref - p) < span) { wired = 1; break; }
        p += 16;
    }
    CHECK_REGRESSION(wired, "FINDING-1");
    free(src);
}

/* ========================================================================== */
/* Finding #6 (gate half) — battery ADC needs the R28 plausibility treatment   */
/* ========================================================================== */
/* SYS_GetBatteryVoltage() returns 0 mV on poll timeout and feeds the power
 * state machine ungated: one bad sample latches SURVIVAL (raw < 4300 floor)
 * and poisons the slope baseline for up to 2 h. Temperature/humidity/pressure/
 * GNSS all have the R28 gate+cache+stale-bit; the battery path must too.
 * (The runtime half of this finding — the slope overflow it enables — is a
 * compiled CHECK_REGRESSION in test_main.c.) Scan: a battery staleness flag
 * must exist on the ADC/sensor path. */
static void test_battery_reading_is_gated(void)
{
    printf("-- finding #6: battery path must carry a plausibility gate / stale flag\n");

    char *adc = slurp("../../Core/Src/adc_if.c");
    char *sns = slurp("../../Core/Src/sys_sensors.c");
    int gated = (strstr(adc, "batt_stale") != NULL) ||
                (strstr(sns, "batt_stale") != NULL);
    CHECK_REGRESSION(gated, "FINDING-6");
    free(adc);
    free(sns);
}

/* ========================================================================== */
/* Finding #7 — MISSION_FLOAT must latch, and must have a cadence consumer     */
/* ========================================================================== */
/* The old detector (|dP|/P < 2% for 3 consecutive samples) demanded < 0.14 hPa
 * stability at float altitude — below MS5607 noise, so FLOAT could never
 * latch; and MissionState_GetStatusBits() had exactly one consumer (telemetry
 * bits), so DDR-0002's adaptive cadence was documented but not implemented.
 * Fix: windowed-range detection (wide band + sustained window + altitude
 * guard, works at any cadence including the 10 s ascent interval) plus the
 * mission cadence override in SendTxData. Scans: the window constants exist
 * and the cadence consumer is wired. */
static void test_float_detection_and_cadence(void)
{
    printf("-- finding #7: FLOAT detection must be windowed and consumed\n");

    char *msh = slurp("../../Core/Inc/mission_state.h");
    CHECK_REGRESSION(strstr(msh, "MISSION_FLOAT_WINDOW_S") != NULL, "FINDING-7");
    free(msh);

    char *app = slurp("../../LoRaWAN/App/lora_app.c");
    CHECK_REGRESSION(strstr(app, "MISSION_ASCENT_TX_INTERVAL_MS") != NULL, "FINDING-7");
    free(app);
}

/* ========================================================================== */
/* BURST-03 (#141) — geofence must run on the GPS-skip path; GPS-loss silence  */
/* ========================================================================== */
/* SelectRegionAndSession (the only rf_silence writer for REGION_RESTRICTED)
 * used to run only on the GPS-acquisition path — every GPS-off cycle (most of
 * a long float) transmitted blind. Maintainer decision 2026-08-11: evaluate
 * the geofence on the backup-register last-known position when GPS is skipped
 * (inhibit only, no auto-switch on stale position), and if no fresh fix for
 * GPS_LOSS_SILENCE_S, stop transmitting but keep logging and keep retrying
 * GPS. Scans: the skip-path geofence helper and the silence knob exist. */
static void test_geofence_runs_when_gps_skipped(void)
{
    printf("-- BURST-03/#141: geofence on GPS-skip path + GPS-loss silence\n");

    char *app = slurp("../../LoRaWAN/App/lora_app.c");
    CHECK_REGRESSION(strstr(app, "GeofenceRestricted") != NULL, "BURST-03");
    CHECK_REGRESSION(strstr(app, "GPS_LOSS_SILENCE_S") != NULL, "BURST-03");
    free(app);

    char *hdr = slurp("../../LoRaWAN/App/lora_app.h");
    CHECK_REGRESSION(strstr(hdr, "GPS_LOSS_SILENCE_S") != NULL, "BURST-03");
    free(hdr);
}

/* ========================================================================== */
/* DDR-0015 BR-STALE-017 — the stale-position RF budget is 24 h, and single     */
/* ========================================================================== */
/* 2026-08-13 intent interview: the GNSS-outage RF silence is a REGULATORY bound
 * owned by DDR-0015, not the energy bound DDR-0016 INV-PWR-009 originally
 * framed it as. It was retimed 6 h -> 24 h, and BR-STALE-020 forbids a SECOND
 * independent time-based cutoff (notably an RTC-sync-age timer).
 *
 * This is the guard that stops the value silently drifting back: nothing else in
 * the tree asserted it numerically before this test existed. Scans, not runtime,
 * because the macro is compile-time and lora_app.c is not host-linkable. */
static void test_stale_position_budget_is_24h(void)
{
    printf("-- DDR-0015/BR-STALE-017: stale-position RF budget is 24 h\n");

    char *hdr = slurp("../../LoRaWAN/App/lora_app.h");

    /* The binding itself. Accept either spelling of the 24 h product value so a
     * deliberate refactor is not a false failure, but a change of VALUE is. */
    int is_24h = (strstr(hdr, "GPS_LOSS_SILENCE_S  (24U * 3600U)") != NULL) ||
                 (strstr(hdr, "GPS_LOSS_SILENCE_S (24U * 3600U)")  != NULL) ||
                 (strstr(hdr, "GPS_LOSS_SILENCE_S  (86400U)")      != NULL);
    CHECK(is_24h);

    /* The retired 6 h value must not come back. */
    CHECK(strstr(hdr, "GPS_LOSS_SILENCE_S  (6U * 3600U)") == NULL);
    CHECK(strstr(hdr, "GPS_LOSS_SILENCE_S (6U * 3600U)")  == NULL);

    /* The DDR citation must travel with the number, so the next reader learns
     * WHO owns it (DDR-0015) rather than re-deriving it from DDR-0016. */
    CHECK(strstr(hdr, "BR-STALE-017") != NULL);
    free(hdr);

    /* Boundary convention (DDR-0015 P-STALE-014): age <= 24 h permitted,
     * > 24 h silent. A `>=` here would silence the mission one wake early. */
    char *app = slurp("../../LoRaWAN/App/lora_app.c");
    CHECK(strstr(app, "> GPS_LOSS_SILENCE_S") != NULL);
    CHECK(strstr(app, ">= GPS_LOSS_SILENCE_S") == NULL);

    /* BR-STALE-019: one quality-valid fix clears the silence on that same wake.
     * The fresh-fix epoch must still participate in the age reference. */
    CHECK(strstr(app, "s_last_fresh_fix_s") != NULL);
    free(app);
}

/* ========================================================================== */
/* MISSION-01 (#142) — launch detection, no float altitude guard, quiet watch  */
/* ========================================================================== */
/* Maintainer decisions 2026-08-11: flight entry is explicit (arming hook or
 * BR-LIFE-007 pressure departure), NOT join-triggered; the FLOAT latch has no
 * altitude ceiling (float is 5-25 km, payload-dependent); a commissioned unit
 * holds a quiet watch until launch. Scans pin all three. */
static void test_mission_entry_and_float_guard(void)
{
    printf("-- MISSION-01/#142: launch detection, no altitude guard, no join-trigger\n");

    char *msh = slurp("../../Core/Inc/mission_state.h");
    CHECK_REGRESSION(strstr(msh, "MISSION_LAUNCH_DP_HPA") != NULL, "MISSION-01");
    CHECK_REGRESSION(strstr(msh, "MISSION_FLOAT_MAX_PRESSURE_HPA") == NULL, "MISSION-01");
    free(msh);

    char *mr = slurp("../../Core/Src/multiregion_context.c");
    CHECK_REGRESSION(strstr(mr, "MissionState_EnterFlight") == NULL, "MISSION-01");
    free(mr);
}

/* ========================================================================== */
/* R2-13 (#117) — no UART IT-receive path on huart1 may remain                 */
/* ========================================================================== */
/* vcom_ReceiveInit called HAL_UART_Receive_IT(&huart1) on the GPS UART:
 * dormant only because UTIL_ADV_TRACE_StartRxProcess has no callers. If ever
 * invoked, RxState goes BUSY_RX permanently and every GNSS HAL_UART_Receive_DMA
 * returns HAL_BUSY — the GPS never receives another byte. Same trap: the
 * trailing HAL_UART_Receive_IT rearm in HAL_UART_RxCpltCallback fired on every
 * DMA full-complete. Fix: stub vcom_ReceiveInit (FW-13 pattern), drop the
 * rearm. GNSS uses Receive_DMA (atgm336h.c), so NO Receive_IT may remain here. */
static void test_no_uart_it_receive_on_gps_uart(void)
{
    printf("-- R2-13/#117: no HAL_UART_Receive_IT anywhere in usart_if.c\n");

    char *src = slurp("../../Core/Src/usart_if.c");
    CHECK(strstr(src, "vcom_ReceiveInit") != NULL);   /* anchor */
    CHECK_REGRESSION(strstr(src, "HAL_UART_Receive_IT") == NULL, "R2-13");
    free(src);
}

/* ========================================================================== */
/* STAB-02 (#149) — fatal degrade escape scoped to degradable faults only      */
/* ========================================================================== */
/* Error_Handler_Fatal returns after 5 boot attempts for ANY code >= 16, but
 * the only real call sites (CLOCK_CONFIG x2, PAYLOAD_FORMAT) have NO designed
 * degraded continuation - execution falls into mission code with a broken
 * clock tree or an unvalidated wire format. The escape must be conditioned on
 * an explicit per-code degradable predicate. */
static void test_fatal_escape_scoped(void)
{
    printf("-- STAB-02/#149: Error_Handler_Fatal degrade escape is per-code gated\n");

    char *src = slurp("../../Core/Src/main.c");
    CHECK(strstr(src, "Error_Handler_Fatal") != NULL);          /* anchor */
    CHECK_REGRESSION(strstr(src, "FatalIsDegradable") != NULL, "STAB-02");
    /* the escape return must be conditioned on the predicate */
    const char *esc = strstr(src, "ResetCause_GetBootAttempts() >= 5U");
    CHECK_REGRESSION(esc != NULL, "STAB-02-esc");
    if (esc) {
        /* the predicate must appear within the escape condition (~200 chars) */
        char window[240];
        memset(window, 0, sizeof(window));
        strncpy(window, esc, 200);
        CHECK_REGRESSION(strstr(window, "FatalIsDegradable") != NULL, "STAB-02-gate");
    }
    /* the predicate body must NOT list the non-degradable codes */
    const char *pred = strstr(src, "FatalIsDegradable(uint16_t code)");
    CHECK_REGRESSION(pred != NULL, "STAB-02-pred");
    if (pred) {
        char body[400];
        memset(body, 0, sizeof(body));
        strncpy(body, pred, 360);
        CHECK(strstr(body, "FAULT_CODE_FLASH_INIT") != NULL);       /* the designed case */
        CHECK_REGRESSION(strstr(body, "FAULT_CODE_CLOCK_CONFIG") == NULL, "STAB-02-clock");
        CHECK_REGRESSION(strstr(body, "FAULT_CODE_PAYLOAD_FORMAT") == NULL, "STAB-02-payload");
    }
    free(src);
}

/* ========================================================================== */
/* STAB-03 (#150) — GPS-loss silence must not force GNSS over the hard floor   */
/* ========================================================================== */
/* GPS-loss recovery may force GNSS only after the first-flight admission gate
 * has accepted the raw electrical floor. Mode names are cadence preferences;
 * they are no longer a substitute for per-wake load admission. */
static void test_silence_force_respects_floor(void)
{
    printf("-- STAB-03/#150: GPS-loss retry follows hard-floor admission\n");

    char *src = slurp("../../LoRaWAN/App/lora_app.c");
    char *cfg = slurp("../../Core/Src/config.c");
    const char *force = strstr(src, "gps_enabled_by_power_mgmt = true;");
    const char *admission = strstr(src,
        "FirstFlightWakeAdmitted(&sensor_data, battery_mv_raw)");
    CHECK(force != NULL);   /* anchor: the silence-path force */
    CHECK(admission != NULL);
    CHECK_REGRESSION(admission != NULL && force != NULL && admission < force,
                     "STAB-03-order");
    CHECK_REGRESSION(strstr(cfg,
        "configured < CONFIG_FIRST_FLIGHT_BATTERY_FLOOR_MV") != NULL,
        "STAB-03-floor");
    free(cfg);
    free(src);
}

/* ========================================================================== */
/* STAB-09 (#156) — CI must run the full host regression gate                  */
/* ========================================================================== */
static void test_ci_runs_all_suites(void)
{
    printf("-- STAB-09/#156: ci.yml runs make -C tests/host all\n");

    char *src = slurp("../../.github/workflows/ci.yml");
    CHECK(strstr(src, "tests/host") != NULL);   /* anchor */
    CHECK_REGRESSION(strstr(src, "make -C tests/host all") != NULL, "STAB-09");
    free(src);
}

/* ========================================================================== */
/* STAB-12 (#159) — timestamp-wrap latch persisted + restored (wiring)         */
/* ========================================================================== */
/* s_ts_wrapped was RAM-only: a reset after the 45.5-day wrap made the next
 * packet's timestamp look like an EARLIER epoch with no indication. The HAL
 * side must restore the latch at boot and persist it on first detection. */
static void test_ts_wrap_persistence_wiring(void)
{
    printf("-- STAB-12/#159: ts-wrap latch persisted via BKP_REG_TS_WRAP\n");

    char *src = slurp("../../LoRaWAN/App/lora_app.c");
    CHECK(strstr(src, "EncodeCompactBinaryPacket") != NULL);   /* anchor */
    /* restore path (read + setter) and persist path (getter + write) */
    CHECK_REGRESSION(strstr(src, "BKP_REG_TS_WRAP") != NULL, "STAB-12");
    CHECK_REGRESSION(strstr(src, "Payload_SetTimestampWrapped") != NULL, "STAB-12-restore");
    CHECK_REGRESSION(strstr(src, "Payload_IsTimestampWrapped") != NULL, "STAB-12-persist");
    free(src);

    char *regs = slurp("../../Core/Inc/backup_regs.h");
    CHECK_REGRESSION(strstr(regs, "BKP_REG_TS_WRAP") != NULL, "STAB-12-reg");
    free(regs);
}

/* ========================================================================== */
/* STAB-01 (#148) — stale-position RF authority must survive reset with age    */
/* ========================================================================== */
/* The position persisted in DR8-DR11 had no acquisition time: every reset
 * restarted the 6 h GPS-loss grace with a stale position RF-authoritative.
 * (The grace window is 24 h since 2026-08-13 — DDR-0015 BR-STALE-017. The
 *  persistence requirement this test guards is unchanged and now matters more,
 *  since a reset must not restart a 24 h regulatory budget either.)
 * The fresh-fix epoch and the GPS-loss epoch must persist (plausibility-gated:
 * no invented freshness) and seed the RAM state at boot. */
static void test_position_age_persistence_wiring(void)
{
    printf("-- STAB-01/#148: fix/loss epochs persisted with the position\n");

    char *regs = slurp("../../Core/Inc/backup_regs.h");
    CHECK_REGRESSION(strstr(regs, "BKP_REG_LASTPOS_EPOCH") != NULL, "STAB-01-regA");
    CHECK_REGRESSION(strstr(regs, "BKP_REG_GPS_LOSS_EPOCH") != NULL, "STAB-01-regB");
    free(regs);

    char *src = slurp("../../LoRaWAN/App/lora_app.c");
    CHECK(strstr(src, "LastPos_Store") != NULL);   /* anchor */
    /* store writes the epoch; restore seeds s_last_fresh_fix_s from it;
     * the loss epoch is persisted and restored; plausibility gate present */
    CHECK_REGRESSION(strstr(src, "BKP_REG_LASTPOS_EPOCH") != NULL, "STAB-01-store");
    CHECK_REGRESSION(strstr(src, "BKP_REG_GPS_LOSS_EPOCH") != NULL, "STAB-01-loss");
    CHECK_REGRESSION(strstr(src, "1700000000UL") != NULL, "STAB-01-plausibility");
    free(src);
}

/* ========================================================================== */
/* STAB-10 (#157) — protocol docs must not drift from the wire format again    */
/* ========================================================================== */
/* The audit found the docs describing PRE_FLIGHT/FLIGHT/RECOVERY mission
 * states (code: ASCENT/FLOAT), a STARTUP/.../GPS_LOCKOUT power table that
 * never matched OperatingMode_t, and "archive v4 current" two formats behind.
 * Pin the corrected statements so the drift cannot silently return. */
static void test_protocol_docs_match_code(void)
{
    printf("-- STAB-10/#157: protocol docs match the wire format\n");

    char *pf = slurp("../../docs/PayloadFormats.md");
    CHECK_REGRESSION(strstr(pf, "0x06") != NULL, "STAB-10-v6");          /* v6 documented */
    CHECK_REGRESSION(strstr(pf, "sensor_quality") != NULL, "STAB-10-quality");
    CHECK_REGRESSION(strstr(pf, "\"ASCENT\"") != NULL, "STAB-10-mission");
    CHECK_REGRESSION(strstr(pf, "SURVIVAL") != NULL, "STAB-10-powermode");
    CHECK_REGRESSION(strstr(pf, "PRE_FLIGHT") == NULL, "STAB-10-nostale");
    free(pf);

    char *proto = slurp("../../docs/LoRaWANApplicationProtocol.md");
    CHECK_REGRESSION(strstr(proto, "1=ASCENT, 2=FLOAT") != NULL, "STAB-10-proto-mission");
    CHECK_REGRESSION(strstr(proto, "Archive v6 (current firmware") != NULL, "STAB-10-proto-v6");
    free(proto);
}

/* ========================================================================== */
/* RV-09 (#165) — LSE failover: full timer re-init, no ISR-context work        */
/* ========================================================================== */
static void test_lse_failover_deferred_and_complete(void)
{
    printf("-- RV-09/#165: LSE failover deferred to main loop + full timer init\n");

    char *src = slurp("../../Core/Src/main.c");
    CHECK(strstr(src, "LSE_FailoverToLSI") != NULL);   /* anchor */
    /* the ISR sets the flag... */
    CHECK_REGRESSION(strstr(src, "s_lse_fail_pending = true") != NULL, "RV-09-flag");
    /* ...and the ISR body itself must not call the failover directly (the
     * legitimate callers are the main loop and RTC_LivenessCheck) */
    const char *cb = strstr(src, "HAL_RCCEx_LSECSS_Callback(void)");
    CHECK(cb != NULL);
    if (cb) {
        char body[300];
        memset(body, 0, sizeof(body));
        strncpy(body, cb, 260);
        CHECK_REGRESSION(strstr(body, "LSE_FailoverToLSI();") == NULL, "RV-09-isr");
    }
    /* the failover body runs the full timer post-init after MX_RTC_Init */
    const char *fo = strstr(src, "MX_RTC_Init();  /* backup domain was reset");
    CHECK(fo != NULL);
    if (fo) {
        char window[700];
        memset(window, 0, sizeof(window));
        strncpy(window, fo, 650);
        CHECK_REGRESSION(strstr(window, "TIMER_IF_Init") != NULL, "RV-09-postinit");
    }
    /* a main-loop consumer of the flag must exist */
    CHECK_REGRESSION(strstr(src, "if (s_lse_fail_pending)") != NULL, "RV-09-consumer");
    free(src);
}

/* ========================================================================== */
/* RV-10 (#166) — mission cadence override must never move toward higher power */
/* ========================================================================== */
static void test_mission_cadence_never_increases_duty(void)
{
    printf("-- RV-10/#166: cadence override state-gated, FLOAT only relaxes\n");

    char *src = slurp("../../LoRaWAN/App/lora_app.c");
    CHECK(strstr(src, "MISSION_ASCENT_TX_INTERVAL_MS") != NULL);   /* anchor */
    /* FLOAT arm must be state-gated and max()-style (only relax) */
    CHECK_REGRESSION(strstr(src, "MISSION_FLOAT_TX_INTERVAL_MS > plan.tx_interval_ms") != NULL,
                     "RV-10");
    /* COMMISSIONING must not fall into a mission interval: explicit state test */
    CHECK_REGRESSION(strstr(src, "ms == MISSION_FLOAT") != NULL, "RV-10-state");
    free(src);
}

/* ========================================================================== */
/* RV-06 (#162) — GPS-loss silence must re-seed on a backward time step        */
/* ========================================================================== */
static void test_silence_backward_step_guard(void)
{
    printf("-- RV-06/#162: GPS-loss silence backward-step guard\n");

    char *src = slurp("../../LoRaWAN/App/lora_app.c");
    const char *sil = strstr(src, "GPS_LOSS_SILENCE_S");
    CHECK(sil != NULL);   /* anchor */
    /* the wrapped-delta guard must appear near the silence evaluation
     * (F-1/#176 moved the silence policy to the UTC clock: utc_now_s) */
    CHECK_REGRESSION(strstr(src, "utc_now_s < ref_s") != NULL, "RV-06-silence");
    free(src);
}

/* ========================================================================== */
/* 2026-08-11 stability review (F1-F14)                                        */
/* ========================================================================== */
static void test_f1_launch_ref_persistence(void)
{
    printf("-- F1/#167: launch reference persists across reset\n");
    char *h = slurp("../../Core/Inc/backup_regs.h");
    CHECK_REGRESSION(strstr(h, "BKP_REG_LAUNCH_REF") != NULL, "F1-reg");
    free(h);
    char *ml = slurp("../../Core/Inc/mission_logic.h");
    CHECK_REGRESSION(strstr(ml, "LaunchDetector_SetRef") != NULL, "F1-api");
    free(ml);
    char *ms = slurp("../../Core/Src/mission_state.c");
    CHECK_REGRESSION(strstr(ms, "BKP_REG_LAUNCH_REF") != NULL, "F1-wiring");
    CHECK_REGRESSION(strstr(ms, "LaunchDetector_SetRef") != NULL, "F1-restore");
    free(ms);
}

static void test_f2_gnss_power_truthfulness(void)
{
    printf("-- F2/#168: commissioning power-up is canonical; Configure is honest\n");
    char *src = slurp("../../Core/Src/atgm336h.c");
    const char *po = strstr(src, "GNSS_StatusTypeDef GNSS_PowerOn(GNSS_HandleTypeDef *hgnss)");
    CHECK(po != NULL);
    if (po) {
        char body[1600];
        memset(body, 0, sizeof(body));
        strncpy(body, po, 1500);
        CHECK_REGRESSION(strstr(body, "GPIO_PIN_SET") != NULL, "F2-pins");
    }
    const char *cf = strstr(src, "GNSS_StatusTypeDef GNSS_Configure(GNSS_HandleTypeDef *hgnss)");
    CHECK(cf != NULL);
    if (cf) {
        char body[3000];
        memset(body, 0, sizeof(body));
        strncpy(body, cf, 2900);
        CHECK_REGRESSION(strstr(body, "cfg_failures") != NULL, "F2-aggregate");
        CHECK_REGRESSION(strstr(body, "return GNSS_ERROR") != NULL, "F2-status");
    }
    free(src);
}

static void test_f3_flash_waitready_iteration_bound(void)
{
    printf("-- F3/#169: flash BUSY wait has a clock-independent bound\n");
    char *src = slurp("../../Core/Src/w25q16jv.c");
    CHECK_REGRESSION(strstr(src, "W25Q_MAX_BUSY_POLLS") != NULL, "F3");
    free(src);
}

static void test_f4_liveness_and_failover_transactional(void)
{
    printf("-- F4/#170: liveness runs on LSI; failover checks HAL results\n");
    char *src = slurp("../../Core/Src/main.c");
    const char *lv = strstr(src, "static void RTC_LivenessCheck(void)\n{");
    if (lv == NULL) lv = strstr(src, "static void RTC_LivenessCheck(void)\r\n{");
    CHECK(lv != NULL);
    if (lv) {
        char body[3400];
        memset(body, 0, sizeof(body));
        strncpy(body, lv, 3300);
        /* no source-based early return: the SSR check is source-agnostic */
        CHECK_REGRESSION(strstr(body, "RCC_RTCCLKSOURCE_LSE) return") == NULL, "F4-lsi-liveness");
        CHECK_REGRESSION(strstr(body, "stalled on LSI") != NULL, "F4-lsi-escalation");
    }
    const char *fo = strstr(src, "HAL_RCCEx_PeriphCLKConfig(&rtcClk)");
    CHECK(fo != NULL);
    if (fo) {
        /* the failover must consume the return code */
        CHECK_REGRESSION(strstr(fo - 60 > src ? fo - 60 : src, "ck_status") != NULL ||
                         strstr(fo - 60 > src ? fo - 60 : src, "!= HAL_OK") != NULL, "F4-hal-check");
    }
    free(src);
}

static void test_f6_critical_init_is_fatal(void)
{
    printf("-- F6/#171: watchdog/RTC init failure is fatal, not degrade-continue\n");
    char *src = slurp("../../Core/Src/main.c");
    CHECK_REGRESSION(strstr(src, "FAULT_CODE_WATCHDOG_INIT") != NULL, "F6-iwdg");
    CHECK_REGRESSION(strstr(src, "FAULT_CODE_RTC_INIT") != NULL, "F6-rtc");
    free(src);
}

static void test_f10_region_policy_comments_aligned(void)
{
    printf("-- F10/#175: stale-position policy stated consistently\n");
    char *src = slurp("../../LoRaWAN/App/lora_app.c");
    CHECK_REGRESSION(strstr(src, "may INHIBIT but never SWITCH") != NULL, "F10");
    free(src);
}

static void test_f12_flight_build_verified(void)
{
    printf("-- F12/#173: flight macro injection is verified + binary marker\n");
    char *ci = slurp("../../.github/workflows/ci.yml");
    CHECK_REGRESSION(strstr(ci, "SONDE_BUILD:flight") != NULL, "F12-ci");
    free(ci);
    char *src = slurp("../../Core/Src/main.c");
    CHECK_REGRESSION(strstr(src, "SONDE_BUILD:") != NULL, "F12-marker");
    free(src);
}

static void test_f13_f14_w25q_hardening(void)
{
    printf("-- F13/#174: exact JEDEC; F14: wrap-safe range checks\n");
    char *src = slurp("../../Core/Src/w25q16jv.c");
    CHECK_REGRESSION(strstr(src, "jedec_id != W25Q16JV_JEDEC_ID") != NULL, "F13");
    CHECK_REGRESSION(strstr(src, "(addr + len)") == NULL, "F14");
    CHECK_REGRESSION(strstr(src, "W25Q_FLASH_SIZE - addr") != NULL, "F14-form");
    free(src);
}

/* ========================================================================== */
/* FR-19 (#296) - failed ADC calibration must return 0, not convert live     */
/* ========================================================================== */
/* adc_if.c ADC_ReadBattery: a failed HAL_ADCEx_Calibration_Start calls the
 * nonfatal Error_Handler and FALLS THROUGH to ConfigChannel/Start/PollFor-
 * Conversion, returning a live conversion from an uncalibrated ADC - an
 * out-of-envelope battery voltage that drives the power-mode state machine.
 * The R10 (#195) pattern already returns 0 for ConfigChannel/Start failures;
 * calibration must fail the same way. */
static void test_fr19_adc_calibration_failure_returns_zero(void)
{
    printf("-- FR-19 (#296): failed ADC calibration returns 0 before ConfigChannel\n");

    char *src = slurp("../../Core/Src/adc_if.c");
    const char *cal = strstr(src, "HAL_ADCEx_Calibration_Start");
    CHECK(cal != NULL);   /* anchor: calibration call still present */
    if (cal) {
        const char *cfg = strstr(cal, "HAL_ADC_ConfigChannel");
        CHECK(cfg != NULL);
        const char *ret = strstr(cal, "return 0");
        CHECK_REGRESSION(ret != NULL && cfg != NULL && ret < cfg, "FR-19");
    }
    free(src);
}

/* ========================================================================== */
/* FR-16 (#283) - LmHandlerDeInit must clear CtxRestoreDone                  */
/* ========================================================================== */
/* CtxRestoreDone is a file-scope flag set at :417/:436 once a context
 * restore has run, but LmHandlerDeInit() never clears it. Every re-init
 * (stack reset, region switch) then runs with a stale restore-done flag.
 * LT_C01 closeout (2026-08-14) explicitly deferred this to the next pass. */
static void test_fr16_deinit_clears_ctx_restore_done(void)
{
    printf("-- FR-16 (#283): LmHandlerDeInit clears CtxRestoreDone\n");

    char *src = slurp("../../Middlewares/Third_Party/LoRaWAN/LmHandler/LmHandler.c");
    const char *deinit = strstr(src, "LmHandlerErrorStatus_t LmHandlerDeInit");
    CHECK(deinit != NULL);   /* anchor */
    if (deinit) {
        const char *end = strstr(deinit, "return LORAMAC_HANDLER_SUCCESS");
        CHECK(end != NULL);
        const char *clr = strstr(deinit, "CtxRestoreDone = false");
        CHECK_REGRESSION(clr != NULL && end != NULL && clr < end, "FR-16");
    }
    free(src);
}

/* ========================================================================== */
/* FR-02 (#282) - NVM restore must select the newest FULLY VALID slot        */
/* ========================================================================== */
/* OnRestoreContextRequest selects the slot by GENERATION ONLY, then reads
 * the payload straight into the caller's nvm and CRCs it there: a torn
 * newest slot both rejects (leaving torn bytes in the live NVM) and never
 * tries the older fully valid slot. Fix contract: read each candidate into
 * a scratch buffer, CRC it there, pick the newest fully valid generation,
 * and copy into the caller's nvm only after selection. */
static void test_fr02_restore_selects_newest_fully_valid_slot(void)
{
    printf("-- FR-02 (#282): restore CRCs each slot in scratch, copies after select\n");

    char *src = slurp("../../LoRaWAN/App/lora_app.c");
    const char *proto = strstr(src, "static void OnRestoreContextRequest");
    CHECK(proto != NULL);   /* anchor: prototype */
    const char *fn = proto ? strstr(proto + 1, "static void OnRestoreContextRequest") : NULL;
    CHECK(fn != NULL);   /* anchor: definition (prototype-vs-definition trap) */
    if (fn) {
        const char *end = strstr(fn, "LoRaApp_EraseNvmSlots");
        CHECK(end != NULL && end > fn);   /* function body bound */
        const char *scratch = strstr(fn, "uint8_t scratch[");
        const char *copy = strstr(fn, "memcpy(nvm,");
        CHECK_REGRESSION(scratch != NULL && end != NULL && scratch < end, "FR-02-scratch");
        CHECK_REGRESSION(copy != NULL && end != NULL && copy < end, "FR-02-copy-after-select");
    }
    free(src);
}

/* ========================================================================== */
/* FR-03 (#290) - detected-but-unjoined region must silence, not transmit    */
/* ========================================================================== */
/* SelectRegionAndSession: when the H3-detected region differs from the
 * active one, MultiRegion_AutoSwitchToRegion returns SUCCESS for a region
 * that has no banked session ("no join needed"), and the gate transmits on
 * the PREVIOUS region's plan. Fail closed instead: detected != active and
 * the detected region is not joined -> VETO_RF_SILENCE (no new wire value). */
static void test_fr03_unjoined_detected_region_silences(void)
{
    printf("-- FR-03 (#290): detected region without a session -> VETO_RF_SILENCE\n");

    char *src = slurp("../../LoRaWAN/App/lora_app.c");
    const char *fn = strstr(src, "SelectRegionAndSession");
    CHECK(fn != NULL);   /* anchor */
    if (fn) {
        /* The fail-closed check must appear inside the function body. */
        const char *chk = strstr(fn, "MultiRegion_IsRegionJoined(detected_region)");
        CHECK_REGRESSION(chk != NULL, "FR-03");
    }
    free(src);
}

/* ========================================================================== */
/* FR-18 (#295) - commissioning switch-back result must be checked           */
/* ========================================================================== */
/* MultiRegion_PreJoinAllRegions switches back to US915 after the join loop
 * but ignores the result: a failed switch-back still sets the PROVISIONED
 * latch while the MAC sits on the wrong session. The result must feed
 * all_success. (The full ceremony is not host-harness-runnable - the fake
 * LmHandlerJoin never completes a join; strengthening that mock is #262.) */
static void test_fr18_switch_back_result_checked(void)
{
    printf("-- FR-18 (#295): commissioning switch-back failure blocks the latch\n");

    char *src = slurp("../../Core/Src/multiregion_context.c");
    const char *fn = strstr(src, "bool MultiRegion_PreJoinAllRegions(void)");
    CHECK(fn != NULL);   /* anchor: definition (a comment mention precedes it) */
    if (fn) {
        const char *end = strstr(fn, "VerifyAndSetProvisioningLatch");
        CHECK(end != NULL && end > fn);   /* the switch-back precedes the latch */
        const char *chk = strstr(fn,
            "if (MultiRegion_SwitchToRegion(LORAMAC_REGION_US915) != LORAMAC_HANDLER_SUCCESS)");
        CHECK_REGRESSION(chk != NULL && end != NULL && chk < end, "FR-18");
    }
    free(src);
}

int main(void)
{
    printf("=== 2026-08-10/11 review findings — source-scan regressions ===\n\n");

    test_f1_launch_ref_persistence();
    test_f2_gnss_power_truthfulness();
    test_f3_flash_waitready_iteration_bound();
    test_f4_liveness_and_failover_transactional();
    test_f6_critical_init_is_fatal();
    test_f10_region_policy_comments_aligned();
    test_f12_flight_build_verified();
    test_f13_f14_w25q_hardening();

    test_lse_failover_deferred_and_complete();
    test_mission_cadence_never_increases_duty();
    test_silence_backward_step_guard();
    test_protocol_docs_match_code();
    test_ts_wrap_persistence_wiring();
    test_position_age_persistence_wiring();
    test_fatal_escape_scoped();
    test_silence_force_respects_floor();
    test_ci_runs_all_suites();
    test_no_uart_it_receive_on_gps_uart();
    test_geofence_runs_when_gps_skipped();
    test_stale_position_budget_is_24h();
    test_mission_entry_and_float_guard();
    test_float_detection_and_cadence();
    test_chunk_bound_covers_survival_interval();
    test_linkcheck_consumed_after_read();
    test_vcom_resume_gated();
    test_nvm_store_task_is_set();
    test_battery_reading_is_gated();

    test_fr19_adc_calibration_failure_returns_zero();
    test_fr16_deinit_clears_ctx_restore_done();
    test_fr02_restore_selects_newest_fully_valid_slot();
    test_fr03_unjoined_detected_region_silences();
    test_fr18_switch_back_result_checked();

    printf("\n%d checks, %d failures (%d expected pre-fix)\n",
           g_checks, g_failures, g_expected_failures);

    const char *expect_unfixed = getenv("EXPECT_UNFIXED");
    if (expect_unfixed != NULL && expect_unfixed[0] == '1') {
        int unexpected = g_failures - g_expected_failures;
        if (unexpected < 0) {
            printf("UNEXPECTED PASS: a documented bug appears FIXED — flip this "
                   "check from CHECK_REGRESSION to CHECK.\n");
            return 1;
        }
        if (unexpected > 0) {
            printf("NEW BREAKAGE beyond the documented findings.\n");
            return 1;
        }
        printf("OK (baseline): only the documented findings fail.\n");
        return 0;
    }

    if (g_failures == 0) { printf("OK\n"); return 0; }
    return 1;
}
