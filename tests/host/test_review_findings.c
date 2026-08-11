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
static void test_chunk_bound_covers_survival_interval(void)
{
    printf("-- finding #3: IWDG chunk bound must cover the 3600 s SURVIVAL interval\n");

    char *src = slurp("../../Core/Src/stm32_lpm_if.c");

    int seconds = 0;
    CHECK(parse_define(src, "IWDG_SAFE_SLEEP_SECONDS", &seconds));

    int bound = 0;
    const char *p = strstr(src, "chunks >");
    if (p) { bound = atoi(p + strlen("chunks >")); }
    CHECK(bound > 0);

    printf("   IWDG_SAFE_SLEEP_SECONDS=%d, chunk bound=%d -> max sleep %d s (need >= 3600)\n",
           seconds, bound, seconds * bound);
    CHECK_REGRESSION(seconds * bound >= 3600, "FINDING-3");

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
/* lora_app.c forced gps_enabled_by_power_mgmt = true in the GPS-loss silence
 * path UNCONDITIONALLY - including MODE_SURVIVAL (raw battery below the LTO
 * floor). Urgency may reprioritize work; it may never bypass the electrical
 * hard limit. The force must be gated on not being at the floor. */
static void test_silence_force_respects_floor(void)
{
    printf("-- STAB-03/#150: GPS-loss silence force is hard-floor gated\n");

    char *src = slurp("../../LoRaWAN/App/lora_app.c");
    const char *force = strstr(src, "gps_enabled_by_power_mgmt = true;");
    CHECK(force != NULL);   /* anchor: the silence-path force */
    if (force) {
        /* the hard-floor guard must appear within ~300 chars before/after */
        const char *start = force - 300;
        if (start < src) start = src;
        size_t n = (size_t)(force - start) + 400;
        char *window = malloc(n + 1);
        memset(window, 0, n + 1);
        memcpy(window, start, n);
        CHECK_REGRESSION(strstr(window, "MODE_SURVIVAL") != NULL, "STAB-03");
        free(window);
    }
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

int main(void)
{
    printf("=== 2026-08-10/11 review findings — source-scan regressions ===\n\n");

    test_ts_wrap_persistence_wiring();
    test_position_age_persistence_wiring();
    test_fatal_escape_scoped();
    test_silence_force_respects_floor();
    test_ci_runs_all_suites();
    test_no_uart_it_receive_on_gps_uart();
    test_geofence_runs_when_gps_skipped();
    test_mission_entry_and_float_guard();
    test_float_detection_and_cadence();
    test_chunk_bound_covers_survival_interval();
    test_linkcheck_consumed_after_read();
    test_vcom_resume_gated();
    test_nvm_store_task_is_set();
    test_battery_reading_is_gated();

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
