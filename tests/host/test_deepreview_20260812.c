/**
  ******************************************************************************
  * @file    test_deepreview_20260812.c
  * @brief   Red-first regressions for the 2026-08-12 stability & predictability
  *          deep review ("R3" series): R3-01
  ******************************************************************************
  * R3-01 (P0/P1, #215): bulk archive recovery can starve current science.
  *
  * WHY THIS IS A SCAN-CONFIGURED REPLAY (same precedent as test_burst_fsm.c)
  * The defect is pure control flow: SendTxData re-arms TxTimer RELATIVE TO
  * INVOCATION TIME, and the bulk-continuation callbacks re-arm the SAME
  * sequencer task without the timer. The starvation only appears when the
  * callback order is replayed over a large backlog. The model's SHAPE is
  * selected by scanning lora_app.c: pre-fix the scan finds the relative-
  * restart shape (red); post-fix it finds the absolute-deadline shape
  * (green). Only a firmware change turns the behavioural check green.
  *
  * Run:
  *   make -C tests/host deep        (red until the fixes land)
  *   make -C tests/host baseline    (EXPECT_UNFIXED=1: green pre-fix CI gate)
  ******************************************************************************
  */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>

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
    size_t r = 0, w = 0;
    while (r < (size_t)n) { if (buf[r] != '\r') buf[w++] = buf[r]; r++; }
    buf[w] = '\0';
    return buf;
}

/* Collapse whitespace runs and strip comments so a scan asserts on CODE,
 * never on prose describing the bug. */
static char *normalize_code(const char *src)
{
    size_t n = strlen(src);
    char *out = (char *)malloc(n + 2);
    if (!out) exit(2);
    size_t o = 0;
    bool in_block = false, in_line = false, sp = true;
    for (size_t i = 0; i < n; i++) {
        if (in_block) {
            if (src[i] == '*' && src[i + 1] == '/') { in_block = false; i++; }
            continue;
        }
        if (in_line) {
            if (src[i] == '\n') in_line = false;
            continue;
        }
        if (src[i] == '/' && src[i + 1] == '*') { in_block = true; i++; continue; }
        if (src[i] == '/' && src[i + 1] == '/') { in_line = true;  i++; continue; }
        if (src[i] == ' ' || src[i] == '\t' || src[i] == '\r' || src[i] == '\n') {
            if (!sp) { out[o++] = ' '; sp = true; }
            continue;
        }
        out[o++] = src[i];
        sp = false;
    }
    out[o] = '\0';
    return out;
}

/* ------------------------------------------------------------------ */
/* R3-01 — scheduler replay                                            */
/* ------------------------------------------------------------------ */

#define SCIENCE_INTERVAL_S   600u    /* 10-minute science cadence */
#define BULK_TURNAROUND_S      5u    /* fastest allowed radio turnaround */
#define BACKLOG_RECORDS     1000u    /* pending archive records */
#define RECORDS_PER_PACKET     5u    /* ~BULK_V6_MAX_RECORDS at good DR */
#define SIM_HORIZON_S      86400u    /* 24 h */
#define ALLOWED_JITTER_S      60u    /* explicitly allowed jitter */

typedef enum {
    SHAPE_RELATIVE_RESTART,  /* pre-fix: every SendTxData re-arms now+interval */
    SHAPE_ABSOLUTE_DEADLINE  /* post-fix: absolute due, bulk yields to science */
} SchedShape_t;

static uint32_t replay_max_science_gap(SchedShape_t shape, uint32_t backlog,
                                       uint32_t *science_cycles_out)
{
    uint64_t now = 0;
    uint64_t next_timer_fire = SCIENCE_INTERVAL_S;
    uint64_t next_bulk = UINT64_MAX;
    uint64_t science_due = 0;
    bool     due_valid = false;
    bool     bulk_active = false;
    uint32_t last_science = 0;
    uint32_t max_gap = 0;
    uint32_t science_cycles = 0;
    bool     first_science = true;

    while (now < SIM_HORIZON_S) {
        uint64_t next = (next_timer_fire < next_bulk) ? next_timer_fire : next_bulk;
        if (next == UINT64_MAX || next > SIM_HORIZON_S) break;
        now = next;

        if (next_timer_fire == now) goto science_cycle;

        /* Bulk-continuation invocation of SendTxData. */
        if (shape == SHAPE_ABSOLUTE_DEADLINE && due_valid && now >= science_due) {
            bulk_active = false;         /* fixed firmware: bulk yields */
            next_bulk = UINT64_MAX;
            goto science_cycle;
        }
        if (backlog == 0) { bulk_active = false; next_bulk = UINT64_MAX; continue; }
        backlog -= (backlog >= RECORDS_PER_PACKET) ? RECORDS_PER_PACKET : backlog;
        if (backlog > 0) {
            next_bulk = now + BULK_TURNAROUND_S;   /* OnTxData re-arm */
        } else {
            bulk_active = false;
            next_bulk = UINT64_MAX;
        }
        if (shape == SHAPE_RELATIVE_RESTART) {
            /* THE DEFECT: bulk SendTxData restarts the science timer
             * relative to NOW - deadline pushed out per packet. */
            next_timer_fire = now + SCIENCE_INTERVAL_S;
        }
        continue;

science_cycle:
        {
            uint32_t gap = first_science ? 0 : (uint32_t)(now - last_science);
            if (!first_science && gap > max_gap) max_gap = gap;
            first_science = false;
            last_science = (uint32_t)now;
            science_cycles++;
            backlog++;  /* the new current record joins the archive */

            if (shape == SHAPE_RELATIVE_RESTART) {
                next_timer_fire = now + SCIENCE_INTERVAL_S;
            } else {
                if (!due_valid) { science_due = now; due_valid = true; }
                science_due += SCIENCE_INTERVAL_S;      /* phase-preserving */
                if (science_due <= now) science_due = now + SCIENCE_INTERVAL_S;
                next_timer_fire = science_due;
            }
            /* Probe ACKed (perfect coverage) -> archive opportunity opens. */
            if (backlog > 0 && !bulk_active) {
                bulk_active = true;
                next_bulk = now + BULK_TURNAROUND_S;
            }
        }
    }

    *science_cycles_out = science_cycles;
    return max_gap;
}

static void test_r301_bulk_starvation(const char *app)
{
    printf("-- R3-01 (P0/P1, #215): bulk recovery must not starve current science\n");

    bool has_deadline  = strstr(app, "g_science_due_ms") != NULL;
    bool has_yield     = strstr(app, "ScienceIsDue") != NULL;
    bool relative_arm  = strstr(app, "UTIL_TIMER_SetPeriod(&TxTimer, plan.tx_interval_ms)") != NULL;

    printf("   scan: absolute deadline var=%s science-due yield=%s relative re-arm=%s\n",
           has_deadline ? "yes" : "NO",
           has_yield ? "yes" : "NO",
           relative_arm ? "YES (defect)" : "no");

    SchedShape_t shape = (has_deadline && has_yield && !relative_arm)
                       ? SHAPE_ABSOLUTE_DEADLINE : SHAPE_RELATIVE_RESTART;

    CHECK_REGRESSION(has_deadline, "R3-01-deadline");
    CHECK_REGRESSION(has_yield, "R3-01-yield");
    CHECK_REGRESSION(!relative_arm, "R3-01-relative-arm");

    /* Review-mandated scenario: 1000-record backlog, perfect coverage,
     * fastest turnaround, 10-minute cadence. */
    uint32_t cycles = 0;
    uint32_t max_gap = replay_max_science_gap(shape, BACKLOG_RECORDS, &cycles);
    printf("   replay (backlog=%u): %u science cycles in %u h, max gap %u s (bound %u s)\n",
           BACKLOG_RECORDS, cycles, SIM_HORIZON_S / 3600u,
           max_gap, SCIENCE_INTERVAL_S + ALLOWED_JITTER_S);
    CHECK_REGRESSION(max_gap <= SCIENCE_INTERVAL_S + ALLOWED_JITTER_S, "R3-01");
    CHECK_REGRESSION(cycles >= (SIM_HORIZON_S / SCIENCE_INTERVAL_S) - 1, "R3-01-cadence");

    uint32_t cycles0 = 0;
    uint32_t max_gap0 = replay_max_science_gap(shape, 0, &cycles0);
    printf("   replay (backlog=0): max gap %u s over %u cycles\n", max_gap0, cycles0);
    CHECK(max_gap0 <= SCIENCE_INTERVAL_S + ALLOWED_JITTER_S);

    uint32_t cycles1 = 0;
    uint32_t max_gap1 = replay_max_science_gap(shape, 1, &cycles1);
    printf("   replay (backlog=1): max gap %u s over %u cycles\n", max_gap1, cycles1);
    CHECK(max_gap1 <= SCIENCE_INTERVAL_S + ALLOWED_JITTER_S);
}

/* True if needle occurs in [start, limit). */
static bool occurs_before(const char *start, const char *limit, const char *needle)
{
    const char *hit = strstr(start, needle);
    return hit != NULL && hit < limit;
}

/* ------------------------------------------------------------------ */
/* R3-02 — GNSS wake failure must not archive a previous fix as FRESH  */
/* ------------------------------------------------------------------ */
static void test_r302_gnss_wake_stale(const char *app)
{
    printf("-- R3-02 (P1, #216): GNSS wake failure must not keep a previous fix FRESH\n");

    const char *sig = strstr(app, "static bool AcquireGnssFix(");
    CHECK(sig != NULL);
    if (!sig) return;
    const char *wake = strstr(sig, "GNSS_WakeFromStandby(&hgnss)");
    CHECK(wake != NULL);
    if (!wake) return;

    /* Conservative provenance BEFORE the wake attempt: if the wake fails the
     * function returns immediately, and any state set only AFTER this point
     * is skipped - the previous cycle's hgnss.data and stale flag then flow
     * into EnvSensors_MergeGnss as a FRESH fix. */
    bool stale_first = occurs_before(sig, wake, "EnvSensors_MarkGnssStale(true)");
    bool inval_first = occurs_before(sig, wake, "memset(&hgnss.data, 0");
    printf("   stale-marked before wake: %s | working data invalidated before wake: %s\n",
           stale_first ? "yes" : "NO (defect)",
           inval_first ? "yes" : "NO (defect)");
    CHECK_REGRESSION(stale_first, "R3-02-stale-first");
    CHECK_REGRESSION(inval_first, "R3-02-invalidate-first");

    /* Guard: the acquisition-timeout path must STILL mark last-known stale
     * (post-fix there must remain a stale mark after the wake call too). */
    CHECK(strstr(wake, "EnvSensors_MarkGnssStale(true)") != NULL);
    /* Guard: a real fix still clears the flag. */
    CHECK(strstr(wake, "EnvSensors_MarkGnssStale(false)") != NULL);
}

/* ------------------------------------------------------------------ */
/* R3-03 — ASCENT must never open the archive-recovery opportunity     */
/* ------------------------------------------------------------------ */
static void test_r303_ascent_no_bulk(const char *app)
{
    printf("-- R3-03 (P1, #217): ASCENT must inhibit historical archive recovery\n");

    /* The archive opportunity opens in OnTxData (probe-ACK branch). That
     * decision must consult the mission state (DDR-0005 BR-TX-016/017,
     * P-TX-008): during ASCENT the backlog keeps until FLOAT. */
    /* Anchor on DEFINITIONS (") {"), not the prototypes near the file top -
     * otherwise the span would swallow SendTxData's own MISSION_ASCENT
     * references and false-pass. */
    const char *sig = strstr(app, "static void OnTxData(LmHandlerTxParams_t *params) {");
    CHECK(sig != NULL);
    if (!sig) return;
    const char *end = strstr(sig, "static void OnJoinRequest(LmHandlerJoinParams_t *joinParams) {");
    CHECK(end != NULL);
    if (!end) return;

    bool gated = occurs_before(sig, end, "MISSION_ASCENT");
    printf("   archive-opportunity gate consults mission state: %s\n",
           gated ? "yes" : "NO (defect)");
    CHECK_REGRESSION(gated, "R3-03");

    /* Guard: the opportunity itself still exists (float recovery intact). */
    CHECK(occurs_before(sig, end, "TX_STATE_BULK_TRANSFER"));
}

/* ------------------------------------------------------------------ */
/* R3-04 — archive recovery implements DDR-0005 one-pass semantics      */
/* ------------------------------------------------------------------ */
static void test_r304_one_pass_protocol(const char *app, const char *flashc,
                                        const char *flashh)
{
    printf("-- R3-04 (P1, #218): archive recovery must implement DDR-0005 one-pass\n");

    /* lora_app.c: newest-first recovery read, send-time watermark advance,
     * UNCONFIRMED archive frames, no commit-on-ACK machinery. */
    bool lifo_read   = strstr(app, "FlashLog_GetRecoveryRecords(&hflashlog") != NULL;
    bool send_mark   = strstr(app, "FlashLog_MarkRecoverySent(&hflashlog") != NULL;
    bool unconfirmed = strstr(app, "LmHandlerSend(&bulkData, LORAMAC_HANDLER_UNCONFIRMED_MSG, 0)") != NULL;
    bool no_commit   = strstr(app, "g_bulk_commit_through") == NULL;
    bool probe_conf  = strstr(app, "LORAMAC_HANDLER_CONFIRMED_MSG") != NULL;

    printf("   lora_app: recovery read=%s send-time mark=%s unconfirmed bulk=%s commit-on-ACK gone=%s probe still confirmed=%s\n",
           lifo_read ? "yes" : "NO", send_mark ? "yes" : "NO",
           unconfirmed ? "yes" : "NO", no_commit ? "yes" : "NO(defect)",
           probe_conf ? "yes" : "NO");

    CHECK_REGRESSION(lifo_read, "R3-04-read");
    CHECK_REGRESSION(send_mark, "R3-04-mark");
    CHECK_REGRESSION(unconfirmed, "R3-04-unconfirmed");
    CHECK_REGRESSION(no_commit, "R3-04-no-commit");
    CHECK(probe_conf);   /* BR-TX-003: the compact probe stays confirmed */

    /* flash_log: walker frontier persisted, header v5, legacy API gone. */
    bool frontier    = strstr(flashc, "recovery_frontier") != NULL;
    bool legacy_gone = strstr(flashc, "FlashLog_CommitThrough") == NULL &&
                       strstr(flashc, "GetUnsentRecordsFIFO") == NULL;
    bool v5          = strstr(flashh, "#define FLASH_LOG_HEADER_VERSION 5") != NULL;

    printf("   flash_log: frontier=%s legacy API gone=%s header v5=%s\n",
           frontier ? "yes" : "NO", legacy_gone ? "yes" : "NO(defect)",
           v5 ? "yes" : "NO");

    CHECK_REGRESSION(frontier, "R3-04-frontier");
    CHECK_REGRESSION(legacy_gone, "R3-04-legacy");
    CHECK_REGRESSION(v5, "R3-04-v5");
}

/* ------------------------------------------------------------------ */
/* R3-06 — GNSS commissioning must not claim unverified success         */
/* ------------------------------------------------------------------ */
static void test_r306_gnss_config_verification(const char *app, const char *gnssc)
{
    printf("-- R3-06 (P2, #220): GNSS config success needs receiver-side evidence\n");

    /* The wake path must check HAL_UART_Init's return (and roll back LPM). */
    bool uart_init_checked =
        strstr(gnssc, "HAL_UART_Init(hgnss->huart) != HAL_OK") != NULL;
    printf("   wake HAL_UART_Init return checked: %s\n",
           uart_init_checked ? "yes" : "NO (defect)");
    CHECK_REGRESSION(uart_init_checked, "R3-06-uartinit");

    /* Some receiver-side verification mechanism must exist (the PCAS write
     * protocol has no query/ACK, so behavioral evidence: the NMEA mask takes
     * visible effect in the output stream). */
    bool has_verification = strstr(gnssc, "receiver-side") != NULL;
    printf("   receiver-side verification mechanism: %s\n",
           has_verification ? "yes" : "NO (defect)");
    CHECK_REGRESSION(has_verification, "R3-06-verify");

    /* The unconditional "reconfigured and saved to flash" banner must be
     * gone from the commissioning path (log strings survive comment
     * stripping - this is CODE output, not prose). */
    bool honest = strstr(app, "GPS reconfigured and saved to flash") == NULL;
    printf("   unconditional saved-to-flash banner removed: %s\n",
           honest ? "yes" : "NO (defect)");
    CHECK_REGRESSION(honest, "R3-06-honest");
}

/* ------------------------------------------------------------------ */
/* R3-08 — HAL_GetTick must convert RTC ticks to real ms at the boundary */
/* ------------------------------------------------------------------ */
static void test_r308_tick_timebase(const char *sysapp)
{
    printf("-- R3-08 (P2, #112): HAL_GetTick must return real milliseconds\n");

    /* TIMER_IF_GetTimerValue() is 1024 Hz RTC ticks; the ms contract of
     * HAL_GetTick holds only if the boundary converts. */
    bool converted =
        strstr(sysapp, "TIMER_IF_Convert_Tick2ms(TIMER_IF_GetTimerValue())") != NULL;
    printf("   HAL_GetTick routes through Tick2ms conversion: %s\n",
           converted ? "yes" : "NO (defect)");
    CHECK_REGRESSION(converted, "R3-08");
}

/* ------------------------------------------------------------------ */
/* S-02 — geofence must also run on the GNSS wake-failure path          */
/* ------------------------------------------------------------------ */
static void test_s02_wakefail_geofence(const char *app)
{
    printf("-- S-02 (P2, #226): GNSS wake failure must still run the geofence\n");

    /* The wake-fail path is the else of the AcquireGnssFix success branch in
     * SendTxData. It must consult the same GeofenceRestricted policy point
     * the BURST-03 GPS-skip path uses (inhibit only, on last-known pos). */
    const char *call = strstr(app, "if (AcquireGnssFix(gps_timeout_ms, &ttf_ms)) {");
    CHECK(call != NULL);
    if (!call) return;
    /* Bounded window: the success branch + the else path, up to the first
     * post-branch statement (comments are stripped by normalize_code, so
     * anchor on the code line that follows the if/else). */
    const char *end = strstr(call, "SONDE_LOG_STR(\"\\r\\n\");");
    CHECK(end != NULL);
    if (!end) return;
    bool else_geofence = occurs_before(call, end, "} else {") &&
                         occurs_before(call, end, "GeofenceRestricted(la, lo)");
    printf("   wake-fail path evaluates GeofenceRestricted on last-known pos: %s\n",
           else_geofence ? "yes" : "NO (defect)");
    CHECK_REGRESSION(else_geofence, "S-02");
}

/* ------------------------------------------------------------------ */
/* S-06 — VDDA cache must be invalidated at the top of each work cycle  */
/* ------------------------------------------------------------------ */
static void test_s06_vdda_invalidation(const char *app, const char *adcc)
{
    printf("-- S-06 (P3, #231): VDDA cache invalidated per work cycle\n");

    bool has_invalidator = strstr(adcc, "SYS_ADC_InvalidateVdda") != NULL;
    printf("   adc_if.c provides SYS_ADC_InvalidateVdda: %s\n",
           has_invalidator ? "yes" : "NO (defect)");
    CHECK_REGRESSION(has_invalidator, "S-06-api");

    /* SendTxData must call it BEFORE EnvSensors_Read (anchor on the
     * definition, not the prototype). */
    const char *sig = strstr(app, "static void SendTxData(void) {");
    CHECK(sig != NULL);
    if (!sig) return;
    const char *read = strstr(sig, "EnvSensors_Read(&sensor_data)");
    CHECK(read != NULL);
    if (!read) return;
    bool called_first = occurs_before(sig, read, "SYS_ADC_InvalidateVdda()");
    printf("   SendTxData invalidates VDDA before EnvSensors_Read: %s\n",
           called_first ? "yes" : "NO (defect)");
    CHECK_REGRESSION(called_first, "S-06");
}

/* ------------------------------------------------------------------ */
/* S-09 — I2C re-init paths must re-apply the MX filter configuration   */
/* ------------------------------------------------------------------ */
static void test_s09_i2c_filter_reapply(const char *sensc, const char *lpm)
{
    printf("-- S-09 (P3, #233): I2C re-init must re-apply MX filter config\n");

    /* I2C_BusRecover (sys_sensors.c): ConfigAnalogFilter after HAL_I2C_Init. */
    const char *rec = strstr(sensc, "HAL_I2C_Init(&hi2c2);");
    bool rec_ok = rec != NULL &&
                  strstr(rec, "HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE)") != NULL;
    printf("   I2C_BusRecover re-applies analog filter: %s\n", rec_ok ? "yes" : "NO (defect)");
    CHECK_REGRESSION(rec_ok, "S-09-recover");

    /* PWR_ExitStopMode (stm32_lpm_if.c): same after its HAL_I2C_Init. */
    const char *wk = strstr(lpm, "HAL_I2C_Init(&hi2c2)");
    bool wk_ok = wk != NULL &&
                 strstr(wk, "HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE)") != NULL;
    printf("   PWR_ExitStopMode re-applies analog filter: %s\n", wk_ok ? "yes" : "NO (defect)");
    CHECK_REGRESSION(wk_ok, "S-09-stop2");
}

int main(void)
{
    char *app    = normalize_code(slurp("../../LoRaWAN/App/lora_app.c"));
    char *flashc = normalize_code(slurp("../../Core/Src/flash_log.c"));
    char *flashh = normalize_code(slurp("../../Core/Inc/flash_log.h"));
    char *gnssc  = normalize_code(slurp("../../Core/Src/atgm336h.c"));
    char *sysapp = normalize_code(slurp("../../Core/Src/sys_app.c"));
    char *adcc   = normalize_code(slurp("../../Core/Src/adc_if.c"));
    char *sensc  = normalize_code(slurp("../../Core/Src/sys_sensors.c"));
    char *lpm    = normalize_code(slurp("../../Core/Src/stm32_lpm_if.c"));

    printf("=== 2026-08-12 deep review regressions (R3 series) ===\n\n");

    test_r301_bulk_starvation(app);
    printf("\n");
    test_r302_gnss_wake_stale(app);
    printf("\n");
    test_r303_ascent_no_bulk(app);
    printf("\n");
    test_r304_one_pass_protocol(app, flashc, flashh);
    printf("\n");
    test_r306_gnss_config_verification(app, gnssc);
    printf("\n");
    test_r308_tick_timebase(sysapp);
    printf("\n");
    test_s02_wakefail_geofence(app);
    printf("\n");
    test_s06_vdda_invalidation(app, adcc);
    printf("\n");
    test_s09_i2c_filter_reapply(sensc, lpm);

    free(lpm);
    free(sensc);
    free(adcc);
    free(sysapp);
    free(gnssc);
    free(flashh);
    free(flashc);

    printf("\n%d checks, %d failures (%d expected pre-fix)\n",
           g_checks, g_failures, g_expected_failures);

    free(app);

    if (getenv("EXPECT_UNFIXED") && g_failures == g_expected_failures) {
        printf("BASELINE OK (all failures are known-unfixed findings)\n");
        return 0;
    }
    return g_failures ? 1 : 0;
}
