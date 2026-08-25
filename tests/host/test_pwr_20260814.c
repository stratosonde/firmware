/* ARCHIVE — PWR admission-gate suite (PWR-SIMPLIFY 2026-08-24). Module
 * contracts live in test_<module>.c (see R2_TEST_MAP.md). (Refactor stage 7.) */

/**
  ******************************************************************************
  * @file    test_pwr_20260814.c
  * @brief   Power admission gates. Gate A = temperature (-60 C default),
  *          Gate B = raw-battery floor (3800 mV). PWR-02 (#297) RESOLVED: the
  *          temperature-blind 4300 mV ladder floor is gone with the ladder;
  *          behavioural checks exercise FirstFlightPolicy_Decide directly,
  *          structural scans verify the shipped defaults and the empty seam.
  *
  *   PWR-01 (P1): compensation-table non-monotonicity remains red-gated as
  *     the LT-06 pair in test_lt_20260813.c (#248). Not duplicated here.
  *
  *   Run:
  *   make -C tests/host pwr
  ******************************************************************************  */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>

#include "first_flight_policy.h"
#include <math.h>

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

/* Source-scan helpers (same shape as test_lt_20260813.c)
 * Phase 3 (#263): the ASan gate runs the scan suites too. Scan buffers live
 * until process exit, so pool them and free once via atexit instead of
 * hand-freeing (or leaking) on individual return paths. */
#define SCAN_POOL_MAX 48
static void *g_scan_pool[SCAN_POOL_MAX];
static int g_scan_pool_n;
static void scan_pool_free(void)
{
    while (g_scan_pool_n > 0) free(g_scan_pool[--g_scan_pool_n]);
}
static void *scan_pool_track(void *p)
{
    if (p && g_scan_pool_n < SCAN_POOL_MAX) {
        if (g_scan_pool_n == 0) atexit(scan_pool_free);
        g_scan_pool[g_scan_pool_n++] = p;
    }
    return p;
}

static char *slurp(const char *path)
{
    FILE *f = fopen(path, "rb");
    if (!f) { printf("FATAL: cannot open %s\n", path); exit(2); }
    fseek(f, 0, SEEK_END);
    long n = ftell(f);
    fseek(f, 0, SEEK_SET);
    char *buf = (char *)scan_pool_track(malloc((size_t)n + 1));
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
    char *out = (char *)scan_pool_track(malloc(n + 1));
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

/* ========================================================================== */
/* GATE-A (temperature): -60 C default in config, -60 NULL fallback in the     */
/* application, reject below the threshold or on a stale/invalid sample.       */
/* ========================================================================== */
static void test_gate_a_temperature(void)
{
    printf("-- GATE-A: temperature admission (default -60 C)\n");

    /* Behavioural (policy runs on first_flight_policy.c): */
    const FirstFlightPolicyConfig_t policy = { -60, 3800 };
    const FirstFlightAdmissionInput_t warm = { -59.9f, true, 5000, true };
    const FirstFlightAdmissionInput_t edge = { -60.0f, true, 5000, true };
    const FirstFlightAdmissionInput_t cold = { -60.1f, true, 5000, true };
    const FirstFlightAdmissionInput_t stale = { -20.0f, false, 5000, true };
    const FirstFlightAdmissionInput_t nan_t = { NAN, true, 5000, true };
    CHECK(FirstFlightPolicy_Decide(&policy, &warm) == FIRST_FLIGHT_RUN_FULL);
    CHECK(FirstFlightPolicy_Decide(&policy, &edge) == FIRST_FLIGHT_RUN_FULL);
    CHECK(FirstFlightPolicy_Decide(&policy, &cold) == FIRST_FLIGHT_RETRY_LOW_ENERGY);
    CHECK(FirstFlightPolicy_Decide(&policy, &stale) == FIRST_FLIGHT_RETRY_LOW_ENERGY);
    CHECK(FirstFlightPolicy_Decide(&policy, &nan_t) == FIRST_FLIGHT_RETRY_LOW_ENERGY);

    /* Structural: the shipped default and the NULL-config fallback agree. */
    char *config = strip_comments(slurp("../../Core/Src/config.c"));
    CHECK(strstr(config, "g_config.gps_temperature_lockout = -60") != NULL);
    char *app = strip_comments(slurp("../../LoRaWAN/App/lora_app.c"));
    CHECK(strstr(app, "gps_temperature_lockout : -60") != NULL);
}

/* ========================================================================== */
/* GATE-B (raw battery floor): 3800 mV, accessor clamps persisted values UP    */
/* (never weaker), rejected ADC sample (0) rejects.                            */
/* ========================================================================== */
static void test_gate_b_voltage_floor(void)
{
    printf("-- GATE-B: raw battery admission (3800 mV floor)\n");

    const FirstFlightPolicyConfig_t policy = { -60, 3800 };
    const FirstFlightAdmissionInput_t above = { 25.0f, true, 3801, true };
    const FirstFlightAdmissionInput_t edge = { 25.0f, true, 3800, true };
    const FirstFlightAdmissionInput_t below = { 25.0f, true, 3799, true };
    const FirstFlightAdmissionInput_t stale = { 25.0f, true, 5000, false };
    const FirstFlightAdmissionInput_t zero = { 25.0f, true, 0, true };
    CHECK(FirstFlightPolicy_Decide(&policy, &above) == FIRST_FLIGHT_RUN_FULL);
    CHECK(FirstFlightPolicy_Decide(&policy, &edge) == FIRST_FLIGHT_RUN_FULL);
    CHECK(FirstFlightPolicy_Decide(&policy, &below) == FIRST_FLIGHT_RETRY_LOW_ENERGY);
    CHECK(FirstFlightPolicy_Decide(&policy, &stale) == FIRST_FLIGHT_RETRY_LOW_ENERGY);
    CHECK(FirstFlightPolicy_Decide(&policy, &zero) == FIRST_FLIGHT_RETRY_LOW_ENERGY);

    /* Structural: the floor macro is 3800, and the persisted threshold can
     * only be stricter (clamped up in ConfigGetFirstFlightBatteryMinMv). */
    char *ch = slurp("../../Core/Inc/config.h");
    CHECK(strstr(ch, "#define CONFIG_FIRST_FLIGHT_BATTERY_FLOOR_MV 3800U") != NULL);
    char *cc = strip_comments(slurp("../../Core/Src/config.c"));
    CHECK(strstr(cc, "configured < CONFIG_FIRST_FLIGHT_BATTERY_FLOOR_MV") != NULL);
}

/* ========================================================================== */
/* EXCISE: no mode ladder remains in the decide seam or the plan. The archive  */
/* record's reserved slope slot carries the named sentinel, not a zero.        */
/* ========================================================================== */
static void test_ladder_excision(void)
{
    printf("-- EXCISE: mode ladder, slope tracking, hysteresis gone\n");

    char *ph = slurp("../../Core/Inc/power_model.h");
    CHECK(strstr(ph, "NormalizeBatteryVoltage") != NULL); /* SoC helper remains */
    CHECK(strstr(ph, "CalculateVoltageSlope") == NULL);
    CHECK(strstr(ph, "PredictTimeToLowerThreshold") == NULL);
    CHECK(strstr(ph, "SelectModeFromPredictions") == NULL);

    char *tp = slurp("../../Core/Inc/transmit_plan.h");
    CHECK(strstr(tp, "DecideTransmitPlan") != NULL);
    CHECK(strstr(tp, "VoltageSlope_t") == NULL);
    char *tc = strip_comments(slurp("../../Core/Src/transmit_plan.c"));
    CHECK(strstr(tc, "SelectModeFromPredictions") == NULL);
    CHECK(strstr(tc, "CalculateVoltageSlope") == NULL);
    CHECK(strstr(tc, "tx_interval_normal") != NULL); /* the fixed cadence source */

    char *app = strip_comments(slurp("../../LoRaWAN/App/lora_app.c"));
    CHECK(strstr(app, "VoltageSlope_t") == NULL);
    CHECK(strstr(app, "SlopePersistToBackup") == NULL);
    /* Sentinel-not-zero: reserved slope slot writes the named invalid value. */
    CHECK(strstr(app, "SLOPE_MV_H_NOT_COMPUTED") != NULL);

    /* Wire layout unchanged: the archive record's voltage_slope field stays
     * at offset 24 (sentinel semantics documented in the schema). */
    char *schema = slurp("../../wire/wire_schema.json");
    CHECK(strstr(schema, "\"voltage_slope\"") != NULL);
    CHECK(strstr(schema, "-32768") != NULL);
}

int main(void)
{
    printf("=== Power admission gates (PWR-SIMPLIFY 2026-08-24) ===\n\n");

    test_gate_a_temperature();
    printf("\n");
    test_gate_b_voltage_floor();
    printf("\n");
    test_ladder_excision();

    /* pooled scan buffers: freed at exit (scan_pool_track) */

    printf("\n%d checks, %d failures (%d expected pre-fix)\n",
           g_checks, g_failures, g_expected_failures);
    return g_failures ? 1 : 0;
}
