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

int main(void)
{
    printf("=== 2026-08-10 review findings — source-scan regressions ===\n\n");

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
