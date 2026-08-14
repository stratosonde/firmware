/**
  ******************************************************************************
  * @file    test_sp_20260813.c
  * @brief   Red-first regressions for the 2026-08-13 "SP" series review
  *          (docs/temp/claude.md).
  ******************************************************************************
  *   SP-01 (#244): the vendored HAL treats ANY UART error as blocking while
  *     CR3.DMAR is set (stm32wlxx_hal_uart.c: UART_EndRxTransfer +
  *     HAL_DMA_Abort_IT) and never consults the AdvancedInit
  *     DMADisableonRxError bit the S-C (#212) fix relied on. Result: one
  *     framing/noise glitch killed the circular GNSS DMA for the rest of the
  *     acquisition window, indistinguishable from "no bytes".
  *     Fix: HAL_UART_ErrorCallback override (usart_if.c) ->
  *     GNSS_UART_ErrorCallback (atgm336h.c) counts, resets ring state, re-arms.
  *
  *   F-01 (#245, P1): LoRaWAN stack lifecycle return values ignored at boot
  *     (LmHandlerInit/Configure) and in the region switch/restore ritual -
  *     R1 (#187) fail-closed steps 4-10, but steps 1-3 were bare and
  *     LoRaApp_ReInitStack returned void. Structural scans only; behavioural
  *     coverage is the fault-injection matrix in test_multiregion.c.
  *
  *   BEHAVIOURAL against the real atgm336h.c (#included below, DR-suite
  *   precedent) + structural scans of usart_if.c / main.c.
  *
  *   Run:
  *   make -C tests/host sp0813        (red until the fix lands)
  *   EXPECT_UNFIXED=1 ./test_sp0813   (green pre-fix gate)
  ******************************************************************************
  */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>

/* GNSS host stub surface (identical to test_dr_20260812.c). */
#include "timer_if.h"
#include "stm32_systime.h"

static uint32_t g_fake_rtc_seconds = 0;
uint32_t TIMER_IF_GetTime(uint16_t *ms) { if (ms) *ms = 0; return g_fake_rtc_seconds; }
uint16_t g_host_dma_cndtr = 0;
uint32_t HAL_GetTick(void) { return g_fake_rtc_seconds * 1000U; }

static uint32_t g_fake_epoch = 1754500000U;
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
    if (!start) { return false; }
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

/* Occurrences of `needle` inside the function body that starts at the first
 * occurrence of `sig` (same bounds as in_function). -1 if sig not found. */
static int count_in_function(const char *src, const char *sig, const char *needle)
{
    const char *start = strstr(src, sig);
    if (!start) return -1;
    const char *end = strstr(start, "\n}\n");
    if (!end) end = src + strlen(src);
    size_t len = (size_t)(end - start);
    char *body = (char *)malloc(len + 1);
    memcpy(body, start, len);
    body[len] = '\0';
    int c = count_occurrences(body, needle);
    free(body);
    return c;
}

/* ========================================================================== */
/* SP-01 (#244) - behavioural: the error callback recovers the circular stream */
/* ========================================================================== */

static UART_HandleTypeDef g_sp01_huart = { USART1, NULL, HAL_UART_ERROR_NONE, 0 };
static GNSS_HandleTypeDef g_sp01_gnss;   /* static: pHgnss (inside the UUT) refs it */

/* Mid-acquisition: stream live, bookkeeping dirty, unrelated state seeded. */
static void seed_mid_acquisition(void)
{
    GNSS_Init(&g_sp01_gnss);             /* sets the UUT's pHgnss */
    g_sp01_gnss.huart         = &g_sp01_huart;
    g_sp01_gnss.is_powered    = true;
    g_sp01_gnss.rx_dma_active = true;
    g_sp01_gnss.dma_head           = 100;
    g_sp01_gnss.dma_tail           = 90;
    g_sp01_gnss.dma_data_ready     = true;
    g_sp01_gnss.dma_produced_total = 256;
    g_sp01_gnss.dma_consumed_total = 200;
    g_sp01_gnss.dma_overrun_count  = 5;    /* must SURVIVE (not an overrun event) */
    g_sp01_gnss.nmea_length        = 42;   /* torn partial sentence */
    g_sp01_gnss.extended.has_prev_altitude = true;  /* must SURVIVE (not a new
                                                       * acquisition - DR-10) */
    g_sp01_huart.ErrorCode = 0x0Eu;        /* FE|NE|ORE-ish */
    g_host_uart_rx_dma_calls = 0;
    g_host_uart_dma_rc = HAL_OK;
}

static void test_sp01_mid_acquisition_error_recovers(void)
{
    printf("-- SP-01 (#244) behavioural: mid-acquisition FE/NE recovers the stream\n");
    seed_mid_acquisition();

    GNSS_UART_ErrorCallback(&g_sp01_huart);

    printf("   after callback: error_count=%lu rearm_calls=%lu rx_dma_active=%d\n",
           (unsigned long)g_sp01_gnss.uart_error_count,
           (unsigned long)g_host_uart_rx_dma_calls,
           (int)g_sp01_gnss.rx_dma_active);

    CHECK(g_sp01_gnss.uart_error_count == 1);        /* event counted */
    CHECK(g_host_uart_rx_dma_calls == 1);            /* re-armed exactly once */
    CHECK(g_sp01_gnss.rx_dma_active);                /* stream still expected live */
    CHECK(g_sp01_huart.ErrorCode == HAL_UART_ERROR_NONE);
    /* ring bookkeeping restarted (torn tail dropped) */
    CHECK(g_sp01_gnss.dma_head == 0);
    CHECK(g_sp01_gnss.dma_tail == 0);
    CHECK(g_sp01_gnss.dma_produced_total == 0);
    CHECK(g_sp01_gnss.dma_consumed_total == 0);
    CHECK(g_sp01_gnss.dma_data_ready == false);
    CHECK(g_sp01_gnss.nmea_length == 0);
    /* NOT a fresh acquisition: health + vertical-speed state survive. */
    CHECK(g_sp01_gnss.dma_overrun_count == 5);
    CHECK(g_sp01_gnss.extended.has_prev_altitude);
    CHECK(g_sp01_gnss.is_powered);
}

static void test_sp01_rearm_failure_disengages_cleanly(void)
{
    printf("-- SP-01 (#244) behavioural: re-arm refusal drops rx_dma_active\n");
    seed_mid_acquisition();
    g_host_uart_dma_rc = HAL_ERROR;    /* HAL refuses the re-arm */

    GNSS_UART_ErrorCallback(&g_sp01_huart);

    printf("   after callback: error_count=%lu rearm_calls=%lu rx_dma_active=%d\n",
           (unsigned long)g_sp01_gnss.uart_error_count,
           (unsigned long)g_host_uart_rx_dma_calls,
           (int)g_sp01_gnss.rx_dma_active);

    CHECK(g_sp01_gnss.uart_error_count == 1);        /* still counted */
    CHECK(g_host_uart_rx_dma_calls == 1);            /* one attempt made */
    CHECK(!g_sp01_gnss.rx_dma_active);               /* honestly disengaged; the
                                                      * GNSS timeout path ends the
                                                      * window, next wake re-inits */
    g_host_uart_dma_rc = HAL_OK;                     /* restore for other tests */
}

static void test_sp01_teardown_error_is_ignored(void)
{
    printf("-- SP-01 (#244) behavioural: error during/after teardown is a no-op\n");
    seed_mid_acquisition();
    g_sp01_gnss.rx_dma_active = false; /* teardown already cleared it */

    GNSS_UART_ErrorCallback(&g_sp01_huart);

    /* Nothing counted, nothing reset, nothing re-armed. */
    CHECK(g_sp01_gnss.uart_error_count == 0);
    CHECK(g_host_uart_rx_dma_calls == 0);
    CHECK(g_sp01_gnss.dma_head == 100);
    CHECK(g_sp01_gnss.nmea_length == 42);
    CHECK(g_sp01_huart.ErrorCode == 0x0Eu);
}

/* ========================================================================== */
/* SP-01 (#244) - structural: the wiring the behaviour depends on              */
/* ========================================================================== */
static void test_sp01_structural(const char *usart, const char *gnss, const char *mainc)
{
    printf("-- SP-01 (#244) structural: callback override, re-arm, S-C anchors\n");

    /* usart_if.c owns the HAL override and delegates to the GNSS driver. */
    CHECK_REGRESSION(strstr(usart, "HAL_UART_ErrorCallback") != NULL, "SP-01-override");
    CHECK_REGRESSION(in_function(usart, "void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)",
                                 "GNSS_UART_ErrorCallback(huart)"),
                     "SP-01-delegates");
    CHECK_REGRESSION(in_function(usart, "void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)",
                                 "huart->Instance == USART1"),
                     "SP-01-instance-guard");

    /* atgm336h.c: the callback re-arms, counts, resets the ring, is guarded. */
    const char *sig = "void GNSS_UART_ErrorCallback(UART_HandleTypeDef *huart)";
    CHECK_REGRESSION(in_function(gnss, sig, "HAL_UART_Receive_DMA"), "SP-01-rearm");
    CHECK_REGRESSION(in_function(gnss, sig, "rx_dma_active"),        "SP-01-live-guard");
    CHECK_REGRESSION(in_function(gnss, sig, "uart_error_count"),     "SP-01-health");
    CHECK_REGRESSION(in_function(gnss, sig, "nmea_length"),          "SP-01-ring-reset");

    /* Arm/teardown discipline: the live flag is set at every DMA start and
     * cleared at every teardown (and on re-arm failure inside the callback). */
    CHECK_REGRESSION(count_occurrences(gnss, "rx_dma_active = true") >= 2,  "SP-01-arm-sites");
    CHECK_REGRESSION(count_occurrences(gnss, "rx_dma_active = false") >= 3, "SP-01-teardown-sites");

    /* S-C (#212) anchors intact - the advanced-feature bits stay set (their
     * ORE hardware behaviour is still wanted); what changed is the comment's
     * claim that nothing restarts the DMA. */
    CHECK(strstr(mainc, "UART_ADVFEATURE_OVERRUN_DISABLE") != NULL);
    CHECK(strstr(mainc, "UART_ADVFEATURE_DMA_ENABLEONRXERROR") != NULL);
    CHECK(strstr(mainc, "is not overridden") == NULL);  /* comment made false by
                                                          * the override */
}

/* ========================================================================== */
/* F-01 (#245) - structural: every lifecycle call returns a CHECKED status     */
/* ========================================================================== */
static void test_f01_structural(const char *app, const char *apph, const char *mreg)
{
    printf("-- F-01 (#245) structural: LoRaWAN lifecycle returns are checked\n");

    /* LoRaApp_ReInitStack now returns a status; the void form is gone. */
    CHECK_REGRESSION(strstr(apph, "LmHandlerErrorStatus_t LoRaApp_ReInitStack") != NULL,
                     "F-01-hdr-status");
    CHECK_REGRESSION(strstr(app, "void LoRaApp_ReInitStack") == NULL, "F-01-no-void");

    /* ReInitStack checks each of Halt/DeInit/Init. */
    const char *rsig = "LmHandlerErrorStatus_t LoRaApp_ReInitStack(LoRaMacRegion_t new_region)";
    CHECK_REGRESSION(count_in_function(app, rsig, "!= LORAMAC_HANDLER_SUCCESS") >= 3,
                     "F-01-reinit-checks");

    /* Boot failure marks SYS_CAP_RADIO (degrade-and-say-so, DDR-0009). */
    CHECK_REGRESSION(count_in_function(app, "void LoRaWAN_Init(void)",
                                       "SysCaps_MarkFailed(SYS_CAP_RADIO)") >= 2,
                     "F-01-boot-marks");  /* LmHandlerInit + LmHandlerConfigure */

    /* Auto-switch: the plain-ERROR outcome gets a real log line. */
    CHECK_REGRESSION(strstr(app, "Switch FAILED") != NULL, "F-01-switch-log");

    /* RestoreSessionToMac: steps 1-3 (reinit/configure/deveui/keys) all
     * fail-closed, joining R1's steps 4-10. */
    CHECK_REGRESSION(count_in_function(mreg, "static LmHandlerErrorStatus_t RestoreSessionToMac",
                                       "!= LORAMAC_HANDLER_SUCCESS") >= 5,
                     "F-01-restore-checks");
    /* Both ReInitStack call sites check the status. */
    CHECK_REGRESSION(count_occurrences(mreg,
                     "if (LoRaApp_ReInitStack(region) != LORAMAC_HANDLER_SUCCESS)") >= 2,
                     "F-01-reinit-callers");
}

/* ========================================================================== */
/* SP-05 (#246) - structural: IN865/KR920/RU864 are compiled AND reachable     */
/* ========================================================================== */
static void test_sp05_structural(const char *conf, const char *app, const char *mreg,
                                 const char *mregh, const char *msstate, const char *seid)
{
    printf("-- SP-05 (#246) structural: three more regions wired end to end\n");

    /* lorawan_conf.h: macros actively defined (comment-stripped text). */
    CHECK_REGRESSION(strstr(conf, "#define REGION_KR920") != NULL, "SP-05-conf-kr920");
    CHECK_REGRESSION(strstr(conf, "#define REGION_IN865") != NULL, "SP-05-conf-in865");
    CHECK_REGRESSION(strstr(conf, "#define REGION_RU864") != NULL, "SP-05-conf-ru864");

    /* Identity table row for RU864 (IN865/KR920 already existed). */
    CHECK_REGRESSION(strstr(mreg, "LORAMAC_REGION_RU864, {LORAWAN_DEVICE_EUI_RU864}") != NULL,
                     "SP-05-identity-ru864");
    CHECK_REGRESSION(strstr(seid, "LORAWAN_DEVICE_EUI_RU864") != NULL, "SP-05-deveui-ru864");

    /* Bank holds 7 slots; persisted format bumped so old banks re-commission. */
    CHECK_REGRESSION(strstr(mregh, "#define MAX_REGION_CONTEXTS              7") != NULL,
                     "SP-05-slots-7");
    CHECK_REGRESSION(strstr(mregh, "#define MULTIREGION_VERSION              4") != NULL,
                     "SP-05-version-4");

    /* Ground-join-all and boot resume-scan cover the new regions. */
    CHECK_REGRESSION(in_function(mreg, "kPreJoinRegions[] =", "LORAMAC_REGION_IN865"), "SP-05-prejoin-in865");
    CHECK_REGRESSION(in_function(mreg, "kPreJoinRegions[] =", "LORAMAC_REGION_KR920"), "SP-05-prejoin-kr920");
    CHECK_REGRESSION(in_function(mreg, "kPreJoinRegions[] =", "LORAMAC_REGION_RU864"), "SP-05-prejoin-ru864");
    CHECK_REGRESSION(strstr(app, "LORAMAC_REGION_RU864") != NULL, "SP-05-loraapp-ru864");
    CHECK_REGRESSION(strstr(app, "DataratesRU864") != NULL, "SP-05-datarates-ru864");

    /* Commissioning door anchor covers all joined regions. */
    CHECK_REGRESSION(strstr(msstate, "MultiRegion_IsRegionJoined(LORAMAC_REGION_IN865)") != NULL,
                     "SP-05-door-in865");
    CHECK_REGRESSION(strstr(msstate, "MultiRegion_IsRegionJoined(LORAMAC_REGION_RU864)") != NULL,
                     "SP-05-door-ru864");
}

/* ========================================================================== */
/* SP-07 (#247) - structural: the FR-23 escape must be REACHABLE                */
/* ========================================================================== */
static void test_sp07_fatal_escape_wired(const char *mainc)
{
    printf("-- SP-07 (#247) structural: deterministic-fatal escape reachable\n");

    /* The escape gates on code >= 16 && Attempts>=5 && FatalIsDegradable().
     * FatalIsDegradable accepts ONLY FAULT_CODE_FLASH_INIT - so the escape is
     * dead code unless that code is actually passed to Error_Handler_Fatal.
     * Pre-fix: zero call sites (a dead W25Q / FlashLog init degraded inline
     * with no breadcrumb, and the documented escape never ran). */
    CHECK_REGRESSION(count_occurrences(mainc,
                     "Error_Handler_Fatal(FAULT_CODE_FLASH_INIT)") >= 2, "SP-07-sites");
    /* Sanity pins: the gate and the policy table still exist as designed. */
    CHECK_REGRESSION(in_function(mainc, "static bool FatalIsDegradable(uint16_t code)",
                                 "FAULT_CODE_FLASH_INIT"), "SP-07-policy");
    CHECK_REGRESSION(in_function(mainc, "void Error_Handler_Fatal(uint16_t code)",
                                 "ResetCause_GetBootAttempts()"), "SP-07-gate");
}

/* ========================================================================== */
/* SP-09 (#249) - structural: ONE owner for the PB6/PB7 sleep policy            */
/* ========================================================================== */
static void test_sp09_single_pin_owner(const char *gnss, const char *lpm)
{
    printf("-- SP-09 (#249) structural: single-owner PB6/PB7 sleep policy\n");

    /* Pre-fix: three owners (MSP AF init, GNSS_EnterStandby, EnterStopMode)
     * and EnterStopMode overrode the anti-parasitic PB6-LOW with ANALOG on
     * every sleep entry - directly contradicting its own 'DO NOT override'
     * comment two paragraphs later. Fix: one policy function in the GNSS
     * driver, called by both teardown sites after HAL_UART_DeInit. */
    CHECK_REGRESSION(strstr(gnss, "GNSS_UARTPins_SleepSafe") != NULL, "SP-09-policy-fn");
    CHECK_REGRESSION(in_function(gnss, "void GNSS_UARTPins_SleepSafe(void)", "GPIO_PIN_6"),
                     "SP-09-policy-pb6");
    CHECK_REGRESSION(in_function(gnss, "GNSS_StatusTypeDef GNSS_EnterStandby",
                                 "GNSS_UARTPins_SleepSafe()"), "SP-09-standby-calls");
    CHECK_REGRESSION(strstr(lpm, "GNSS_UARTPins_SleepSafe()") != NULL, "SP-09-lpm-calls");
    /* The contradictory blanket-analog override must be gone (code scan,
     * not prose: comments are stripped before this point). */
    CHECK_REGRESSION(strstr(lpm, "GPIO_UART.Pin = GPIO_PIN_6 | GPIO_PIN_7") == NULL,
                     "SP-09-lpm-no-override");
}

/* ========================================================================== */
/* SP-14 (#251) - structural: compact converters guard NaN BEFORE the cast     */
/* ========================================================================== */
static void test_sp14_nan_guards(const char *pend)
{
    printf("-- SP-14 (#251) structural: NaN/negative guards on temp + battery\n");
    /* Value-based NaN probes are a toolchain lottery (float->int UB is
     * diagnosable but not deterministic), so the deterministic assertion is
     * the guard's presence; test_main.c carries the behavioural mirrors. */
    /* Anchor on the DEFINITION (the ')\n{' suffix separates it from the
     * forward prototype at the file top - the notes' scan pit). */
    CHECK_REGRESSION(in_function(pend, "static int8_t ConvertTemperatureToCompact(float temperature_c)\n{",
                                 "isnan"), "SP-14-temp-guard");
    CHECK_REGRESSION(in_function(pend, "static uint8_t ConvertBatteryVoltageToCompact(float voltage_volts)\n{",
                                 "isnan"), "SP-14-batt-guard");
}

/* ========================================================================== */
/* SP-15 (#253) - structural: the unreachable WAIT_PROBE_ACK case is gone      */
/* ========================================================================== */
static void test_sp15_unreachable_case_gone(const char *app)
{
    printf("-- SP-15 (#253) structural: no dead WAIT_PROBE_ACK switch case\n");
    /* The pre-switch reset guarantees g_tx_state is PROBE_SF10 or
     * BULK_TRANSFER at the switch; the case could never run, and its
     * 'fall through' comment was falsified by its own break. */
    CHECK_REGRESSION(strstr(app, "case TX_STATE_WAIT_PROBE_ACK:") == NULL, "SP-15-case-gone");
    /* The reset itself must still exist (it is the actual mechanism). */
    CHECK_REGRESSION(strstr(app, "Resetting stale TX state") != NULL, "SP-15-reset-kept");
}

/* ========================================================================== */
/* SP-16 (#254) - structural: auto-switch logs distinguish switch vs no-op     */
/* ========================================================================== */
static void test_sp16_honest_switch_log(const char *app)
{
    printf("-- SP-16 (#254) structural: honest auto-switch logging\n");
    /* The old banner claimed success for genuinely-switched, same-region,
     * not-joined-stay, and disabled-build outcomes alike. */
    CHECK_REGRESSION(strstr(app, "Auto-switch completed successfully") == NULL,
                     "SP-16-banner-gone");
    CHECK_REGRESSION(strstr(app, "Auto-switch: no change") != NULL, "SP-16-nochange-log");
    CHECK_REGRESSION(count_occurrences(app, "MultiRegion_GetActiveRegion()") >= 2,
                     "SP-16-before-after");
}

int main(void)
{
    char *usart = strip_comments(slurp("../../Core/Src/usart_if.c"));
    char *gnss  = strip_comments(slurp("../../Core/Src/atgm336h.c"));
    char *mainc = strip_comments(slurp("../../Core/Src/main.c"));
    char *lpm   = strip_comments(slurp("../../Core/Src/stm32_lpm_if.c"));
    char *pend  = strip_comments(slurp("../../Core/Src/payload_encode.c"));
    char *app   = strip_comments(slurp("../../LoRaWAN/App/lora_app.c"));
    char *apph  = strip_comments(slurp("../../LoRaWAN/App/lora_app.h"));
    char *mreg  = strip_comments(slurp("../../Core/Src/multiregion_context.c"));
    char *conf  = strip_comments(slurp("../../LoRaWAN/Target/lorawan_conf.h"));
    char *mregh = strip_comments(slurp("../../Core/Inc/multiregion_context.h"));
    char *msstate = strip_comments(slurp("../../Core/Src/mission_state.c"));
    /* se-identity.h is gitignored (real credentials) - scan the committed
     * TEMPLATE so CI/non-flashed checkouts pass too. */
    char *seid  = strip_comments(slurp("../../LoRaWAN/App/se-identity-template.h"));

    printf("=== SP-series review regressions (2026-08-13) ===\n\n");

    test_sp01_mid_acquisition_error_recovers();
    printf("\n");
    test_sp01_rearm_failure_disengages_cleanly();
    printf("\n");
    test_sp01_teardown_error_is_ignored();
    printf("\n");
    test_sp01_structural(usart, gnss, mainc);
    printf("\n");
    test_f01_structural(app, apph, mreg);
    printf("\n");
    test_sp05_structural(conf, app, mreg, mregh, msstate, seid);
    printf("\n");
    test_sp07_fatal_escape_wired(mainc);
    printf("\n");
    test_sp09_single_pin_owner(gnss, lpm);
    printf("\n");
    test_sp14_nan_guards(pend);
    printf("\n");
    test_sp15_unreachable_case_gone(app);
    printf("\n");
    test_sp16_honest_switch_log(app);

    free(usart); free(gnss); free(mainc); free(app); free(apph); free(mreg); free(lpm); free(pend);
    free(conf); free(mregh); free(msstate); free(seid);

    printf("\n%d checks, %d failures (%d expected pre-fix)\n",
           g_checks, g_failures, g_expected_failures);

    if (getenv("EXPECT_UNFIXED") && g_failures == g_expected_failures) {
        printf("BASELINE OK (all failures are known-unfixed findings)\n");
        return 0;
    }
    return g_failures ? 1 : 0;
}
