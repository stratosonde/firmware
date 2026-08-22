/* ARCHIVE — regression record for the 2026-08-13 LT review. Module contracts
 * live in test_<module>.c (see R2_TEST_MAP.md). Do not extend; extend the
 * contract suite for the owning module instead. (Refactor stage 7.) */

/**
 ******************************************************************************
 * @file    test_lt_20260813.c
 * @brief   Red-first regressions for the 2026-08-13 "LT" series review
 *          (long-term stability & predictability pass).
 ******************************************************************************
 *   LT-01 (CRITICAL): RescheduleScienceTimer's re-base guard lives inside
 *     `if (science_cycle)`, but the `delay_ms = g_science_due_ms - now_ms`
 *     subtraction that sets the period is OUTSIDE it. On the bulk-continuation
 *     path (science_cycle == false) the deadline can legally be in the past -
 *     the ~40-60 ms between SendTxData's entry ScienceIsDue() check and this
 *     call is enough (EnvSensors_Read runs the MS5607 OSR_4096 pair). The
 *     unsigned wrap yields delay_ms = 4294967246, which TIMER_IF_Convert_ms2Tick
 *     ALIASES (not saturates) into 103079163 ticks = 28.0 h of ReloadValue.
 *     UTIL_TIMER_Start has no upper bound. Nothing re-arms SendTxData, so the
 *     only escape is Deadman_Check at max(3 h, 3x survival): a 3-6 h blackout
 *     plus a reset that discards slope history and mode-hysteresis commit.
 *     BEHAVIOURAL (model gated by a source scan) + structural.
 *
 *     NOTE on DR-09: sys_app.c's HAL_GetTick audit note lists
 *     "RescheduleScienceTimer re-base guard" as absorbing one bad delta
 *     benignly. True for the guard and for the 4,194,304,000 ms tick-era wrap
 *     (the wrap makes ScienceIsDue() fire, routing to the guarded branch).
 *     It does not cover the unguarded branch's subtraction. Right function,
 *     wrong line - test_lt01_wrap_era_still_absorbed() pins both halves.
 *
 *   LT-06: NormalizeBatteryVoltage's compensation table is non-monotonic
 *     between -40 (700) and -50 (450) - the "Approximate" rows contradict the
 *     measured -55 row. Cooling at constant charge state DROPS the normalized
 *     voltage ~250 mV inside the float temperature band, manufacturing an
 *     approach to the 4500 mV critical threshold. R10 (#37) fixed the
 *     -50/-55 pair and left this one. BEHAVIOURAL against the real
 *     power_model.c.
 *
 *   LT-02/03/04/05/07/08/10/11: structural scans (HAL- or flow-bound).
 *
 *   Run:
 *   make -C tests/host lt0813         (red until the fixes land)
 *   EXPECT_UNFIXED=1 ./test_lt0813    (green pre-fix gate)
 ******************************************************************************
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "power_model.h"

static int g_failures = 0;
static int g_checks = 0;
static int g_expected_failures = 0;

#define CHECK(cond)                                          \
  do {                                                       \
    g_checks++;                                              \
    if (!(cond)) {                                           \
      g_failures++;                                          \
      printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
    }                                                        \
  } while (0)

#define CHECK_REGRESSION(cond, id)                                    \
  do {                                                                \
    g_checks++;                                                       \
    if (!(cond)) {                                                    \
      g_failures++;                                                   \
      g_expected_failures++;                                          \
      printf("FAIL [%s] %s:%d: %s\n", id, __FILE__, __LINE__, #cond); \
    }                                                                 \
  } while (0)

/* ========================================================================== */
/* Source-scan helpers (same shape as test_sp_20260813.c / test_dr_20260812.c) */
/* ========================================================================== */

/* Phase 3 (#263): the ASan gate runs the scan suites too. Scan buffers live
 * until process exit, so pool them and free once via atexit instead of
 * hand-freeing (or leaking) on individual return paths. */
#define SCAN_POOL_MAX 48
static void *g_scan_pool[SCAN_POOL_MAX];
static int g_scan_pool_n;
static void scan_pool_free(void) {
  while (g_scan_pool_n > 0)
    free(g_scan_pool[--g_scan_pool_n]);
}
static void *scan_pool_track(void *p) {
  if (p && g_scan_pool_n < SCAN_POOL_MAX) {
    if (g_scan_pool_n == 0)
      atexit(scan_pool_free);
    g_scan_pool[g_scan_pool_n++] = p;
  }
  return p;
}

static char *slurp(const char *path) {
  FILE *f = fopen(path, "rb");
  if (!f) {
    printf("FATAL: cannot open %s\n", path);
    exit(2);
  }
  fseek(f, 0, SEEK_END);
  long n = ftell(f);
  fseek(f, 0, SEEK_SET);
  char *buf = (char *)scan_pool_track(malloc((size_t)n + 1));
  if (!buf)
    exit(2);
  if (fread(buf, 1, (size_t)n, f) != (size_t)n)
    exit(2);
  buf[n] = '\0';
  fclose(f);
  /* Firmware sources are CRLF; scan anchors are written LF. */
  {
    size_t r = 0, w = 0;
    while (r < (size_t)n) {
      if (buf[r] != '\r')
        buf[w++] = buf[r];
      r++;
    }
    buf[w] = '\0';
  }
  return buf;
}

/* Strip block and line comments so a scan asserts on CODE, not prose.
 * Load-bearing here: every LT finding has a comment nearby that quotes the
 * very token the scan looks for. */
static char *strip_comments(const char *src) {
  size_t n = strlen(src);
  char *out = (char *)scan_pool_track(malloc(n + 1));
  size_t o = 0;
  for (size_t i = 0; i < n; i++) {
    if (src[i] == '/' && src[i + 1] == '*') {
      i += 2;
      while (i < n && !(src[i] == '*' && src[i + 1] == '/'))
        i++;
      i++;
      out[o++] = ' ';
    } else if (src[i] == '/' && src[i + 1] == '/') {
      while (i < n && src[i] != '\n')
        i++;
      out[o++] = '\n';
    } else {
      out[o++] = src[i];
    }
  }
  out[o] = '\0';
  return out;
}

static int count_occurrences(const char *hay, const char *needle) {
  int c = 0;
  const char *p = hay;
  size_t nl = strlen(needle);
  while ((p = strstr(p, needle)) != NULL) {
    c++;
    p += nl;
  }
  return c;
}

/* Body of the function starting at the first occurrence of `sig`, bounded by
 * real brace matching.
 *
 * NOTE: the sibling suites bound bodies with the next "\n}\n". That is fine for
 * short functions but SILENTLY TRUNCATES the long ones - SendTxData, OnTxData
 * and RunTxStateMachine all contain preprocessor-bracketed and nested blocks,
 * so a "\n}\n" scan of SendTxData stops long before the archive write and every
 * in_function() against it reads FALSE for the wrong reason. A scan that cannot
 * see the code it is asserting about is worse than no scan: it goes green on a
 * defect. String literals are skipped so a brace inside a log format cannot
 * unbalance the count; comments are already stripped by the caller. */
static char *function_body(const char *src, const char *sig) {
  /* lora_app.c declares SendTxData/OnTxData/OnRxData in its prototype block
   * before defining them. A bare strstr() lands on the DECLARATION, whose
   * next '{' belongs to some unrelated later construct - the body then looks
   * plausible and every assertion against it is meaningless. Take the first
   * occurrence whose parameter list is followed by '{' and not ';'. */
  const char *start = NULL;
  const char *p = NULL;
  for (const char *cand = strstr(src, sig); cand != NULL;
       cand = strstr(cand + 1, sig)) {
    const char *q = cand + strlen(sig);
    while (*q != '\0' && *q != '{' && *q != ';')
      q++;
    if (*q == '{') {
      start = cand;
      p = q;
      break;
    }
  }
  if (!start || !p)
    return NULL;

  int depth = 0;
  const char *end = NULL;
  for (; *p != '\0'; p++) {
    if (*p == '"' || *p == '\'') {
      char q = *p;
      p++;
      while (*p != '\0' && *p != q) {
        if (*p == '\\' && p[1] != '\0')
          p++;
        p++;
      }
      if (*p == '\0')
        break;
      continue;
    }
    if (*p == '{')
      depth++;
    else if (*p == '}') {
      depth--;
      if (depth == 0) {
        end = p + 1;
        break;
      }
    }
  }
  if (!end)
    end = src + strlen(src);

  size_t len = (size_t)(end - start);
  char *body = (char *)malloc(len + 1);
  memcpy(body, start, len);
  body[len] = '\0';
  return body;
}

static bool in_function(const char *src, const char *sig, const char *needle) {
  char *body = function_body(src, sig);
  if (!body)
    return false;
  bool found = strstr(body, needle) != NULL;
  free(body);
  return found;
}

static int count_in_function(const char *src, const char *sig, const char *needle) {
  char *body = function_body(src, sig);
  if (!body)
    return -1;
  int c = count_occurrences(body, needle);
  free(body);
  return c;
}

/* ========================================================================== */
/* LT-01 - BEHAVIOURAL: the science timer period can never exceed one interval */
/* ========================================================================== */

/* Stage 5.4b: the wrapper kept its home in lora_app.c (one signature change:
 * the science_cycle flag is now derived from the FSM state inside the pure
 * step) and the deadline arithmetic moved to TxFsm_Reschedule in
 * Core/Src/tx_fsm.c. */
#define LT_RESCHED_SIG "static void RescheduleScienceTimer(uint32_t interval_ms)"

/* Core/Src/timer_if.c: ret = ((uint32_t)((((uint64_t) timeMilliSec) << RTC_N_PREDIV_S) / 1000));
 * The uint32_t cast ALIASES rather than saturates - that is what turns the
 * wrapped delay into a plausible-looking 28 h instead of an obvious overflow. */
#define LT_RTC_N_PREDIV_S 10
static uint32_t model_ms2tick(uint32_t ms) {
  return (uint32_t)((((uint64_t)ms) << LT_RTC_N_PREDIV_S) / 1000U);
}

/* Model of RescheduleScienceTimer's period computation, transcribed from
 * lora_app.c. `clamped` selects pre-fix vs post-fix shape; which one runs is
 * decided by a scan of the real source below (the test_burst_fsm.c pattern),
 * so this cannot go green without a firmware change. */
static uint32_t model_reschedule_period_ms(uint32_t *due, uint32_t now,
                                           uint32_t interval, bool science_cycle,
                                           bool clamped) {
  uint32_t delay_ms;

  if (science_cycle) {
    if (*due == 0U)
      *due = now; /* establish phase */
    *due += interval;
    if ((int32_t)(*due - now) <= 0) {
      *due = now + interval; /* fell >= 1 period behind */
    }
  } else if (*due == 0U) {
    *due = now + interval;
  }

  if (clamped) {
    /* Proposed fix: overdue -> fire immediately, never wrap. */
    int32_t remain = (int32_t)(*due - now);
    if (remain <= 0)
      remain = 1;
    delay_ms = (uint32_t)remain;
  } else {
    delay_ms = *due - now; /* HEAD: unsigned, unguarded */
    if (delay_ms == 0U)
      delay_ms = 1U;
  }
  return delay_ms;
}

/* The fix is present when the bare unsigned subtraction is gone and the
 * signed clamp is present. Stage 5.4b: the arithmetic lives in
 * TxFsm_Reschedule (Core/Src/tx_fsm.c). */
static bool lt01_fix_present(const char *txfsm) {
  bool bare_subtraction_gone =
      !in_function(txfsm, "uint32_t TxFsm_Reschedule(TxFsm_t *fsm, uint32_t now_ms, uint32_t interval_ms)",
                   "uint32_t delay_ms = fsm->science_due_ms - now_ms;");
  bool signed_clamp_present =
      (strstr(txfsm, "if (remain_ms <= 0)\n    remain_ms = 1;") != NULL);
  return bare_subtraction_gone && signed_clamp_present;
}

static void test_lt01_bulk_continuation_never_overshoots(bool fixed) {
  printf("LT-01: bulk continuation with a just-missed deadline\n");

  const uint32_t interval = 300000U; /* FLOAT science cadence, 5 min */
  const uint32_t now = 1000000U;

  /* The deadline crossed 50 ms ago - inside the EnvSensors_Read window
   * between SendTxData's ScienceIsDue() check and this call. State is still
   * TX_STATE_BULK_TRANSFER, so science_cycle is false. */
  uint32_t due = now - 50U;
  uint32_t period = model_reschedule_period_ms(&due, now, interval, false, fixed);
  uint32_t ticks = model_ms2tick(period);

  printf("  period=%u ms (%.1f h of timer), ReloadValue=%u ticks\n",
         period, (double)ticks / 1024.0 / 3600.0, ticks);

  CHECK_REGRESSION(period >= 1U, "LT-01-period-nonzero");
  CHECK_REGRESSION(period <= interval, "LT-01-period-bounded");
  /* The aliased ReloadValue: 103079163 ticks = 28.0 h. Deadman rescues at
   * 3-6 h, so this is a multi-hour blackout plus a reset, not a hang. */
  CHECK_REGRESSION(ticks <= model_ms2tick(interval), "LT-01-reload-bounded");
}

static void test_lt01_sweep_deadline_past_and_future(bool fixed) {
  printf("LT-01: sweep - deadline from +1 interval to -1 interval\n");

  const uint32_t interval = 300000U;
  const uint32_t now = 4000000U;
  int violations = 0;

  /* Every offset a bulk continuation can legally observe. */
  for (int32_t offset = (int32_t)interval; offset >= -(int32_t)interval; offset -= 977) {
    uint32_t due = (uint32_t)((int32_t)now + offset);
    if (due == 0U)
      continue; /* 0 means "unscheduled" */
    uint32_t period = model_reschedule_period_ms(&due, now, interval, false, fixed);
    if (period < 1U || period > interval)
      violations++;
  }

  printf("  %d offsets out of bounds\n", violations);
  CHECK_REGRESSION(violations == 0, "LT-01-sweep");
}

static void test_lt01_science_path_still_phase_preserving(bool fixed) {
  printf("LT-01: R3-01 phase preservation must survive the fix\n");

  const uint32_t interval = 300000U;
  uint32_t due = 0U;

  /* Cycle 1 establishes phase. */
  uint32_t now = 100000U;
  (void)model_reschedule_period_ms(&due, now, interval, true, fixed);
  CHECK(due == now + interval);

  /* Cycle 2 arrives 1 s late: the deadline advances by exactly one interval
   * (phase-preserving), it does NOT re-base to now + interval. */
  now = due + 1000U;
  uint32_t before = due;
  uint32_t period = model_reschedule_period_ms(&due, now, interval, true, fixed);
  CHECK(due == before + interval);
  CHECK(period == interval - 1000U);

  /* Cycle 3 arrives a FULL period late: re-base, never a catch-up storm. */
  now = due + interval + 5000U;
  (void)model_reschedule_period_ms(&due, now, interval, true, fixed);
  CHECK(due == now + interval);

  /* A bulk continuation must never move the deadline. */
  before = due;
  (void)model_reschedule_period_ms(&due, now + 100U, interval, false, fixed);
  CHECK(due == before);
}

static void test_lt01_wrap_era_still_absorbed(bool fixed) {
  printf("LT-01 / DR-09: the 4,194,304,000 ms tick-era wrap stays benign\n");

  /* DR-09: HAL_GetTick wraps at 4,194,304,000 ms, not 2^32. After the wrap
   * `now` is small while `due` is near the old ceiling. ScienceIsDue()'s
   * (int32_t)(now - due) >= 0 reads TRUE there, so SendTxData yields the
   * burst and this call takes the GUARDED science branch. Pin both halves:
   * the wrap routes to science_cycle == true, and the guard re-bases. */
  const uint32_t interval = 300000U;
  const uint32_t tick_era = 4194304000U;
  uint32_t due = tick_era - 10000U;
  uint32_t now = 0U; /* just wrapped */

  bool science_is_due = ((int32_t)(now - due) >= 0);
  CHECK(science_is_due); /* the yield fires */

  uint32_t period = model_reschedule_period_ms(&due, now, interval, true, fixed);
  CHECK(due == now + interval); /* re-based, not wrapped */
  CHECK(period == interval);
}

static void test_lt01_structural(const char *app, const char *txfsm) {
  printf("LT-01: structural - the unguarded subtraction must be gone\n");

  /* The timer is still programmed in exactly one place (the wrapper). */
  CHECK(count_in_function(app, LT_RESCHED_SIG, "UTIL_TIMER_SetPeriod") == 1);

  CHECK_REGRESSION(
      !in_function(txfsm, "uint32_t TxFsm_Reschedule(TxFsm_t *fsm, uint32_t now_ms, uint32_t interval_ms)",
                   "uint32_t delay_ms = fsm->science_due_ms - now_ms;"),
      "LT-01-unguarded-subtraction");
  CHECK_REGRESSION(
      strstr(txfsm, "if (remain_ms <= 0)\n    remain_ms = 1;") != NULL,
      "LT-01-signed-clamp-present");
}

/* ========================================================================== */
/* LT-06 - BEHAVIOURAL: cold compensation must not fall as temperature falls   */
/* ========================================================================== */

static void test_lt06_compensation_monotonic(void) {
  printf("LT-06: NormalizeBatteryVoltage monotonicity across the float band\n");

  /* At a constant charge state, a COLDER cell may need more compensation but
   * never less. Any dip means cooling alone manufactures a lower normalized
   * voltage - and normalized voltage is what feeds
   * PredictTimeToLowerThreshold(..., 4500, ...) and SelectModeFromPredictions. */
  const uint16_t raw = 5000U;
  uint16_t prev = NormalizeBatteryVoltage(raw, -25.0f);
  int dips = 0;
  float worst_t = 0.0f;
  int32_t worst_drop = 0;

  for (float t = -26.0f; t >= -66.0f; t -= 1.0f) {
    uint16_t v = NormalizeBatteryVoltage(raw, t);
    int32_t d = (int32_t)v - (int32_t)prev;
    if (d < 0) {
      dips++;
      if (d < worst_drop) {
        worst_drop = d;
        worst_t = t;
      }
    }
    prev = v;
  }

  printf("  %d dips; worst %d mV at %.0f C\n", dips, worst_drop, (double)worst_t);
  printf("  -40C=%u  -45C=%u  -50C=%u  -55C=%u  -56C=%u\n",
         NormalizeBatteryVoltage(raw, -40.0f), NormalizeBatteryVoltage(raw, -45.0f),
         NormalizeBatteryVoltage(raw, -50.0f), NormalizeBatteryVoltage(raw, -55.0f),
         NormalizeBatteryVoltage(raw, -56.0f));

  CHECK_REGRESSION(dips == 0, "LT-06-monotonic");

  /* The specific inversion R10 (#37) left behind: -40 (+700) -> -55 (+430). */
  CHECK_REGRESSION(NormalizeBatteryVoltage(raw, -55.0f) >=
                       NormalizeBatteryVoltage(raw, -40.0f),
                   "LT-06-m40-vs-m55");
}

/* ========================================================================== */
/* LT-02 - failed region restore must roll back, and must not lie about it     */
/* ========================================================================== */

static void test_lt02_restore_rolls_back(const char *mreg, const char *app) {
  printf("LT-02: region-switch teardown must be undone on failure\n");

  /* RestoreSessionToMac step 1 calls LoRaApp_ReInitStack(), which DeInits the
   * stack AND sets LmHandlerParams.ActiveRegion = new_region. Every failure
   * after that returns ERROR with the MAC sessionless and the app's two views
   * of "active region" disagreeing (g_storage.active_slot = old,
   * LmHandlerParams.ActiveRegion = new). DatarateFromSF resolves against the
   * new one; every policy gate reads the old one. */
  CHECK(in_function(mreg, "static LmHandlerErrorStatus_t RestoreSessionToMac",
                    "LoRaApp_ReInitStack") == true);

  CHECK_REGRESSION(
      in_function(mreg, "static LmHandlerErrorStatus_t RestoreSessionToMac",
                  "rollback") ||
          in_function(mreg, "LmHandlerErrorStatus_t MultiRegion_SwitchToRegion",
                      "rollback"),
      "LT-02-rollback-exists");

  /* The log line must be conditional on the old session actually still being
   * loaded - at HEAD it is printed unconditionally on the ERROR branch. */
  CHECK_REGRESSION(
      !in_function(app, "static void SelectRegionAndSession",
                   "Switch FAILED (restore error) - staying on current region"),
      "LT-02-honest-log");
}

/* ========================================================================== */
/* LT-03 - archive must pair environment and position from the same moment     */
/* ========================================================================== */

static void test_lt03_env_resampled_after_fix(const char *app) {
  printf("LT-03: environment must be re-sampled after GNSS acquisition\n");

  char *body = function_body(app, "static void SendTxData(void)");
  CHECK(body != NULL);
  if (!body)
    return;

  /* FR-15 (#96) removed the post-fix sensor re-read on the grounds it added
   * "no new information". With gps_timeout up to 255 s that is ~300 m of
   * ascent between the barometric and GNSS halves of one record. Require an
   * environment sample AFTER the acquisition block. */
  const char *merge = strstr(body, "EnvSensors_MergeGnss(&sensor_data);");
  const char *archive = strstr(body, "ArchiveSample(&sensor_data");
  CHECK(merge != NULL && archive != NULL);

  bool resample_after_fix = false;
  if (merge && archive && archive > merge) {
    /* Any environment read between the merge and the archive write. */
    char *window = (char *)malloc((size_t)(archive - merge) + 1);
    memcpy(window, merge, (size_t)(archive - merge));
    window[archive - merge] = '\0';
    resample_after_fix = (strstr(window, "EnvSensors_Read(") != NULL) ||
                         (strstr(window, "EnvSensors_ReadEnv(") != NULL);
    free(window);
  }
  CHECK_REGRESSION(resample_after_fix, "LT-03-env-resample");

  free(body);
}

/* ========================================================================== */
/* LT-04 - GNSS wake failure paths must return the pins to sleep-safe          */
/* ========================================================================== */

static void test_lt04_wake_failure_teardown(const char *gnss) {
  printf("LT-04: GNSS_WakeFromStandby failure paths must not leave PB6 high\n");

  /* SP-09 (#249): PB6 (MCU TX -> GPS RX) must be OUTPUT-LOW whenever the
   * module is depowered - AF push-pull idles HIGH and leaks through the
   * depowered module's ESD clamp. R9 (#194) gave GNSS_PowerOn transactional
   * cleanup; the FLIGHT path (WakeFromStandby) never got it. */
  CHECK(count_occurrences(gnss, "GNSS_UARTPins_SleepSafe") >= 1);

  CHECK_REGRESSION(
      in_function(gnss, "GNSS_StatusTypeDef GNSS_WakeFromStandby",
                  "GNSS_UARTPins_SleepSafe") ||
          in_function(gnss, "GNSS_StatusTypeDef GNSS_WakeFromStandby",
                      "GNSS_TeardownToOff"),
      "LT-04-wake-teardown");

  CHECK_REGRESSION(
      in_function(gnss, "GNSS_StatusTypeDef GNSS_PowerOn",
                  "GNSS_UARTPins_SleepSafe") ||
          in_function(gnss, "GNSS_StatusTypeDef GNSS_PowerOn",
                      "GNSS_TeardownToOff"),
      "LT-04-poweron-teardown");
}

/* ========================================================================== */
/* LT-05 - STOP2 wake re-init failures must mark capabilities                  */
/* ========================================================================== */

static void test_lt05_wake_reinit_marks_caps(const char *lpm, const char *mainc) {
  printf("LT-05: STOP2 re-init failures must reach the capability mask\n");

  /* SP-17 (#255) established the rule on the boot path (runs once). The wake
   * path runs thousands of times per mission and only logs - and in a flight
   * build SONDE_LOG_STR compiles away, so a persistently dead I2C2 or a flash
   * chip that stops waking from deep power-down is completely invisible. */
  CHECK(count_in_function(mainc, "static void MX_I2C2_Init(void)",
                          "SysCaps_MarkFailed") >= 1);

  CHECK_REGRESSION(
      count_in_function(lpm, "void PWR_ExitStopMode(void)", "SysCaps_MarkFailed") >= 1,
      "LT-05-wake-marks-caps");

  /* stop2_reinit_fail_count is written and never read - it is a log-only
   * counter standing in for real degraded-state signalling. */
  CHECK_REGRESSION(
      count_occurrences(lpm, "stop2_reinit_fail_count") == 0 ||
          in_function(lpm, "void PWR_ExitStopMode(void)", "SysCaps_MarkFailed"),
      "LT-05-counter-not-a-signal");
}

/* ========================================================================== */
/* LT-07 - the burst FSM needs a timeout of its own                            */
/* ========================================================================== */

static void test_lt07_burst_has_own_timeout(const char *app) {
  printf("LT-07: burst must not depend on the science deadline to unstick\n");

  /* First archive packet is UNCONFIRMED with a piggybacked LinkCheckReq. If
   * the network answers nothing, OnRxData never runs, OnTxData declines
   * (BURST-01, correctly), and g_tx_state parks in BULK_TRANSFER with nothing
   * scheduled. The only escape is SendTxData's entry ScienceIsDue() yield -
   * accidental, not designed. And when TxTimer fires EARLY (OnTxTimerEvent
   * re-arms with a stale partial ReloadValue) the yield does not fire, so
   * AcquireGnssFix and ArchiveSample are both skipped: one science record
   * silently lost. */
  CHECK(in_function(app, "static void OnTxData(LmHandlerTxParams_t *params)",
                    "FlashLog_DeferHeaderSync"));

  CHECK_REGRESSION(
      count_occurrences(app, "g_burst_opened_ms") >= 2 ||
          count_occurrences(app, "BURST_MAX_OPEN_MS") >= 1,
      "LT-07-burst-age-bound");

  /* The redundant re-arm is what makes the early fire routine: the period is
   * owned by RescheduleScienceTimer. */
  CHECK_REGRESSION(
      !in_function(app, "static void OnTxTimerEvent(void *context)",
                   "UTIL_TIMER_Start(&TxTimer);"),
      "LT-07-no-blind-rearm");
}

/* ========================================================================== */
/* LT-08 - forcing gps_enabled without a budget is a no-op (S-A #211 pattern)  */
/* ========================================================================== */

static void test_lt08_ascent_gps_budget(const char *app) {
  printf("LT-08: ASCENT override must guarantee a non-zero GPS budget\n");

  char *body = function_body(app, "static void SendTxData(void)");
  CHECK(body != NULL);
  if (!body)
    return;

  /* S-A (#211): with gps_timeout_ms == 0 AcquireGnssFix powers the receiver
   * up, runs the acquisition loop zero times, takes the stale branch and
   * powers it down. The GPS-loss path got an explicit clamp; the ASCENT
   * override did not. Safe today only because power_mode <= MODE_CONSERVATIVE
   * happens to select the two modes ApplyOperatingMode gives a 60 s budget -
   * a coincidence between two files, not an invariant. */
  const char *asc = strstr(body, "MISSION_ASCENT_TX_INTERVAL_MS;");
  bool clamped = false;
  if (asc) {
    const char *blk_end = strstr(asc, "MISSION_FLOAT_TX_INTERVAL_MS");
    if (!blk_end)
      blk_end = asc + strlen(asc);
    char *win = (char *)malloc((size_t)(blk_end - asc) + 1);
    memcpy(win, asc, (size_t)(blk_end - asc));
    win[blk_end - asc] = '\0';
    clamped = (strstr(win, "gps_timeout_ms") != NULL);
    free(win);
  }
  CHECK(asc != NULL);
  CHECK_REGRESSION(clamped, "LT-08-ascent-gps-budget");

  free(body);
}

/* ========================================================================== */
/* LT-10 - vertical speed must not update after a REJECTED GGA                 */
/* ========================================================================== */

static void test_lt10_vspeed_gated_on_parse(const char *gnss) {
  printf("LT-10: GNSS_UpdateVerticalSpeed must be gated on ParseGGA success\n");

  /* DR-01 (#236) made ParseGGA leave hgnss->data untouched on rejection.
   * UpdateVerticalSpeed did not learn about it: it still runs, publishes
   * 0 m/s (dalt over an unchanged altitude), and advances prev_timestamp -
   * destroying the base the next GOOD sentence would difference against.
   * Contained today (vertical_speed_ms is only read by the gated
   * ENABLE_GNSS_DETAIL_PACKET encoder), live the day it feeds burst or float
   * detection. The return value already exists and is already correct. */
  /* Anchor on the FIXED form, not on exact adjacency of the broken one:
   * strip_comments collapses the comment between the two calls, so an
   * adjacency match is brittle and would pass for the wrong reason. */
  CHECK(in_function(gnss, "GNSS_StatusTypeDef GNSS_ParseNMEA",
                    "GNSS_UpdateVerticalSpeed(hgnss)"));
  CHECK_REGRESSION(
      in_function(gnss, "GNSS_StatusTypeDef GNSS_ParseNMEA", "if (GNSS_ParseGGA(") ||
          in_function(gnss, "GNSS_StatusTypeDef GNSS_ParseNMEA", "== 0) {\n      GNSS_UpdateVerticalSpeed"),
      "LT-10-vspeed-gate");
}

/* ========================================================================== */
/* LT-11 - the dead dual-domain now_timestamp parameter                        */
/* ========================================================================== */

static void test_lt11_dead_timestamp_param(const char *app) {
  printf("LT-11: RunTxStateMachine's now_timestamp is dead and dual-domain\n");

  /* SendTxData sets it from TIMER_IF_GetTime (boot-relative seconds).
   * ArchiveSample overwrites it with SysTimeGet().Seconds (UTC epoch) - but
   * only on non-bulk cycles. RunTxStateMachine receives one of two
   * incompatible clock domains depending on control flow, and reads neither
   * (SP-11/SP-12 removed the last consumer). This is the exact conflation
   * F-1 (#176), R45 and SP-11/12 each eliminated; the last carrier is inert
   * but still in the signature. */
  CHECK_REGRESSION(
      !in_function(app, "static void RunTxStateMachine(const sensor_t *sensor_data",
                   "now_timestamp"),
      "LT-11-dead-param");
}

/* ========================================================================== */
/* C-01 - the one-way flight door must be gated on a PROVISIONED latch         */
/* ========================================================================== */

static void test_c01_flight_door_gated(const char *mstate, const char *mregh) {
  printf("C-01: launch detection must not open the flight door unprovisioned\n");

  /* MissionState_Update's commissioning branch called the one-way
   * MissionState_EnterFlight() on pressure departure alone, while joins are
   * commissioning-only (DDR-0018 INV-COMM-001): a virgin or partially
   * provisioned unit latching ASCENT could never join - an archive-only
   * mission. The door must be gated on the durable Tier-1 PROVISIONED
   * latch, and EnterFlight must not be reachable before that gate. */
  char *body = function_body(mstate, "void MissionState_Update(");
  CHECK(body != NULL);
  bool gated = false;
  if (body) {
    const char *gate = strstr(body, "MultiRegion_IsProvisioningComplete");
    const char *door = strstr(body, "MissionState_EnterFlight();");
    gated = (gate != NULL) && (door != NULL) && (gate < door);
    free(body);
  }
  CHECK_REGRESSION(gated, "C-01-door-gate");

  /* The latch lives in the Tier-1 bank: persisted-format bump v4 -> v5
   * (v4 banks mismatch-reject and re-commission on the bench, same policy
   * as the v3 -> v4 bump in #246). */
  CHECK_REGRESSION(strstr(mregh, "#define MULTIREGION_VERSION 5") != NULL,
                   "C-01-version-5");

  /* H-11 (#287): GNSS-accepted latch is the third door gate. */
  char *ms_body_h11 = function_body(mstate, "void MissionState_Update(");
  CHECK(ms_body_h11 != NULL);
  if (ms_body_h11) {
    const char *g = strstr(ms_body_h11, "MissionState_GnssAccepted");
    const char *d = strstr(ms_body_h11, "MissionState_EnterFlight();");
    CHECK_REGRESSION(g != NULL && d != NULL && g < d, "H-11-gnss-gate");
    free(ms_body_h11);
  }

  /* SP-06 (#256): launch-detector input is a 3-sample mean inside the
   * mission_state.c wrapper, so a single spike cannot false-arm. */
  char *ms_body2 = function_body(mstate, "void MissionState_Update(");
  CHECK(ms_body2 != NULL);
  if (ms_body2) {
    const char *w = strstr(ms_body2, "window[");
    CHECK_REGRESSION(w != NULL, "SP-06-window");
    free(ms_body2);
  }
}

/* ========================================================================== */
/* H-02 - Config_Init must run before MX_LoRaWAN_Init                          */
/* ========================================================================== */

static void test_h02_config_before_lorawan_init(const char *mainc) {
  printf("H-02: Config_Init must run before MX_LoRaWAN_Init\n");

  /* The frame-counter restore inside MultiRegion_Init (called from
   * MX_LoRaWAN_Init) reads CfgFrameCounterSaveInterval(), which falls back
   * to the macro default (10) while Config_Get() is NULL. With a configured
   * interval != 10 the restore margin and the save cadence disagree -
   * counter replay/rollback, the DR-07/#239 defect class via init order. */
  const char *cfg = strstr(mainc, "Config_Init();");
  const char *lw = strstr(mainc, "MX_LoRaWAN_Init();");
  CHECK(cfg != NULL && lw != NULL);
  CHECK_REGRESSION(cfg != NULL && lw != NULL && cfg < lw, "H-02-config-first");
}

/* ========================================================================== */

int main(void) {
  char *app = strip_comments(slurp("../../LoRaWAN/App/lora_app.c"));
  char *gnss = strip_comments(slurp("../../Core/Src/atgm336h.c"));
  char *mreg = strip_comments(slurp("../../Core/Src/multiregion_context.c"));
  char *lpm = strip_comments(slurp("../../Core/Src/stm32_lpm_if.c"));
  char *mainc = strip_comments(slurp("../../Core/Src/main.c"));
  char *mstate = strip_comments(slurp("../../Core/Src/mission_state.c"));
  char *mregh = strip_comments(slurp("../../Core/Inc/multiregion_context.h"));
  char *txfsm = strip_comments(slurp("../../Core/Src/tx_fsm.c"));

  printf("=== LT-series review regressions (2026-08-13) ===\n\n");

  /* The behavioural LT-01 model runs in the shape the REAL source has, so it
   * cannot go green by editing the test (test_burst_fsm.c precedent). */
  bool fixed = lt01_fix_present(txfsm);
  printf("LT-01 fix detected in lora_app.c: %s\n\n", fixed ? "YES" : "NO");

  test_lt01_bulk_continuation_never_overshoots(fixed);
  printf("\n");
  test_lt01_sweep_deadline_past_and_future(fixed);
  printf("\n");
  test_lt01_science_path_still_phase_preserving(fixed);
  printf("\n");
  test_lt01_wrap_era_still_absorbed(fixed);
  printf("\n");
  test_lt01_structural(app, txfsm);
  printf("\n");
  test_lt06_compensation_monotonic();
  printf("\n");
  test_lt02_restore_rolls_back(mreg, app);
  printf("\n");
  test_lt03_env_resampled_after_fix(app);
  printf("\n");
  test_lt04_wake_failure_teardown(gnss);
  printf("\n");
  test_lt05_wake_reinit_marks_caps(lpm, mainc);
  printf("\n");
  test_lt07_burst_has_own_timeout(app);
  printf("\n");
  test_lt08_ascent_gps_budget(app);
  printf("\n");
  test_lt10_vspeed_gated_on_parse(gnss);
  printf("\n");
  test_lt11_dead_timestamp_param(app);
  printf("\n");
  test_c01_flight_door_gated(mstate, mregh);
  printf("\n");
  test_h02_config_before_lorawan_init(mainc);

  /* pooled scan buffers: freed at exit (scan_pool_track) */
  /* pooled scan buffers: freed at exit (scan_pool_track) */

  printf("\n%d checks, %d failures (%d expected pre-fix)\n",
         g_checks, g_failures, g_expected_failures);

  if (getenv("EXPECT_UNFIXED") && g_failures == g_expected_failures) {
    printf("BASELINE OK (all failures are known-unfixed findings)\n");
    return 0;
  }
  return g_failures ? 1 : 0;
}
