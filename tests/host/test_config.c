/**
  ******************************************************************************
  * @file    test_config.c
  * @brief   Behavioural regressions for the REAL config.c (S-04, S-07)
  ******************************************************************************
  * S-04 (P2, #228): the deadman timeout must be DERIVED from the configured
  * survival cadence (max(3 h, 3x survival)), not fixed at 3 h while the
  * validator accepts a 2 h cadence (1.5x margin).
  * S-07 (P3, #229): gps_timeout_normal / gps_timeout_conservative must be
  * range-validated.
  *
  * The REAL config.c is #included with a minimal stub flash (reads fail ->
  * defaults path). g_config is a same-TU static, so tests poke it directly.
  ******************************************************************************
  */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "stm32wlxx_hal.h"
#include "flash_if.h"

/* Minimal stub flash: reads fail (defaults path), writes/erases succeed. */
FLASH_IF_StatusTypedef FLASH_IF_Init(void *p) { (void)p; return FLASH_IF_OK; }
FLASH_IF_StatusTypedef FLASH_IF_DeInit(void) { return FLASH_IF_OK; }
FLASH_IF_StatusTypedef FLASH_IF_Read(void *d, const void *s, uint32_t n)
{ (void)d; (void)s; (void)n; return FLASH_IF_ERROR; }
FLASH_IF_StatusTypedef FLASH_IF_Write(void *d, const void *s, uint32_t n)
{ (void)d; (void)s; (void)n; return FLASH_IF_OK; }
FLASH_IF_StatusTypedef FLASH_IF_Erase(void *s, uint32_t n)
{ (void)s; (void)n; return FLASH_IF_OK; }

#include "../../Core/Src/config.c"

/* Config_Validate CRC-checks the struct - re-stamp after every poke. */
static void restamp(void)
{
    SystemConfig_t t = g_config;
    t.crc32 = 0;
    g_config.crc32 = Config_CRC32((const uint8_t *)&t, sizeof(SystemConfig_t));
}

static int g_checks = 0, g_failures = 0, g_expected_failures = 0;

#define CHECK(cond) do { \
    g_checks++; \
    if (!(cond)) { g_failures++; printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); } \
} while (0)

#define CHECK_REGRESSION(cond, id) do { \
    g_checks++; \
    if (!(cond)) { \
        g_failures++; g_expected_failures++; \
        printf("FAIL [%s] %s:%d: %s\n", id, __FILE__, __LINE__, #cond); \
    } \
} while (0)

int main(void)
{
    printf("=== config.c behavioural regressions (S-04, S-07) ===\n\n");

    CHECK(Config_Init() == CONFIG_OK);

    /* -- S-04 (#228): deadman timeout derived from survival cadence -------- */
    printf("-- S-04 (P2, #228): deadman timeout derived from survival cadence\n");

    /* Defaults (survival = 1 h): the 3 h floor holds. */
    printf("   defaults: timeout=%lu s (want %u)\n",
           (unsigned long)ConfigGetDeadmanTimeoutS(), 3U * 3600U);
    CHECK_REGRESSION(ConfigGetDeadmanTimeoutS() == 3U * 3600U, "S-04-floor");

    /* 2 h survival (the validator's old ceiling): timeout must become 6 h,
     * preserving the 3x margin. */
    g_config.tx_interval_survival = 7200000U;
    restamp();
    printf("   survival=2h: timeout=%lu s (want %u)\n",
           (unsigned long)ConfigGetDeadmanTimeoutS(), 6U * 3600U);
    CHECK_REGRESSION(ConfigGetDeadmanTimeoutS() == 6U * 3600U, "S-04-derive");

    /* Validator: 2 h survival is ACCEPTABLE now (the margin comes from the
     * derivation, not from a fixed ceiling). */
    CHECK(Config_Validate(&g_config) == CONFIG_OK);

    /* Absurd survival (24 h) is still rejected (would strand the mission). */
    g_config.tx_interval_survival = 86400000U;
    restamp();
    CHECK(Config_Validate(&g_config) == CONFIG_ERROR_RANGE);
    g_config.tx_interval_survival = 3600000U;   /* restore */
    restamp();

    /* -- S-07 (#229): gps_timeout range validation ------------------------- */
    printf("\n-- S-07 (P3, #229): gps_timeout_normal/_conservative validated\n");

    /* NOTE: the fields are uint8_t (0-255 s), so the enforceable hazard is
     * the LOW bound - an acquisition bound too short to ever succeed. */
    g_config.gps_timeout_normal = 5;    /* absurdly short acquisition bound */
    restamp();
    CHECK_REGRESSION(Config_Validate(&g_config) == CONFIG_ERROR_RANGE, "S-07-low");
    g_config.gps_timeout_normal = 60;
    restamp();

    g_config.gps_timeout_conservative = 0;    /* zero: GPS never attempted */
    restamp();
    CHECK_REGRESSION(Config_Validate(&g_config) == CONFIG_ERROR_RANGE, "S-07-zero");
    g_config.gps_timeout_conservative = 60;
    restamp();

    g_config.gps_timeout_normal = 10;   /* boundary: shortest sane bound */
    restamp();
    CHECK(Config_Validate(&g_config) == CONFIG_OK);
    g_config.gps_timeout_normal = 60;
    restamp();

    CHECK(Config_Validate(&g_config) == CONFIG_OK);

    printf("\n%d checks, %d failures (%d expected pre-fix)\n",
           g_checks, g_failures, g_expected_failures);

    if (getenv("EXPECT_UNFIXED") && g_failures == g_expected_failures) {
        printf("BASELINE OK (all failures are known-unfixed findings)\n");
        return 0;
    }
    return g_failures ? 1 : 0;
}
