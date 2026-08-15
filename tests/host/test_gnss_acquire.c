/**
  ******************************************************************************
  * @file    test_gnss_acquire.c
  * @brief   Module contract suite for Core/Src/gnss_acquire.c (refactor stage 4)
  ******************************************************************************
  * CONTRACT suite, not a findings archive. The civil-date algorithm underpins
  * every timestamp in the archive, so it is verified EXHAUSTIVELY: every day
  * from 1970-01-01 to 2100-01-01 against an independent brute-force
  * reference (leap-rule day counter, not the era arithmetic under test).
  *
  * Also covered: DDMMYY/HHMMSS epoch composition (round-trip vs the date
  * function), the S-A (#211) iteration-budget contract (strictly positive
  * for every nonzero timeout; zero only for the explicit non-acquisition
  * path), and the first-flight package-completeness truth table (GGA
  * position alone is never ACCEPT; RMC time alone is never ACCEPT).
  ******************************************************************************
  */

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include "gnss_acquire.h"

static int g_failures = 0;
static int g_checks = 0;

#define CHECK(cond) do { \
    g_checks++; \
    if (!(cond)) { \
        g_failures++; \
        printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
    } \
} while (0)

static bool ref_leap(int y)
{
    return (y % 4 == 0 && y % 100 != 0) || (y % 400 == 0);
}

/* Independent reference: count days forward from 1970-01-01. */
static uint32_t ref_days_from_civil(int y, unsigned m, unsigned d)
{
    static const unsigned mdays[12] = { 31,28,31,30,31,30,31,31,30,31,30,31 };
    uint32_t days = 0;
    for (int yy = 1970; yy < y; yy++) days += ref_leap(yy) ? 366U : 365U;
    for (unsigned mm = 1; mm < m; mm++) {
        days += mdays[mm - 1];
        if (mm == 2 && ref_leap(y)) days += 1U;
    }
    return days + d - 1U;
}

int main(void)
{
    /* ---- known anchors ---- */
    CHECK(GnssAcquire_DaysFromCivil(1970, 1, 1) == 0U);
    CHECK(GnssAcquire_DaysFromCivil(1970, 1, 2) == 1U);
    CHECK(GnssAcquire_DaysFromCivil(2000, 2, 29) == 11016U);   /* Y2K leap day */
    CHECK(GnssAcquire_DaysFromCivil(2024, 2, 29) == 19782U);   /* recent leap day */
    CHECK(GnssAcquire_DaysFromCivil(2100, 3, 1) == 47541U);    /* 2100 is NOT leap */

    /* ---- exhaustive: every civil day 1970-01-01 .. 2100-01-01 ---- */
    {
        static const unsigned mdays[12] = { 31,28,31,30,31,30,31,31,30,31,30,31 };
        uint32_t mismatches = 0;
        for (int y = 1970; y <= 2100; y++) {
            for (unsigned m = 1; m <= 12; m++) {
                unsigned dim = mdays[m - 1] + ((m == 2 && ref_leap(y)) ? 1U : 0U);
                for (unsigned d = 1; d <= dim; d++) {
                    if (GnssAcquire_DaysFromCivil(y, m, d) != ref_days_from_civil(y, m, d)) {
                        if (mismatches++ == 0)
                            printf("FAIL DaysFromCivil %d-%02u-%02u\n", y, m, d);
                    }
                }
            }
        }
        g_checks++;
        if (mismatches != 0) g_failures++;
        printf("   DaysFromCivil exhaustive 1970..2100: %s\n",
               mismatches == 0 ? "all days match reference" : "MISMATCH");
    }

    /* ---- epoch composition from DDMMYY/HHMMSS (YY maps to 20YY) ----
     * NB: field literals carry NO leading zero - a leading zero would make
     * them octal in C; the NMEA parser hands over decimal field values. */
    CHECK(GnssAcquire_EpochFromUtc(10124U, 0U) == 1704067200U);            /* 2024-01-01T00:00:00Z; t==0 valid (#132) */
    CHECK(GnssAcquire_EpochFromUtc(10170U, 100U) ==
          GnssAcquire_DaysFromCivil(2070, 1, 1) * 86400U + 60U);           /* YY=70 -> 2070, 00:01:00 */
    CHECK(GnssAcquire_EpochFromUtc(311226U, 235959U) ==
          GnssAcquire_DaysFromCivil(2026, 12, 31) * 86400U + 23U*3600U + 59U*60U + 59U);
    CHECK(GnssAcquire_EpochFromUtc(290224U, 120000U) ==
          GnssAcquire_DaysFromCivil(2024, 2, 29) * 86400U + 12U*3600U);    /* leap day valid */

    /* ---- S-A iteration-budget contract ---- */
    CHECK(GnssAcquire_IterationBudget(0U) == 0U);          /* the ONLY zero: explicit non-acquisition */
    {
        bool positive = true;
        for (uint32_t t = 1U; t <= 3600000U; t += 997U) {  /* sweep past all configured budgets */
            if (GnssAcquire_IterationBudget(t) == 0U) positive = false;
        }
        CHECK(positive);
    }
    CHECK(GnssAcquire_IterationBudget(60000U) == 1920000U);

    /* ---- first-flight package completeness (GGA alone never accepts) ---- */
    CHECK(GnssAcquire_PackageComplete(true, true) == true);
    CHECK(GnssAcquire_PackageComplete(true, false) == false);   /* position w/o valid RMC time */
    CHECK(GnssAcquire_PackageComplete(false, true) == false);   /* time w/o good-quality fix */
    CHECK(GnssAcquire_PackageComplete(false, false) == false);

    printf("%d checks, %d failures\n", g_checks, g_failures);
    return g_failures != 0;
}
