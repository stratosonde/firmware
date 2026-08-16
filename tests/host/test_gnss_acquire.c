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
#include <math.h>

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

    /* ---- A5 (#284/H-08): GnssAcquire_FixAccepted contract ----
     * The coordinate/token facts arrive as candidate fields; their
     * computation (GNSS_ValidateCoordinates range check, genuine (0,0)
     * acceptance, token presence) is covered by the DR-02/R2-16 suites in
     * test_dr_20260812.c. The time/position package split is the
     * PackageComplete truth table above. */
    const GnssFixLimits_t limits = { .minimum_satellites = 4, .maximum_hdop_x10 = 25 };
    GnssFixCandidate_t c = {
        .valid = true, .position_present = true, .fix_quality_valid = true,
        .coordinates_valid = true, .satellites = 6, .hdop = 1.5f
    };
    CHECK(GnssAcquire_FixAccepted(&c, &limits));                 /* all good: accept */

    /* satellites: one below / exactly at / above the configured minimum */
    c.satellites = 3; CHECK(!GnssAcquire_FixAccepted(&c, &limits));
    c.satellites = 4; CHECK(GnssAcquire_FixAccepted(&c, &limits));   /* boundary equality accepts */
    c.satellites = 5; CHECK(GnssAcquire_FixAccepted(&c, &limits));

    /* HDOP: below / exactly at / above the configured maximum (x10 exact) */
    c.hdop = 2.4f;  CHECK(GnssAcquire_FixAccepted(&c, &limits));
    c.hdop = 2.5f;  CHECK(GnssAcquire_FixAccepted(&c, &limits));     /* boundary equality accepts */
    c.hdop = 2.55f; CHECK(!GnssAcquire_FixAccepted(&c, &limits));

    /* non-finite / negative / unrepresentable HDOP: reject before conversion */
    c.hdop = NAN;       CHECK(!GnssAcquire_FixAccepted(&c, &limits));
    c.hdop = INFINITY;  CHECK(!GnssAcquire_FixAccepted(&c, &limits));
    c.hdop = -INFINITY; CHECK(!GnssAcquire_FixAccepted(&c, &limits));
    c.hdop = -0.1f;     CHECK(!GnssAcquire_FixAccepted(&c, &limits));
    c.hdop = 1e38f;     CHECK(!GnssAcquire_FixAccepted(&c, &limits)); /* *10 overflows or compares huge */
    c.hdop = 1.5f;

    /* every candidate invariant is load-bearing */
    c.valid = false;             CHECK(!GnssAcquire_FixAccepted(&c, &limits)); c.valid = true;
    c.position_present = false;  CHECK(!GnssAcquire_FixAccepted(&c, &limits)); c.position_present = true;
    c.fix_quality_valid = false; CHECK(!GnssAcquire_FixAccepted(&c, &limits)); c.fix_quality_valid = true;
    c.coordinates_valid = false; CHECK(!GnssAcquire_FixAccepted(&c, &limits)); c.coordinates_valid = true;

    /* null safety */
    CHECK(!GnssAcquire_FixAccepted(NULL, &limits));
    CHECK(!GnssAcquire_FixAccepted(&c, NULL));

    /* the limits are CONSUMED, not hardcoded: HDOP 3.0 (accepted by the old
     * hardcoded 5.0 driver predicate - see test_gnss_fix_acceptance.c) is
     * rejected under the configured 2.5, and the boundary moves with config */
    c.hdop = 3.0f; CHECK(!GnssAcquire_FixAccepted(&c, &limits));
    {
        const GnssFixLimits_t loose = { .minimum_satellites = 4, .maximum_hdop_x10 = 50 };
        CHECK(GnssAcquire_FixAccepted(&c, &loose));
    }
    {
        const GnssFixLimits_t strict = { .minimum_satellites = 6, .maximum_hdop_x10 = 25 };
        c.hdop = 1.5f; c.satellites = 5; CHECK(!GnssAcquire_FixAccepted(&c, &strict));
        c.satellites = 6; CHECK(GnssAcquire_FixAccepted(&c, &strict));
    }

    /* ---- BEH-02 (#284): weak/basic position never becomes TRUSTED ----
     * GnssAcquire_Disposition is the one decision for what a
     * non-package-complete acquisition may touch. Required: trusted
     * position + freshness update only under the configured accepted-fix
     * predicate; anything else keeps stale/weak provenance in the current
     * sample; valid date/time may discipline the RTC on its OWN validity,
     * never as proof of accepted position quality. */
    {
        GnssFixDisposition_t d;

        /* poor HDOP (3.0 > configured 2.5): position stays rejected + stale */
        c.satellites = 6; c.hdop = 3.0f; c.valid = true; c.position_present = true;
        c.fix_quality_valid = true; c.coordinates_valid = true;
        CHECK(!GnssAcquire_FixAccepted(&c, &limits));
        d = GnssAcquire_Disposition(GnssAcquire_FixAccepted(&c, &limits), true, true);
        CHECK(!d.update_trusted_position);
        CHECK(d.mark_gnss_stale);
        CHECK(d.discipline_time);   /* valid RMC time may still discipline the RTC */

        /* too few satellites (3 < configured 4) */
        c.satellites = 3; c.hdop = 1.5f;
        CHECK(!GnssAcquire_FixAccepted(&c, &limits));
        d = GnssAcquire_Disposition(GnssAcquire_FixAccepted(&c, &limits), true, true);
        CHECK(!d.update_trusted_position);
        CHECK(d.mark_gnss_stale);

        /* invalid fix quality */
        c.satellites = 6; c.fix_quality_valid = false;
        CHECK(!GnssAcquire_FixAccepted(&c, &limits));
        d = GnssAcquire_Disposition(GnssAcquire_FixAccepted(&c, &limits), true, true);
        CHECK(!d.update_trusted_position);
        CHECK(d.mark_gnss_stale);
        c.fix_quality_valid = true;

        /* partial sentence state (RMC 'A' latch without position tokens):
         * nothing trusted, still stale, no discipline from this path */
        d = GnssAcquire_Disposition(false, false, true);
        CHECK(!d.update_trusted_position);
        CHECK(d.mark_gnss_stale);
        CHECK(!d.discipline_time);

        /* weak fix with INVALID RMC time: no RTC discipline either */
        d = GnssAcquire_Disposition(false, true, false);
        CHECK(!d.update_trusted_position);
        CHECK(d.mark_gnss_stale);
        CHECK(!d.discipline_time);

        /* boundary-valid accepted fix (sats == min, hdop == max): ALL
         * authoritative updates occur together */
        c.satellites = 4; c.hdop = 2.5f;
        CHECK(GnssAcquire_FixAccepted(&c, &limits));
        d = GnssAcquire_Disposition(GnssAcquire_FixAccepted(&c, &limits), true, true);
        CHECK(d.update_trusted_position);
        CHECK(!d.mark_gnss_stale);
        CHECK(d.discipline_time);

        /* accepted fix but invalid RMC date/time: trusted position updates,
         * but the RTC is not disciplined on invalid time (and the position
         * was never trusted BECAUSE time disciplined) */
        d = GnssAcquire_Disposition(true, true, false);
        CHECK(d.update_trusted_position);
        CHECK(!d.mark_gnss_stale);
        CHECK(!d.discipline_time);
    }

    printf("%d checks, %d failures\n", g_checks, g_failures);
    return g_failures != 0;
}
