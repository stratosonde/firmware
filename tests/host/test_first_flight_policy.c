#include "first_flight_policy.h"
#include <math.h>
#include <stdio.h>

static int checks;
static int failures;
#define CHECK(x) do { checks++; if (!(x)) { failures++; \
    printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #x); } } while (0)

static FirstFlightAdmissionInput_t make_input(float temp, bool temp_fresh,
                                               uint16_t battery,
                                               bool battery_fresh)
{
    FirstFlightAdmissionInput_t i = {
        temp, temp_fresh, battery, battery_fresh
    };
    return i;
}

int main(void)
{
    const FirstFlightPolicyConfig_t cfg = { -55, 4300 };
    FirstFlightAdmissionInput_t i;
    i = make_input(-55.0f, true, 4300, true);
    CHECK(FirstFlightPolicy_Decide(&cfg, &i) == FIRST_FLIGHT_RUN_FULL);
    CHECK(FirstFlightPolicy_Decide(NULL, &i) == FIRST_FLIGHT_RETRY_LOW_ENERGY);
    CHECK(FirstFlightPolicy_Decide(&cfg, NULL) == FIRST_FLIGHT_RETRY_LOW_ENERGY);
    i = make_input(-55.01f, true, 4300, true);
    CHECK(FirstFlightPolicy_Decide(&cfg, &i) == FIRST_FLIGHT_RETRY_LOW_ENERGY);
    i = make_input(-54.9f, true, 4299, true);
    CHECK(FirstFlightPolicy_Decide(&cfg, &i) == FIRST_FLIGHT_RETRY_LOW_ENERGY);
    i = make_input(-54.9f, false, 5000, true);
    CHECK(FirstFlightPolicy_Decide(&cfg, &i) == FIRST_FLIGHT_RETRY_LOW_ENERGY);
    i = make_input(-54.9f, true, 5000, false);
    CHECK(FirstFlightPolicy_Decide(&cfg, &i) == FIRST_FLIGHT_RETRY_LOW_ENERGY);
    i = make_input(NAN, true, 5000, true);
    CHECK(FirstFlightPolicy_Decide(&cfg, &i) == FIRST_FLIGHT_RETRY_LOW_ENERGY);
    i = make_input(-54.9f, true, 0, true);
    CHECK(FirstFlightPolicy_Decide(&cfg, &i) == FIRST_FLIGHT_RETRY_LOW_ENERGY);
    {
        const FirstFlightPolicyConfig_t stricter = { -50, 4400 };
        i = make_input(-50.0f, true, 4399, true);
        CHECK(FirstFlightPolicy_Decide(&stricter, &i) == FIRST_FLIGHT_RETRY_LOW_ENERGY);
    }
    {
        FirstFlightSciencePackage_t p = { true, true, true, true, true, true, true };
        CHECK(FirstFlightPolicy_PackageComplete(&p));
        p.fresh_pressure = false;
        CHECK(!FirstFlightPolicy_PackageComplete(&p));
        CHECK(!FirstFlightPolicy_PackageComplete(NULL));
    }
    CHECK(FirstFlightPolicy_GnssDateTimeValid(290224U, 0U));
    CHECK(FirstFlightPolicy_GnssDateTimeValid(311226U, 235960U));
    CHECK(!FirstFlightPolicy_GnssDateTimeValid(290225U, 120000U));
    CHECK(!FirstFlightPolicy_GnssDateTimeValid(311124U, 120000U));
    CHECK(!FirstFlightPolicy_GnssDateTimeValid(150826U, 236000U));
    CHECK(!FirstFlightPolicy_GnssDateTimeValid(0U, 120000U));

    /* ---- stage 6: FirstFlightPolicy_VoltsToMvOrZero ---- */
    CHECK(FirstFlightPolicy_VoltsToMvOrZero(5.0f) == 5000U);
    CHECK(FirstFlightPolicy_VoltsToMvOrZero(3.4567f) == 3457U);   /* rounds to nearest mV */
    CHECK(FirstFlightPolicy_VoltsToMvOrZero(65.534f) == 65534U);  /* in range */
    CHECK(FirstFlightPolicy_VoltsToMvOrZero(65.536f) == 0U);      /* out of range -> conservative zero */
    /* NB the razor edge (65.535f) is deliberately NOT asserted: it is
     * evaluation-model dependent - host x87 compares the literal at 80-bit
     * (guard fires -> 0), the ARM FPU compares float32 (accepted -> 65535).
     * The contract pins the unambiguous points on either side. */
    CHECK(FirstFlightPolicy_VoltsToMvOrZero(0.0f) == 0U);         /* zero reads as a sensor fault */
    CHECK(FirstFlightPolicy_VoltsToMvOrZero(-1.2f) == 0U);
    CHECK(FirstFlightPolicy_VoltsToMvOrZero(NAN) == 0U);
    CHECK(FirstFlightPolicy_VoltsToMvOrZero(INFINITY) == 0U);
    CHECK(FirstFlightPolicy_VoltsToMvOrZero(-INFINITY) == 0U);
    CHECK(FirstFlightPolicy_VoltsToMvOrZero(0.0004f) == 0U);      /* sub-mV rounds to a zero sample */

    /* ---- stage 6: FirstFlightPolicy_GnssPackagePresent ---- */
    /* cached-bulk exemption: a continuation never needs a fresh package */
    CHECK(FirstFlightPolicy_GnssPackagePresent(true, false, false));
    CHECK(FirstFlightPolicy_GnssPackagePresent(true, true, true));
    /* science wake: this wake's accepted fix AND disciplined time, together */
    CHECK(FirstFlightPolicy_GnssPackagePresent(false, true, true));
    CHECK(!FirstFlightPolicy_GnssPackagePresent(false, false, true));   /* GNSS failure */
    CHECK(!FirstFlightPolicy_GnssPackagePresent(false, true, false));   /* fix without time = partial */
    CHECK(!FirstFlightPolicy_GnssPackagePresent(false, false, false));

    printf("%d checks, %d failures\n", checks, failures);
    return failures != 0;
}
