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
    printf("%d checks, %d failures\n", checks, failures);
    return failures != 0;
}
