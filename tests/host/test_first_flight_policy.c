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

    /* ---- BEH-01 (#300): admitted-wake outcome (DecideWakeOutcome) ----
     * The all-fresh abort is removed: an admitted science wake ARCHIVES its
     * record and RUNS the TX FSM regardless of package freshness. The
     * freshness inputs are record-quality diagnostics; the stale bits ride
     * the flash record (flash_log.c WriteRecord flags b0-b4 -> the v6 wire
     * sensor_quality byte, pinned by the STAB-04/flash/payload suites), and
     * transmission itself is decided downstream by the plan veto / RF
     * silence (transmit_plan, region_policy and tx_fsm suites). */
    {
        FirstFlightSciencePackage_t pkg;
        FirstFlightWakeState_t st;
        FirstFlightWakeOutcome_t o;

        /* scenario 1: admitted energy + humidity stale -> archive the record
         * (hum stale bit b2 is set by WriteRecord); one failed humidity
         * channel must not decide whether pressure, temperature, battery and
         * time history exist */
        pkg = (FirstFlightSciencePackage_t){ true, true, true, true, false, true, true };
        st = (FirstFlightWakeState_t){ true, false, true,
                                       FirstFlightPolicy_PackageComplete(&pkg) };
        o = FirstFlightPolicy_DecideWakeOutcome(&st);
        CHECK(o.archive_record);
        CHECK(o.run_tx_fsm);

        /* scenario 2: admitted energy + pressure stale -> archive the other fields */
        pkg = (FirstFlightSciencePackage_t){ true, true, true, true, true, false, true };
        st = (FirstFlightWakeState_t){ true, false, true,
                                       FirstFlightPolicy_PackageComplete(&pkg) };
        o = FirstFlightPolicy_DecideWakeOutcome(&st);
        CHECK(o.archive_record);
        CHECK(o.run_tx_fsm);

        /* scenario 3: admitted energy + GNSS timeout + trusted stale position
         * within legal age -> archive (gnss stale bit b3 rides the record;
         * position age is the geo authority's concern, not this gate's) */
        st = (FirstFlightWakeState_t){ true, false,
                                       FirstFlightPolicy_GnssPackagePresent(false, false, false),
                                       false };
        o = FirstFlightPolicy_DecideWakeOutcome(&st);
        CHECK(o.archive_record);
        CHECK(o.run_tx_fsm);

        /* scenario 4: same but RF unauthorized -> archive yes, transmit no.
         * The outcome carries no RF input by construction:
         * VETO_RESTRICTED_REGION / RF silence decide transmission downstream
         * (never this decision), so the RF state cannot block archiving. */
        CHECK(o.archive_record);

        /* scenario 5: battery or temperature admission failure -> no archive,
         * no TX, survival-cadence retry (the production admission path
         * FirstFlightWakeAdmitted parks the TX FSM and rebases the timer) */
        i = make_input(-60.0f, true, 5000, true);
        CHECK(FirstFlightPolicy_Decide(&cfg, &i) == FIRST_FLIGHT_RETRY_LOW_ENERGY);
        i = make_input(-20.0f, true, 4000, true);
        CHECK(FirstFlightPolicy_Decide(&cfg, &i) == FIRST_FLIGHT_RETRY_LOW_ENERGY);
        st = (FirstFlightWakeState_t){ false, false, true, true };
        o = FirstFlightPolicy_DecideWakeOutcome(&st);
        CHECK(!o.archive_record);
        CHECK(!o.run_tx_fsm);

        /* scenario 6: bulk continuation stays independent of live-package
         * freshness (it services cached recovery, never a live record) */
        st = (FirstFlightWakeState_t){ false, true, false, false };
        o = FirstFlightPolicy_DecideWakeOutcome(&st);
        CHECK(o.archive_record);
        CHECK(o.run_tx_fsm);

        /* NULL state -> conservative park */
        o = FirstFlightPolicy_DecideWakeOutcome(NULL);
        CHECK(!o.archive_record);
        CHECK(!o.run_tx_fsm);
    }

    printf("%d checks, %d failures\n", checks, failures);
    return failures != 0;
}
