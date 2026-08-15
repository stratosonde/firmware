/* First-flight admission and science-package policy.  Pure, host-testable
 * predicates; no hardware or persistence is touched here. */
#ifndef FIRST_FLIGHT_POLICY_H
#define FIRST_FLIGHT_POLICY_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    FIRST_FLIGHT_RETRY_LOW_ENERGY = 0,
    FIRST_FLIGHT_RUN_FULL = 1
} FirstFlightAdmission_t;

typedef struct {
    int16_t minimum_temperature_c;
    uint16_t minimum_battery_mv;
} FirstFlightPolicyConfig_t;

typedef struct {
    float temperature_c;
    bool temperature_fresh;
    uint16_t battery_mv_raw;
    bool battery_fresh;
} FirstFlightAdmissionInput_t;

typedef struct {
    bool disciplined_time;
    bool fresh_good_fix;
    bool fresh_position;
    bool fresh_temperature;
    bool fresh_humidity;
    bool fresh_pressure;
    bool fresh_battery;
} FirstFlightSciencePackage_t;

FirstFlightAdmission_t FirstFlightPolicy_Decide(
    const FirstFlightPolicyConfig_t *config,
    const FirstFlightAdmissionInput_t *input);

bool FirstFlightPolicy_PackageComplete(
    const FirstFlightSciencePackage_t *package);

/** Validate the GNSS DDMMYY / HHMMSS fields used to discipline science time. */
bool FirstFlightPolicy_GnssDateTimeValid(uint32_t date_ddmmyy,
                                         uint32_t time_hhmmss);

/** Defensive float -> millivolt conversion (refactor stage 6, moved verbatim
 *  from lora_app.c's FirstFlightVoltsToMvOrZero). A float-to-integer
 *  conversion outside the destination range is undefined in C, so sensor
 *  faults (NaN, Inf, non-positive, > 65.535 V) become a conservative zero
 *  sample BEFORE the policy sees them, never undefined flight behavior. */
uint16_t FirstFlightPolicy_VoltsToMvOrZero(float voltage);

/** Early GNSS-package gate (refactor stage 6): a bulk continuation services
 *  cached recovery - never a live record - so it is exempt. A science wake
 *  may archive/transmit a live record only with THIS wake's accepted GNSS
 *  fix AND disciplined time. The full 7-field package check
 *  (FirstFlightPolicy_PackageComplete) still runs later, after the post-GNSS
 *  sensor re-sample. */
bool FirstFlightPolicy_GnssPackagePresent(bool is_bulk_continuation,
                                          bool fresh_good_fix,
                                          bool time_disciplined);

#endif /* FIRST_FLIGHT_POLICY_H */
