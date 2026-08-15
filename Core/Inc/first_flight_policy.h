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

#endif /* FIRST_FLIGHT_POLICY_H */
