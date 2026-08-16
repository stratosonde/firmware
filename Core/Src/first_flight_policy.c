#include "first_flight_policy.h"
#include <math.h>
#include <stddef.h>

FirstFlightAdmission_t FirstFlightPolicy_Decide(
    const FirstFlightPolicyConfig_t *config,
    const FirstFlightAdmissionInput_t *input) {
  if (config == NULL || input == NULL ||
      !input->temperature_fresh ||
      !isfinite(input->temperature_c) ||
      !input->battery_fresh ||
      input->battery_mv_raw == 0U ||
      input->temperature_c < (float)config->minimum_temperature_c ||
      input->battery_mv_raw < config->minimum_battery_mv) {
    return FIRST_FLIGHT_RETRY_LOW_ENERGY;
  }
  return FIRST_FLIGHT_RUN_FULL;
}

bool FirstFlightPolicy_PackageComplete(
    const FirstFlightSciencePackage_t *package) {
  return package != NULL && package->disciplined_time &&
         package->fresh_good_fix && package->fresh_position &&
         package->fresh_temperature && package->fresh_humidity &&
         package->fresh_pressure && package->fresh_battery;
}

static bool IsLeapYear(uint32_t year) {
  return ((year % 4U) == 0U && (year % 100U) != 0U) ||
         (year % 400U) == 0U;
}

bool FirstFlightPolicy_GnssDateTimeValid(uint32_t date_ddmmyy,
                                         uint32_t time_hhmmss) {
  static const uint8_t days_per_month[12] = {
      31U, 28U, 31U, 30U, 31U, 30U, 31U, 31U, 30U, 31U, 30U, 31U};

  uint32_t day = date_ddmmyy / 10000U;
  uint32_t month = (date_ddmmyy / 100U) % 100U;
  uint32_t year = 2000U + (date_ddmmyy % 100U);
  uint32_t hour = time_hhmmss / 10000U;
  uint32_t minute = (time_hhmmss / 100U) % 100U;
  uint32_t second = time_hhmmss % 100U;

  if (date_ddmmyy == 0U || year < 2024U || month < 1U || month > 12U ||
      hour > 23U || minute > 59U || second > 60U) {
    return false;
  }

  uint32_t max_day = days_per_month[month - 1U];
  if (month == 2U && IsLeapYear(year)) {
    max_day = 29U;
  }
  return day >= 1U && day <= max_day;
}

uint16_t FirstFlightPolicy_VoltsToMvOrZero(float voltage) {
  /* A float-to-integer conversion outside the destination range is
   * undefined in C. Sensor faults must therefore become a conservative
   * zero sample before the policy sees them, never undefined flight
   * behavior. */
  if (!isfinite(voltage) || voltage <= 0.0f || voltage > 65.535f) {
    return 0U;
  }
  return (uint16_t)(voltage * 1000.0f + 0.5f);
}

bool FirstFlightPolicy_GnssPackagePresent(bool is_bulk_continuation,
                                          bool fresh_good_fix,
                                          bool time_disciplined) {
  /* Bulk continuations service cached recovery and are exempt; a science
   * wake needs this wake's accepted fix AND disciplined time. */
  if (is_bulk_continuation) {
    return true;
  }
  return fresh_good_fix && time_disciplined;
}
