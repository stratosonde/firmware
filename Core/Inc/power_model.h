/**
 ******************************************************************************
 * @file    power_model.h
 * @brief   Operating-mode enum + battery temperature normalization (SoC only)
 ******************************************************************************
 * @attention
 *
 * PWR-SIMPLIFY (2026-08-24, docs/temp/stratosonde-power-simplification.md):
 * the voltage-slope predictive ladder is DELETED. Safety decisions are now
 * made by exactly two hard admission gates in first_flight_policy.c:
 *
 *   Gate A — fresh temperature >= configured minimum (default -60 C)
 *   Gate B — fresh raw battery   >= CONFIG_FIRST_FLIGHT_BATTERY_FLOOR_MV
 *             (default 3800 mV)
 *
 * What remains here is deliberate:
 *   - OperatingMode_t / GetModeName: the telemetry mode field's value space.
 *     An admitted plan always reports MODE_NORMAL.
 *   - NormalizeBatteryVoltage: 25 C-equivalent SoC for telemetry only. It is
 *     deliberately NOT an input to any safety decision.
 *
 ******************************************************************************/

#ifndef POWER_MODEL_H
#define POWER_MODEL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

/* Power Management - Operating Modes */
typedef enum {
  MODE_NORMAL = 0,       // 5 min cadence
  MODE_CONSERVATIVE = 1, // 10 min cadence
  MODE_REDUCED = 2,      // 15 min cadence
  MODE_RECOVERY = 3,     // 30 min cadence
  MODE_SURVIVAL = 4      // 60 min retry cadence
} OperatingMode_t;

/* Identity of the temperature-normalization table below. PWR-SIMPLIFY keeps
 * both macros: the release-manifest evidence and the CI identity gate read
 * them, and the table itself remains (SoC telemetry only). The predictive
 * PowerProfile_t apparatus (validate/select/active, slope fields) is deleted
 * with the ladder. */
#define POWER_PROFILE_SCHEMA_VERSION 1U
#define POWER_PROFILE_UNQUALIFIED_LEGACY_ID 0x554C4547U /* "ULEG" */

typedef struct {
  int8_t temp_c; /* knots in DESCENDING temperature order */
  int16_t compensation_mv;
} PowerProfileTempKnot_t;

/** 25 C-equivalent battery voltage for SoC telemetry. NOT a safety input. */
uint16_t NormalizeBatteryVoltage(uint16_t measured_mv, float temp_c);
const char *GetModeName(OperatingMode_t mode);

#ifdef __cplusplus
}
#endif

#endif /* POWER_MODEL_H */
