/**
 ******************************************************************************
 * @file    power_model.c
 * @brief   Voltage-based predictive power management — pure decision logic
 ******************************************************************************
 * @attention
 *
 * R49 (#46): extracted verbatim from lora_app.c for host testability.
 * No hardware access in this file — SEGGER_RTT calls compile to no-ops in
 * the host harness (tests/host/stubs/SEGGER_RTT.h).
 *
 ******************************************************************************
 */

#include "power_model.h"
#include "SEGGER_RTT.h"
#include "sonde_log.h" /* R50 (#47): compile-time log gate */
#include <stdlib.h>    /* abs */

/* BEH-06 (#297): the UNQUALIFIED_LEGACY profile - the current, unmeasured
 * values preserved as named DATA until the Nichicon cold bench campaign
 * (#248) replaces them. The decision code consumes the profile; it never
 * embeds a table. */
static const PowerProfileTempKnot_t s_legacy_knots[] = {
    {25, 0},     // 5500 + 0 = 5500mV (reference)
    {0, 200},    // Approximate for 0°C to 25°C range
    {-10, 350},  // Approximate
    {-20, 500},  // Approximate
    {-30, 600},  // Approximate
    {-40, 700},  // Approximate
    {-50, 450},  // R10 (#37): was 400 — non-monotonic between -40(700) and -55(430)
    {-55, 430},  // 5070 + 430 = 5500
    {-56, 660},  // 4840 + 660 = 5500
    {-57, 690},  // 4810 + 690 = 5500
    {-58, 700},  // 4800 + 700 = 5500
    {-59, 730},  // 4770 + 730 = 5500
    {-60, 800},  // 4700 + 800 = 5500
    {-61, 950},  // 4550 + 950 = 5500
    {-62, 1100}, // 4400 + 1100 = 5500
    {-63, 1400}, // 4100 + 1400 = 5500
    {-64, 1690}, // 3810 + 1690 = 5500
    {-65, 2170}, // 3330 + 2170 = 5500 (massive drop!)
    {-66, 2700}, // 2800 + 2700 = 5500 (non-operational)
};

static const PowerProfile_t s_legacy_profile = {
    POWER_PROFILE_UNQUALIFIED_LEGACY_ID,
    POWER_PROFILE_SCHEMA_VERSION,
    (uint16_t)(sizeof(s_legacy_knots) / sizeof(s_legacy_knots[0])),
    s_legacy_knots,
    4300U, /* raw floor: UNQUALIFIED until the bench floor(T) lands */
    -30, -15, -5, 20,
    6U, 12U,
    3U /* F8 hysteresis */
};

static const PowerProfile_t *s_active_profile = NULL;

const PowerProfile_t *PowerProfile_Legacy(void) {
  return &s_legacy_profile;
}

bool PowerProfile_Validate(const PowerProfile_t *profile) {
  if (profile == NULL || profile->knots == NULL ||
      profile->schema_version != POWER_PROFILE_SCHEMA_VERSION ||
      profile->knot_count < 2U || profile->knot_count > 32U ||
      profile->raw_floor_mv < 1000U || profile->raw_floor_mv > 6000U ||
      profile->upgrade_confirm < 1U || profile->upgrade_confirm > 10U ||
      !(profile->slope_emergency_mv_h < profile->slope_warning_mv_h) ||
      !(profile->slope_warning_mv_h < profile->slope_caution_mv_h) ||
      !(profile->slope_caution_mv_h < 0) ||
      !(profile->slope_charging_mv_h > 0) ||
      profile->hours_emergency == 0U ||
      profile->hours_emergency >= profile->hours_warning) {
    return false;
  }
  /* Knots must be strictly descending in temperature with sane
   * compensations. */
  for (uint16_t i = 1; i < profile->knot_count; i++) {
    if (profile->knots[i].temp_c >= profile->knots[i - 1U].temp_c) {
      return false;
    }
  }
  for (uint16_t i = 0; i < profile->knot_count; i++) {
    if (profile->knots[i].compensation_mv < 0 ||
        profile->knots[i].compensation_mv > 4000) {
      return false;
    }
  }
  return true;
}

const PowerProfile_t *PowerProfile_Select(const PowerProfile_t *candidate) {
  return PowerProfile_Validate(candidate) ? candidate : PowerProfile_Legacy();
}

const PowerProfile_t *PowerProfile_Active(void) {
  return (s_active_profile != NULL) ? s_active_profile : PowerProfile_Legacy();
}

void PowerProfile_SetActive(const PowerProfile_t *candidate) {
  s_active_profile = PowerProfile_Select(candidate);
}

uint16_t PowerModel_Normalize(const PowerProfile_t *profile,
                              uint16_t measured_mv, float temp_c) {
  const PowerProfile_t *p = PowerProfile_Select(profile);
  const PowerProfileTempKnot_t *knots = p->knots;
  const uint16_t count = p->knot_count;

  // At or above the top knot: its compensation applies as-is
  if (temp_c >= (float)knots[0].temp_c) {
    return measured_mv + (uint16_t)knots[0].compensation_mv;
  }

  // Find bracketing points for linear interpolation
  for (uint16_t i = 0; i + 1U < count; i++) {
    if (temp_c >= (float)knots[i + 1U].temp_c &&
        temp_c <= (float)knots[i].temp_c) {
      // Linear interpolation between two points
      float temp_range = (float)(knots[i].temp_c - knots[i + 1U].temp_c);
      float temp_fraction = (temp_c - (float)knots[i + 1U].temp_c) / temp_range;
      int16_t comp_range = knots[i].compensation_mv - knots[i + 1U].compensation_mv;
      int16_t compensation = knots[i + 1U].compensation_mv +
                             (int16_t)(temp_fraction * comp_range);

      return measured_mv + (uint16_t)compensation;
    }
  }

  // Below the bottom knot: use its (maximum) compensation - the battery is
  // non-functional down there anyway.
  return measured_mv + (uint16_t)knots[count - 1U].compensation_mv;
}

uint16_t NormalizeBatteryVoltage(uint16_t measured_mv, float temp_c) {
  return PowerModel_Normalize(PowerProfile_Active(), measured_mv, temp_c);
}

int16_t CalculateVoltageSlope(VoltageSlope_t *slope, uint16_t battery_mv, uint32_t now_timestamp) {
  // First sample? Initialize baseline and current
  if (slope->baseline_timestamp == 0) {
    slope->baseline_voltage_mv = battery_mv;
    slope->baseline_timestamp = now_timestamp;
    slope->current_voltage_mv = battery_mv;
    slope->current_timestamp = now_timestamp;
    return 0; // No slope yet (same value)
  }

  /* S-B (#213, 2026-08-12, RV-06 class): backward time step. The LSE->LSI
   * failover re-inits the RTC and restarts the counter near zero, but
   * this state lives in RAM and survives (no MCU reset), so
   * (current - baseline) wraps. The SMALL backward step is the dangerous
   * one: it casts to a small NEGATIVE int32_t below and (dv * 3600) / -dt
   * both SIGN-FLIPS and MAGNIFIES. Measured on the host harness: a
   * genuine -108 mV/h discharge reported as +14400 mV/h fast charging,
   * then republished for the whole FW-6 600 s window - enough separated
   * observations to confirm an F8 mode UPGRADE at ASCENT cadence. Re-seed
   * and report no slope - the same guard Deadman_Check,
   * LaunchDetector_Update and FloatDetector_Update already carry
   * (RV-06/#162). (The F-002/#201 fix makes the failover RTC restart
   * actually happen, which makes this guard load-bearing, not
   * theoretical.) */
  if (now_timestamp < slope->baseline_timestamp) {
    slope->baseline_voltage_mv = battery_mv;
    slope->baseline_timestamp = now_timestamp;
    slope->current_voltage_mv = battery_mv;
    slope->current_timestamp = now_timestamp;
    slope->last_slope_mv_per_hour = 0;
    return 0;
  }

  // Update current values
  slope->current_voltage_mv = battery_mv;
  slope->current_timestamp = now_timestamp;

  // Calculate time difference
  uint32_t time_change_sec = slope->current_timestamp - slope->baseline_timestamp;

  /* FW-6: a tiny dt amplifies ~10 mV of ADC noise into a huge spurious
   * slope (dv*3600/dt) — e.g. back-to-back SendTxData cycles re-armed by
   * bulk transfer (C7b). Require 600 s of history before recomputing;
   * below that return the last valid slope (also covers dt == 0). */
  if (time_change_sec < 600)
    return slope->last_slope_mv_per_hour;

  // Calculate voltage change
  int32_t voltage_change = slope->current_voltage_mv - slope->baseline_voltage_mv;

  // Convert to mV/hour
  /* F-01 (#62): cast the denominator to int32_t — without it the uint32_t
   * time_change_sec promotes the WHOLE division to unsigned, wrapping a
   * negative numerator to a huge positive (a discharging battery read as
   * charging: -100 mV/h came back as +13298 mV/h). Safe: dt >= 600 above. */
  int32_t slope32 = (voltage_change * 3600) / (int32_t)time_change_sec;
  /* #136 (2026-08-10 finding #6): saturate, never wrap. A large legitimate
   * swing (e.g. recovery after a rejected/poisoned baseline) overflows the
   * int16_t cast and flips sign — a RECOVERING battery read as catastrophic
   * discharge (+38400 mV/h came back as -27136 mV/h). */
  if (slope32 > INT16_MAX)
    slope32 = INT16_MAX;
  if (slope32 < INT16_MIN)
    slope32 = INT16_MIN;
  int16_t slope_mv_per_hour = (int16_t)slope32;

  // Every 2 hours (7200 seconds), shift baseline forward
  if (time_change_sec >= 7200) {
    slope->baseline_voltage_mv = slope->current_voltage_mv;
    slope->baseline_timestamp = slope->current_timestamp;
  }

  slope->last_slope_mv_per_hour = slope_mv_per_hour;
  return slope_mv_per_hour;
}

/* STAB-08 (#155): the old PredictTimeToVoltage tested (voltage_delta > 0 &&
 * current >= target) and its mirror - both contradictory by construction, so
 * already-past was unreachable there and fell into the same 0xFFFF as
 * stable/never. Direction-specific functions with explicit states replace
 * it; hours are computed only in the REACHABLE state and saturate at 9999. */
PredictionState_t PredictTimeToLowerThreshold(uint16_t current_voltage_mv,
                                              int16_t slope_mv_per_hour,
                                              uint16_t lower_mv,
                                              uint16_t *hours_out) {
  if (current_voltage_mv <= lower_mv) {
    return PRED_AT_OR_PAST;
  }
  if (slope_mv_per_hour == 0) {
    return PRED_STABLE;
  }
  if (slope_mv_per_hour > 0) {
    return PRED_MOVING_AWAY;
  }
  uint32_t hours = (uint32_t)(current_voltage_mv - lower_mv) /
                   (uint32_t)(-slope_mv_per_hour);
  if (hours > 9999)
    hours = 9999;
  if (hours_out)
    *hours_out = (uint16_t)hours;
  return PRED_REACHABLE;
}

PredictionState_t PredictTimeToUpperThreshold(uint16_t current_voltage_mv,
                                              int16_t slope_mv_per_hour,
                                              uint16_t upper_mv,
                                              uint16_t *hours_out) {
  if (current_voltage_mv >= upper_mv) {
    return PRED_AT_OR_PAST;
  }
  if (slope_mv_per_hour == 0) {
    return PRED_STABLE;
  }
  if (slope_mv_per_hour < 0) {
    return PRED_MOVING_AWAY;
  }
  uint32_t hours = (uint32_t)(upper_mv - current_voltage_mv) /
                   (uint32_t)slope_mv_per_hour;
  if (hours > 9999)
    hours = 9999;
  if (hours_out)
    *hours_out = (uint16_t)hours;
  return PRED_REACHABLE;
}

OperatingMode_t PowerModel_SelectMode(const PowerProfile_t *profile,
                                      int16_t current_slope,
                                      uint16_t current_voltage,
                                      uint16_t time_to_critical,
                                      uint16_t raw_voltage_mv) {
  const PowerProfile_t *p = PowerProfile_Select(profile);

  // VOLTAGE-BASED FLOOR: Emergency low voltage (LTO threshold)
  // BUG 1.5 FIX: This MUST be first — at sunrise (positive slope, near-dead battery),
  // the slope branches would select NORMAL/CONSERVATIVE and re-enable GPS + frequent TX,
  // causing brownout. Below the absolute floor = SURVIVAL, regardless of slope.
  // R10 (#37): the floor reads the RAW voltage. Feeding it the normalized
  // value let up to +2700 mV of cold compensation (at -66C) mask a real
  // brownout. Normalization is for slope/prediction only.
  // BEH-06 (#297): the floor is profile DATA (UNQUALIFIED_LEGACY until the
  // bench campaign), not a literal embedded in the decision.
  if (raw_voltage_mv < p->raw_floor_mv) {
    SONDE_LOG_STR("PREDICT: V below raw floor (LTO critical) -> SURVIVAL\r\n");
    return MODE_SURVIVAL;
  }

  // EMERGENCY: Depleting fast and will hit critical soon
  if (current_slope < p->slope_emergency_mv_h && time_to_critical != 0xFFFF &&
      time_to_critical < p->hours_emergency) {
    SONDE_LOG_STR("PREDICT: Critical in <6h -> SURVIVAL\r\n");
    return MODE_SURVIVAL;
  }

  // WARNING: Moderate depletion with limited time
  if (current_slope < p->slope_warning_mv_h && time_to_critical != 0xFFFF &&
      time_to_critical < p->hours_warning) {
    SONDE_LOG_STR("PREDICT: Critical in <12h -> RECOVERY\r\n");
    return MODE_RECOVERY;
  }

  // CAUTION: Slow depletion
  if (current_slope < p->slope_caution_mv_h) {
    SONDE_LOG_STR("PREDICT: Slow depletion -> REDUCED\r\n");
    return MODE_REDUCED;
  }

  // CHARGING: Good charging rate
  if (current_slope > p->slope_charging_mv_h) {
    SONDE_LOG_STR("PREDICT: Fast charging -> NORMAL\r\n");
    return MODE_NORMAL;
  }

  // STABLE/SLIGHT CHARGE: Gentle charging or stable
  if (current_slope > 0) {
    SONDE_LOG_STR("PREDICT: Stable/charging -> CONSERVATIVE\r\n");
    return MODE_CONSERVATIVE;
  }

  // DEFAULT: Conservative
  return MODE_CONSERVATIVE;
}

OperatingMode_t SelectModeFromPredictions(int16_t current_slope,
                                          uint16_t current_voltage,
                                          uint16_t time_to_critical,
                                          uint16_t raw_voltage_mv) {
  return PowerModel_SelectMode(PowerProfile_Active(), current_slope,
                               current_voltage, time_to_critical,
                               raw_voltage_mv);
}

const char *GetModeName(OperatingMode_t mode) {
  switch (mode) {
  case MODE_NORMAL:
    return "NORMAL";
  case MODE_CONSERVATIVE:
    return "CONSERVATIVE";
  case MODE_REDUCED:
    return "REDUCED";
  case MODE_RECOVERY:
    return "RECOVERY";
  case MODE_SURVIVAL:
    return "SURVIVAL";
  default:
    return "UNKNOWN";
  }
}
