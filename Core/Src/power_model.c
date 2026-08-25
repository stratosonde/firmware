/**
 ******************************************************************************
 * @file    power_model.c
 * @brief   Operating-mode enum + battery temperature normalization (SoC only)
 ******************************************************************************
 * @attention
 *
 * PWR-SIMPLIFY (2026-08-24): the predictive ladder (slope, predictions, mode
 * selection, profiles) is deleted; safety is the two hard admission gates in
 * first_flight_policy.c. Remaining here: the mode enum's display names and
 * the 25 C-equivalent SoC normalization (telemetry only).
 *
 ******************************************************************************
 */

#include "power_model.h"
#include "SEGGER_RTT.h"
#include "sonde_log.h" /* R50 (#47): compile-time log gate */
#include <stdlib.h>    /* abs */

/* PWR-SIMPLIFY: only the normalization table survives the ladder deletion,
 * and only for SoC telemetry (synthetic level, never a safety input).
 * LT-06 (#248): the -40..-55 span is known non-monotonic; recalibration is
 * tracked separately. */
static const PowerProfileTempKnot_t s_soc_knots[] = {
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

uint16_t NormalizeBatteryVoltage(uint16_t measured_mv, float temp_c) {
  const PowerProfileTempKnot_t *knots = s_soc_knots;
  const uint16_t count =
      (uint16_t)(sizeof(s_soc_knots) / sizeof(s_soc_knots[0]));

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
