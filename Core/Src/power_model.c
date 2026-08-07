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
#include "sonde_log.h"  /* R50 (#47): compile-time log gate */
#include <stdlib.h>  /* abs */

uint16_t NormalizeBatteryVoltage(uint16_t measured_mv, float temp_c) {
    // No compensation needed at or above room temperature
    if (temp_c >= 25.0f) {
        return measured_mv;
    }

    // Lookup table from empirical data (Vmax column - no load voltage)
    // Compensation values bring voltage back to 5500mV (fully charged at 25°C)
    typedef struct {
        int8_t temp;
        int16_t compensation_mv;  // mV to add to measured voltage
    } TempCompPoint_t;

    static const TempCompPoint_t comp_table[] = {
        {25,  0},      // 5500 + 0 = 5500mV (reference)
        {0,   200},    // Approximate for 0°C to 25°C range
        {-10, 350},    // Approximate
        {-20, 500},    // Approximate
        {-30, 600},    // Approximate
        {-40, 700},    // Approximate
        {-50, 450},    // R10 (#37): was 400 — non-monotonic between -40(700) and -55(430)
        {-55, 430},    // 5070 + 430 = 5500
        {-56, 660},    // 4840 + 660 = 5500
        {-57, 690},    // 4810 + 690 = 5500
        {-58, 700},    // 4800 + 700 = 5500
        {-59, 730},    // 4770 + 730 = 5500
        {-60, 800},    // 4700 + 800 = 5500
        {-61, 950},    // 4550 + 950 = 5500
        {-62, 1100},   // 4400 + 1100 = 5500
        {-63, 1400},   // 4100 + 1400 = 5500
        {-64, 1690},   // 3810 + 1690 = 5500
        {-65, 2170},   // 3330 + 2170 = 5500 (massive drop!)
        {-66, 2700},   // 2800 + 2700 = 5500 (non-operational)
    };

    const int table_size = sizeof(comp_table) / sizeof(comp_table[0]);

    // Find bracketing points for linear interpolation
    for (int i = 0; i < table_size - 1; i++) {
        if (temp_c >= comp_table[i+1].temp && temp_c <= comp_table[i].temp) {
            // Linear interpolation between two points
            float temp_range = (float)(comp_table[i].temp - comp_table[i+1].temp);
            float temp_fraction = (temp_c - comp_table[i+1].temp) / temp_range;
            int16_t comp_range = comp_table[i].compensation_mv - comp_table[i+1].compensation_mv;
            int16_t compensation = comp_table[i+1].compensation_mv +
                                 (int16_t)(temp_fraction * comp_range);

            return measured_mv + compensation;
        }
    }

    // Below -66°C: use maximum compensation (battery non-functional anyway)
    return measured_mv + 2700;
}

int16_t CalculateVoltageSlope(VoltageSlope_t *slope, uint16_t battery_mv, uint32_t now_timestamp) {
    // First sample? Initialize baseline and current
    if (slope->baseline_timestamp == 0) {
        slope->baseline_voltage_mv = battery_mv;
        slope->baseline_timestamp = now_timestamp;
        slope->current_voltage_mv = battery_mv;
        slope->current_timestamp = now_timestamp;
        return 0;  // No slope yet (same value)
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
    if (time_change_sec < 600) return slope->last_slope_mv_per_hour;

    // Calculate voltage change
    int32_t voltage_change = slope->current_voltage_mv - slope->baseline_voltage_mv;

    // Convert to mV/hour
    int16_t slope_mv_per_hour = (int16_t)((voltage_change * 3600) / time_change_sec);

    // Every 2 hours (7200 seconds), shift baseline forward
    if (time_change_sec >= 7200) {
        slope->baseline_voltage_mv = slope->current_voltage_mv;
        slope->baseline_timestamp = slope->current_timestamp;
    }

    slope->last_slope_mv_per_hour = slope_mv_per_hour;
    return slope_mv_per_hour;
}


uint16_t PredictTimeToVoltage(uint16_t current_voltage_mv,
                                      int16_t slope_mv_per_hour,
                                      uint16_t target_voltage_mv) {

    int32_t voltage_delta = target_voltage_mv - current_voltage_mv;

    // Stable voltage - never reaches target
    if (slope_mv_per_hour == 0) {
        return 0xFFFF;
    }

    // Check if moving toward or away from target
    if ((voltage_delta > 0 && slope_mv_per_hour < 0) ||  // Target higher but discharging
        (voltage_delta < 0 && slope_mv_per_hour > 0)) {  // Target lower but charging
        return 0xFFFF;  // Moving away from target
    }

    // Already at or past target
    if (voltage_delta == 0 ||
        (voltage_delta > 0 && current_voltage_mv >= target_voltage_mv) ||
        (voltage_delta < 0 && current_voltage_mv <= target_voltage_mv)) {
        return 0;
    }

    // Calculate time: |voltage_delta| / |slope|
    int32_t hours_to_target = abs(voltage_delta) / abs(slope_mv_per_hour);

    // Clamp to reasonable range
    if (hours_to_target > 9999) {
        hours_to_target = 9999;
    }

    return (uint16_t)hours_to_target;
}

OperatingMode_t SelectModeFromPredictions(int16_t current_slope,
                                                   uint16_t current_voltage,
                                                   uint16_t time_to_critical,
                                                   uint16_t raw_voltage_mv) {

    // VOLTAGE-BASED FLOOR: Emergency low voltage (LTO threshold)
    // BUG 1.5 FIX: This MUST be first — at sunrise (positive slope, near-dead battery),
    // the slope branches would select NORMAL/CONSERVATIVE and re-enable GPS + frequent TX,
    // causing brownout. Below the absolute floor = SURVIVAL, regardless of slope.
    // R10 (#37): the floor reads the RAW voltage. Feeding it the normalized
    // value let up to +2700 mV of cold compensation (at -66C) mask a real
    // brownout. Normalization is for slope/prediction only.
    if (raw_voltage_mv < 4300) {
        SONDE_LOG_STR("PREDICT: V<4.3V raw (LTO critical) -> SURVIVAL\r\n");
        return MODE_SURVIVAL;
    }

    // EMERGENCY: Depleting fast and will hit critical soon
    if (current_slope < -30 && time_to_critical != 0xFFFF && time_to_critical < 6) {
        SONDE_LOG_STR("PREDICT: Critical in <6h -> SURVIVAL\r\n");
        return MODE_SURVIVAL;
    }

    // WARNING: Moderate depletion with limited time
    if (current_slope < -15 && time_to_critical != 0xFFFF && time_to_critical < 12) {
        SONDE_LOG_STR("PREDICT: Critical in <12h -> RECOVERY\r\n");
        return MODE_RECOVERY;
    }

    // CAUTION: Slow depletion
    if (current_slope < -5) {
        SONDE_LOG_STR("PREDICT: Slow depletion -> REDUCED\r\n");
        return MODE_REDUCED;
    }

    // CHARGING: Good charging rate
    if (current_slope > 20) {
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

const char* GetModeName(OperatingMode_t mode) {
    switch(mode) {
        case MODE_NORMAL: return "NORMAL";
        case MODE_CONSERVATIVE: return "CONSERVATIVE";
        case MODE_REDUCED: return "REDUCED";
        case MODE_RECOVERY: return "RECOVERY";
        case MODE_SURVIVAL: return "SURVIVAL";
        default: return "UNKNOWN";
    }
}
