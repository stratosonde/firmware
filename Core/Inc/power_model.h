/**
  ******************************************************************************
  * @file    power_model.h
  * @brief   Voltage-based predictive power management — pure decision logic
  ******************************************************************************
  * @attention
  *
  * R49 (#46): extracted from lora_app.c so the power model is host-testable
  * with zero hardware. These functions touch NO peripherals — feed them fake
  * battery/temp values and assert the outputs. This is the decide-half
  * pattern that R47 (#44) generalizes to the whole transmit cycle.
  *
  ******************************************************************************
  */

#ifndef POWER_MODEL_H
#define POWER_MODEL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/* Power Management - Operating Modes */
typedef enum {
    MODE_NORMAL = 0,       // 5min interval, GPS enabled
    MODE_CONSERVATIVE = 1, // 10min interval, GPS enabled
    MODE_REDUCED = 2,      // 15min interval, GPS disabled
    MODE_RECOVERY = 3,     // 30min interval, GPS disabled
    MODE_SURVIVAL = 4      // 60min interval, minimal activity
} OperatingMode_t;

/* Simplified voltage tracking for slope calculation (2-hour window) */
typedef struct {
    uint16_t baseline_voltage_mv;   // Voltage at baseline (updated every 2 hours)
    uint32_t baseline_timestamp;     // Timestamp at baseline
    uint16_t current_voltage_mv;     // Most recent voltage reading
    uint32_t current_timestamp;      // Most recent timestamp
    int16_t  last_slope_mv_per_hour; // FW-6: last valid slope (returned when dt < MIN_SLOPE_DT)
    /* F8 (#172): mode-change hysteresis (RAM-only, like the slope history).
     * One ADC sample of noise (~10 mV) over the ~600 s slope window is
     * ~60 mV/h — larger than every mode threshold — so raw mode selection
     * chatters. Upgrades (toward higher power) need consecutive confirmation;
     * downgrades apply immediately. */
    uint8_t  mode_hyst_valid;    // committed_mode is meaningful
    uint8_t  committed_mode;     // OperatingMode_t last handed out
    uint8_t  upgrade_streak;     // consecutive cycles proposing a higher-power mode
} VoltageSlope_t;

uint16_t NormalizeBatteryVoltage(uint16_t measured_mv, float temp_c);
int16_t  CalculateVoltageSlope(VoltageSlope_t *slope, uint16_t battery_mv, uint32_t now_timestamp);
/* STAB-08 (#155): direction-aware threshold prediction. The old
 * PredictTimeToVoltage had logically dead boundary tests and folded
 * "already past" into the same 0xFFFF as "stable/never". These return an
 * explicit state; hours are meaningful only for PRED_REACHABLE. */
typedef enum {
    PRED_AT_OR_PAST,   /* current already at/beyond the threshold */
    PRED_STABLE,       /* slope == 0: never reaches it */
    PRED_MOVING_AWAY,  /* slope points away from the threshold */
    PRED_REACHABLE     /* hours_out = hours to the threshold (clamped 9999) */
} PredictionState_t;

PredictionState_t PredictTimeToLowerThreshold(uint16_t current_voltage_mv,
                                              int16_t slope_mv_per_hour,
                                              uint16_t lower_mv,
                                              uint16_t *hours_out);
PredictionState_t PredictTimeToUpperThreshold(uint16_t current_voltage_mv,
                                              int16_t slope_mv_per_hour,
                                              uint16_t upper_mv,
                                              uint16_t *hours_out);
OperatingMode_t SelectModeFromPredictions(int16_t current_slope, uint16_t current_voltage, uint16_t time_to_critical, uint16_t raw_voltage_mv);
const char* GetModeName(OperatingMode_t mode);

#ifdef __cplusplus
}
#endif

#endif /* POWER_MODEL_H */
