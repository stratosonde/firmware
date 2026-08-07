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
} VoltageSlope_t;

uint16_t NormalizeBatteryVoltage(uint16_t measured_mv, float temp_c);
int16_t  CalculateVoltageSlope(VoltageSlope_t *slope, uint16_t battery_mv, uint32_t now_timestamp);
uint16_t PredictTimeToVoltage(uint16_t current_voltage_mv, int16_t slope_mv_per_hour, uint16_t target_voltage_mv);
OperatingMode_t SelectModeFromPredictions(int16_t current_slope, uint16_t current_voltage, uint16_t time_to_critical, uint16_t raw_voltage_mv);
const char* GetModeName(OperatingMode_t mode);

#ifdef __cplusplus
}
#endif

#endif /* POWER_MODEL_H */
