/**
  ******************************************************************************
  * @file    transmit_plan.c
  * @brief   R47 (#44): transmit-cycle decide half — pure logic, host-testable
  ******************************************************************************
  * @attention
  *
  * Extracted from lora_app.c. Everything here is decision, not doing:
  * no GPIO, no radio, no timers. Host tests feed fake inputs and assert the
  * plan (tests/host/test_main.c).
  *
  ******************************************************************************
  */

#include "transmit_plan.h"
#include "config.h"
#include "SEGGER_RTT.h"
#include <stdio.h>
#include <stdlib.h>

/**
 * @brief Apply operating mode configuration (moved verbatim from lora_app.c)
 * @param mode Operating mode to apply
 * @param gps_enabled Output parameter for GPS enable state
 * @param gps_timeout_ms Output parameter for GPS timeout
 * @return New transmission interval in milliseconds
 */
static uint32_t ApplyOperatingMode(OperatingMode_t mode, bool *gps_enabled, uint32_t *gps_timeout_ms) {
    uint32_t interval_ms;

    // Get configuration pointer (use defaults if not available)
    const SystemConfig_t *config = Config_Get();
    if (config == NULL) {
        // Fallback to hardcoded values if config not available
        switch(mode) {
            case MODE_NORMAL:
                interval_ms = 300000;
                *gps_enabled = true;
                *gps_timeout_ms = 60000;
                break;
            case MODE_CONSERVATIVE:
                interval_ms = 600000;
                *gps_enabled = true;
                *gps_timeout_ms = 60000;
                break;
            case MODE_REDUCED:
                interval_ms = 900000;
                *gps_enabled = false;
                *gps_timeout_ms = 0;
                break;
            case MODE_RECOVERY:
                interval_ms = 1800000;
                *gps_enabled = false;
                *gps_timeout_ms = 0;
                break;
            case MODE_SURVIVAL:
                interval_ms = 3600000;
                *gps_enabled = false;
                *gps_timeout_ms = 0;
                break;
            default:
                interval_ms = 600000;
                *gps_enabled = true;
                *gps_timeout_ms = 60000;  // 60 seconds (was 30s - bug fix)
                break;
        }
    } else {
        // Use configuration values
        switch(mode) {
            case MODE_NORMAL:
                interval_ms = config->tx_interval_normal;
                *gps_enabled = true;
                *gps_timeout_ms = config->gps_timeout_normal * 1000;  // Convert to ms
                break;

            case MODE_CONSERVATIVE:
                interval_ms = config->tx_interval_conservative;
                *gps_enabled = true;
                *gps_timeout_ms = config->gps_timeout_conservative * 1000;  // Convert to ms
                break;

            case MODE_REDUCED:
                interval_ms = config->tx_interval_reduced;
                *gps_enabled = false;
                *gps_timeout_ms = 0;
                break;

            case MODE_RECOVERY:
                interval_ms = config->tx_interval_recovery;
                *gps_enabled = false;
                *gps_timeout_ms = 0;
                break;

            case MODE_SURVIVAL:
                interval_ms = config->tx_interval_survival;
                *gps_enabled = false;
                *gps_timeout_ms = 0;
                break;

            default:
                interval_ms = config->tx_interval_conservative;  // Conservative default
                *gps_enabled = true;
                *gps_timeout_ms = config->gps_timeout_conservative * 1000;
                break;
        }
    }

    return interval_ms;
}


TransmitPlan_t DecideTransmitPlan(VoltageSlope_t *slope_state,
                                  uint16_t battery_mv_raw,
                                  float temperature_c,
                                  bool temp_stale,
                                  uint32_t now_timestamp,
                                  bool joined,
                                  bool commissioning)
{
    TransmitPlan_t plan;

    /* Temperature compensation: normalize battery voltage to 25C equivalent.
     * R10 (#37): only with a FRESH temperature — a stale temp feeding
     * normalization can fabricate confidence; raw voltage is the safe input. */
    plan.battery_mv_normalized = temp_stale ? battery_mv_raw
                                            : NormalizeBatteryVoltage(battery_mv_raw, temperature_c);

    /* Slope + predictions on the NORMALIZED voltage (LTO: 4.5V crit, 5.5V full) */
    plan.voltage_slope_mv_per_hour =
        CalculateVoltageSlope(slope_state, plan.battery_mv_normalized, now_timestamp);

    uint16_t time_to_critical =
        PredictTimeToVoltage(plan.battery_mv_normalized, plan.voltage_slope_mv_per_hour, 4500);
    uint16_t time_to_full =
        PredictTimeToVoltage(plan.battery_mv_normalized, plan.voltage_slope_mv_per_hour, 5500);

    if (time_to_critical != 0xFFFF) {
        plan.time_to_target_h = -(int16_t)time_to_critical;
    } else if (time_to_full != 0xFFFF) {
        plan.time_to_target_h = (int16_t)time_to_full;
    } else {
        plan.time_to_target_h = 0;
    }

    /* Mode selection + application (R10: floor reads RAW voltage) */
    plan.power_mode = SelectModeFromPredictions(plan.voltage_slope_mv_per_hour,
                                                plan.battery_mv_normalized,
                                                time_to_critical,
                                                battery_mv_raw);
    plan.tx_interval_ms = ApplyOperatingMode(plan.power_mode,
                                             &plan.gps_enabled,
                                             &plan.gps_timeout_ms);

    /* Veto evaluation — first veto wins, record WHY (DDR-0007).
     * BUG 1.4: lockout check runs AFTER ApplyOperatingMode so it has final say. */
    plan.veto = VETO_NONE;

    const SystemConfig_t *config = Config_Get();
    int8_t gps_lockout_temp = (config != NULL) ? config->gps_temperature_lockout : -55;

    /* F9/T2 (DDR-0007): stale/unknown temperature is treated as COLD — the GPS
     * stays locked out. Fail safe, not fail sunny. */
    if (temp_stale) {
        plan.gps_enabled = false;
        plan.veto = VETO_TEMP_STALE;
        int temp_deci = (int)(temperature_c * 10.0f);
        char temp_msg[96];
        snprintf(temp_msg, sizeof(temp_msg), "GPS LOCKOUT: Temperature STALE (treated as COLD, last=%d.%d C)\r\n",
                 temp_deci / 10, abs(temp_deci % 10));
        SEGGER_RTT_WriteString(0, temp_msg);
    } else if (temperature_c < gps_lockout_temp) {
        plan.gps_enabled = false;
        plan.veto = VETO_TEMP_LOCKOUT;
        int temp_deci = (int)(temperature_c * 10.0f);
        char temp_msg[96];
        snprintf(temp_msg, sizeof(temp_msg), "GPS LOCKOUT: Temperature %d.%d C < %d C (supercap inoperative)\r\n",
                 temp_deci / 10, abs(temp_deci % 10), gps_lockout_temp);
        SEGGER_RTT_WriteString(0, temp_msg);
    }

    /* T1 ladder (DDR-0006): FLIGHT with no session = RF silence. The cycle
     * still runs (GPS + flash logging); only the radio stays dark. */
    if (!joined && !commissioning) {
        plan.veto = VETO_RF_SILENCE;
    }

    return plan;
}
