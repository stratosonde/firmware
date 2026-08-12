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
#include "sonde_log.h"  /* R50 (#47): compile-time log gate */
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
                                  bool commissioning,
                                  bool batt_stale)
{
    TransmitPlan_t plan;

    /* Temperature compensation: normalize battery voltage to 25C equivalent.
     * R10 (#37): only with a FRESH temperature — a stale temp feeding
     * normalization can fabricate confidence; raw voltage is the safe input. */
    plan.battery_mv_normalized = temp_stale ? battery_mv_raw
                                            : NormalizeBatteryVoltage(battery_mv_raw, temperature_c);

    /* Slope on the RAW voltage, predictions against the normalized level.
     * R2-10 (#114): computing the slope on the normalized voltage let the
     * temperature compensation itself masquerade as charging - below -55C
     * NormalizeBatteryVoltage adds up to +2170 mV, so cooling at constant
     * charge state produced a steeply POSITIVE slope (measured +1740 mV/h
     * for a 10C drop) and flipped the mode to GPS-on NORMAL. The raw-voltage
     * slope is the true charge/discharge trend; normalization only maps the
     * instantaneous level onto the 4.5V-crit/5.5V-full thresholds. */
    /* R2-11 (#115): the slope state is RAM-only, so after every reset there
     * is no history. Remember that BEFORE sampling so the no-history default
     * below can derive from raw voltage instead of falling through. */
    /* RV-02/03 (#161): a stale battery sample must not touch the slope
     * estimator — a rejected 0 mV read would become the baseline (the next
     * real reading then reports tens of thousands of mV/h of "charging"),
     * and a frozen cached read would hold the slope at exactly 0
     * (fabricated stability). Skip the update and keep the last slope;
     * treat staleness as no-history for the conservative defaults below. */
    const bool have_history = (slope_state->baseline_timestamp != 0) && !batt_stale;

    plan.voltage_slope_mv_per_hour = batt_stale
        ? slope_state->last_slope_mv_per_hour
        : CalculateVoltageSlope(slope_state, battery_mv_raw, now_timestamp);

    /* STAB-08 (#155): direction-aware prediction states. Wire semantics
     * unchanged: negative = hours to critical, -1 = critical now (R2-17),
     * positive = hours to full, 0 = genuinely stable. New: PRED_AT_OR_PAST on
     * the lower threshold covers ANY slope (the old rounding hole rendered a
     * below-critical but slowly-charging battery as "Stable": 50/150 = 0h). */
    uint16_t crit_hours = 0, full_hours = 0;
    PredictionState_t crit_state =
        PredictTimeToLowerThreshold(plan.battery_mv_normalized,
                                    plan.voltage_slope_mv_per_hour, 4500, &crit_hours);
    PredictionState_t full_state =
        PredictTimeToUpperThreshold(plan.battery_mv_normalized,
                                    plan.voltage_slope_mv_per_hour, 5500, &full_hours);

    /* time_to_critical keeps the SelectModeFromPredictions contract:
     * 0xFFFF = not approaching; a real hour count otherwise (0 = now). */
    uint16_t time_to_critical = (crit_state == PRED_AT_OR_PAST) ? 0 :
                                (crit_state == PRED_REACHABLE)  ? crit_hours : 0xFFFF;

    if (crit_state == PRED_AT_OR_PAST) {
        plan.time_to_target_h = -1;          /* critical now, no computable ETA */
    } else if (crit_state == PRED_REACHABLE) {
        plan.time_to_target_h = (crit_hours > 0) ? -(int16_t)crit_hours : -1;
    } else if (full_state == PRED_REACHABLE) {
        plan.time_to_target_h = (int16_t)full_hours;
    } else {
        plan.time_to_target_h = 0;           /* genuinely stable / moving away */
    }

    /* Mode selection + application (R10: floor reads RAW voltage) */
    plan.power_mode = SelectModeFromPredictions(plan.voltage_slope_mv_per_hour,
                                                plan.battery_mv_normalized,
                                                time_to_critical,
                                                battery_mv_raw);

    /* R2-11 (#115): with no slope history SelectModeFromPredictions falls
     * through every branch to MODE_CONSERVATIVE - 10-min cadence, GPS ON -
     * even on a marginal battery right after a brownout reset. Fail the
     * other way: below 5000 mV raw (marginal supercap), start REDUCED
     * (GPS off) until a real slope exists. Never loosen a stricter mode
     * (the R10 raw floor may already have picked SURVIVAL). NOTE (finding #9,
     * 2026-08-10): the slope baseline is RAM-ONLY — the backup-register
     * persistence (DR12-15) once described here was never implemented. This
     * no-history REDUCED fallback is the live mitigation; if reset-stable
     * history is ever wanted, implement DR persistence for real. */
    if (!have_history && battery_mv_raw < 5000 && plan.power_mode < MODE_REDUCED) {
        SONDE_LOG_STR("PREDICT: no slope history + marginal raw V -> REDUCED\r\n");
        plan.power_mode = MODE_REDUCED;
    }
    /* RV-03 (#161): a battery nobody has measured this cycle is not entitled
     * to the GPS-on modes — cap at REDUCED until a real reading returns. */
    if (batt_stale && plan.power_mode < MODE_REDUCED) {
        SONDE_LOG_STR("PREDICT: battery reading stale -> cap at REDUCED\r\n");
        plan.power_mode = MODE_REDUCED;
    }
    /* F8 (#172): asymmetric mode hysteresis. DOWNGRADES apply immediately
     * (energy protection must never wait); UPGRADES (toward higher power)
     * require F8_UPGRADE_CONFIRM consecutive cycles proposing the upgrade —
     * one 10 mV ADC deviation is ~60 mV/h of slope noise, larger than every
     * mode threshold, so unfiltered selection chatters at dawn/dusk/load. */
    #define F8_UPGRADE_CONFIRM  3U
    {
        OperatingMode_t proposed = plan.power_mode;
        if (!slope_state->mode_hyst_valid) {
            slope_state->mode_hyst_valid = 1;
            slope_state->committed_mode = (uint8_t)proposed;
            slope_state->upgrade_streak = 0;
            slope_state->hyst_last_ts = now_timestamp;
        } else {
            /* F-4 (#179): a streak advance requires REAL elapsed time since
             * the last evaluation. OnTxData re-arms SendTxData once per bulk
             * burst packet (up to 20) with the same now_timestamp; without
             * this gate "three consecutive work cycles" became "three
             * seconds" mid-burst - hysteresis and the RV-07 float guard both
             * bypassed exactly when the radio is most expensive. A zero-dt
             * (or backward-step) call is the same observation: the proposal
             * is held, not counted. */
            bool new_observation = (now_timestamp > slope_state->hyst_last_ts);
            if (new_observation) {
                slope_state->hyst_last_ts = now_timestamp;
            }
            if (proposed < (OperatingMode_t)slope_state->committed_mode) {
                /* upgrade requested */
                bool confirm = false;
                if (new_observation) {
                    /* S-E (#214): the streak must count a CONSISTENT
                     * proposal - previously any upgrade proposal advanced
                     * it, so NORMAL->CONSERVATIVE->NORMAL confirmed NORMAL
                     * on cycle 3 (a two-level jump on mixed evidence). A
                     * changed target restarts the streak at 1. */
                    if (slope_state->upgrade_streak == 0 ||
                        (uint8_t)proposed != slope_state->hyst_last_proposal) {
                        slope_state->upgrade_streak = 1;
                    } else {
                        slope_state->upgrade_streak++;
                    }
                    slope_state->hyst_last_proposal = (uint8_t)proposed;
                    confirm = (slope_state->upgrade_streak >= F8_UPGRADE_CONFIRM);
                }
                if (confirm) {
                    slope_state->committed_mode = (uint8_t)proposed;
                    slope_state->upgrade_streak = 0;
                    SONDE_LOG_STR("PREDICT: sustained upgrade confirmed -> mode change\r\n");
                } else {
                    plan.power_mode = (OperatingMode_t)slope_state->committed_mode;
                }
            } else {
                /* downgrade or unchanged: immediate */
                slope_state->committed_mode = (uint8_t)proposed;
                slope_state->upgrade_streak = 0;
            }
        }
    }
    plan.tx_interval_ms = ApplyOperatingMode(plan.power_mode,
                                             &plan.gps_enabled,
                                             &plan.gps_timeout_ms);

    /* Veto evaluation — first veto wins, record WHY (DDR-0003). */
    plan.veto = VETO_NONE;

    /* RV-08 (#164, DDR-0021 conformance): the temperature-based GPS lockout
     * is REMOVED. Float-altitude ambient is genuinely -50 to -70 C, so the
     * veto made GPS impossible exactly when the airframe is at float; composed
     * with #141's GPS-loss silence it produced a 6 h dark sawtooth for the
     * entire multi-week float (veto -> dark -> forced fix -> one TX -> veto).
     * The field evidence (#128) is that the ATGM336H operates at cold/altitude
     * without special handling. temp_stale still skips normalization above
     * (R2-10) — data honesty, not a veto. The config field
     * gps_temperature_lockout remains for backcompat but is no longer read. */

    /* T1 ladder (DDR-0018): FLIGHT with no session = RF silence. The cycle
     * still runs (GPS + flash logging); only the radio stays dark. */
    if (!joined && !commissioning) {
        plan.veto = VETO_RF_SILENCE;
    }

    return plan;
}
