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
#include "SEGGER_RTT.h"
#include "config.h"
#include "sonde_log.h" /* R50 (#47): compile-time log gate */
#include <stdio.h>
#include <stdlib.h>

/**
 * @brief Apply operating mode configuration (moved verbatim from lora_app.c)
 * @param mode Operating mode to apply
 * @param gps_enabled Output parameter for GPS enable state
 * @param gps_timeout_ms Output parameter for GPS timeout
 * @return New transmission interval in milliseconds
 */
static uint32_t ApplyOperatingMode(OperatingMode_t mode, bool *gps_enabled,
                                   uint32_t *gps_timeout_ms) {
  const SystemConfig_t *config = Config_Get();
  uint32_t interval_ms;

  /* First flight has no intentional GNSS-less degradation mode. Admission
   * happens before this planner; every admitted mode gets a real budget. */
  *gps_enabled = true;
  *gps_timeout_ms = (config != NULL)
                        ? (uint32_t)config->gps_timeout_conservative * 1000U
                        : 60000U;

  switch (mode) {
  case MODE_NORMAL:
    interval_ms = (config != NULL) ? config->tx_interval_normal : 300000U;
    if (config != NULL) {
      *gps_timeout_ms = (uint32_t)config->gps_timeout_normal * 1000U;
    }
    break;
  case MODE_CONSERVATIVE:
    interval_ms = (config != NULL) ? config->tx_interval_conservative : 600000U;
    break;
  case MODE_REDUCED:
    interval_ms = (config != NULL) ? config->tx_interval_reduced : 900000U;
    break;
  case MODE_RECOVERY:
    interval_ms = (config != NULL) ? config->tx_interval_recovery : 1800000U;
    break;
  case MODE_SURVIVAL:
    interval_ms = (config != NULL) ? config->tx_interval_survival : 3600000U;
    break;
  default:
    interval_ms = (config != NULL) ? config->tx_interval_conservative : 600000U;
    break;
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
                                  bool batt_stale) {
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
  uint16_t time_to_critical = (crit_state == PRED_AT_OR_PAST) ? 0 : (crit_state == PRED_REACHABLE) ? crit_hours
                                                                                                   : 0xFFFF;

  if (crit_state == PRED_AT_OR_PAST) {
    plan.time_to_target_h = -1; /* critical now, no computable ETA */
  } else if (crit_state == PRED_REACHABLE) {
    plan.time_to_target_h = (crit_hours > 0) ? -(int16_t)crit_hours : -1;
  } else if (full_state == PRED_REACHABLE) {
    plan.time_to_target_h = (int16_t)full_hours;
  } else {
    plan.time_to_target_h = 0; /* genuinely stable / moving away */
  }

  /* Mode selection + application (R10: floor reads RAW voltage) */
  plan.power_mode = SelectModeFromPredictions(plan.voltage_slope_mv_per_hour,
                                              plan.battery_mv_normalized,
                                              time_to_critical,
                                              battery_mv_raw);

  /* R2-11 (#115): with no slope history SelectModeFromPredictions falls
   * through every branch to MODE_CONSERVATIVE (10-min cadence)
   * even on a marginal battery right after a brownout reset. Fail the
   * other way: below 5000 mV raw (marginal supercap), start REDUCED cadence
   * until a real slope exists. Never loosen a stricter mode
   * (the R10 raw floor may already have picked SURVIVAL). NOTE (finding #9,
   * 2026-08-10): the slope baseline is RAM-ONLY — the backup-register
   * persistence (DR12-15) once described here was never implemented. This
   * no-history REDUCED fallback is the live mitigation; if reset-stable
   * history is ever wanted, implement DR persistence for real. */
  if (!have_history && battery_mv_raw < 5000 && plan.power_mode < MODE_REDUCED) {
    SONDE_LOG_STR("PREDICT: no slope history + marginal raw V -> REDUCED\r\n");
    plan.power_mode = MODE_REDUCED;
  }
  /* RV-03 (#161): keep a conservative cadence for pure-planner callers with
   * stale battery input. First-flight orchestration rejects this input before
   * reaching the planner. */
  if (batt_stale && plan.power_mode < MODE_REDUCED) {
    SONDE_LOG_STR("PREDICT: battery reading stale -> cap at REDUCED\r\n");
    plan.power_mode = MODE_REDUCED;
  }
/* F8 (#172): asymmetric mode hysteresis. DOWNGRADES apply immediately
 * (energy protection must never wait); UPGRADES (toward higher power)
 * require F8_UPGRADE_CONFIRM consecutive cycles proposing the upgrade —
 * one 10 mV ADC deviation is ~60 mV/h of slope noise, larger than every
 * mode threshold, so unfiltered selection chatters at dawn/dusk/load. */
#define F8_UPGRADE_CONFIRM 3U
  {
    OperatingMode_t proposed = plan.power_mode;
    if (!slope_state->mode_hyst_valid) {
      slope_state->mode_hyst_valid = 1;
      slope_state->committed_mode = (uint8_t)proposed;
      slope_state->upgrade_streak = 0;
      slope_state->hyst_last_ts = now_timestamp;
    } else {
      /* DR-03: RV-06 (#162) re-seed, the consumer that was missed. The
       * LSE->LSI failover re-inits the RTC and restarts the boot-relative
       * counter near zero (F-002/#201) while this state survives in RAM.
       * Without the re-seed, new_observation is false for the whole
       * elapsed-uptime window: upgrades can never confirm while
       * downgrades still apply immediately - a one-way ratchet toward
       * SURVIVAL triggered by exactly the event the failover machinery
       * was built to survive. A backward step is a new clock domain:
       * re-seed the epoch, drop the streak (its time base is gone), and
       * treat this call as the same observation. */
      if (now_timestamp < slope_state->hyst_last_ts) {
        slope_state->hyst_last_ts = now_timestamp;
        slope_state->upgrade_streak = 0;
      }
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

  /* The old temperature-only GNSS veto remains absent here. Temperature and
   * raw battery are now handled together by first-flight admission before
   * this planner runs; admitted wakes always keep GNSS in the package. */

  /* T1 ladder (DDR-0018): FLIGHT with no session = RF silence. The cycle
   * still runs (GPS + flash logging); only the radio stays dark. */
  if (!joined && !commissioning) {
    plan.veto = VETO_RF_SILENCE;
  }

  return plan;
}
