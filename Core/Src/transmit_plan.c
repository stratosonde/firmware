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

uint32_t OutageBackoff_Interval(uint32_t base_ms, uint32_t no_ack_s) {
  /* 1h/4h/12h rungs -> x2/x4/x8, capped at the shared max interval. */
  uint32_t mult = 1U;
  if (no_ack_s >= 43200U) { /* 12 h */
    mult = 8U;
  } else if (no_ack_s >= 14400U) { /* 4 h */
    mult = 4U;
  } else if (no_ack_s >= 3600U) { /* 1 h */
    mult = 2U;
  }
  uint32_t scaled = base_ms * mult;
  if (scaled > CONFIG_MAX_TX_INTERVAL_MS || scaled < base_ms /* overflow */) {
    scaled = CONFIG_MAX_TX_INTERVAL_MS;
  }
  return scaled;
}

TransmitPlan_t DecideTransmitPlan(uint16_t battery_mv_raw,
                                  float temperature_c,
                                  bool temp_stale,
                                  bool joined,
                                  bool commissioning) {
  TransmitPlan_t plan;

  /* PWR-SIMPLIFY: fixed configured cadence; the two hard admission gates run
   * upstream in first_flight_policy.c. The ladder (slope, predictions,
   * hysteresis, mode fallbacks) is deleted; every admitted cycle budgets
   * GNSS and reports MODE_NORMAL. */
  plan.power_mode = MODE_NORMAL;
  plan.gps_enabled = true;

  /* Cadence: the single configured science target. Admission rejects never
   * reach this planner — they retry at tx_interval_survival upstream
   * (FirstFlightWakeAdmitted in lora_app.c). */
  const SystemConfig_t *config = Config_Get();
  plan.tx_interval_ms = (config != NULL) ? config->tx_interval_normal : 300000U;
  plan.gps_timeout_ms = (config != NULL)
                            ? (uint32_t)config->gps_timeout_normal * 1000U
                            : 60000U;

  /* SoC telemetry normalization (not a safety input). R10 (#37): only with
   * a FRESH temperature — a stale temp feeding normalization can fabricate
   * confidence; raw voltage is the safe input. */
  plan.battery_mv_normalized = temp_stale ? battery_mv_raw
                                          : NormalizeBatteryVoltage(battery_mv_raw, temperature_c);



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
