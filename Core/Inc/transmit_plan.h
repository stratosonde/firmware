/**
  ******************************************************************************
  * @file    transmit_plan.h
  * @brief   R47 (#44): transmit-cycle decide half — pure logic, host-testable
  ******************************************************************************
  * @attention
  *
  * The decide half of SendTxData(): takes raw inputs (voltages, temperature,
  * time, join state) and produces a TransmitPlan_t — pure data, no hardware.
  * The executor (SendTxData) reads the plan and does the doing. A skipped or
  * degraded cycle records WHY (veto), not just THAT (DDR-0007).
  *
  ******************************************************************************
  */

#ifndef TRANSMIT_PLAN_H
#define TRANSMIT_PLAN_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "power_model.h"

/** @brief Why a cycle was degraded/skipped — first veto wins */
typedef enum {
    VETO_NONE = 0,          /**< go */
    VETO_TEMP_STALE,        /**< stale temperature treated as COLD (fail-safe) */
    VETO_TEMP_LOCKOUT,      /**< below GPS temperature lockout */
    VETO_RF_SILENCE,        /**< FLIGHT with no valid session (DDR-0006) */
    VETO_RESTRICTED_REGION  /**< regulatory RF prohibition (set by executor) */
} TransmitVeto_t;

/** @brief Pure-data output of the decide half */
typedef struct {
    OperatingMode_t power_mode;       /**< selected operating mode */
    bool     gps_enabled;             /**< wake GPS this cycle */
    uint32_t gps_timeout_ms;          /**< acquisition bound */
    uint32_t tx_interval_ms;          /**< next wake interval */
    int16_t  voltage_slope_mv_per_hour; /**< temperature-normalized slope */
    int16_t  time_to_target_h;        /**< signed: +hours to full, -hours to critical, 0 stable */
    uint16_t battery_mv_normalized;   /**< 25C-equivalent battery voltage */
    TransmitVeto_t veto;              /**< VETO_NONE = go */
} TransmitPlan_t;

/**
  * @brief  Decide what this work cycle should do. Touches NO hardware.
  * @param  slope_state: persistent voltage-slope tracker (caller-owned)
  * @param  battery_mv_raw: raw battery voltage (normalization happens inside)
  * @param  temperature_c / temp_stale: temperature + honesty flag
  * @param  now_timestamp: monotonic seconds (RTC-derived)
  * @param  joined: LoRaWAN session status
  * @param  commissioning: mission state
  * @retval plan (pure data)
  */
TransmitPlan_t DecideTransmitPlan(VoltageSlope_t *slope_state,
                                  uint16_t battery_mv_raw,
                                  float temperature_c,
                                  bool temp_stale,
                                  uint32_t now_timestamp,
                                  bool joined,
                                  bool commissioning);

#ifdef __cplusplus
}
#endif

#endif /* TRANSMIT_PLAN_H */
