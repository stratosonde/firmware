/**
  ******************************************************************************
  * @file    lora_app_adapters.c
  * @brief   Production-compiled adapter builders (MAINT-01). Pure: no HAL,
  *          no LoRaMac, no flash, no timers, no globals - see the header.
  ******************************************************************************
  */

#include "lora_app_adapters.h"

TxFsmConfirmInput_t
AppAdapters_BuildTxConfirm(const AppTxConfirmSnapshot_t *snap) {
  TxFsmConfirmInput_t in;
  in.now_ms = snap->now_ms;
  in.status_ok = (snap->tx_status == APP_TX_STATUS_OK);
  in.ack_received = (snap->ack_received_raw != 0);
  in.battery_good = (snap->battery_mv >= snap->bulk_batt_min_mv);
  in.has_unsent = snap->has_cache;
  /* TX-ADAPTER-01: the positive mapping, exactly here and nowhere else. */
  in.mission_ascent = (snap->mission_state == MISSION_ASCENT);
  in.max_bulk_packets = snap->max_bulk_packets;
  return in;
}

TxFsmCycleInput_t AppAdapters_BuildTxCycle(const AppTxCycleSnapshot_t *snap) {
  TxFsmCycleInput_t in;
  in.now_ms = snap->now_ms;
  in.interval_ms = 0; /* dispatch phase never reschedules (see header) */
  in.rf_silence = snap->rf_silence;
  in.has_unsent = snap->has_unsent;
  in.recovery_empty = snap->recovery_empty;
  in.unconvertible = snap->unconvertible;
  in.no_budget = snap->no_budget;
  in.max_bulk_packets = snap->max_bulk_packets;
  in.burst_max_open_ms = snap->burst_max_open_ms;
  return in;
}

TxFsmRxInput_t AppAdapters_BuildRx(const AppRxSnapshot_t *snap) {
  TxFsmRxInput_t in;
  in.linkcheck_received = snap->linkcheck_received;
  in.margin = snap->margin;
  in.gateways = snap->gateways;
  in.margin_min = snap->margin_min;
  in.gateways_min = snap->gateways_min;
  in.has_unsent = snap->has_unsent;
  in.max_bulk_packets = snap->max_bulk_packets;
  return in;
}

bool AppAdapters_RegionDiffers(uint8_t detected_region,
                               uint8_t active_region) {
  return detected_region != active_region;
}

bool AppAdapters_SwitchWasRequired(uint8_t before_region,
                                   uint8_t detected_region) {
  return before_region != detected_region;
}

bool AppAdapters_ActiveMatchesDetected(uint8_t active_region_after,
                                       uint8_t detected_region) {
  return active_region_after == detected_region;
}

FirstFlightAdmissionInput_t
AppAdapters_BuildFirstFlightAdmission(const AppFirstFlightSnapshot_t *snap) {
  FirstFlightAdmissionInput_t in;
  in.temperature_c = snap->temperature_c;
  in.temperature_fresh = (snap->temperature_stale == 0U);
  in.battery_mv_raw = snap->battery_mv_raw;
  in.battery_fresh = (snap->battery_stale == 0U);
  return in;
}
