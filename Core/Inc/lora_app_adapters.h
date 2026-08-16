/**
  ******************************************************************************
  * @file    lora_app_adapters.h
  * @brief   Production-compiled adapter builders (MAINT-01, pretest-hardening
  *          handoff 2026-08-15, Phase 5)
  ******************************************************************************
  * The five high-consequence mappings between lora_app.c's HAL/Mac-facing
  * world and the pure policy/FSM modules, extracted into ONE pure module so
  * each mapping is linked-testable:
  *   1. TxFsmConfirmInput_t  (OnTxData; TX-ADAPTER-01's inverted polarity)
  *   2. TxFsmCycleInput_t    (SendTxData dispatch phase)
  *   3. TxFsmRxInput_t       (OnRxData LinkCheck verdict)
  *   4. region-policy comparisons / post-switch final authorization
  *   5. FirstFlightAdmissionInput_t (FirstFlightWakeAdmitted)
  *
  * No HAL, LoRaMac, flash, timer, or global dependencies: every input arrives
  * as a raw domain value in a snapshot (raw enums, raw staleness counters,
  * raw status integers), so the polarity-bearing derivations - stale==0 means
  * fresh, state==MISSION_ASCENT, status==OK, detected!=active - live HERE,
  * under direct linked test, instead of inline in the callback where an
  * inversion once shipped (TX-ADAPTER-01).
  ******************************************************************************
  */

#ifndef LORA_APP_ADAPTERS_H
#define LORA_APP_ADAPTERS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "mission_state.h"       /* MissionState_t */
#include "tx_fsm.h"              /* TxFsm*Input_t */
#include "first_flight_policy.h" /* FirstFlightAdmissionInput_t */

/* LoRaMac-free mirror of LORAMAC_EVENT_INFO_STATUS_OK (0 in LoRaMac.h). The
 * host suite source-pins the LoRaMac enum value so a renumbering breaks the
 * build here, not the polarity on target. */
#define APP_TX_STATUS_OK 0

/* ---- OnTxData -> TxFsm_OnTxConfirm ---- */

typedef struct {
  uint32_t now_ms;               /* HAL_GetTick() at confirm time */
  int tx_status;                 /* raw params->Status */
  uint8_t ack_received_raw;      /* raw params->AckReceived */
  uint16_t battery_mv;           /* s_cycle_batt_mv (admitted post-GNSS) */
  uint16_t bulk_batt_min_mv;     /* CfgBulkBattMin() */
  MissionState_t mission_state;  /* raw enum - NOT a derived bool (R3-03) */
  bool has_cache;                /* FlashLog_HasUnsentData */
  uint8_t max_bulk_packets;      /* CfgMaxBulkPkts() */
} AppTxConfirmSnapshot_t;

TxFsmConfirmInput_t
AppAdapters_BuildTxConfirm(const AppTxConfirmSnapshot_t *snap);

/* ---- SendTxData dispatch phase -> TxFsm_Dispatch ---- */

typedef struct {
  uint32_t now_ms;
  bool rf_silence;            /* T1 (DDR-0018): any veto silenced RF */
  bool has_unsent;            /* FlashLog_HasUnsentData at case entry */
  bool recovery_empty;        /* recovery read: error or 0 records */
  bool unconvertible;         /* every read record failed conversion */
  bool no_budget;             /* LoRaMacQueryTxPossible failed for all n */
  uint8_t max_bulk_packets;   /* CfgMaxBulkPkts() */
  uint32_t burst_max_open_ms; /* BURST_MAX_OPEN_MS */
} AppTxCycleSnapshot_t;

/* The dispatch-phase input never reschedules: interval_ms is always 0 here.
 * The plan interval belongs to the separate TxFsm_Reschedule phase, which
 * lora_app.c calls with a fresh tick (phased-cycle fidelity note in
 * tx_fsm.h). */
TxFsmCycleInput_t AppAdapters_BuildTxCycle(const AppTxCycleSnapshot_t *snap);

/* ---- OnRxData LinkCheck verdict -> TxFsm_OnRx ---- */

typedef struct {
  bool linkcheck_received;  /* a fresh LinkCheckAns arrived this RX */
  uint8_t margin;           /* dB, valid only when linkcheck_received */
  uint8_t gateways;
  uint8_t margin_min;       /* CfgLinkMargin() */
  uint8_t gateways_min;     /* CfgGatewayCount() */
  bool has_unsent;          /* FlashLog_HasUnsentData */
  uint8_t max_bulk_packets; /* CfgMaxBulkPkts() */
} AppRxSnapshot_t;

TxFsmRxInput_t AppAdapters_BuildRx(const AppRxSnapshot_t *snap);

/* ---- Region-policy comparisons / final authorization (BEH-03) ----
 * Region identities arrive as integers (LoRaMacRegion_t values) so this
 * module stays LoRaMac-free. The comparisons are the polarity surface. */
bool AppAdapters_RegionDiffers(uint8_t detected_region, uint8_t active_region);
bool AppAdapters_SwitchWasRequired(uint8_t before_region,
                                   uint8_t detected_region);
bool AppAdapters_ActiveMatchesDetected(uint8_t active_region_after,
                                       uint8_t detected_region);

/* ---- FirstFlightWakeAdmitted -> FirstFlightPolicy_Decide ---- */

typedef struct {
  float temperature_c;
  uint8_t temperature_stale; /* raw staleness counter; 0 = fresh */
  uint16_t battery_mv_raw;   /* FirstFlightPolicy_VoltsToMvOrZero output */
  uint8_t battery_stale;     /* raw staleness counter; 0 = fresh */
} AppFirstFlightSnapshot_t;

FirstFlightAdmissionInput_t
AppAdapters_BuildFirstFlightAdmission(const AppFirstFlightSnapshot_t *snap);

#ifdef __cplusplus
}
#endif

#endif /* LORA_APP_ADAPTERS_H */
