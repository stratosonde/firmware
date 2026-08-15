/**
  ******************************************************************************
  * @file    tx_fsm.h
  * @brief   Pure TX state machine decision logic (refactor stage 5)
  ******************************************************************************
  * The decision core of lora_app.c's adaptive transmit strategy, extracted
  * as a HAL-free step module. This module owns ONLY the transitions of
  * (state, bulk_packets_sent, probe_sent_ms, burst_opened_ms, science_due_ms)
  * and the decisions derived from them. Every side effect (LmHandlerSend,
  * flash watermark/header ops, UTIL_SEQ arming, UTIL_TIMER programming,
  * MultiRegion accounting, packet-queue drain) stays in the lora_app.c
  * adapters, which call these functions and then perform the mandated
  * actions.
  *
  * Adapter contract (fidelity notes the adapters MUST honour):
  * - BURST-02: never call TxFsm_OnTxConfirm for an MlmeConfirm re-entry
  *   (params->IsMcpsConfirm == 0) - only the MCPS confirm describes an
  *   application uplink.
  * - BURST-01/FR-14: burst termination at packet counts 0 and 1 is not the
  *   confirm path's to decide; this module's confirm tail acts on > 1 only.
  * - R3-01 (#215): call TxFsm_OnWorkCycle exactly once per SendTxData
  *   invocation; program TxTimer from out->timer_delay_ms every time (the
  *   sole arming path, LT-07/#277).
  * - R3-04 (#218): for TXFSM_ACT_SEND_BULK the adapter gathers the pipeline
  *   facts FIRST (read records, convert, query the payload budget - the
  *   firmware order), calls the step, then acts: retire when told, else
  *   encode -> LinkCheckReq when flagged (first packet only, post-encode)
  *   -> send -> TxFsm_OnSendResult. Marking records sent at send success is
  *   the adapter's job.
  * - DDR-0005 (#34): the probe is a CONFIRMED uplink; the archive
  *   opportunity opens only via TxFsm_OnTxConfirm with ack_received.
  ******************************************************************************
  */

#ifndef TX_FSM_H
#define TX_FSM_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/* Values match lora_app.h TxState_t (PROBE_SF10=0 .. COMPLETE=3). */
typedef enum {
    TX_FSM_PROBE_SF10 = 0,
    TX_FSM_WAIT_PROBE_ACK,
    TX_FSM_BULK_TRANSFER,
    TX_FSM_COMPLETE
} TxFsmState_t;

typedef struct {
    TxFsmState_t state;
    uint8_t  bulk_packets_sent;
    uint32_t probe_sent_ms;      /* LT-07: probe-ACK wait stamp (0 = none) */
    uint32_t burst_opened_ms;    /* LT-07: burst-open stamp (0 = none) */
    uint32_t science_due_ms;     /* R3-01: absolute science deadline */
} TxFsm_t;

void TxFsm_Init(TxFsm_t *fsm);   /* PROBE_SF10, counters/stamps zeroed */

/* ---- SendTxData (one work-cycle invocation) ---- */

typedef struct {
    uint32_t now_ms;
    uint32_t interval_ms;        /* plan.tx_interval_ms */
    bool     rf_silence;         /* T1 (DDR-0018) */
    bool     has_unsent;         /* FlashLog_HasUnsentData at case entry */
    bool     recovery_empty;     /* recovery read: error or 0 records */
    bool     unconvertible;      /* every read record failed conversion */
    bool     no_budget;          /* LoRaMacQueryTxPossible failed for all n */
    uint8_t  max_bulk_packets;   /* CfgMaxBulkPkts() */
    uint32_t burst_max_open_ms;  /* BURST_MAX_OPEN_MS */
} TxFsmCycleInput_t;

typedef enum {
    TXFSM_ACT_NONE = 0,          /* cycle ends without an uplink */
    TXFSM_ACT_SEND_PROBE,        /* send the confirmed compact probe */
    TXFSM_ACT_SEND_BULK          /* send one archive packet */
} TxFsmAction_t;

typedef struct {
    TxFsmAction_t action;
    bool     linkcheck_req;      /* LinkCheckReq rides this bulk send */
    bool     flush_header_sync;  /* FlashLog_FlushHeaderSync this invocation */
    bool     retire_batch;       /* unconvertible batch: retire without TX */
    uint32_t timer_delay_ms;     /* program TxTimer with this (always valid) */
} TxFsmCycleOutput_t;

/* The work cycle is phased to mirror SendTxData's real call sites:
 * entry yield (R3-01), then the timer reschedule, then the dispatch
 * (LT-07 forcing, stale reset, RF-silence park, probe/bulk). The firmware
 * adapter calls them with a FRESH HAL_GetTick at each phase (a GPS
 * acquisition can elapse between them); equal now_ms values compose to
 * the atomic step the shadow-run validates. */
void     TxFsm_OnCycleEntry(TxFsm_t *fsm, uint32_t now_ms);
uint32_t TxFsm_Reschedule(TxFsm_t *fsm, uint32_t now_ms, uint32_t interval_ms);
void     TxFsm_Dispatch(TxFsm_t *fsm, const TxFsmCycleInput_t *in,
                        TxFsmCycleOutput_t *out);

/* Pure queries for adapter reachability guards (e.g. the adapter must not
 * read flash for a bulk dispatch that LT-07 forcing will preempt). */
bool TxFsm_ScienceIsDue(const TxFsm_t *fsm, uint32_t now_ms);
bool TxFsm_BurstStale(const TxFsm_t *fsm, uint32_t now_ms,
                      uint32_t burst_max_open_ms);
bool TxFsm_ProbeStale(const TxFsm_t *fsm, uint32_t now_ms,
                      uint32_t burst_max_open_ms);

/* Direct deadline re-base (first-flight survival retry path re-arms at
 * now + retry_ms, bypassing the phase-preserving advance). */
void TxFsm_SetScienceDue(TxFsm_t *fsm, uint32_t due_ms);

/* Report the outcome of a mandated send. has_unsent_now is re-queried AFTER
 * the adapter's send-time watermark advance. Probe: ok -> WAIT (stamped),
 * fail -> COMPLETE. Bulk: ok -> count and stay-or-complete, fail ->
 * COMPLETE. */
void TxFsm_OnSendResult(TxFsm_t *fsm, uint32_t now_ms, bool send_ok,
                        bool has_unsent_now, uint8_t max_bulk_packets);

/* ---- OnTxData (MCPS confirm only - see BURST-02 above) ---- */

typedef struct {
    uint32_t now_ms;
    bool     status_ok;
    bool     ack_received;
    bool     battery_good;       /* s_cycle_batt_mv >= CfgBulkBattMin() */
    bool     has_unsent;
    bool     mission_ascent;     /* R3-03 (#217) */
    uint8_t  max_bulk_packets;
} TxFsmConfirmInput_t;

typedef struct {
    bool arm_send_task;          /* UTIL_SEQ_SetTask the send task */
    bool defer_header_sync;      /* FlashLog_DeferHeaderSync (burst open) */
} TxFsmEventOutput_t;

void TxFsm_OnTxConfirm(TxFsm_t *fsm, const TxFsmConfirmInput_t *in,
                       TxFsmEventOutput_t *out);

/* ---- OnRxData ---- */

typedef struct {
    bool    linkcheck_received;
    uint8_t margin;
    uint8_t gateways;
    uint8_t margin_min;          /* CfgLinkMargin() */
    uint8_t gateways_min;        /* CfgGatewayCount() */
    bool    has_unsent;
    uint8_t max_bulk_packets;
} TxFsmRxInput_t;

void TxFsm_OnRx(TxFsm_t *fsm, const TxFsmRxInput_t *in,
                TxFsmEventOutput_t *out);

/* ---- OnTxTimerEvent: arms the send task; never touches the timer ---- */

void TxFsm_OnTxTimer(TxFsmEventOutput_t *out);

/* ---- FirstFlightAbortTransmitCycle -------------------------------
 * No-op from PROBE_SF10 (returns false); otherwise parks the FSM and
 * discards every burst-scoped stamp (returns true: caller flushes). */
bool TxFsm_OnAbort(TxFsm_t *fsm);

#ifdef __cplusplus
}
#endif

#endif /* TX_FSM_H */
