/**
  ******************************************************************************
  * @file    tx_fsm.c
  * @brief   Pure TX state machine decision logic (refactor stage 5)
  ******************************************************************************
  * Every transition below is a faithful transcription of the behaviour
  * characterised in tests/host/test_burst_fsm.c (stage 5 step 1); the shadow
  * suite (test_tx_fsm_shadow.c) asserts step/model agreement on scripted and
  * randomized event streams. No HAL, no globals, no side effects.
  ******************************************************************************
  */

#include "tx_fsm.h"

void TxFsm_Init(TxFsm_t *fsm)
{
    fsm->state = TX_FSM_PROBE_SF10;
    fsm->bulk_packets_sent = 0;
    fsm->probe_sent_ms = 0;
    fsm->burst_opened_ms = 0;
    fsm->science_due_ms = 0;
}

static bool science_is_due(const TxFsm_t *fsm, uint32_t now_ms)
{
    /* R3-01: wrap-safe absolute deadline; false while unscheduled. */
    return fsm->science_due_ms != 0 &&
           (int32_t)(now_ms - fsm->science_due_ms) >= 0;
}

static uint32_t reschedule(TxFsm_t *fsm, uint32_t now_ms,
                           uint32_t interval_ms, bool science_cycle)
{
    if (science_cycle) {
        if (fsm->science_due_ms == 0) fsm->science_due_ms = now_ms;
        fsm->science_due_ms += interval_ms;
        if ((int32_t)(fsm->science_due_ms - now_ms) <= 0) {
            fsm->science_due_ms = now_ms + interval_ms;  /* >=1 period behind */
        }
    } else if (fsm->science_due_ms == 0) {
        fsm->science_due_ms = now_ms + interval_ms;
    }
    /* LT-01 (#269): clamp in the signed domain - overdue means "fire now". */
    int32_t remain_ms = (int32_t)(fsm->science_due_ms - now_ms);
    if (remain_ms <= 0) remain_ms = 1;
    return (uint32_t)remain_ms;
}

void TxFsm_OnWorkCycle(TxFsm_t *fsm, const TxFsmCycleInput_t *in,
                       TxFsmCycleOutput_t *out)
{
    out->action = TXFSM_ACT_NONE;
    out->linkcheck_req = false;
    out->flush_header_sync = false;
    out->retire_batch = false;

    /* R3-01 (#215): a bulk continuation yields when the science deadline
     * arrives - current science always wins (DDR-0005 BR-TX-001/002). */
    if (fsm->state == TX_FSM_BULK_TRANSFER && science_is_due(fsm, in->now_ms)) {
        fsm->state = TX_FSM_COMPLETE;
        fsm->bulk_packets_sent = 0;
    }

    out->timer_delay_ms = reschedule(fsm, in->now_ms, in->interval_ms,
                                     fsm->state != TX_FSM_BULK_TRANSFER);

    /* LT-07 (#277): burst-scoped deadlines force a stuck network-wait state
     * out; the stale-state reset below then runs as for any completed
     * cycle. */
    if (fsm->state == TX_FSM_WAIT_PROBE_ACK && fsm->probe_sent_ms != 0 &&
        (uint32_t)(in->now_ms - fsm->probe_sent_ms) > in->burst_max_open_ms) {
        fsm->state = TX_FSM_COMPLETE;
        fsm->probe_sent_ms = 0;
    }
    if (fsm->state == TX_FSM_BULK_TRANSFER && fsm->burst_opened_ms != 0 &&
        (uint32_t)(in->now_ms - fsm->burst_opened_ms) > in->burst_max_open_ms) {
        fsm->state = TX_FSM_COMPLETE;
        fsm->burst_opened_ms = 0;
        fsm->bulk_packets_sent = 0;
    }

    /* C2/SP-15: stale states reset to PROBE; the deferred header sync flushes
     * exactly once here (Finding #8). BULK_TRANSFER is NOT reset. */
    if (fsm->state == TX_FSM_WAIT_PROBE_ACK || fsm->state == TX_FSM_COMPLETE) {
        out->flush_header_sync = true;
        fsm->state = TX_FSM_PROBE_SF10;
        fsm->bulk_packets_sent = 0;
    }

    /* T1 (DDR-0018) + F-5 (#180): RF silence parks the FSM (with a flush so a
     * deferred watermark is never stranded) and skips all transmit work. */
    if (in->rf_silence) {
        out->flush_header_sync = true;
        fsm->state = TX_FSM_PROBE_SF10;
        return;
    }

    if (fsm->state == TX_FSM_PROBE_SF10) {
        out->action = TXFSM_ACT_SEND_PROBE;   /* DDR-0005: confirmed uplink */
        return;
    }
    if (fsm->state == TX_FSM_BULK_TRANSFER) {
        /* Firmware order: entry check -> read -> convert -> budget; every
         * failure path ends the cycle without an uplink. */
        if (!in->has_unsent || fsm->bulk_packets_sent >= in->max_bulk_packets) {
            fsm->state = TX_FSM_COMPLETE;
            return;
        }
        if (in->recovery_empty) { fsm->state = TX_FSM_COMPLETE; return; }
        if (in->unconvertible) {
            /* R21 (#51)/FR-09 (#92): retire the consumed batch with no TX -
             * leaving it pending re-probes it forever and wedges bulk. */
            out->retire_batch = true;
            fsm->state = TX_FSM_COMPLETE;
            return;
        }
        if (in->no_budget) { fsm->state = TX_FSM_COMPLETE; return; }
        out->action = TXFSM_ACT_SEND_BULK;
        out->linkcheck_req = (fsm->bulk_packets_sent == 0);  /* protocol §5.2 */
        return;
    }
    /* TX_FSM_COMPLETE / unreachable: reset for the next cycle. */
    fsm->state = TX_FSM_PROBE_SF10;
    fsm->bulk_packets_sent = 0;
}


void TxFsm_OnSendResult(TxFsm_t *fsm, uint32_t now_ms, bool send_ok,
                        bool has_unsent_now, uint8_t max_bulk_packets)
{
    if (fsm->state == TX_FSM_PROBE_SF10) {
        if (send_ok) {
            fsm->probe_sent_ms = now_ms;    /* LT-07: bound the wait */
            fsm->state = TX_FSM_WAIT_PROBE_ACK;
        } else {
            fsm->state = TX_FSM_COMPLETE;   /* complete cycle on error */
        }
        return;
    }
    if (fsm->state == TX_FSM_BULK_TRANSFER) {
        if (!send_ok) {
            fsm->state = TX_FSM_COMPLETE;   /* nothing was marked (adapter) */
            return;
        }
        fsm->bulk_packets_sent++;
        if (!has_unsent_now || fsm->bulk_packets_sent >= max_bulk_packets) {
            fsm->state = TX_FSM_COMPLETE;
        }
        return;
    }
    /* A send result for any other state is a stray callback: ignore. */
}

void TxFsm_OnTxConfirm(TxFsm_t *fsm, const TxFsmConfirmInput_t *in,
                       TxFsmEventOutput_t *out)
{
    out->arm_send_task = false;
    out->defer_header_sync = false;

    /* DDR-0005 (#34): the confirmed probe heartbeat opens the archive
     * opportunity. No ACK -> stay in long-range mode (protocol §5.1/§15). */
    if (fsm->state == TX_FSM_WAIT_PROBE_ACK) {
        fsm->probe_sent_ms = 0;  /* LT-07: the wait is resolved either way */
        if (in->status_ok && in->ack_received) {
            /* R3-03 (#217): during ASCENT live science outranks recovery. */
            if (in->battery_good && in->has_unsent && !in->mission_ascent) {
                out->defer_header_sync = true;   /* Finding #8 */
                fsm->burst_opened_ms = in->now_ms;  /* LT-07 */
                fsm->state = TX_FSM_BULK_TRANSFER;
                fsm->bulk_packets_sent = 0;
                out->arm_send_task = true;
            } else {
                fsm->state = TX_FSM_COMPLETE;
            }
        } else {
            fsm->state = TX_FSM_COMPLETE;
        }
    }

    /* BURST-01: packet counts 0 and 1 are NOT this path's to terminate
     * (0: the ACK branch above just opened the opportunity; 1: OnRxData's
     * LinkCheck verdict owns it). Only sent > 1 is decidable here. */
    if (fsm->state == TX_FSM_BULK_TRANSFER && fsm->bulk_packets_sent > 1) {
        if (in->has_unsent && fsm->bulk_packets_sent < in->max_bulk_packets) {
            out->arm_send_task = true;
        } else {
            fsm->state = TX_FSM_COMPLETE;
            fsm->bulk_packets_sent = 0;
        }
    }
}

void TxFsm_OnRx(TxFsm_t *fsm, const TxFsmRxInput_t *in,
                TxFsmEventOutput_t *out)
{
    out->arm_send_task = false;
    out->defer_header_sync = false;

    /* FR-14 (#88): the packet-1 LinkCheck verdict owns burst continuation. */
    if (fsm->state == TX_FSM_BULK_TRANSFER && fsm->bulk_packets_sent == 1) {
        bool link_good = (in->linkcheck_received &&
                          in->margin >= in->margin_min &&
                          in->gateways >= in->gateways_min);
        if (!link_good) {
            /* Protocol §5.4: end the burst (one-pass: records already marked
             * at send time - no loss, no retry). */
            fsm->state = TX_FSM_COMPLETE;
            fsm->bulk_packets_sent = 0;
        } else if (in->has_unsent &&
                   fsm->bulk_packets_sent < in->max_bulk_packets) {
            out->arm_send_task = true;
        } else {
            fsm->state = TX_FSM_COMPLETE;
            fsm->bulk_packets_sent = 0;
        }
    }
}

void TxFsm_OnTxTimer(TxFsmEventOutput_t *out)
{
    /* LT-07 (#277): arm the task only - the timer period is owned solely by
     * the work-cycle step (timer_delay_ms). */
    out->arm_send_task = true;
    out->defer_header_sync = false;
}

bool TxFsm_OnAbort(TxFsm_t *fsm)
{
    /* A low-energy decision wins over every open transmit state. */
    if (fsm->state == TX_FSM_PROBE_SF10) return false;
    fsm->state = TX_FSM_PROBE_SF10;
    fsm->bulk_packets_sent = 0;
    fsm->probe_sent_ms = 0;
    fsm->burst_opened_ms = 0;
    return true;   /* caller flushes the deferred header sync */
}

