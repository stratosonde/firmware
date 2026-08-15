/**
  ******************************************************************************
  * @file    test_tx_fsm_shadow.c
  * @brief   Stage-5 shadow-run: Core/Src/tx_fsm.c vs the characterised model
  ******************************************************************************
  * Drives the pure step module (tx_fsm.c) and the scan-locked replay model
  * (test_burst_fsm.c, included below) with IDENTICAL event streams -
  * scripted scenarios plus a long randomized fuzz - and asserts the FSM
  * state (state, bulk count, probe/burst stamps, science deadline) and the
  * mandated actions agree after every single event.
  *
  * The model is locked to lora_app.c by the source scan (test_shape_anchors),
  * so step == model implies step == firmware behaviour for every
  * characterised decision.
  ******************************************************************************
  */

#define main burst_suite_main   /* park the included suite's entry point */
#include "test_burst_fsm.c"
#undef main

#include "tx_fsm.h"             /* Core/Inc via -I../../Core/Inc */

/* ---- state agreement ------------------------------------------------ */

static bool states_equal(const TxFsm_t *f)
{
    return (int)f->state == (int)g_tx_state &&
           (int)f->bulk_packets_sent == g_bulk_packets_sent &&
           f->probe_sent_ms == g_probe_sent_ms &&
           f->burst_opened_ms == g_burst_opened_ms &&
           f->science_due_ms == g_science_due_ms;
}

static void report_mismatch(const char *ev, const TxFsm_t *f)
{
    printf("MISMATCH at %s:\n", ev);
    printf("  model: state=%d bulk=%d probe=%lu burst=%lu due=%lu\n",
           (int)g_tx_state, g_bulk_packets_sent,
           (unsigned long)g_probe_sent_ms, (unsigned long)g_burst_opened_ms,
           (unsigned long)g_science_due_ms);
    printf("  step : state=%d bulk=%d probe=%lu burst=%lu due=%lu\n",
           (int)f->state, (int)f->bulk_packets_sent,
           (unsigned long)f->probe_sent_ms, (unsigned long)f->burst_opened_ms,
           (unsigned long)f->science_due_ms);
}

/* ---- shadow drivers: one event applied to both sides ----------------- */

/* Work cycle. probe_fail/bulk_fail/bulk_encode_fail simulate the adapter's
 * encode/send outcomes; the model encodes them as env knobs (its SendTxData
 * owns the whole pipeline), the step gets them via TxFsm_OnSendResult. */
static void shadow_work(TxFsm_t *f, bool probe_fail, bool bulk_fail,
                        bool bulk_encode_fail)
{
    bool pre_has_unsent = has_unsent();
    env_probe_send_fail   = probe_fail;
    env_bulk_send_fail    = bulk_fail;
    env_bulk_encode_fail  = bulk_encode_fail;

    int hb0 = obs_heartbeats, ar0 = obs_archive_packets;
    int rt0 = obs_retire_no_tx, fl0 = obs_flush_calls, lc0 = obs_linkcheck_req;

    bool mlme;
    (void)SendTxData(&mlme);                 /* model side */
    bool post_has_unsent = has_unsent();

    TxFsmCycleInput_t in = {
        now_ms, env_interval_ms, env_rf_silence, pre_has_unsent,
        env_recovery_empty, env_unconvertible, env_no_budget,
        (uint8_t)MAX_BULK_PACKETS_PER_CYCLE, (uint32_t)BURST_MAX_OPEN_MS
    };
    TxFsmCycleOutput_t out;
    TxFsm_OnCycleEntry(f, in.now_ms);        /* step side: the firmware's */
    out.timer_delay_ms = TxFsm_Reschedule(f, in.now_ms, in.interval_ms);
    TxFsm_Dispatch(f, &in, &out);            /* three phase call sites     */

    bool send_ok = true;
    bool lc_issued = false;
    if (out.action == TXFSM_ACT_SEND_PROBE) {
        send_ok = !probe_fail;
        TxFsm_OnSendResult(f, now_ms, send_ok, post_has_unsent,
                           (uint8_t)MAX_BULK_PACKETS_PER_CYCLE);
    } else if (out.action == TXFSM_ACT_SEND_BULK) {
        lc_issued = out.linkcheck_req && !bulk_encode_fail;
        send_ok = !bulk_fail && !bulk_encode_fail;
        TxFsm_OnSendResult(f, now_ms, send_ok, post_has_unsent,
                           (uint8_t)MAX_BULK_PACKETS_PER_CYCLE);
    }

    if (!states_equal(f)) { report_mismatch("work", f); g_failures++; }
    CHECK(out.timer_delay_ms == obs_timer_delay_ms);
    CHECK(out.retire_batch == (obs_retire_no_tx > rt0));
    CHECK(out.flush_header_sync == (obs_flush_calls > fl0));
    CHECK((out.action == TXFSM_ACT_SEND_PROBE && send_ok) == (obs_heartbeats > hb0));
    CHECK((out.action == TXFSM_ACT_SEND_BULK && send_ok) == (obs_archive_packets > ar0));
    CHECK(lc_issued == (obs_linkcheck_req > lc0));
}

static void shadow_confirm(TxFsm_t *f, bool status_ok, bool ack)
{
    seq_task_armed = false;
    int df0 = obs_defer_calls;
    OnTxData(true, status_ok, ack);          /* model side (MCPS only) */

    TxFsmConfirmInput_t in = {
        now_ms, status_ok, ack, env_batt_good, has_unsent(),
        env_mission_ascent, (uint8_t)MAX_BULK_PACKETS_PER_CYCLE
    };
    TxFsmEventOutput_t out;
    TxFsm_OnTxConfirm(f, &in, &out);         /* step side */

    if (!states_equal(f)) { report_mismatch("confirm", f); g_failures++; }
    CHECK(out.arm_send_task == seq_task_armed);
    CHECK(out.defer_header_sync == (obs_defer_calls > df0));
}

static void shadow_rx(TxFsm_t *f)
{
    seq_task_armed = false;
    OnRxData();                              /* model side */

    TxFsmRxInput_t in = {
        env_linkcheck, (uint8_t)env_margin, (uint8_t)env_gateways,
        (uint8_t)LINK_MARGIN_THRESHOLD, (uint8_t)GATEWAY_COUNT_THRESHOLD,
        has_unsent(), (uint8_t)MAX_BULK_PACKETS_PER_CYCLE
    };
    TxFsmEventOutput_t out;
    TxFsm_OnRx(f, &in, &out);                /* step side */

    if (!states_equal(f)) { report_mismatch("rx", f); g_failures++; }
    CHECK(out.arm_send_task == seq_task_armed);
}

static void shadow_timer(TxFsm_t *f)
{
    seq_task_armed = false;
    OnTxTimerEvent();                        /* model side */
    TxFsmEventOutput_t out;
    TxFsm_OnTxTimer(&out);                   /* step side */
    if (!states_equal(f)) { report_mismatch("timer", f); g_failures++; }
    CHECK(out.arm_send_task == seq_task_armed);
}

static void shadow_abort(TxFsm_t *f)
{
    int fl0 = obs_flush_calls;
    FirstFlightAbortTransmitCycle();         /* model side */
    bool flushed = TxFsm_OnAbort(f);         /* step side */
    if (!states_equal(f)) { report_mismatch("abort", f); g_failures++; }
    CHECK(flushed == (obs_flush_calls > fl0));
}

/* ---- scripted scenarios ---------------------------------------------- */

/* A healthy 3-packet burst, event by event through the shadow drivers. */
static void test_scripted_burst(void)
{
    printf("-- scripted: healthy 3-packet burst, step-by-step agreement\n");
    reset_world();
    env_pending_batches = 3;
    TxFsm_t f;
    TxFsm_Init(&f);

    shadow_work(&f, false, false, false);          /* probe out */
    CHECK(obs_heartbeats == 1);
    shadow_confirm(&f, true, true);                /* ACK -> burst opens */
    CHECK((int)f.state == ST_BULK);
    shadow_work(&f, false, false, false);          /* archive packet 1 */
    CHECK(obs_linkcheck_req == 1);
    shadow_confirm(&f, true, true);                /* MCPS confirm (tail: 1) */
    shadow_rx(&f);                                 /* LinkCheckAns verdict */
    shadow_work(&f, false, false, false);          /* packet 2 */
    shadow_confirm(&f, true, true);                /* tail: sent > 1 */
    shadow_rx(&f);
    shadow_work(&f, false, false, false);          /* packet 3 drains backlog */
    CHECK(obs_archive_packets == 3);
    shadow_confirm(&f, true, true);
    shadow_rx(&f);
    CHECK((int)f.state == ST_COMPLETE);
    CHECK(obs_commits == 3);
}

/* Poor link at the packet-1 verdict ends the burst on both sides. */
static void test_scripted_poor_link(void)
{
    printf("-- scripted: poor link after packet 1\n");
    reset_world();
    env_margin = LINK_MARGIN_THRESHOLD - 1;
    TxFsm_t f;
    TxFsm_Init(&f);

    shadow_work(&f, false, false, false);
    shadow_confirm(&f, true, true);
    shadow_work(&f, false, false, false);          /* packet 1 */
    shadow_confirm(&f, true, true);
    shadow_rx(&f);                                 /* fallback */
    CHECK((int)f.state == ST_COMPLETE);
    CHECK(obs_archive_packets == 1);
    CHECK(!seq_task_armed);
}


/* ---- randomized fuzz -------------------------------------------------- */

static uint32_t rng_state = 0x2F6E2B1Cu;
static uint32_t rnd(void)
{
    rng_state = rng_state * 1664525u + 1013904223u;
    return rng_state >> 8;
}

/* Random event streams across the whole input space, including clock wrap
 * (now_ms advances up to 70 s per event over 200k events -> many wraps).
 * Any single disagreement stops the run with a full state dump. */
static void test_fuzz(void)
{
    printf("-- fuzz: 200000 randomized events, per-event agreement\n");
    TxFsm_t f;
    TxFsm_Init(&f);
    reset_world();

    for (int i = 0; i < 200000; i++) {
        now_ms += rnd() % 70000u;
        env_interval_ms     = (rnd() % 4 == 0) ? 1000u :
                              (rnd() % 2 == 0) ? 60000u : 3600000u;
        env_rf_silence      = (rnd() % 12) == 0;
        env_mission_ascent  = (rnd() % 10) == 0;
        env_batt_good       = (rnd() % 5) != 0;
        env_has_unsent      = (rnd() % 5) != 0;
        env_pending_batches = (rnd() % 8 == 0) ? (int)(rnd() % 5) : -1;
        env_recovery_empty  = (rnd() % 20) == 0;
        env_unconvertible   = (rnd() % 20) == 0;
        env_no_budget       = (rnd() % 20) == 0;
        env_linkcheck       = (rnd() % 10) != 0;
        env_margin          = (int)(rnd() % 20);
        env_gateways        = (int)(rnd() % 4);

        switch (rnd() % 20) {
        case 0:
            shadow_timer(&f);
            break;
        case 1: case 2:
            shadow_abort(&f);
            break;
        case 3: case 4: case 5:
            shadow_rx(&f);
            break;
        case 6: case 7: case 8: case 9:
            shadow_confirm(&f, (rnd() % 4) != 0, (rnd() % 3) != 0);
            break;
        default:
            shadow_work(&f, (rnd() % 10) == 0, (rnd() % 10) == 0,
                        (rnd() % 10) == 0);
            break;
        }
        if (g_failures > 0) {
            printf("fuzz stopped at iteration %d\n", i);
            return;
        }
    }
    printf("   200000 events: full agreement\n");
}

int main(void)
{
    printf("=== TX FSM shadow-run: tx_fsm.c vs characterised model ===\n\n");
    scan_firmware();
    if (cfg_tail_terminates_low_counts || cfg_ontxdata_reentrant) {
        printf("FATAL: firmware shows a pre-fix BURST shape; shadow-run is meaningless\n");
        return 2;
    }

    test_scripted_burst();
    test_scripted_poor_link();
    test_fuzz();

    printf("\n%d checks, %d failures\n", g_checks, g_failures);
    if (g_failures == 0) { printf("OK\n"); return 0; }
    return 1;
}

