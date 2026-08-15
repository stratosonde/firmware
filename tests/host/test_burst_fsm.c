/**
  ******************************************************************************
  * @file    test_burst_fsm.c
  * @brief   Behavioural regression for the archive-burst state machine
  *          (2026-08-11 review, BURST-01 / BURST-02) EXTENDED (refactor
  *          stage 5, step 1) into a CHARACTERISATION model of the whole TX
  *          FSM decision space.
  ******************************************************************************
  * WHY THIS IS NOT A PLAIN SOURCE SCAN
  *
  * BURST-01 is a pure control-flow defect: every individual branch in
  * OnTxData/OnRxData/RunTxStateMachine reads correctly, and no grep-able
  * token is wrong. The bug only appears when the three callbacks are replayed
  * in the order LoRaMac actually calls them. A grep cannot see it; a replay
  * can. So this suite REPLAYS the state machine.
  *
  * WHY THE REPLAY CANNOT DRIFT FROM THE FIRMWARE
  *
  * A hand-copied model would go green the moment someone edits the model
  * instead of the firmware. Instead the model is CONFIGURED AND LOCKED BY A
  * SOURCE SCAN: at startup it reads LoRaWAN/App/lora_app.c, detects which
  * shape each decision point currently has, and replays that shape. The
  * stage-5 extension adds scan anchors for every newly characterised
  * decision point (LT-07, R3-01, R3-03, T1/F-5, the deadline arithmetic,
  * the bulk failure paths); if the firmware's shape changes, this suite
  * goes red until the model is updated to match.
  *
  * CALLBACK ORDER (Middlewares/Third_Party/LoRaWAN/Mac/LoRaMac.c,
  * LoRaMacProcess): LoRaMacHandleRequestEvents() dispatches McpsConfirm
  * (-> LmHandler McpsConfirm -> OnTxData) and then MlmeConfirm (-> LmHandler
  * MlmeConfirm -> OnTxData AGAIN), and only afterwards does
  * LoRaMacHandleIndicationEvents() dispatch McpsIndication (-> OnRxData).
  * OnTxData therefore always runs BEFORE OnRxData for the same uplink, and
  * runs twice whenever an MLME request rode along.
  *
  * Run:
  *   make -C tests/host burst      (red until the fixes land)
  *   make -C tests/host baseline   (EXPECT_UNFIXED=1: green pre-fix CI gate)
  ******************************************************************************
  */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

/* Mirrors of the firmware constants under test. Parsed from the firmware at
 * runtime so a retune there cannot silently invalidate the test. */
static int MAX_BULK_PACKETS_PER_CYCLE = 0;   /* lora_app.h */
static int LINK_MARGIN_THRESHOLD      = 0;   /* lora_app.h */
static int GATEWAY_COUNT_THRESHOLD    = 0;   /* lora_app.h */
static int BULK_BATTERY_MIN_MV        = 0;   /* lora_app.h */
static int BURST_MAX_OPEN_MS          = 0;   /* lora_app.c (LT-07) */

static int g_failures = 0;
static int g_checks = 0;
static int g_expected_failures = 0;

#define CHECK(cond) do { \
    g_checks++; \
    if (!(cond)) { \
        g_failures++; \
        printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
    } \
} while (0)

#define CHECK_REGRESSION(cond, id) do { \
    g_checks++; \
    if (!(cond)) { \
        g_failures++; \
        g_expected_failures++; \
        printf("FAIL [%s] %s:%d: %s\n", id, __FILE__, __LINE__, #cond); \
    } \
} while (0)

/* ------------------------------------------------------------------ */
/* Source scan: read the firmware and decide which shape to replay      */
/* ------------------------------------------------------------------ */

static char *slurp(const char *path)
{
    FILE *f = fopen(path, "rb");
    if (!f) { printf("FATAL: cannot open %s\n", path); exit(2); }
    fseek(f, 0, SEEK_END);
    long n = ftell(f);
    fseek(f, 0, SEEK_SET);
    char *buf = (char *)malloc((size_t)n + 1);
    if (!buf) exit(2);
    if (fread(buf, 1, (size_t)n, f) != (size_t)n) exit(2);
    buf[n] = '\0';
    fclose(f);
    return buf;
}


/* Collapse every whitespace run to one space so the scan survives reindenting
 * and line rewrapping. Comments are stripped first: a fix described in prose
 * must never be mistaken for a fix in code. */
static char *normalize_code(const char *src)
{
    size_t n = strlen(src);
    char *out = (char *)malloc(n + 2);
    if (!out) exit(2);
    size_t o = 0;
    bool in_block = false, in_line = false, sp = true;
    for (size_t i = 0; i < n; i++) {
        if (in_block) {
            if (src[i] == '*' && src[i + 1] == '/') { in_block = false; i++; }
            continue;
        }
        if (in_line) {
            if (src[i] == '\n') in_line = false;
            continue;
        }
        if (src[i] == '/' && src[i + 1] == '*') { in_block = true; i++; continue; }
        if (src[i] == '/' && src[i + 1] == '/') { in_line = true;  i++; continue; }
        if (src[i] == ' ' || src[i] == '\t' || src[i] == '\r' || src[i] == '\n') {
            if (!sp) { out[o++] = ' '; sp = true; }
            continue;
        }
        out[o++] = src[i];
        sp = false;
    }
    out[o] = '\0';
    return out;
}

static int parse_define(const char *src, const char *name, int *out)
{
    const char *p = src;
    size_t nl = strlen(name);
    while ((p = strstr(p, "#define")) != NULL) {
        const char *q = p + 7;
        while (*q == ' ' || *q == '\t') q++;
        if (strncmp(q, name, nl) == 0 && (q[nl] == ' ' || q[nl] == '\t')) {
            q += nl;
            while (*q == ' ' || *q == '\t') q++;
            *out = atoi(q);
            return 1;
        }
        p += 7;
    }
    return 0;
}

/* Crude function slice on normalized code: from the signature to the start
 * of the next "static " definition. */
static char *func_slice(const char *norm, const char *sig)
{
    const char *b = strstr(norm, sig);
    if (!b) return NULL;
    const char *e = strstr(b + strlen(sig), " static ");
    if (!e) e = b + strlen(b);
    size_t n = (size_t)(e - b);
    char *out = (char *)malloc(n + 1);
    if (!out) exit(2);
    memcpy(out, b, n);
    out[n] = '\0';
    return out;
}

/* Which shape does the firmware have right now? */
static bool cfg_tail_terminates_low_counts = true;  /* BURST-01: buggy default */
static bool cfg_ontxdata_reentrant         = true;  /* BURST-02: buggy default */

/* Stage-5 characterisation anchors: each locks one decision point the model
 * replays. All are protective shapes the current firmware HAS; if a future
 * edit removes one, test_shape_anchors goes red. */
static bool cfg_yield_science_due;      /* R3-01 (#215): bulk yields to science */
static bool cfg_lt07_probe;             /* LT-07 (#277): probe-ACK wait bound */
static bool cfg_lt07_burst;             /* LT-07 (#277): burst-open bound */
static bool cfg_ascent_gate;            /* R3-03 (#217): ASCENT inhibits archive */
static bool cfg_defer_at_open;          /* Finding #8: header sync deferred at open */
static bool cfg_flush_on_reset;         /* Finding #8: flush at stale-state reset */
static bool cfg_phase_advance;          /* R3-01: deadline advances phase-wise */
static bool cfg_rebase;                 /* R3-01: re-base when a period behind */
static bool cfg_lt01_clamp;             /* LT-01 (#269): overdue -> fire now */
static bool cfg_timer_no_bare_start;    /* LT-07: OnTxTimerEvent never re-arms */
static bool cfg_abort_park;             /* first-flight abort parks the FSM */
static bool cfg_retire_unconvertible;   /* R21 (#51)/FR-09 (#92): retire w/o TX */
static bool cfg_no_budget_complete;     /* no LoRaMac payload budget -> retry */

static void scan_firmware(void)
{
    char *hdr = slurp("../../LoRaWAN/App/lora_app.h");
    if (!parse_define(hdr, "MAX_BULK_PACKETS_PER_CYCLE", &MAX_BULK_PACKETS_PER_CYCLE) ||
        !parse_define(hdr, "LINK_MARGIN_THRESHOLD", &LINK_MARGIN_THRESHOLD) ||
        !parse_define(hdr, "GATEWAY_COUNT_THRESHOLD", &GATEWAY_COUNT_THRESHOLD) ||
        !parse_define(hdr, "BULK_BATTERY_MIN_MV", &BULK_BATTERY_MIN_MV)) {
        printf("FATAL: burst constants not found in lora_app.h\n");
        exit(2);
    }
    free(hdr);

    char *app  = slurp("../../LoRaWAN/App/lora_app.c");
    if (!parse_define(app, "BURST_MAX_OPEN_MS", &BURST_MAX_OPEN_MS)) {
        printf("FATAL: BURST_MAX_OPEN_MS not found in lora_app.c\n");
        exit(2);
    }
    char *norm = normalize_code(app);

    /* BURST-01: the unqualified terminal `else if` is the defect. Its presence
     * means packet counts 0 and 1 are reset to COMPLETE by OnTxData's tail. */
    cfg_tail_terminates_low_counts =
        (strstr(norm, "else if (g_tx_state == TX_STATE_BULK_TRANSFER) {") != NULL);

    /* BURST-02: an early return on the MLME re-entry. */
    cfg_ontxdata_reentrant =
        (strstr(norm, "if (params->IsMcpsConfirm == 0) { return; }") == NULL);

    /* Stage 5.4b: the decision shapes moved VERBATIM into the pure module
     * Core/Src/tx_fsm.c (fsm-> fields instead of the old globals). The
     * anchors below lock them there; the adapter-side shapes (IsMcpsConfirm
     * early return, the absent tail, the timer never re-arming, the retire
     * loop, the no-budget log) stay locked on lora_app.c. */
    char *fsm  = slurp("../../Core/Src/tx_fsm.c");
    char *fnorm = normalize_code(fsm);
    free(fsm);

    cfg_yield_science_due =
        (strstr(fnorm, "if (fsm->state == TX_FSM_BULK_TRANSFER && TxFsm_ScienceIsDue(fsm, now_ms)) {") != NULL);
    cfg_lt07_probe =
        (strstr(fnorm, "fsm->state == TX_FSM_WAIT_PROBE_ACK && fsm->probe_sent_ms != 0 && (uint32_t)(in->now_ms - fsm->probe_sent_ms) > in->burst_max_open_ms") != NULL);
    cfg_lt07_burst =
        (strstr(fnorm, "fsm->state == TX_FSM_BULK_TRANSFER && fsm->burst_opened_ms != 0 && (uint32_t)(in->now_ms - fsm->burst_opened_ms) > in->burst_max_open_ms") != NULL);
    cfg_ascent_gate =
        (strstr(fnorm, "!in->mission_ascent") != NULL);
    cfg_defer_at_open =
        (strstr(fnorm, "out->defer_header_sync = true; fsm->burst_opened_ms = in->now_ms;") != NULL);
    cfg_flush_on_reset =
        (strstr(fnorm, "out->flush_header_sync = true; fsm->state = TX_FSM_PROBE_SF10; fsm->bulk_packets_sent = 0;") != NULL);
    cfg_phase_advance =
        (strstr(fnorm, "fsm->science_due_ms += interval_ms;") != NULL);
    cfg_rebase =
        (strstr(fnorm, "fsm->science_due_ms = now_ms + interval_ms;") != NULL);
    cfg_lt01_clamp =
        (strstr(fnorm, "if (remain_ms <= 0) remain_ms = 1;") != NULL);
    cfg_abort_park =
        (strstr(fnorm, "if (fsm->state == TX_FSM_PROBE_SF10) return false;") != NULL);
    free(fnorm);

    cfg_retire_unconvertible =
        (strstr(norm, "FlashLog_MarkRecoverySent(&hflashlog, flash_records[i].sequence);") != NULL);
    cfg_no_budget_complete =
        (strstr(norm, "Bulk: no payload budget at current DR") != NULL);

    char *timer_fn = func_slice(norm, "static void OnTxTimerEvent(void *context)");
    if (!timer_fn) { printf("FATAL: OnTxTimerEvent not found\n"); exit(2); }
    cfg_timer_no_bare_start = (strstr(timer_fn, "UTIL_TIMER_Start") == NULL);
    free(timer_fn);

    free(norm);
    free(app);

    printf("source scan: OnTxData tail terminates sent<=1 : %s\n",
           cfg_tail_terminates_low_counts ? "YES (BURST-01 present)" : "no");
    printf("source scan: OnTxData re-entrant on MlmeConfirm: %s\n",
           cfg_ontxdata_reentrant ? "YES (BURST-02 present)" : "no");
    printf("source scan: MAX_BULK_PACKETS_PER_CYCLE=%d margin>=%d gateways>=%d batt_min=%dmV burst_max_open=%dms\n\n",
           MAX_BULK_PACKETS_PER_CYCLE, LINK_MARGIN_THRESHOLD,
           GATEWAY_COUNT_THRESHOLD, BULK_BATTERY_MIN_MV, BURST_MAX_OPEN_MS);
}



/* ------------------------------------------------------------------ */
/* The replay                                                          */
/* ------------------------------------------------------------------ */

typedef enum { ST_PROBE, ST_WAIT_PROBE_ACK, ST_BULK, ST_COMPLETE } TxState_t;

static TxState_t g_tx_state;
static int  g_bulk_packets_sent;
static uint32_t g_probe_sent_ms;     /* LT-07: probe-ACK wait stamp */
static uint32_t g_burst_opened_ms;   /* LT-07: burst-open stamp */
static uint32_t g_science_due_ms;    /* R3-01: absolute science deadline */
/* R3-04 (#218): g_bulk_commit_through deleted - one-pass advance at send. */

static uint32_t now_ms;              /* the model clock (HAL_GetTick) */

/* environment knobs */
static bool env_has_unsent, env_ack, env_linkcheck, env_batt_good;
static int  env_margin, env_gateways;
static bool env_mission_ascent;      /* R3-03 (#217) */
static bool env_rf_silence;          /* T1 (DDR-0018) */
static uint32_t env_interval_ms;     /* plan.tx_interval_ms */
static int  env_pending_batches;     /* -1 = inexhaustible backlog */
static bool env_recovery_empty;      /* GetRecoveryRecords returns nothing */
static bool env_unconvertible;       /* every read record fails conversion */
static bool env_no_budget;           /* LoRaMacQueryTxPossible fails for all n */
static bool env_bulk_encode_fail, env_bulk_send_fail;
static bool env_probe_encode_fail, env_probe_send_fail;

/* observations */
static bool seq_task_armed;
static int  obs_archive_packets;     /* archive uplinks actually built */
static int  obs_heartbeats;          /* confirmed probe uplinks */
static int  obs_commits;             /* R3-04: send-time watermark advances */
static int  obs_nvm_tx_counter;      /* the every-10th-TX NVM store counter */
static int  obs_ctx_saves;           /* MultiRegion_SaveCurrentContext calls */
static bool obs_onrxdata_branch;     /* OnRxData packet-1 branch reached */
static int  obs_flush_calls;         /* FlashLog_FlushHeaderSync calls */
static int  obs_defer_calls;         /* FlashLog_DeferHeaderSync calls */
static int  obs_retire_no_tx;        /* unconvertible-batch retirements */
static int  obs_linkcheck_req;       /* LinkCheckReq piggybacks (first archive) */
static uint32_t obs_timer_delay_ms;  /* last TxTimer period programmed */

static void reset_world(void)
{
    g_tx_state = ST_PROBE;
    g_bulk_packets_sent = 0;
    g_probe_sent_ms = 0;
    g_burst_opened_ms = 0;
    g_science_due_ms = 0;
    now_ms = 1000;
    env_has_unsent = true; env_ack = true; env_linkcheck = true; env_batt_good = true;
    env_margin = LINK_MARGIN_THRESHOLD + 5;
    env_gateways = GATEWAY_COUNT_THRESHOLD + 2;
    env_mission_ascent = false;
    env_rf_silence = false;
    env_interval_ms = 60000;
    env_pending_batches = -1;
    env_recovery_empty = env_unconvertible = env_no_budget = false;
    env_bulk_encode_fail = env_bulk_send_fail = false;
    env_probe_encode_fail = env_probe_send_fail = false;
    seq_task_armed = false;
    obs_archive_packets = obs_heartbeats = obs_commits = 0;
    obs_nvm_tx_counter = obs_ctx_saves = 0;
    obs_onrxdata_branch = false;
    obs_flush_calls = obs_defer_calls = obs_retire_no_tx = obs_linkcheck_req = 0;
    obs_timer_delay_ms = 0;
}

static bool has_unsent(void)
{
    return env_has_unsent && env_pending_batches != 0;
}

/* lora_app.c ScienceIsDue(): wrap-safe absolute deadline test */
static bool ScienceIsDue(void)
{
    return g_science_due_ms != 0 &&
           (int32_t)(now_ms - g_science_due_ms) >= 0;
}

/* lora_app.c RescheduleScienceTimer(): phase-preserving advance on science
 * cycles, re-point only on bulk continuations, LT-01 "fire now" clamp. */
static void RescheduleScienceTimer(uint32_t interval_ms, bool science_cycle)
{
    if (science_cycle) {
        if (g_science_due_ms == 0) g_science_due_ms = now_ms;
        g_science_due_ms += interval_ms;
        if ((int32_t)(g_science_due_ms - now_ms) <= 0) {
            g_science_due_ms = now_ms + interval_ms;
        }
    } else if (g_science_due_ms == 0) {
        g_science_due_ms = now_ms + interval_ms;
    }
    int32_t remain_ms = (int32_t)(g_science_due_ms - now_ms);
    if (remain_ms <= 0) remain_ms = 1;
    obs_timer_delay_ms = (uint32_t)remain_ms;
}

/* lora_app.c OnTxData() */
static void OnTxData(bool is_mcps_confirm, bool status_ok, bool ack_received)
{
    if (!cfg_ontxdata_reentrant && !is_mcps_confirm) return;

    if (status_ok) {
        obs_ctx_saves++;
        obs_nvm_tx_counter++;
    }

    /* R3-04 (#218): commit-on-ACK block deleted - unconfirmed one-pass sends
     * advance the watermark at send time (modeled in SendTxData). */

    if (g_tx_state == ST_WAIT_PROBE_ACK) {
        g_probe_sent_ms = 0;   /* LT-07: the wait is resolved one way or the other */
        if (status_ok && ack_received) {
            if (env_batt_good && has_unsent() && !env_mission_ascent) {
                obs_defer_calls++;
                g_burst_opened_ms = now_ms;
                g_tx_state = ST_BULK;
                g_bulk_packets_sent = 0;
                seq_task_armed = true;
            } else {
                g_tx_state = ST_COMPLETE;
            }
        } else {
            g_tx_state = ST_COMPLETE;
        }
    }

    /* R3-04 (#218): the unACKed-archive fallback is deleted - archive frames
     * are unconfirmed (BR-TX-011); AckReceived is meaningless for them. */

    /* the tail under test */
    if (g_tx_state == ST_BULK && g_bulk_packets_sent > 1) {
        if (has_unsent() && g_bulk_packets_sent < MAX_BULK_PACKETS_PER_CYCLE) {
            seq_task_armed = true;
        } else {
            g_tx_state = ST_COMPLETE;
            g_bulk_packets_sent = 0;
        }
    } else if (cfg_tail_terminates_low_counts && g_tx_state == ST_BULK) {
        g_tx_state = ST_COMPLETE;
        g_bulk_packets_sent = 0;
    }
}

/* lora_app.c OnRxData() */
static void OnRxData(void)
{
    if (g_tx_state == ST_BULK && g_bulk_packets_sent == 1) {
        obs_onrxdata_branch = true;
        bool link_good = (env_linkcheck &&
                          env_margin >= LINK_MARGIN_THRESHOLD &&
                          env_gateways >= GATEWAY_COUNT_THRESHOLD);
        if (!link_good) {
            g_tx_state = ST_COMPLETE;
            g_bulk_packets_sent = 0;
        } else if (has_unsent() && g_bulk_packets_sent < MAX_BULK_PACKETS_PER_CYCLE) {
            seq_task_armed = true;
        } else {
            g_tx_state = ST_COMPLETE;
            g_bulk_packets_sent = 0;
        }
    }
}

/* lora_app.c OnTxTimerEvent(): arms the send task; NEVER re-arms the timer
 * itself (LT-07 - the period is owned solely by RescheduleScienceTimer). */
static void OnTxTimerEvent(void)
{
    seq_task_armed = true;
}

/* lora_app.c FirstFlightAbortTransmitCycle(): a low-energy decision wins over
 * every open transmit state; parks the FSM and discards burst-scoped timing. */
static void FirstFlightAbortTransmitCycle(void)
{
    if (g_tx_state == ST_PROBE) return;
    obs_flush_calls++;
    g_tx_state = ST_PROBE;
    g_bulk_packets_sent = 0;
    g_probe_sent_ms = 0;
    g_burst_opened_ms = 0;
}


/* lora_app.c SendTxData()/RunTxStateMachine(); returns true if an uplink went out.
 * Firmware order: entry yield -> (power/GPS/archive, not modelled) ->
 * RescheduleScienceTimer -> RunTxStateMachine. */
static bool SendTxData(bool *carried_linkcheckreq)
{
    seq_task_armed = false;
    *carried_linkcheckreq = false;

    /* R3-01 (#215): a bulk continuation yields when the science deadline
     * arrives - current science always wins (DDR-0005 BR-TX-001/002). */
    if (g_tx_state == ST_BULK && ScienceIsDue()) {
        g_tx_state = ST_COMPLETE;
        g_bulk_packets_sent = 0;
    }

    RescheduleScienceTimer(env_interval_ms, g_tx_state != ST_BULK);

    /* --- RunTxStateMachine --- */
    /* LT-07 (#277): burst-scoped deadlines force a stuck network-wait state
     * out; the stale-state reset below then runs as for any completed cycle. */
    if (g_tx_state == ST_WAIT_PROBE_ACK && g_probe_sent_ms != 0 &&
        (uint32_t)(now_ms - g_probe_sent_ms) > (uint32_t)BURST_MAX_OPEN_MS) {
        g_tx_state = ST_COMPLETE;
        g_probe_sent_ms = 0;
    }
    if (g_tx_state == ST_BULK && g_burst_opened_ms != 0 &&
        (uint32_t)(now_ms - g_burst_opened_ms) > (uint32_t)BURST_MAX_OPEN_MS) {
        g_tx_state = ST_COMPLETE;
        g_burst_opened_ms = 0;
        g_bulk_packets_sent = 0;
    }

    /* C2/SP-15: stale states reset to PROBE; the deferred header sync flushes
     * exactly once here (Finding #8). BULK_TRANSFER is NOT reset. */
    if (g_tx_state == ST_WAIT_PROBE_ACK || g_tx_state == ST_COMPLETE) {
        obs_flush_calls++;
        g_tx_state = ST_PROBE;
        g_bulk_packets_sent = 0;
    }

    /* T1 (DDR-0018) + F-5 (#180): RF silence parks the FSM (with a flush so a
     * deferred watermark is never stranded) and skips all transmit work. */
    if (env_rf_silence) {
        obs_flush_calls++;
        g_tx_state = ST_PROBE;
        return false;
    }

    if (g_tx_state == ST_PROBE) {
        if (env_probe_encode_fail || env_probe_send_fail) {
            g_tx_state = ST_COMPLETE;   /* complete cycle on error */
            return false;
        }
        g_probe_sent_ms = now_ms;       /* LT-07: bound the wait */
        g_tx_state = ST_WAIT_PROBE_ACK;
        obs_heartbeats++;
        return true;
    }
    if (g_tx_state == ST_BULK) {
        if (!has_unsent() || g_bulk_packets_sent >= MAX_BULK_PACKETS_PER_CYCLE) {
            g_tx_state = ST_COMPLETE;
            return false;
        }
        /* firmware order: read -> convert -> budget -> encode -> LinkCheckReq
         * (first packet) -> send. Every failure path ends the cycle. */
        if (env_recovery_empty)   { g_tx_state = ST_COMPLETE; return false; }
        if (env_unconvertible) {
            obs_retire_no_tx++;         /* R21/FR-09: retire batch, no TX */
            g_tx_state = ST_COMPLETE;
            return false;
        }
        if (env_no_budget)        { g_tx_state = ST_COMPLETE; return false; }
        if (env_bulk_encode_fail) { g_tx_state = ST_COMPLETE; return false; }
        *carried_linkcheckreq = (g_bulk_packets_sent == 0);  /* protocol §5.2 */
        if (*carried_linkcheckreq) obs_linkcheck_req++;
        if (env_bulk_send_fail)   { g_tx_state = ST_COMPLETE; return false; }
        g_bulk_packets_sent++;
        obs_archive_packets++;
        obs_commits++;   /* R3-04: watermark advance AT SEND TIME */
        if (env_pending_batches > 0) env_pending_batches--;
        if (!has_unsent() || g_bulk_packets_sent >= MAX_BULK_PACKETS_PER_CYCLE)
            g_tx_state = ST_COMPLETE;
        return true;
    }
    g_tx_state = ST_PROBE;
    g_bulk_packets_sent = 0;
    return false;
}

/* one LoRaMacProcess pass after an uplink completes */
static void mac_process(bool carried_mlme)
{
    OnTxData(true, true, env_ack);                 /* McpsConfirm  */
    if (carried_mlme) OnTxData(false, true, env_ack); /* MlmeConfirm: same callback */
    OnRxData();                                    /* McpsIndication */
}

/* Drive one full work cycle chain to quiescence (or the wedge bound). */
static int run_cycle(int bound)
{
    bool mlme;
    int iterations = 0;
    bool sent = SendTxData(&mlme);
    if (!sent) return 0;
    mac_process(mlme);
    while (seq_task_armed && iterations < bound) {
        iterations++;
        sent = SendTxData(&mlme);
        if (!sent) break;
        mac_process(mlme);
    }
    return iterations;
}


/* ------------------------------------------------------------------ */
/* Tests                                                               */
/* ------------------------------------------------------------------ */

/* T-B1: the probe ACK opens the archive opportunity. OnTxData's tail must not
 * close it in the same call. */
static void test_probe_ack_opportunity_survives(void)
{
    printf("-- T-B1 / BURST-01: probe ACK must leave the archive opportunity open\n");
    reset_world();
    bool mlme;
    (void)SendTxData(&mlme);                  /* confirmed heartbeat */
    mac_process(false);
    printf("   after probe ACK: state=%d bulk_sent=%d armed=%d (want state=BULK)\n",
           g_tx_state, g_bulk_packets_sent, seq_task_armed);
    CHECK_REGRESSION(g_tx_state == ST_BULK, "BURST-01");
    CHECK(seq_task_armed);
}

/* T-B2: a healthy link with backlog must drain up to MAX_BULK_PACKETS_PER_CYCLE
 * archive packets, and must not burn extra heartbeats doing it. */
static void test_full_burst_reaches_cap(void)
{
    printf("-- T-B2 / BURST-01: healthy burst drains to the packet cap\n");
    reset_world();
    int iterations = run_cycle(200);
    printf("   archive packets=%d (want %d)  heartbeats=%d (want 1)  "
           "commits=%d  iterations=%d\n",
           obs_archive_packets, MAX_BULK_PACKETS_PER_CYCLE,
           obs_heartbeats, obs_commits, iterations);
    CHECK_REGRESSION(obs_archive_packets == MAX_BULK_PACKETS_PER_CYCLE, "BURST-01");
    CHECK_REGRESSION(obs_heartbeats == 1, "BURST-01");
    CHECK_REGRESSION(obs_commits == MAX_BULK_PACKETS_PER_CYCLE, "BURST-01");
}

/* T-B3: the wedge. With the tail terminating low counts, every ACKed heartbeat
 * re-opens and re-kills the opportunity, and the armed task spends a full work
 * cycle sending another heartbeat. The cycle must terminate. */
static void test_no_unbounded_heartbeat_loop(void)
{
    printf("-- T-B3 / BURST-01: work cycle must not self-perpetuate\n");
    reset_world();
    const int bound = 500;
    int iterations = run_cycle(bound);
    printf("   iterations=%d (bound %d)  heartbeats=%d  archive packets=%d\n",
           iterations, bound, obs_heartbeats, obs_archive_packets);
    CHECK_REGRESSION(iterations < bound, "BURST-01");
    CHECK_REGRESSION(obs_heartbeats <= 1, "BURST-01");
}

/* T-B4: OnRxData owns the packet-1 verdict (FR-14/#88 must stay fixed) and
 * must actually be reachable. */
static void test_onrxdata_owns_packet_one(void)
{
    printf("-- T-B4 / BURST-01: OnRxData packet-1 LinkCheck branch is reachable\n");
    reset_world();
    (void)run_cycle(200);
    CHECK_REGRESSION(obs_onrxdata_branch, "BURST-01");

    printf("-- T-B4b: poor link after packet 1 falls back to heartbeat mode\n");
    reset_world();
    env_margin = LINK_MARGIN_THRESHOLD - 1;
    (void)run_cycle(200);
    printf("   archive packets=%d (want exactly 1)\n", obs_archive_packets);
    CHECK_REGRESSION(obs_archive_packets == 1, "BURST-01");

    printf("-- T-B4c: missing LinkCheckAns after packet 1 also falls back\n");
    reset_world();
    env_linkcheck = false;
    (void)run_cycle(200);
    printf("   archive packets=%d (want exactly 1)\n", obs_archive_packets);
    CHECK_REGRESSION(obs_archive_packets == 1, "BURST-01");
}

/* T-B5: DDR-0005 BR-TX-003/004 — the compact probe stays CONFIRMED and its
 * ACK gates the archive opportunity: no probe ACK, no recovery at all.
 * (R3-04/#218 reframing: archive frames themselves are unconfirmed
 * one-pass; the legacy "unACKed archive ends the burst" rule is gone.) */
static void test_unacked_archive_packet_falls_back(void)
{
    printf("-- T-B5 / DDR-0005: no probe ACK -> no archive opportunity\n");
    reset_world();
    env_ack = false;
    (void)run_cycle(200);
    printf("   archive packets=%d  sends-marked=%d (want 0)\n",
           obs_archive_packets, obs_commits);
    CHECK(obs_archive_packets == 0);
    CHECK(obs_commits == 0);
    CHECK(g_tx_state != ST_BULK);
}

/* T-B6: BURST-02 — one uplink, one accounting event. The first archive packet
 * carries LinkCheckReq, so LmHandler calls OnTxData twice for it. */
static void test_one_uplink_one_accounting_event(void)
{
    printf("-- T-B6 / BURST-02: MlmeConfirm must not double-count an uplink\n");
    reset_world();
    (void)run_cycle(200);
    int uplinks = obs_heartbeats + obs_archive_packets;
    printf("   uplinks=%d  context saves=%d  NVM-store ticks=%d (want == uplinks)\n",
           uplinks, obs_ctx_saves, obs_nvm_tx_counter);
    CHECK_REGRESSION(obs_ctx_saves == uplinks, "BURST-02");
    CHECK_REGRESSION(obs_nvm_tx_counter == uplinks, "BURST-02");
}



/* ------------------------------------------------------------------ */
/* Stage-5 characterisation: the rest of the TX FSM decision space.    */
/* Every assertion below encodes CURRENT firmware behaviour, locked    */
/* to the source by the scan anchors (test_shape_anchors).             */
/* ------------------------------------------------------------------ */

static void test_shape_anchors(void)
{
    printf("-- T-C0: firmware shape anchors for the characterised decisions\n");
    CHECK(cfg_yield_science_due);      /* R3-01: bulk yields to science */
    CHECK(cfg_lt07_probe);             /* LT-07: probe-ACK wait bound */
    CHECK(cfg_lt07_burst);             /* LT-07: burst-open bound */
    CHECK(cfg_ascent_gate);            /* R3-03: ASCENT inhibits archive */
    CHECK(cfg_defer_at_open);          /* header sync deferred at burst open */
    CHECK(cfg_flush_on_reset);         /* flush at stale-state reset */
    CHECK(cfg_phase_advance);          /* deadline advances phase-wise */
    CHECK(cfg_rebase);                 /* re-base when a period behind */
    CHECK(cfg_lt01_clamp);             /* overdue deadline -> fire now */
    CHECK(cfg_timer_no_bare_start);    /* OnTxTimerEvent never re-arms timer */
    CHECK(cfg_abort_park);             /* first-flight abort parks the FSM */
    CHECK(cfg_retire_unconvertible);   /* unconvertible batch retired w/o TX */
    CHECK(cfg_no_budget_complete);     /* no payload budget -> retry next cycle */
}

/* T-C1: LT-07 - a probe the network never confirms is forcibly completed
 * once the wait outlives BURST_MAX_OPEN_MS; the same invocation then resets
 * (flushing the deferred sync) and sends a fresh probe. */
static void test_lt07_stale_probe_wait(void)
{
    printf("-- T-C1 / LT-07: stale probe-ACK wait is forced out\n");
    reset_world();
    bool mlme;
    (void)SendTxData(&mlme);              /* probe out; network stays silent */
    now_ms += (uint32_t)BURST_MAX_OPEN_MS + 1;
    (void)SendTxData(&mlme);
    printf("   heartbeats=%d flushes=%d state=%d (want 2, >=1, WAIT)\n",
           obs_heartbeats, obs_flush_calls, g_tx_state);
    CHECK(obs_heartbeats == 2);
    CHECK(obs_flush_calls >= 1);
    CHECK(g_tx_state == ST_WAIT_PROBE_ACK);
    CHECK(g_probe_sent_ms == now_ms);     /* fresh wait stamped */
}

/* T-C2: LT-07 - a burst that outlives BURST_MAX_OPEN_MS is forced complete
 * (timestamps zeroed), then the cycle proceeds as a fresh science probe.
 * env_interval_ms > BURST_MAX_OPEN_MS so the R3-01 yield cannot fire first. */
static void test_lt07_stale_burst(void)
{
    printf("-- T-C2 / LT-07: stale burst is forced out\n");
    reset_world();
    env_interval_ms = 3600000;
    bool mlme;
    (void)SendTxData(&mlme);
    mac_process(false);                   /* ACK -> burst open */
    CHECK(g_tx_state == ST_BULK);
    now_ms += (uint32_t)BURST_MAX_OPEN_MS + 1;
    (void)SendTxData(&mlme);
    printf("   heartbeats=%d flushes=%d opened=%lu (want 2, >=1, 0)\n",
           obs_heartbeats, obs_flush_calls, (unsigned long)g_burst_opened_ms);
    CHECK(g_tx_state == ST_WAIT_PROBE_ACK);
    CHECK(obs_heartbeats == 2);
    CHECK(obs_flush_calls >= 1);
    CHECK(g_burst_opened_ms == 0);
}

/* T-C3: a RESOLVED wait (NACK arrived, probe stamp zeroed) needs no timeout -
 * the plain stale-state reset flushes and re-probes. */
static void test_stale_reset_after_nack(void)
{
    printf("-- T-C3: stale-state reset after a resolved wait\n");
    reset_world();
    bool mlme;
    (void)SendTxData(&mlme);
    OnTxData(true, true, false);          /* NACK resolves the wait */
    CHECK(g_tx_state == ST_COMPLETE);
    CHECK(g_probe_sent_ms == 0);
    now_ms += 1000;
    (void)SendTxData(&mlme);
    CHECK(obs_heartbeats == 2);
    CHECK(obs_flush_calls >= 1);
    CHECK(g_tx_state == ST_WAIT_PROBE_ACK);
}

/* T-C4: T1/F-5 - RF silence parks a mid-burst FSM (with flush) and sends
 * nothing. */
static void test_rf_silence_parks_mid_burst(void)
{
    printf("-- T-C4 / T1+F-5: RF silence parks the FSM mid-burst\n");
    reset_world();
    env_interval_ms = 3600000;
    bool mlme;
    (void)SendTxData(&mlme);
    mac_process(false);                   /* burst open */
    env_rf_silence = true;
    bool sent = SendTxData(&mlme);
    CHECK(!sent);
    CHECK(g_tx_state == ST_PROBE);
    CHECK(obs_flush_calls >= 1);
    CHECK(obs_archive_packets == 0);
    CHECK(obs_heartbeats == 1);
}

/* T-C5: R3-01 - a bulk continuation arriving at/after the science deadline
 * yields: the burst is completed and the same invocation runs a fresh
 * science probe. The deadline advances phase-preservingly. */
static void test_science_deadline_yield(void)
{
    printf("-- T-C5 / R3-01: science deadline preempts a bulk continuation\n");
    reset_world();
    bool mlme;
    (void)SendTxData(&mlme);
    mac_process(false);                   /* burst open; due was set to now+interval */
    CHECK(g_tx_state == ST_BULK);
    now_ms += 60000;                      /* exactly at the deadline */
    (void)SendTxData(&mlme);
    CHECK(g_tx_state == ST_WAIT_PROBE_ACK);   /* fresh probe, not another bulk */
    CHECK(obs_heartbeats == 2);
    CHECK(obs_archive_packets == 0);
    CHECK(g_science_due_ms == 1000 + 120000); /* phase-advanced from 61000 */
}

/* T-C6/7/8: R3-03 / battery / cache gates at the probe-ACK - each one alone
 * is sufficient to refuse the archive opportunity. */
static void test_burst_open_gates(void)
{
    printf("-- T-C6 / R3-03: ASCENT inhibits the archive opportunity\n");
    reset_world();
    env_mission_ascent = true;
    bool mlme;
    (void)SendTxData(&mlme);
    mac_process(false);
    CHECK(g_tx_state == ST_COMPLETE);
    CHECK(obs_defer_calls == 0);
    CHECK(!seq_task_armed);

    printf("-- T-C7: low battery refuses the archive opportunity\n");
    reset_world();
    env_batt_good = false;
    (void)SendTxData(&mlme);
    mac_process(false);
    CHECK(g_tx_state == ST_COMPLETE);
    CHECK(obs_defer_calls == 0);
    CHECK(!seq_task_armed);

    printf("-- T-C8: empty backlog refuses the archive opportunity\n");
    reset_world();
    env_has_unsent = false;
    (void)SendTxData(&mlme);
    mac_process(false);
    CHECK(g_tx_state == ST_COMPLETE);
    CHECK(obs_defer_calls == 0);
    CHECK(!seq_task_armed);
}

/* T-C9: RescheduleScienceTimer arithmetic - phase advance, re-base when a
 * full period behind, bulk continuations never move the deadline, and the
 * LT-01 "fire now" clamp on an overdue deadline. */
static void test_reschedule_semantics(void)
{
    printf("-- T-C9 / R3-01+LT-01: science-deadline arithmetic\n");
    reset_world();                        /* now_ms = 1000 */
    RescheduleScienceTimer(60000, true);  /* establish phase */
    CHECK(g_science_due_ms == 61000);
    CHECK(obs_timer_delay_ms == 60000);
    now_ms = 2000;
    RescheduleScienceTimer(60000, true);  /* phase advance, not now+interval */
    CHECK(g_science_due_ms == 121000);
    CHECK(obs_timer_delay_ms == 119000);
    now_ms = 200000;                      /* more than one period behind */
    RescheduleScienceTimer(60000, true);  /* re-base: never storm catch-up */
    CHECK(g_science_due_ms == 260000);
    CHECK(obs_timer_delay_ms == 60000);
    now_ms = 201000;
    RescheduleScienceTimer(60000, false); /* bulk continuation: re-point only */
    CHECK(g_science_due_ms == 260000);
    CHECK(obs_timer_delay_ms == 59000);
    now_ms = 300000;                      /* deadline already past */
    RescheduleScienceTimer(60000, false); /* LT-01: clamp to "fire now" */
    CHECK(g_science_due_ms == 260000);
    CHECK(obs_timer_delay_ms == 1);
}

/* T-C10: ScienceIsDue - unscheduled is never due, exact arrival is due,
 * and the comparison is wrap-safe. */
static void test_science_is_due(void)
{
    printf("-- T-C10 / R3-01: ScienceIsDue boundary and wrap\n");
    reset_world();
    CHECK(!ScienceIsDue());               /* unscheduled */
    g_science_due_ms = 5000;
    now_ms = 4999;
    CHECK(!ScienceIsDue());
    now_ms = 5000;
    CHECK(ScienceIsDue());                /* exact arrival counts */
    g_science_due_ms = 0xFFFFFF00u;
    now_ms = 0xFFFFFE00u;
    CHECK(!ScienceIsDue());               /* 256 ms before due (pre-wrap) */
    now_ms = 200;                         /* wrapped past due */
    CHECK(ScienceIsDue());
}

/* T-C11: OnTxTimerEvent arms the send task and nothing else (the scan anchor
 * proves it never touches the timer). */
static void test_tx_timer_event_arms_only(void)
{
    printf("-- T-C11 / LT-07: OnTxTimerEvent arms the task, never the timer\n");
    reset_world();
    OnTxTimerEvent();
    CHECK(seq_task_armed);
    CHECK(obs_timer_delay_ms == 0);       /* no reschedule happened */
}

/* T-C12: FirstFlightAbortTransmitCycle - a no-op from PROBE; from BULK it
 * flushes, parks, and discards every burst-scoped stamp. */
static void test_first_flight_abort(void)
{
    printf("-- T-C12: first-flight abort parks the FSM\n");
    reset_world();
    FirstFlightAbortTransmitCycle();      /* already parked: no-op */
    CHECK(g_tx_state == ST_PROBE);
    CHECK(obs_flush_calls == 0);

    bool mlme;
    (void)SendTxData(&mlme);
    mac_process(false);                   /* burst open */
    CHECK(g_tx_state == ST_BULK);
    FirstFlightAbortTransmitCycle();
    CHECK(g_tx_state == ST_PROBE);
    CHECK(g_bulk_packets_sent == 0);
    CHECK(g_probe_sent_ms == 0);
    CHECK(g_burst_opened_ms == 0);
    CHECK(obs_flush_calls >= 1);
}


/* T-C13: no LoRaMac payload budget at the current DR - the cycle ends with
 * NOTHING sent and NOTHING marked; the records stay pending for next cycle.
 * The budget check runs BEFORE the first-packet LinkCheckReq. */
static void test_bulk_no_payload_budget(void)
{
    printf("-- T-C13: no payload budget -> retry next cycle, nothing marked\n");
    reset_world();
    env_no_budget = true;
    (void)run_cycle(50);
    CHECK(obs_archive_packets == 0);
    CHECK(obs_commits == 0);
    CHECK(obs_heartbeats == 1);
    CHECK(obs_linkcheck_req == 0);
    CHECK(g_tx_state == ST_COMPLETE);
}

/* T-C14: R21/FR-09 - a batch that reads CRC-clean but fails conversion is
 * retired WITHOUT a transmission (leaving it pending would wedge bulk
 * transfer re-probing it forever). */
static void test_bulk_unconvertible_retired(void)
{
    printf("-- T-C14 / R21+FR-09: unconvertible batch retired without TX\n");
    reset_world();
    env_unconvertible = true;
    (void)run_cycle(50);
    CHECK(obs_retire_no_tx == 1);
    CHECK(obs_archive_packets == 0);
    CHECK(obs_commits == 0);
    CHECK(g_tx_state == ST_COMPLETE);
}

/* T-C15: send/encode failures end the cycle. A failed bulk send marks
 * NOTHING (the watermark advanced at send time only for accepted sends);
 * a failed probe never enters the wait. */
static void test_send_failure_paths(void)
{
    printf("-- T-C15: send/encode failure paths complete the cycle\n");
    reset_world();
    env_bulk_send_fail = true;
    (void)run_cycle(50);
    CHECK(obs_linkcheck_req == 1);        /* LinkCheckReq precedes the send */
    CHECK(obs_archive_packets == 0);
    CHECK(obs_commits == 0);
    CHECK(g_tx_state == ST_COMPLETE);

    reset_world();
    env_probe_send_fail = true;
    (void)run_cycle(50);
    CHECK(obs_heartbeats == 0);
    CHECK(g_tx_state == ST_COMPLETE);

    reset_world();
    env_probe_encode_fail = true;
    (void)run_cycle(50);
    CHECK(obs_heartbeats == 0);
    CHECK(g_tx_state == ST_COMPLETE);

    reset_world();
    env_bulk_encode_fail = true;
    (void)run_cycle(50);
    CHECK(obs_archive_packets == 0);
    CHECK(obs_linkcheck_req == 0);        /* encode fails before LinkCheckReq */
    CHECK(g_tx_state == ST_COMPLETE);
}

/* T-C16: when the first archive packet exactly drains the backlog, the cycle
 * completes AT THE SEND SITE - the OnRxData packet-1 verdict is never
 * evaluated (moot: nothing left to send). */
static void test_exact_drain_completes_at_send(void)
{
    printf("-- T-C16: exact one-packet drain completes at the send site\n");
    reset_world();
    env_pending_batches = 1;
    (void)run_cycle(50);
    CHECK(obs_archive_packets == 1);
    CHECK(obs_commits == 1);
    CHECK(!obs_onrxdata_branch);          /* verdict moot, never evaluated */
    CHECK(g_tx_state == ST_COMPLETE);
}

/* T-C17: recovery read returning nothing (empty archive between the
 * HasUnsentData check and the read) completes the cycle without sends. */
static void test_bulk_recovery_empty(void)
{
    printf("-- T-C17: empty recovery read completes the cycle\n");
    reset_world();
    env_recovery_empty = true;
    (void)run_cycle(50);
    CHECK(obs_archive_packets == 0);
    CHECK(obs_commits == 0);
    CHECK(g_tx_state == ST_COMPLETE);
}

int main(void)
{
    printf("=== archive burst FSM regressions (BURST-01, BURST-02) + TX FSM characterisation ===\n\n");
    scan_firmware();

    test_probe_ack_opportunity_survives();
    test_full_burst_reaches_cap();
    test_no_unbounded_heartbeat_loop();
    test_onrxdata_owns_packet_one();
    test_unacked_archive_packet_falls_back();
    test_one_uplink_one_accounting_event();

    test_shape_anchors();
    test_lt07_stale_probe_wait();
    test_lt07_stale_burst();
    test_stale_reset_after_nack();
    test_rf_silence_parks_mid_burst();
    test_science_deadline_yield();
    test_burst_open_gates();
    test_reschedule_semantics();
    test_science_is_due();
    test_tx_timer_event_arms_only();
    test_first_flight_abort();
    test_bulk_no_payload_budget();
    test_bulk_unconvertible_retired();
    test_send_failure_paths();
    test_exact_drain_completes_at_send();
    test_bulk_recovery_empty();

    printf("\n%d checks, %d failures (%d expected pre-fix)\n",
           g_checks, g_failures, g_expected_failures);

    const char *expect_unfixed = getenv("EXPECT_UNFIXED");
    if (expect_unfixed != NULL && expect_unfixed[0] == '1') {
        int unexpected = g_failures - g_expected_failures;
        if (unexpected < 0) {
            printf("UNEXPECTED PASS: a documented bug appears FIXED — flip this "
                   "check from CHECK_REGRESSION to CHECK.\n");
            return 1;
        }
        if (unexpected > 0) {
            printf("NEW BREAKAGE beyond the documented findings.\n");
            return 1;
        }
        printf("OK (baseline): only the documented findings fail.\n");
        return 0;
    }

    if (g_failures == 0) { printf("OK\n"); return 0; }
    return 1;
}

